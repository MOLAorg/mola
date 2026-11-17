/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola
 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/
/**
 * @file   VideoRecorder.cpp
 * @brief  Records the GUI 3D view to a video file, through ffmpeg.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include "VideoRecorder.h"

#include <mrpt/opengl/opengl_api.h>

#include <algorithm>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace mola
{
namespace
{
/// The string in single quotes, for a shell command line.
std::string shell_quote(const std::string& s)
{
  std::string out = "'";
  for (const char c : s)
  {
    if (c == '\'')
    {
      out += "'\\''";
    }
    else
    {
      out += c;
    }
  }
  return out + "'";
}
}  // namespace

VideoRecorder::VideoRecorder(Params p) : params_(std::move(p))
{
  if (params_.width <= 0 || params_.height <= 0 || params_.fps <= 0)
  {
    throw std::invalid_argument("VideoRecorder: width, height and fps must be positive");
  }
}

VideoRecorder::~VideoRecorder()
{
  // The GL objects are left to the context (it may not be current here), but
  // the video must be finished:
  if (pipe_ != nullptr)
  {
    pclose(pipe_);
    pipe_ = nullptr;
  }
}

void VideoRecorder::create_gl_objects()
{
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

  glGenRenderbuffers(1, &color_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, color_rb_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, params_.width, params_.height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rb_);

  glGenRenderbuffers(1, &depth_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rb_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, params_.width, params_.height);
  glFramebufferRenderbuffer(
      GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    throw std::runtime_error("VideoRecorder: incomplete framebuffer");
  }
}

void VideoRecorder::open_pipe()
{
  // If ffmpeg dies, writing to its pipe must fail, not kill the process:
  std::signal(SIGPIPE, SIG_IGN);

  const std::string cmd = "ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -s " +
                          std::to_string(params_.width) + "x" + std::to_string(params_.height) +
                          " -r " + std::to_string(params_.fps) + " -i - -vf vflip " +
                          params_.ffmpeg_options + " " + shell_quote(params_.output);
  pipe_ = popen(cmd.c_str(), "w");
  if (pipe_ == nullptr)
  {
    throw std::runtime_error("VideoRecorder: cannot run: " + cmd);
  }
  std::cout << "[VideoRecorder] Recording " << params_.width << "x" << params_.height << " @ "
            << params_.fps << " fps to: " << params_.output << "\n";
}

void VideoRecorder::begin_frame()
{
  if (failed_)
  {
    return;
  }
  try
  {
    if (fbo_ == 0)
    {
      create_gl_objects();
      open_pipe();
      pixels_.resize(static_cast<std::size_t>(params_.width) * params_.height * 3);
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "[VideoRecorder] Disabled: " << e.what() << "\n";
    failed_ = true;
    return;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glViewport(0, 0, params_.width, params_.height);
}

void VideoRecorder::end_frame(int windowWidth, int windowHeight)
{
  if (failed_ || fbo_ == 0)
  {
    return;
  }

  // One video frame per rendered frame: the GUI loop runs at the video rate.
  const auto now = std::chrono::steady_clock::now();
  if (!t0_)
  {
    t0_ = now;
  }
  last_frame_ = now;
  if (pipe_ != nullptr)
  {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, params_.width, params_.height, GL_RGB, GL_UNSIGNED_BYTE, pixels_.data());
    if (fwrite(pixels_.data(), 1, pixels_.size(), pipe_) != pixels_.size())
    {
      std::cerr << "[VideoRecorder] Writing to ffmpeg failed: recording stopped.\n";
      pclose(pipe_);
      pipe_   = nullptr;
      failed_ = true;
    }
    else
    {
      frames_written_++;
    }
    // Report the rendered rate now and then (a rate below fps speeds up the video):
    if (!window_start_)
    {
      window_start_  = now;
      window_frames_ = frames_written_;
    }
    if (const double dt = std::chrono::duration<double>(now - *window_start_).count(); dt > 30.0)
    {
      std::cout << "[VideoRecorder] " << frames_written_ << " frames, last " << dt << " s at "
                << static_cast<double>(frames_written_ - window_frames_) / dt << " fps\n";
      std::cout.flush();
      window_start_  = now;
      window_frames_ = frames_written_;
    }
  }

  // Preview in the window, keeping the aspect ratio:
  glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glViewport(0, 0, windowWidth, windowHeight);
  glClearColor(0, 0, 0, 1);
  glClear(GL_COLOR_BUFFER_BIT);
  const double s = std::min(
      static_cast<double>(windowWidth) / params_.width,
      static_cast<double>(windowHeight) / params_.height);
  const int w  = static_cast<int>(s * params_.width);
  const int h  = static_cast<int>(s * params_.height);
  const int x0 = (windowWidth - w) / 2;
  const int y0 = (windowHeight - h) / 2;
  glBlitFramebuffer(
      0, 0, params_.width, params_.height, x0, y0, x0 + w, y0 + h, GL_COLOR_BUFFER_BIT, GL_LINEAR);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

double VideoRecorder::effective_fps() const
{
  if (!t0_ || !last_frame_ || frames_written_ < 2)
  {
    return 0;
  }
  const double dt = std::chrono::duration<double>(*last_frame_ - *t0_).count();
  return dt > 0 ? static_cast<double>(frames_written_ - 1) / dt : 0;
}

void VideoRecorder::close()
{
  if (pipe_ != nullptr)
  {
    pclose(pipe_);
    pipe_ = nullptr;
    std::cout << "[VideoRecorder] Done: " << frames_written_ << " frames, " << effective_fps()
              << " frames per second, in: " << params_.output << "\n";
  }
  if (fbo_ != 0)
  {
    glDeleteFramebuffers(1, &fbo_);
    glDeleteRenderbuffers(1, &color_rb_);
    glDeleteRenderbuffers(1, &depth_rb_);
    fbo_ = 0;
  }
}

}  // namespace mola
