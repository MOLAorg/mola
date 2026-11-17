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

#include <mrpt/core/format.h>
#include <mrpt/opengl/opengl_api.h>
#include <pthread.h>
#include <sys/wait.h>

#include <algorithm>
#include <csignal>
#include <cstring>
#include <ctime>
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

/** Blocks SIGPIPE in the calling thread (the one writing to ffmpeg), so a
 *  write to a dead ffmpeg fails with EPIPE instead of killing the process,
 *  without changing the signal disposition of the rest of the process. */
void block_sigpipe_in_this_thread()
{
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGPIPE);
  pthread_sigmask(SIG_BLOCK, &set, nullptr);
}

/// Discards a SIGPIPE left pending in this thread by a failed write.
void consume_pending_sigpipe()
{
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGPIPE);
  const timespec zero{0, 0};
  while (sigtimedwait(&set, nullptr, &zero) == SIGPIPE)
  {
  }
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
  close_pipe();
}

void VideoRecorder::log(mrpt::system::VerbosityLevel level, const std::string& msg) const
{
  if (params_.logger)
  {
    params_.logger(level, msg);
    return;
  }
  (level >= mrpt::system::LVL_WARN ? std::cerr : std::cout) << "[VideoRecorder] " << msg << "\n";
}

void VideoRecorder::fail(const std::string& why)
{
  log(mrpt::system::LVL_ERROR, "Recording stopped: " + why);
  failed_ = true;
  close_pipe();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void VideoRecorder::create_gl_objects()
{
  // Same color encoding as the window, so the video looks like it (the scene
  // is gamma-encoded only into sRGB-capable framebuffers):
  GLint encoding = GL_LINEAR;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glGetFramebufferAttachmentParameteriv(
      GL_FRAMEBUFFER, GL_BACK_LEFT, GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING, &encoding);
  while (glGetError() != GL_NO_ERROR)
  {
  }
  const GLenum colorFormat = encoding == GL_SRGB ? GL_SRGB8_ALPHA8 : GL_RGBA8;

  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

  glGenRenderbuffers(1, &color_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, color_rb_);
  glRenderbufferStorage(GL_RENDERBUFFER, colorFormat, params_.width, params_.height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rb_);

  glGenRenderbuffers(1, &depth_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rb_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, params_.width, params_.height);
  glFramebufferRenderbuffer(
      GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  if (!complete)
  {
    throw std::runtime_error("incomplete off-screen framebuffer");
  }
}

void VideoRecorder::open_pipe()
{
  block_sigpipe_in_this_thread();

  // In its own session: a terminal CTRL+C must not stop ffmpeg before us.
  const std::string cmd = "setsid ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgba -s " +
                          std::to_string(params_.width) + "x" + std::to_string(params_.height) +
                          " -r " + std::to_string(params_.fps) + " -i - " + params_.ffmpeg_options +
                          " " + shell_quote(params_.output);
  pipe_ = popen(cmd.c_str(), "w");
  if (pipe_ == nullptr)
  {
    throw std::runtime_error("cannot run: " + cmd);
  }
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
      pixels_.resize(static_cast<std::size_t>(params_.width) * params_.height * 4);
      row_.resize(static_cast<std::size_t>(params_.width) * 4);
      open_pipe();
    }
  }
  catch (const std::exception& e)
  {
    fail(e.what());
    return;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glViewport(0, 0, params_.width, params_.height);
}

void VideoRecorder::write_frame()
{
  glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  glReadPixels(0, 0, params_.width, params_.height, GL_RGBA, GL_UNSIGNED_BYTE, pixels_.data());

  // OpenGL rows go bottom-up, video rows top-down:
  const std::size_t rowSize = row_.size();
  for (std::size_t top = 0, bottom = params_.height - 1; top < bottom; top++, bottom--)
  {
    uint8_t* a = &pixels_[top * rowSize];
    uint8_t* b = &pixels_[bottom * rowSize];
    std::memcpy(row_.data(), a, rowSize);
    std::memcpy(a, b, rowSize);
    std::memcpy(b, row_.data(), rowSize);
  }

  if (fwrite(pixels_.data(), 1, pixels_.size(), pipe_) != pixels_.size())
  {
    consume_pending_sigpipe();
    fail("writing to ffmpeg failed (is ffmpeg installed, and the output writable?)");
    return;
  }
  frames_written_++;
}

void VideoRecorder::end_frame(int windowWidth, int windowHeight)
{
  if (failed_ || fbo_ == 0)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!t0_)
  {
    t0_           = now;
    window_start_ = now;
  }
  last_frame_ = now;
  write_frame();
  if (failed_)
  {
    return;
  }

  // Report the rendered rate now and then: a rate below fps speeds up the video.
  if (const double dt = std::chrono::duration<double>(now - *window_start_).count(); dt > 30.0)
  {
    const double fps = static_cast<double>(frames_written_ - window_frames_) / dt;
    log(fps < 0.95 * params_.fps ? mrpt::system::LVL_WARN : mrpt::system::LVL_INFO,
        mrpt::format(
            "%zu frames, last %.0f s rendered at %.2f fps (video: %d fps)", frames_written_, dt,
            fps, params_.fps));
    window_start_  = now;
    window_frames_ = frames_written_;
  }

  // Preview in the window, keeping the aspect ratio:
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

void VideoRecorder::close_pipe()
{
  if (pipe_ == nullptr)
  {
    return;
  }
  const int status = pclose(pipe_);
  pipe_            = nullptr;
  if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
  {
    log(mrpt::system::LVL_ERROR, mrpt::format(
                                     "ffmpeg failed (status %d): the video may be incomplete: %s",
                                     status, params_.output.c_str()));
  }
}

void VideoRecorder::close()
{
  if (pipe_ != nullptr)
  {
    close_pipe();
    log(mrpt::system::LVL_INFO,
        mrpt::format(
            "Video finished: %zu frames, rendered at %.2f fps (video: %d fps), in: %s",
            frames_written_, effective_fps(), params_.fps, params_.output.c_str()));
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
