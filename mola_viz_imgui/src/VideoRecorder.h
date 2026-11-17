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
 * @file   VideoRecorder.h
 * @brief  Records the GUI 3D view to a video file, through ffmpeg.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/system/COutputLogger.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace mola
{
/** Records frames rendered off-screen (an OpenGL framebuffer object of a fixed
 *  size, independent of the window size and visibility) to a video file, by
 *  piping raw RGBA to an ffmpeg process. Every rendered frame becomes one video
 *  frame at `fps`: the render loop should run at that rate (if it is slower,
 *  the video plays faster than real time, but without repeated frames).
 *
 *  All methods must be called from the thread owning the OpenGL context.
 */
class VideoRecorder
{
 public:
  using Logger = std::function<void(mrpt::system::VerbosityLevel, const std::string&)>;

  struct Params
  {
    /// Video file, overwritten if it exists (e.g. ".mkv": still playable if
    /// the recording is cut short).
    std::string output;
    int         fps    = 30;
    int         width  = 1920;
    int         height = 1080;
    /// ffmpeg output options (codec, quality, filters).
    std::string ffmpeg_options = "-c:v libx264 -preset veryfast -crf 16 -pix_fmt yuv420p";
    /// Where messages go (default: stdout/stderr).
    Logger logger;
  };

  /** \throw std::invalid_argument on non-positive sizes or rate. */
  explicit VideoRecorder(Params p);
  ~VideoRecorder();

  VideoRecorder(const VideoRecorder&)            = delete;
  VideoRecorder& operator=(const VideoRecorder&) = delete;
  VideoRecorder(VideoRecorder&&)                 = delete;
  VideoRecorder& operator=(VideoRecorder&&)      = delete;

  /** Binds the off-screen framebuffer (created on first use): draw the frame
   *  next. On failure (see failed()), the window framebuffer stays bound. */
  void begin_frame();

  /** Records the frame drawn since begin_frame(), binds the window
   *  framebuffer again and copies the frame into it, scaled to the given
   *  window framebuffer size, as a preview. */
  void end_frame(int windowWidth, int windowHeight);

  /// Finishes the video (waits for ffmpeg) and releases the GL objects.
  void close();

  /// True once recording failed (no ffmpeg, write error...): nothing is recorded.
  [[nodiscard]] bool failed() const { return failed_; }

  [[nodiscard]] std::size_t frames_written() const { return frames_written_; }
  /// Frames per second actually rendered so far.
  [[nodiscard]] double effective_fps() const;

 private:
  Params                                               params_;
  unsigned int                                         fbo_      = 0;
  unsigned int                                         color_rb_ = 0;
  unsigned int                                         depth_rb_ = 0;
  FILE*                                                pipe_     = nullptr;
  bool                                                 failed_   = false;
  std::vector<uint8_t>                                 pixels_;
  std::vector<uint8_t>                                 row_;
  std::optional<std::chrono::steady_clock::time_point> t0_;
  std::optional<std::chrono::steady_clock::time_point> last_frame_;
  std::size_t                                          frames_written_ = 0;
  std::optional<std::chrono::steady_clock::time_point> window_start_;
  std::size_t                                          window_frames_ = 0;

  void log(mrpt::system::VerbosityLevel level, const std::string& msg) const;
  void create_gl_objects();
  void open_pipe();
  void write_frame();
  /// Closes the pipe, reporting a failed ffmpeg exit status.
  void close_pipe();
  void fail(const std::string& why);
};

}  // namespace mola
