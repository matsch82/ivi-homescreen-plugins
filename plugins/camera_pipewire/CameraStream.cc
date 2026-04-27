/*
 * Copyright 2020-2024 Toyota Connected North America
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "CameraStream.h"

#include <cstdio>
#include <filesystem>
#include <future>
#include <sstream>
#include <utility>
#include <vector>

#include <GLES2/gl2.h>
#include <jpeglib.h>
#include <spa/param/format-utils.h>
#include <spa/param/format.h>
#include <spa/param/video/raw-utils.h>
#include <spa/param/video/raw.h>
#include <spa/pod/builder.h>

#include "PipewireGraph.h"
#include "plugins/common/common.h"

static constexpr char kPictureCaptureExtension[] = "jpeg";

//------------------------------------------------------------------------------
// A helper function for MJPEG decoding
//------------------------------------------------------------------------------
static int decode_mjpeg(const uint8_t* input,
                        size_t input_size,
                        uint8_t* output,
                        int out_width,
                        int out_height) {
  jpeg_decompress_struct cinfo{};
  jpeg_error_mgr jerr{};
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  jpeg_mem_src(&cinfo, input, input_size);
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
    spdlog::error("[decode_mjpeg] failed to read JPEG header.");
    jpeg_destroy_decompress(&cinfo);
    return -1;
  }

  jpeg_start_decompress(&cinfo);
  if (static_cast<int>(cinfo.output_width) != out_width ||
      static_cast<int>(cinfo.output_height) != out_height ||
      cinfo.output_components != 3) {
    spdlog::error("[decode_mjpeg] unexpected size.");
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return -1;
  }

  const unsigned long long row_stride =
      cinfo.output_width * cinfo.output_components;
  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW row[1];
    row[0] = &output[cinfo.output_scanline * row_stride];
    jpeg_read_scanlines(&cinfo, row, 1);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return 0;
}

static int decode_yuy2(const uint8_t* input,
                       size_t input_size,
                       uint8_t* output,
                       const int width,
                       const int height) {
  if (const size_t expected_size = width * height * 2;
      input_size < expected_size) {
    spdlog::error("[decode_yuy2] input size too small: {} < {}", input_size,
                  expected_size);
    return -1;
  }

  const uint8_t* in = input;
  uint8_t* out = output;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; x += 2) {
      // Read 4 bytes: Y0 U Y1 V
      const uint8_t y0 = *in++;
      const uint8_t u = *in++;
      const uint8_t y1 = *in++;
      const uint8_t v = *in++;

      auto yuv_to_rgb = [](uint8_t y, uint8_t u, uint8_t v, uint8_t* rgb) {
        const int c = y - 16;
        const int d = u - 128;
        const int e = v - 128;

        const int r = (298 * c + 409 * e + 128) >> 8;
        const int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
        const int b = (298 * c + 516 * d + 128) >> 8;

        rgb[0] = std::clamp(r, 0, 255);
        rgb[1] = std::clamp(g, 0, 255);
        rgb[2] = std::clamp(b, 0, 255);
      };

      // Write pixel 1 (Y0)
      yuv_to_rgb(y0, u, v, out);
      out += 3;

      // Write pixel 2 (Y1)
      yuv_to_rgb(y1, u, v, out);
      out += 3;
    }
  }

  return 0;
}
//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
CameraStream::CameraStream(flutter::PluginRegistrarDesktop* plugin_registrar,
                           std::string camera_id,
                           const int width,
                           const int height)
    : registrar_(plugin_registrar),
      width_(width),
      height_(height),
      camera_id_(std::move(camera_id)) {
  registrar_->texture_registrar()->TextureMakeCurrent();

  glGenFramebuffers(1, &framebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

  glGenTextures(1, &texture_id_);
  glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  // RGB24 rows may not be 4-byte aligned; set once here so HandleProcess
  // never has to touch pixel-store state per frame.
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
               GL_UNSIGNED_BYTE, nullptr);
  glBindTexture(GL_TEXTURE_2D, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         texture_id_, 0);

  if (auto status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      status != GL_FRAMEBUFFER_COMPLETE) {
    spdlog::error("[camera_plugin] framebufferStatus: 0x{:X}", status);
  }

  glFinish();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  registrar_->texture_registrar()->TextureClearCurrent();

  descriptor = {
      .struct_size = sizeof(FlutterDesktopGpuSurfaceDescriptor),
      .handle = &texture_id_,
      .width = static_cast<size_t>(width),
      .height = static_cast<size_t>(height),
      .visible_width = static_cast<size_t>(width),
      .visible_height = static_cast<size_t>(height),
      .format = kFlutterDesktopPixelFormatRGBA8888,
      .release_callback = [](void* /* release_context */) {},
      .release_context = this,
  };

  gpu_surface_texture = std::make_unique<flutter::GpuSurfaceTexture>(
      kFlutterDesktopGpuSurfaceTypeGlTexture2D,
      [&](size_t /* width */, size_t /* height */)
          -> const FlutterDesktopGpuSurfaceDescriptor* { return &descriptor; });

  flutter::TextureVariant texture = *gpu_surface_texture;
  registrar_->texture_registrar()->RegisterTexture(&texture);
  registrar_->texture_registrar()->MarkTextureFrameAvailable(texture_id_);
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
CameraStream::~CameraStream() {
  Stop();
}

//------------------------------------------------------------------------------
// Start capturing from the given node ID
//------------------------------------------------------------------------------
bool CameraStream::Start(const std::string& camera_id) {
  // 1) Ensure the manager is running
  auto& mgr = PipewireGraph::instance();
  if (!mgr.initialize()) {
    spdlog::error("[CameraStream] fail to initialize PipewireGraph.");
    return false;
  }

  auto* loop = mgr.threadLoop();
  if (!loop) {
    spdlog::error("[CameraStream] threadLoop is null!");
    return false;
  }

  // 2) Lock the thread loop while creating the stream
  pw_thread_loop_lock(loop);
  {
    auto* core = mgr.core();
    if (!core) {
      spdlog::error("[CameraStream] no valid PipeWire core.");
      pw_thread_loop_unlock(loop);
      return false;
    }

    // Create the pw_stream
    pw_properties* props = pw_properties_new(
        PW_KEY_MEDIA_TYPE, "Video", PW_KEY_MEDIA_CATEGORY, "Capture",
        PW_KEY_MEDIA_ROLE, "Camera", "node.target", camera_id.c_str(), nullptr);

    pw_stream_ = pw_stream_new(core, "MyCameraStream", props);
    if (!pw_stream_) {
      spdlog::error("[CameraStream] failed to create pw_stream.");
      pw_thread_loop_unlock(loop);
      return false;
    }

    // Set up the stream events
    static pw_stream_events streamEvents{};
    streamEvents.version = PW_VERSION_STREAM_EVENTS;
    streamEvents.state_changed = OnStreamStateChanged;
    streamEvents.process = OnStreamProcess;

    pw_stream_add_listener(pw_stream_, &stream_listener_, &streamEvents, this);

    // For example, request an MJPEG format or any other video format
    // building an SPA_POD with resolution, etc. This is just a stub:

    // Build the SPA format param
    std::vector<uint8_t> pod_buffer(1024);
    auto builder = ((struct spa_pod_builder){
        (pod_buffer.data()),
        (static_cast<unsigned int>(pod_buffer.size())),
        0,
        {},
        {}});
    spa_rectangle rect = {static_cast<uint32_t>(width_),
                          static_cast<uint32_t>(height_)};
    spa_fraction fps = {30, 1};

    const spa_pod* params[1];

    const char* env_value = std::getenv("CAMERA_OUTPUT_FORMAT");
    if (std::string format_env = env_value ? env_value : "";
        format_env == "MJPEG") {
      camera_output_format = "MJPEG";
    } else if (format_env == "RGB3") {
      camera_output_format = "RGB3";
    } else if (format_env == "YUV2" || format_env.empty()) {
      camera_output_format = "YUV2";
    } else {
      spdlog::warn(
          "[CameraStream] CAMERA_OUTPUT_FORMAT='{}' is unsupported. "
          "Supported values: YUV2, RGB3, MJPEG. Defaulting to YUV2.",
          format_env);
      camera_output_format = "YUV2";
    }

    spdlog::debug("[CameraStream] camera_output_format is set to {}",
                  camera_output_format);

    if (camera_output_format == "MJPEG") {
      params[0] = static_cast<const spa_pod*>(spa_pod_builder_add_object(
          &builder, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
          SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video),
          SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_mjpg),
          SPA_FORMAT_VIDEO_size, SPA_POD_Rectangle(&rect),
          SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&fps)));
    } else if (camera_output_format == "YUV2") {
      params[0] = static_cast<const spa_pod*>(spa_pod_builder_add_object(
          &builder, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
          SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video),
          SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
          SPA_FORMAT_VIDEO_format, SPA_POD_Id(SPA_VIDEO_FORMAT_YUY2),
          SPA_FORMAT_VIDEO_size, SPA_POD_Rectangle(&rect),
          SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&fps)));
    } else {  // RGB3 (default)
      params[0] = static_cast<const spa_pod*>(spa_pod_builder_add_object(
          &builder, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
          SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video),
          SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
          SPA_FORMAT_VIDEO_format, SPA_POD_Id(SPA_VIDEO_FORMAT_RGB),
          SPA_FORMAT_VIDEO_size, SPA_POD_Rectangle(&rect),
          SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&fps)));
    }

    // Actually connect the stream
    spdlog::debug("[CameraStream] connecting to camera_id: {}", camera_id);
    if (int res = pw_stream_connect(
            pw_stream_, PW_DIRECTION_INPUT, PW_ID_ANY,
            static_cast<pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT |
                                         PW_STREAM_FLAG_MAP_BUFFERS),
            params, 1);
        res < 0) {
      spdlog::error("[CameraStream] pw_stream_connect() error: {}", res);
      pw_stream_destroy(pw_stream_);
      pw_stream_ = nullptr;
      pw_thread_loop_unlock(loop);
      return false;
    }
  }
  pw_thread_loop_unlock(loop);

  return true;
}

//------------------------------------------------------------------------------
// Stop capturing
//------------------------------------------------------------------------------
void CameraStream::Stop() {
  if (!pw_stream_) {
    return;  // already stopped
  }

  auto& mgr = PipewireGraph::instance();
  auto* loop = mgr.threadLoop();

  // Lock while destroying
  pw_thread_loop_lock(loop);
  {
    pw_stream_destroy(pw_stream_);
    pw_stream_ = nullptr;
  }
  pw_thread_loop_unlock(loop);
}

void save_image_to_jpeg(const std::string& filename,
                        const unsigned char* image_data,
                        const int width,
                        const int height,
                        const int channels,
                        const int quality) {
  jpeg_compress_struct cinfo{};
  jpeg_error_mgr jerr{};

  // Setup error handling
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Open a file for writing
  FILE* outfile = fopen(filename.c_str(), "wb");
  if (!outfile) {
    spdlog::error("error: unable to open {} for writing", filename);
    return;
  }

  jpeg_stdio_dest(&cinfo, outfile);

  // Set image properties
  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = channels;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);

  // Start compression
  jpeg_start_compress(&cinfo, TRUE);

  // Write scan lines
  JSAMPROW row_pointer;
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer = const_cast<JSAMPROW>(
        &image_data[cinfo.next_scanline * width * channels]);
    jpeg_write_scanlines(&cinfo, &row_pointer, 1);
  }

  // Finish compression
  jpeg_finish_compress(&cinfo);
  fclose(outfile);
  jpeg_destroy_compress(&cinfo);
  spdlog::debug("image saved to {}", filename);
}

//------------------------------------------------------------------------------
// Private method: called each time there's a new MJPEG frame
//------------------------------------------------------------------------------
void CameraStream::HandleProcess() {
  if (!pw_stream_)
    return;
  pw_buffer* buf = pw_stream_dequeue_buffer(pw_stream_);
  if (!buf)
    return;

  // Guard against malformed/early buffers that arrive before the CSI hardware
  // link is fully established (rp1-cfe probe race): chunk may be null even
  // when data is mapped, and n_datas may be 0 on the very first callback.
  if (buf->buffer->n_datas == 0 ||
      !buf->buffer->datas[0].data ||
      !buf->buffer->datas[0].chunk ||
      buf->buffer->datas[0].chunk->size == 0) {
    pw_stream_queue_buffer(pw_stream_, buf);
    return;
  }

  const auto* frameData =
      static_cast<const uint8_t*>(buf->buffer->datas[0].data);
  const size_t frameSize = buf->buffer->datas[0].chunk->size;

  if (camera_output_format == "RGB3") {
    // Direct GPU upload: the camera delivers packed RGB24 — no CPU conversion.
    const size_t expected = static_cast<size_t>(width_ * height_ * 3);
    if (frameSize < expected) {
      spdlog::error("[CameraStream] RGB3 frame too small: {} < {}",
                    frameSize, expected);
      pw_stream_queue_buffer(pw_stream_, buf);
      return;
    }
    registrar_->texture_registrar()->TextureMakeCurrent();
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_,
                    GL_RGB, GL_UNSIGNED_BYTE, frameData);
    glBindTexture(GL_TEXTURE_2D, 0);
    registrar_->texture_registrar()->TextureClearCurrent();
    registrar_->texture_registrar()->MarkTextureFrameAvailable(texture_id_);
  } else {
    // Legacy CPU-decode paths (YUV2 / MJPEG).
    if (!decoded_buffer_) {
      decoded_buffer_.reset(new uint8_t[width_ * height_ * 3]);
    }
    int ret = -1;
    if (camera_output_format == "YUV2") {
      ret = decode_yuy2(frameData, frameSize, decoded_buffer_.get(),
                        width_, height_);
    } else if (camera_output_format == "MJPEG") {
      ret = decode_mjpeg(frameData, frameSize, decoded_buffer_.get(),
                         width_, height_);
    }
    if (ret == 0) {
      registrar_->texture_registrar()->TextureMakeCurrent();
      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_,
                      GL_RGB, GL_UNSIGNED_BYTE, decoded_buffer_.get());
      glBindTexture(GL_TEXTURE_2D, 0);
      registrar_->texture_registrar()->TextureClearCurrent();
      registrar_->texture_registrar()->MarkTextureFrameAvailable(texture_id_);
    } else {
      spdlog::error("[CameraStream] frame decode failed.");
    }
  }
  pw_stream_queue_buffer(pw_stream_, buf);
}

//------------------------------------------------------------------------------
// Static callback proxies
//------------------------------------------------------------------------------
const char* StreamStateToString(const pw_stream_state state) {
  switch (state) {
    case PW_STREAM_STATE_ERROR:
      return "PW_STREAM_STATE_ERROR";
    case PW_STREAM_STATE_UNCONNECTED:
      return "PW_STREAM_STATE_UNCONNECTED";
    case PW_STREAM_STATE_CONNECTING:
      return "PW_STREAM_STATE_CONNECTING";
    case PW_STREAM_STATE_PAUSED:
      return "PW_STREAM_STATE_PAUSED";
    case PW_STREAM_STATE_STREAMING:
      return "PW_STREAM_STATE_STREAMING";
    default:
      return "PW_STREAM_STATE_UNKNOWN";
  }
}

void CameraStream::OnStreamStateChanged(void* /*data*/,
                                        const pw_stream_state old_state,
                                        const pw_stream_state new_state,
                                        const char* /*error*/) {
  spdlog::debug("[CameraStream] stream state changed from {} to {}",
                StreamStateToString(old_state), StreamStateToString(new_state));
}

void CameraStream::OnStreamProcess(void* data) {
  auto* self = static_cast<CameraStream*>(data);
  (void)self;
  self->HandleProcess();
}

void CameraStream::PauseStream() const {
  if (!pw_stream_)
    return;

  auto& mgr = PipewireGraph::instance();
  if (!mgr.initialize()) {
    spdlog::error("[CameraStream] failed to initialize PipewireGraph.");
    return;
  }

  auto* loop = mgr.threadLoop();
  if (!loop) {
    spdlog::error("[CameraStream] threadLoop is null!");
    return;
  }

  pw_thread_loop_lock(loop);
  { pw_stream_set_active(pw_stream_, false); }
  pw_thread_loop_unlock(loop);
}

void CameraStream::ResumeStream() const {
  if (!pw_stream_)
    return;

  auto& mgr = PipewireGraph::instance();
  if (!mgr.initialize()) {
    spdlog::error("[CameraStream] failed to initialize PipewireGraph.");
    return;
  }

  auto* loop = mgr.threadLoop();
  if (!loop) {
    spdlog::error("[CameraStream] threadLoop is null!");
    return;
  }

  pw_thread_loop_lock(loop);
  { pw_stream_set_active(pw_stream_, true); }
  pw_thread_loop_unlock(loop);
}
std::optional<std::string> CameraStream::GetFilePathForPicture() {
  std::ostringstream oss;
  oss << "xdg-user-dir PICTURES";
  std::string picture_path;
  if (!plugin_common::Command::Execute(oss.str().c_str(), picture_path)) {
    return std::nullopt;
  }
  std::filesystem::path path(
      plugin_common::StringTools::trim(picture_path, "\n"));

  path /= "PhotoCapture_" + plugin_common::TimeTools::GetCurrentTimeString() +
          "." + kPictureCaptureExtension;
  return path;
}

std::string CameraStream::takePicture() {
  auto filename = GetFilePathForPicture();
  if (!filename.has_value()) {
    spdlog::error("[CameraStream] takePicture: could not determine output path.");
    return {};
  }

  if (camera_output_format == "RGB3") {
    // No persistent CPU copy in RGB3 mode; read back from the GPU framebuffer.
    // GLES2 only guarantees GL_RGBA/GL_UNSIGNED_BYTE for glReadPixels.
    registrar_->texture_registrar()->TextureMakeCurrent();
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    std::vector<uint8_t> rgba(static_cast<size_t>(width_ * height_ * 4));
    glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    registrar_->texture_registrar()->TextureClearCurrent();

    // Strip alpha: the JPEG encoder expects RGB24.
    std::vector<uint8_t> rgb(static_cast<size_t>(width_ * height_ * 3));
    const int pixels = width_ * height_;
    for (int i = 0; i < pixels; ++i) {
      rgb[i * 3 + 0] = rgba[i * 4 + 0];
      rgb[i * 3 + 1] = rgba[i * 4 + 1];
      rgb[i * 3 + 2] = rgba[i * 4 + 2];
    }
    save_image_to_jpeg(filename.value(), rgb.data(), width_, height_, 3, 90);
  } else {
    if (!decoded_buffer_) {
      spdlog::error("[CameraStream] takePicture: no frame decoded yet.");
      return {};
    }
    save_image_to_jpeg(filename.value(), decoded_buffer_.get(), width_, height_,
                       3, 90);
  }

  return filename.value();
}
