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
#include <GLES2/gl2.h>
#include <jpeglib.h>
#include <spa/param/format-utils.h>
#include <spa/param/format.h>
#include <spa/param/video/raw-utils.h>
#include <spa/param/video/mjpg-utils.h>
#include <spa/param/video/h264-utils.h>
#include <spa/param/video/raw.h>
#include <spa/pod/builder.h>
#include <spdlog/spdlog.h>
#include <string/string_tools.h>
#include <time/time_tools.h>
#include <spa/debug/format.h>
#include <spa/debug/pod.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <future>
#include <iostream>
#include <sstream>
#include <utility>
#include "CameraManager.h"
#include "tools/command.h"

static constexpr char kPictureCaptureExtension[] = "jpeg";

//------------------------------------------------------------------------------
// A helper function for MJPEG decoding
//------------------------------------------------------------------------------
static int decode_mjpeg(const uint8_t* input,
    size_t input_size,
                        uint8_t* output,
                        int out_width,
                        int out_height) {
  spdlog::debug("[CameraStream.cc:decode_mjpeg] Decoding MJPEG frame\n");
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
                       int width,
                       int height) {
  spdlog::debug("[CameraStream.cc:decode_yuy2] Decoding YUY2 frame");
  const size_t expected_size = width * height * 2;  // 2 bytes per pixel
  if (input_size < expected_size) {
    spdlog::error("[decode_yuy2] input size too small: {} < {}", input_size,
                  expected_size);
    return -1;
  }

  const uint8_t* in = input;
  uint8_t* out = output;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; x += 2) {
      // Read 4 bytes: Y0 U Y1 V
      uint8_t y0 = *in++;
      uint8_t u = *in++;
      uint8_t y1 = *in++;
      uint8_t v = *in++;

      auto yuv_to_rgb = [](uint8_t y, uint8_t u, uint8_t v, uint8_t* rgb) {
        int c = y - 16;
        int d = u - 128;
        int e = v - 128;

        int r = (298 * c + 409 * e + 128) >> 8;
        int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
        int b = (298 * c + 516 * d + 128) >> 8;

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
// Helper function to dump camera format capabilities
//------------------------------------------------------------------------------
static void dump_format_info(const struct spa_pod* param, int index) {
  if (!param) return;
  
  spdlog::info("---------- Format #{} ----------", index);
  // Parse as video format
  uint32_t media_type, media_subtype;
  if (spa_format_parse(param, &media_type, &media_subtype) < 0 ||
      media_type != SPA_MEDIA_TYPE_video) {
    spdlog::warn("  Failed to parse format or not a video format");
    return;
  }
  
  // Get media subtype string
  const char* subtype_str = "Unknown";
  switch (media_subtype) {
    case SPA_MEDIA_SUBTYPE_raw: subtype_str = "RAW"; break;
    case SPA_MEDIA_SUBTYPE_mjpg: subtype_str = "MJPEG"; break;
    case SPA_MEDIA_SUBTYPE_h264: subtype_str = "H264"; break;
  }
  spdlog::info("  Media Subtype: {}", subtype_str);
  
  // Parse detailed format info
  if (media_subtype == SPA_MEDIA_SUBTYPE_raw) {
    struct spa_video_info_raw raw_info;
    if (spa_format_video_raw_parse(param, &raw_info) >= 0) {
      // Get format string
      const char* format_str = spa_debug_type_find_name(spa_type_video_format, raw_info.format);
      if (!format_str) format_str = "Unknown";
      
      spdlog::info("  Pixel Format: {}", format_str);
      spdlog::info("  Resolution: {}x{}", raw_info.size.width, raw_info.size.height);
      spdlog::info("  Framerate: {}/{} ({:.2f} fps)", 
                  raw_info.framerate.num, raw_info.framerate.denom,
                  raw_info.framerate.denom > 0 ? 
                    static_cast<double>(raw_info.framerate.num) / raw_info.framerate.denom : 0.0);
      
      // Check for framerate range
      if (raw_info.max_framerate.num > 0 && raw_info.max_framerate.denom > 0) {
        spdlog::info("  Max Framerate: {}/{} ({:.2f} fps)",
                    raw_info.max_framerate.num, raw_info.max_framerate.denom,
                    static_cast<double>(raw_info.max_framerate.num) / raw_info.max_framerate.denom);
      }
    }
  } else if (media_subtype == SPA_MEDIA_SUBTYPE_mjpg) {
    struct spa_video_info_mjpg mjpg_info;
    if (spa_format_video_mjpg_parse(param, &mjpg_info) >= 0) {
      spdlog::info("  Pixel Format: MJPEG (encoded)");
      spdlog::info("  Resolution: {}x{}", mjpg_info.size.width, mjpg_info.size.height);
      spdlog::info("  Framerate: {}/{} ({:.2f} fps)",
                  mjpg_info.framerate.num, mjpg_info.framerate.denom,
                  mjpg_info.framerate.denom > 0 ?
                    static_cast<double>(mjpg_info.framerate.num) / mjpg_info.framerate.denom : 0.0);
      
      if (mjpg_info.max_framerate.num > 0 && mjpg_info.max_framerate.denom > 0) {
        spdlog::info("  Max Framerate: {}/{} ({:.2f} fps)",
                    mjpg_info.max_framerate.num, mjpg_info.max_framerate.denom,
                    static_cast<double>(mjpg_info.max_framerate.num) / mjpg_info.max_framerate.denom);
      }
    }
  } else if (media_subtype == SPA_MEDIA_SUBTYPE_h264) {
    struct spa_video_info_h264 h264_info;
    if (spa_format_video_h264_parse(param, &h264_info) >= 0) {
      spdlog::info("  Pixel Format: H264 (encoded)");
      spdlog::info("  Resolution: {}x{}", h264_info.size.width, h264_info.size.height);
      spdlog::info("  Framerate: {}/{} ({:.2f} fps)",
                  h264_info.framerate.num, h264_info.framerate.denom,
                  h264_info.framerate.denom > 0 ?
                    static_cast<double>(h264_info.framerate.num) / h264_info.framerate.denom : 0.0);
    }
  }
}

//------------------------------------------------------------------------------
CameraStream::CameraStream(flutter::PluginRegistrarDesktop* plugin_registrar,

    std::string camera_id, int width,
                           int height)
    : registrar_(plugin_registrar),
      width_(width),
      height_(height),
      camera_id_(std::move(camera_id)) {
  spdlog::debug("[CameraStream.cc:CameraStream] Constructor called");
  // Allocate RGB buffer for frames
  decoded_buffer_.reset(new uint8_t[width_ * height_ * 3]);
  std::memset(decoded_buffer_.get(), 0, width_ * height_ * 3);

  // Create the Flutter PixelBufferTexture
  auto pixel_buffer_texture = std::make_unique<flutter::PixelBufferTexture>(
      [this](size_t /*width*/,
             size_t /*height*/) -> const FlutterDesktopPixelBuffer* {
        static FlutterDesktopPixelBuffer pixel_buffer = {};
        static std::mutex s_mutex;
        std::lock_guard<std::mutex> lock(s_mutex);

        pixel_buffer.width = width_;
        pixel_buffer.height = height_;
        pixel_buffer.buffer = decoded_buffer_.get();

        pixel_buffer.release_context = nullptr;
        pixel_buffer.release_callback = nullptr;
        return &pixel_buffer;
      });

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
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
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
  spdlog::debug("[CameraStream.cc:~CameraStream] Destructor called");
  Stop();
}

//------------------------------------------------------------------------------
// Start capturing from the given node ID
//------------------------------------------------------------------------------
bool CameraStream::Start(const std::string& camera_id) {
  spdlog::debug("[CameraStream.cc:Start] Starting camera stream");
  // 1) Ensure the manager is running
  auto& mgr = CameraManager::instance();
  if (!mgr.initialize()) {
    spdlog::error("[CameraStream] fail to initialize CameraManager.");
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
    streamEvents.param_changed = OnStreamParamChanged;
    streamEvents.process = OnStreamProcess;

    pw_stream_add_listener(pw_stream_, &stream_listener_, &streamEvents, this);

    // For example, request an MJPEG format or any other video format
    // building an SPA_POD with resolution, etc. This is just a stub:

    // Build the SPA format param
    // Build the SPA format param
    std::vector<uint8_t> pod_buffer(1024);
    spa_pod_builder builder = SPA_POD_BUILDER_INIT(pod_buffer.data(), pod_buffer.size());

    // Verify builder initialization
    if (builder.data == nullptr || builder.size == 0) {
      spdlog::error("[CameraStream] failed to initialize spa_pod_builder");
      pw_stream_destroy(pw_stream_);
      pw_stream_ = nullptr;
      pw_thread_loop_unlock(loop);
      return false;
    }
    spa_rectangle rect = {static_cast<uint32_t>(width_),
                          static_cast<uint32_t>(height_)};
    spa_fraction fps = {30, 1};

    // Query supported formats - request enumeration of ALL possible formats
    // Build a comprehensive format query that will enumerate all camera capabilities
    uint8_t buffer[4096];  // Larger buffer for comprehensive enumeration
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));

    // Request ALL possible video formats (raw, MJPEG, H264, etc.)
    // with wildcard/choice for resolution and framerate
    const struct spa_pod* formats[1];
    formats[0] = static_cast<const spa_pod*>(spa_pod_builder_add_object(
        &b, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
        SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video),
        // Enumerate all subtypes
        SPA_FORMAT_mediaSubtype, SPA_POD_CHOICE_ENUM_Id(3,
                               SPA_MEDIA_SUBTYPE_raw,
                               SPA_MEDIA_SUBTYPE_mjpg,
                               SPA_MEDIA_SUBTYPE_h264)));
    
    // Connect with EnumFormat to get all possible formats enumerated
    // This will trigger OnStreamParamChanged for each supported format
    spdlog::info("[CameraStream] Requesting enumeration of all supported camera formats...");
    if (int res = pw_stream_connect(
            pw_stream_, PW_DIRECTION_INPUT, PW_ID_ANY,
            static_cast<pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT |
                                         PW_STREAM_FLAG_MAP_BUFFERS),
            formats, 1);
        res < 0) {
      spdlog::error("[CameraStream] pw_stream_connect() error during format enumeration: {}", res);
      pw_stream_destroy(pw_stream_);
      pw_stream_ = nullptr;
      pw_thread_loop_unlock(loop);
      return false;
    }
    
    // Wait a bit for format enumeration to complete
    pw_thread_loop_unlock(loop);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pw_thread_loop_lock(loop);
    
    // Now disconnect and reconnect with the desired format
    pw_stream_disconnect(pw_stream_);
    spdlog::info("========== END OF CAMERA CAPABILITIES ==========");
    
    // Rebuild stream for actual usage with desired format
    const spa_pod* params[1];

    

    std::string format_env;
    if (const char* env = std::getenv("CAMERA_OUTPUT_FORMAT")) {
      format_env = env;
    } else {
      format_env = "";
    }

    if (format_env == "MJPEG") {
      camera_output_format = "MJPEG";
    } else if (format_env == "YUV2") {
      camera_output_format = "YUV2";
    } else {
      spdlog::error(
          "CAMERA_OUTPUT_FORMAT is set to an unsupported value ('{}'). "
          "Supported values: MJPEG, YUV2. Defaulting to YUV2.",
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

      if (!params[0] || builder.state.offset > pod_buffer.size()) {
        spdlog::error("[CameraStream] failed to build MJPEG format parameters (buffer overflow)");
        pw_stream_destroy(pw_stream_);
        pw_stream_ = nullptr;
        pw_thread_loop_unlock(loop);
        return false;
      }
    } else if (camera_output_format == "YUV2") {
      params[0] = static_cast<const spa_pod*>(spa_pod_builder_add_object(
          &builder, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
          SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video),
          SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
          SPA_FORMAT_VIDEO_format, SPA_POD_Id(SPA_VIDEO_FORMAT_YUY2),
          SPA_FORMAT_VIDEO_size, SPA_POD_Rectangle(&rect),
          SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&fps)));

      if (!params[0] || builder.state.offset > pod_buffer.size()) {
        spdlog::error("[CameraStream] failed to build YUV2 format parameters (buffer overflow)");
        pw_stream_destroy(pw_stream_);
        pw_stream_ = nullptr;
        pw_thread_loop_unlock(loop);
        return false;
      }
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
  spdlog::debug("[CameraStream.cc:Stop] Stopping camera stream");
  if (!pw_stream_) {
    return;  // already stopped
  }

  auto& mgr = CameraManager::instance();
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
                        int width,
                        int height,
                        int channels,
                        int quality) {
  struct jpeg_compress_struct cinfo {};
  struct jpeg_error_mgr jerr {};

  // Setup error handling
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Open file for writing
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

  // Write scanlines
  JSAMPROW row_pointer;
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer = (JSAMPROW)&image_data[cinfo.next_scanline * width * channels];
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
  spdlog::debug("[CameraStream.cc:HandleProcess] Processing new frame");
  if (!pw_stream_)
    return;
  pw_buffer* buf = pw_stream_dequeue_buffer(pw_stream_);
  if (!buf)
    return;

  if (!buf->buffer->datas[0].data) {
    pw_stream_queue_buffer(pw_stream_, buf);
    return;
  }

  const auto* compressedData =
      static_cast<uint8_t*>(buf->buffer->datas[0].data);
  const size_t compressedSize = buf->buffer->datas[0].chunk->size;

  if (!decoded_buffer_) {
    decoded_buffer_.reset(new uint8_t[width_ * height_ * 3]);
  }

  int ret = -1;
  if (camera_output_format == "YUV2") {
    ret = decode_yuy2(compressedData, compressedSize, decoded_buffer_.get(),
                      width_, height_);
  } else if (camera_output_format == "MJPEG") {
    ret = decode_mjpeg(compressedData, compressedSize, decoded_buffer_.get(),
                       width_, height_);
  }

  if (ret == 0) {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      new_frame_available_ = true;
      registrar_->texture_registrar()->TextureMakeCurrent();
      glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
      glViewport(0, 0, width_, height_);

      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glUniform1i(0, 0);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                      GL_LINEAR_MIPMAP_LINEAR);

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB,
                   GL_UNSIGNED_BYTE, decoded_buffer_.get());
      glGenerateMipmap(GL_TEXTURE_2D);

      glBindFramebuffer(GL_FRAMEBUFFER, 0);

      registrar_->texture_registrar()->TextureClearCurrent();
      registrar_->texture_registrar()->MarkTextureFrameAvailable(texture_id_);
    }
  } else {
    spdlog::error("[CameraStream] mjpeg decode failed.");
  }
  pw_stream_queue_buffer(pw_stream_, buf);
}

//------------------------------------------------------------------------------
// Static callback proxies
//------------------------------------------------------------------------------
const char* StreamStateToString(enum pw_stream_state state) {
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

void  CameraStream::OnStreamParamChanged(void* data, uint32_t id, const struct spa_pod* param) {
  auto* self = reinterpret_cast<CameraStream*>(data);
  
  spdlog::debug("[CameraStream] param_changed: id={}", id);
  
  // Dump all enumerated formats
  if (param && id == SPA_PARAM_EnumFormat) {
    static int format_index = 0;
    if (format_index == 0) {
      spdlog::info("========== ENUMERATING ALL CAMERA CAPABILITIES ==========");
      spdlog::info("Camera ID: {}", self->camera_id_);
    }
    dump_format_info(param, ++format_index);
  } else if (param && id == SPA_PARAM_Format) {
    spdlog::info("---------- NEGOTIATED FORMAT ----------");
    dump_format_info(param, 0);
    spdlog::debug("[CameraStream] Format negotiated successfully");
  }
}

void CameraStream::OnStreamStateChanged(void* /*data*/,
    pw_stream_state old_state,
    pw_stream_state new_state,
                                        const char* /*error*/) {
  spdlog::debug("[CameraStream.cc:OnStreamStateChanged] Stream state changed callback");
  spdlog::debug("[CameraStream] stream state changed from {} to {}",
                StreamStateToString(old_state), StreamStateToString(new_state));

}


void CameraStream::OnStreamProcess(void* data) {
  spdlog::debug("[CameraStream.cc:OnStreamProcess] Stream process callback");
  auto* self = reinterpret_cast<CameraStream*>(data);
  (void)self;
  self->HandleProcess();
}

void CameraStream::PauseStream() const {
  spdlog::debug("[CameraStream.cc:PauseStream] Pausing camera stream");
  if (!pw_stream_)
    return;

  auto& mgr = CameraManager::instance();
  if (!mgr.initialize()) {
    spdlog::error("[CameraStream] failed to initialize CameraManager.");
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
  spdlog::debug("[CameraStream.cc:ResumeStream] Resuming camera stream");
  if (!pw_stream_)
    return;

  auto& mgr = CameraManager::instance();
  if (!mgr.initialize()) {
    spdlog::error("[CameraStream] failed to initialize CameraManager.");
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
  spdlog::debug("[CameraStream.cc:GetFilePathForPicture] Getting file path for picture");
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

std::string CameraStream::takePicture() const {
  spdlog::debug("[CameraStream.cc:takePicture] Taking picture");
  auto filename = GetFilePathForPicture();
  save_image_to_jpeg(filename.value(), decoded_buffer_.get(), width_, height_,
                     3, 90);

  return filename.value();
}
