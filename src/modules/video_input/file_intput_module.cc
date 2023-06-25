/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/video_input/file_input_module.h"

#include <string>
#include <thread>

#include "hobotlog/hobotlog.hpp"
#include "modules/vio_module.h"
#include "utils/protocol/test.pb.h"

namespace J5Sample {

std::once_flag FileInputModule::fb_init_flag_;

struct strMediaCodecCtx {
  strMediaCodecCtx(FileInputModule *hModule, media_codec_context_t *mCtx,
                   bool &isLoop, bool &isFinish, FILE *hFile)
      : h_module(hModule),
        media_ctx(mCtx),
        is_loop(isLoop),
        is_finish(isFinish),
        h_file(hFile) {}

  FileInputModule *h_module;
  media_codec_context_t *media_ctx;
  bool &is_loop;
  bool &is_finish;
  FILE *h_file;
};
typedef std::shared_ptr<strMediaCodecCtx> spMediaCodecCtx;

static u_int64_t CurrentTs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

int FileInputModule::LoadConfig(const std::string &config_file) {
  std::shared_ptr<JsonConfigWrapper> cfg_json = nullptr;
  int ret = this->LoadConfigFile(config_file, cfg_json);
  if (ret) {
    LOGE << "chnl[" << channel_id_ << "] load config file failed, ret: " << ret;
    return ret;
  }

  std::string chn_num = "chn_" + std::to_string(channel_id_);
  auto file_cfg_json = cfg_json->GetSubConfig(chn_num);
  if (file_cfg_json == nullptr) {
    LOGE << chn_num << ": parameter is not exit!";
    return -1;
  }

  file_input_cfg_ = std::make_shared<FileInputConfig>();
  file_input_cfg_->codec_id = MEDIA_CODEC_ID_H264;
  if (file_cfg_json->HasMember("data_type")) {
    file_input_cfg_->codec_id =
        media_codec_id_t(file_cfg_json->GetIntValue("data_type"));
  }
  file_input_cfg_->frame_rate = 30;
  if (file_cfg_json->HasMember("frame_rate")) {
    file_input_cfg_->frame_rate = file_cfg_json->GetIntValue("frame_rate");
  }
  if (file_cfg_json->HasMember("file_path")) {
    file_input_cfg_->file_path = file_cfg_json->GetSTDStringValue("file_path");
  } else {
    LOGE << "chnl[" << channel_id_
         << "] video file path parameter is not exist";
    return -1;
  }

  return 0;
}

static void on_decoder_input_buffer_available(
    hb_ptr userdata, media_codec_buffer_t *inputBuffer) {
  auto mc_ctx = reinterpret_cast<strMediaCodecCtx *>(userdata);
  if (mc_ctx->is_finish) {
    return;
  }
  auto h_file = mc_ctx->h_file;
  hb_s32 ret = fread(inputBuffer->vstream_buf.vir_ptr, 1,
                     inputBuffer->vstream_buf.size, h_file);
  if (ret <= 0) {
    if (!mc_ctx->is_loop) {
      mc_ctx->is_finish = true;
      return;
    }
    if (fseek(h_file, 0, SEEK_SET)) {
      LOGE << "chnl[" << mc_ctx->h_module->GetChannelId()
           << "] Failed to rewind input file";
    } else {
      ret = fread(inputBuffer->vstream_buf.vir_ptr, 1,
                  inputBuffer->vstream_buf.size, h_file);
      if (ret <= 0) {
        LOGE << "chnl[" << mc_ctx->h_module->GetChannelId()
             << "] Failed to read input file";
      }
    }
  }
}

static void on_decoder_output_buffer_available(
    hb_ptr userdata, media_codec_buffer_t *outputBuffer,
    media_codec_output_buffer_info_t *extraInfo) {
  auto mc_ctx = reinterpret_cast<strMediaCodecCtx *>(userdata);
  if (!mc_ctx || mc_ctx->is_finish || !outputBuffer || !extraInfo) {
    return;
  }
  assert(MC_VIDEO_FRAME_BUFFER == outputBuffer->type);
  auto &vframe_buf = outputBuffer->vframe_buf;
  auto y_buf = vframe_buf.vir_ptr[0];
  auto y_len = vframe_buf.compSize[0];
  auto uv_buf = vframe_buf.vir_ptr[1];
  auto uv_len = vframe_buf.compSize[1];
  auto data_len = vframe_buf.size;
  assert(data_len == (y_len + uv_len));

  // Get a buffer from memory pool
  auto spFrameBuf = mc_ctx->h_module->GetFrameDataBuf(data_len);
  if (nullptr == spFrameBuf) {
    return;
  }

  // For abnormal case
  // 每个chnl都有自己的同mempool，理论上每个buffer一旦构建完他的width和height应该是不会变的
  auto &ctrl_info = spFrameBuf->ctrl;
  if ((ctrl_info.width != 0 && ctrl_info.width != vframe_buf.width) ||
      (ctrl_info.height != 0 && ctrl_info.height != vframe_buf.height)) {
    LOGE << "chnl[" << mc_ctx->h_module->GetChannelId() << "]["
         << std::this_thread::get_id() << "] width or height changed!!! "
         << "Last: " << ctrl_info.usr_ctx << " / " << ctrl_info.width << " * "
         << ctrl_info.height << ", Now: " << (void *)(mc_ctx->media_ctx)
         << " / " << vframe_buf.width << " * " << vframe_buf.height << ".";
    return;
  }

  // ctrl_info process
  ctrl_info.usr_ctx = mc_ctx->media_ctx;
  ctrl_info.pts = outputBuffer->vframe_buf.pts;
  ctrl_info.frame_rate.numerator =
      extraInfo->video_frame_info.frame_rate_numerator;
  ctrl_info.frame_rate.denominator =
      extraInfo->video_frame_info.frame_rate_denominator;
  ctrl_info.width = outputBuffer->vframe_buf.width;
  ctrl_info.height = outputBuffer->vframe_buf.height;

  // data copy
  memcpy(spFrameBuf->p_data, y_buf, y_len);
  memcpy(spFrameBuf->p_data + y_len, uv_buf, uv_len);
#if 0
  {  // test
    static bool my_flag = true;
    if (my_flag) {
      my_flag = false;
      char fntmp[1024] = {0};
      sprintf(fntmp, "chnl%d-%d_%d.yuv", mc_ctx->h_module->GetChannelId(),
              ctrl_info.width, ctrl_info.height);
      FILE *tf = fopen(fntmp, "wb");
      if (tf) {
        fwrite(spFrameBuf->p_data, 1, data_len, tf);
        fclose(tf);
      }
    }
  }
#endif
  mc_ctx->h_module->FrameDataInput(spFrameBuf);
}

static void on_decoder_media_codec_message(hb_ptr userdata, hb_s32 error) {
  auto mc_ctx = reinterpret_cast<strMediaCodecCtx *>(userdata);
  LOGD << "chnl[" << mc_ctx->h_module->GetChannelId()
       << "] on_decoder_media_codec_message";
  if (error) {
    mc_ctx->is_finish = true;
    LOGE << "chnl[" << mc_ctx->h_module->GetChannelId()
         << "] ERROR happened! ecode = " << error;
  }
}

//#define TEST_BY_NV12_IMG
#ifdef TEST_BY_NV12_IMG
static void on_nv12_read(void *userdata) {
  auto mc_ctx = reinterpret_cast<strMediaCodecCtx *>(userdata);
  if (!mc_ctx || mc_ctx->is_finish) {
    return;
  }

  uint32_t width(384), height(288);
  uint32_t data_len = width * height * 3 / 2;

  while (!mc_ctx->is_finish) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // Get a buffer from memory pool
    auto spFrameBuf = mc_ctx->h_module->GetFrameDataBuf(data_len);
    if (nullptr == spFrameBuf || mc_ctx->is_finish) {
      continue;
    }

    // ctrl_info process
    auto &ctrl_info = spFrameBuf->ctrl;
    ctrl_info.usr_ctx = mc_ctx->media_ctx;
    ctrl_info.pts = 0;
    ctrl_info.frame_rate.numerator = 0;
    ctrl_info.frame_rate.denominator = 0;
    ctrl_info.width = width;
    ctrl_info.height = height;

    // data read
    auto h_file = mc_ctx->h_file;
    auto ret = fread(spFrameBuf->p_data, 1, data_len, h_file);
    if (ret <= 0) {
      if (!mc_ctx->is_loop) {
        mc_ctx->is_finish = true;
        continue;
      }

      if (fseek(h_file, 0, SEEK_SET)) {
        LOGE << "chnl[" << mc_ctx->h_module->GetChannelId()
             << "] Failed to rewind input file";
      } else {
        ret = fread(spFrameBuf->p_data, 1, data_len, h_file);
        if (ret <= 0) {
          LOGE << "chnl[" << mc_ctx->h_module->GetChannelId()
               << "] Failed to read input file";
        } else {
          mc_ctx->h_module->FrameDataInput(spFrameBuf);
        }
      }
    } else {
      mc_ctx->h_module->FrameDataInput(spFrameBuf);
    }
  }

  LOGE << "chnl[" << mc_ctx->h_module->GetChannelId() << "] on_nv12_read";
}
#endif

int FileInputModule::ModuleInit() {
  HOBOT_CHECK(input_cfg_) << "chnl[" << channel_id_
                          << "] input cfg has not set";
  int ret = this->LoadConfig(input_cfg_->source_cfg_file);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] load file config failed, ret: " << ret
                        << ", path: " << input_cfg_->source_cfg_file;

  LOGI << "chnl[" << channel_id_ << "] File input module init,"
       << " chn_id: " << channel_id_
       << " file_path: " << file_input_cfg_->file_path
       << " codec_id: " << file_input_cfg_->codec_id;

  // Step1: Open file
  inFile_ = fopen(file_input_cfg_->file_path.c_str(), "rb");
  if (!inFile_) {
    LOGE << "chnl[" << channel_id_ << "] Open file["
         << file_input_cfg_->file_path << "] failed!";
    return -1;
  }

#ifndef TEST_BY_NV12_IMG
  // Step2: Init hb_mm_mc
  if (nullptr == media_ctx_) {
    const media_codec_descriptor_t *desc =
        hb_mm_mc_get_descriptor(file_input_cfg_->codec_id);
    LOGD << "chnl[" << channel_id_ << "] codec description, "
         << " codec_id: " << desc->id << " mode: " << desc->mode
         << " name: " << desc->name;

    media_ctx_ = new media_codec_context_t();
    if (!media_ctx_) {
      LOGE << "chnl[" << channel_id_ << "] New media_codec_context_t failed!";
      return -1;
    }
    memset(media_ctx_, 0, sizeof(media_codec_context_t));
    ret =
        hb_mm_mc_get_default_context(file_input_cfg_->codec_id, 0, media_ctx_);
    if (ret) {
      LOGE << "chnl[" << channel_id_
           << "] hb_mm_mc_get_default_context failed, ret:" << ret;
      return ret;
    }
    assert(media_ctx_->codec_id == file_input_cfg_->codec_id);
    LOGI << "chnl[" << channel_id_ << "] get default context: " << media_ctx_;

    do {
      // 解码参数设置
      auto &dec_params = media_ctx_->video_dec_params;
      dec_params.feed_mode = MC_FEEDING_MODE_STREAM_SIZE;
      dec_params.pix_fmt = MC_PIXEL_FORMAT_NV12;
      dec_params.bitstream_buf_size = 1024 * 1024;  // default: 10*1024*1024
      dec_params.bitstream_buf_count = 3;           // default:5
      ret = hb_mm_mc_initialize(media_ctx_);
      if (ret) {
        LOGE << "chnl[" << channel_id_
             << "] hb_mm_mc_initialize failed, ret = " << ret;
        break;
      }

      // 解码回调设置
      is_finish_ = false;
      media_codec_callback_t callback;
      callback.on_input_buffer_available = on_decoder_input_buffer_available;
      callback.on_output_buffer_available = on_decoder_output_buffer_available;
      callback.on_media_codec_message = on_decoder_media_codec_message;
      auto mc_ctx =
          new strMediaCodecCtx(this, media_ctx_, is_loop_, is_finish_, inFile_);
      ret = hb_mm_mc_set_callback(media_ctx_, &callback, mc_ctx);
      if (ret) {
        LOGE << "chnl[" << channel_id_
             << "] hb_mm_mc_set_callback failed, ret = " << ret;
        break;
      }

      ret = hb_mm_mc_configure(media_ctx_);
      if (ret) {
        LOGE << "chnl[" << channel_id_
             << "] hb_mm_mc_configure failed, ret = " << ret;
        break;
      }

      //设置队列缓冲池
      frames_queue_buf_count_ = input_cfg_->max_vio_buffer;

      // 启动服务
      mc_av_codec_startup_params_t startup_params;
      ret = hb_mm_mc_start(media_ctx_, &startup_params);
      if (ret) {
        LOGE << "chnl[" << channel_id_
             << "] hb_mm_mc_start failed, ret=" << ret;
        break;
      }
    } while (0);

    if (ret) {
      media_codec_state_t status;
      if (hb_mm_mc_get_state(media_ctx_, &status) >= 0) {
        if (MEDIA_CODEC_STATE_UNINITIALIZED != status) {
          hb_mm_mc_stop(media_ctx_);
          hb_mm_mc_release(media_ctx_);
        }
      } else {
        LOGE << "chnl[" << channel_id_ << "] hb_mm_mc_get_state failed!";
      }
      return ret;
    }
  }
#else
  frames_queue_buf_count_ = input_cfg_->max_vio_buffer;
  if (nullptr == worker_) {
    auto ctx =
        new strMediaCodecCtx(this, media_ctx_, is_loop_, is_finish_, inFile_);
    is_finish_ = false;
    worker_ = std::make_shared<std::thread>(&on_nv12_read, ctx);
    LOGE << "chnl[" << channel_id_
         << "] nv12 read thread created! ctx= " << this;
  }
#endif

  // Step3: Init hb_vio
  std::call_once(fb_init_flag_, [&]() {
    // vio init all device
    ret = hb_vio_init(input_cfg_->vpm_cfg_file.c_str());
    HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                          << "] hb_vio_init failed, ret: " << ret
                          << " vpm cfg file: " << input_cfg_->vpm_cfg_file;
  });

  ret = hb_vio_start_pipeline(channel_id_);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] hb_vio_start_pipeline failed, ret: " << ret
                        << ", pls check config file: "
                        << input_cfg_->vpm_cfg_file;

  init_flag_ = true;
  return 0;
}

int FileInputModule::ModuleDeInit() {
  if (init_flag_ == false) {
    LOGE << "chnl[" << channel_id_
         << "] module has not init, init_flag: " << init_flag_;
    return -1;
  }

  // Step-1: Deinit hb_mm_mc
  is_finish_ = true;
  if (inFile_) {
    fclose(inFile_);
    inFile_ = nullptr;
  }

  if (nullptr != worker_) {
    worker_->join();
    worker_ = nullptr;
  }

  // Step-3: Deinit hb_vio
  int ret = hb_vio_stop_pipeline(channel_id_);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] hb_vio_stop_pipeline failed, ret: " << ret
                        << " channel_id: " << channel_id_;

  if (vio_module_->GetTaskNum() == 0) {
    ret = hb_vio_deinit();
    HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                          << "] hb_vio_deinit failed, ret: " << ret;
  }

  init_flag_ = false;
  return 0;
}

spNV12FrameData FileInputModule::GetFrameDataBuf(uint32_t item_size) {
  if (nullptr == frames_pool_) {
    frames_pool_ = Pool<NV12FrameData>::Create(
        frames_queue_buf_count_, frames_queue_buf_count_, item_size);
    if (nullptr == frames_pool_) {
      LOGF << "chnl[" << channel_id_ << "] frames pool Create failed!";
      return nullptr;
    }
    LOGW << "chnl[" << channel_id_
         << "] frames pool Created, item_size = " << item_size;
  }
  return frames_pool_->GetSharedPtr(1000 / file_input_cfg_->frame_rate);
}

void FileInputModule::FrameDataInput(spNV12FrameData spFrame) {
  std::lock_guard<std::mutex> dq_guard(frames_queue_mutex_);
  frames_queue_.push(spFrame);
}

bool FileInputModule::HasNext() {
  std::lock_guard<std::mutex> dq_guard(frames_queue_mutex_);
  return !frames_queue_.empty();
}

int FileInputModule::Next(std::shared_ptr<PyramidFrame> &output_image) {
  //帧率判断
  auto _now_ts = CurrentTs();
  auto _ts_diff_ms = (_now_ts - last_frame_timestamp_) / 1000;
  if (_ts_diff_ms < uint32_t(1000 / file_input_cfg_->frame_rate - 2)) {
    return -1;
  }

  // 从队列中取数据
  spNV12FrameData nv12_frame;
  {
    std::lock_guard<std::mutex> dq_guard(frames_queue_mutex_);
    if (frames_queue_.empty()) {
      return -1;
    }
    nv12_frame = frames_queue_.front();
    frames_queue_.pop();
  }

  // 首次获取帧数据，动态提取信息更新帧率
  if (0 == last_frame_timestamp_) {
    // FrameRate = frame_rate_numerator / frame_rate_denominator.
    auto &tmp_fr = nv12_frame->ctrl.frame_rate;
    if (tmp_fr.numerator && tmp_fr.denominator) {
      file_input_cfg_->frame_rate = tmp_fr.numerator / tmp_fr.denominator;
      LOGW << "chnl[" << channel_id_ << "] The FrameRate is reflushed to "
           << tmp_fr.numerator << "/" << tmp_fr.denominator;
    }
    LOGW << "chnl[" << channel_id_
         << "] frame_rate: " << file_input_cfg_->frame_rate
         << ", width: " << nv12_frame->ctrl.width
         << ", height: " << nv12_frame->ctrl.height;
  }
  last_frame_timestamp_ = _now_ts;

  // 转换为PyramidFrame
  hb_vio_buffer_t *src_vpm_buf = reinterpret_cast<hb_vio_buffer_t *>(
      std::calloc(1, sizeof(hb_vio_buffer_t)));
  if (nullptr == src_vpm_buf) {
    LOGE << "chnl[" << channel_id_ << "] std::calloc src vpm buffer failed";
    return -1;
  }

  int ret = -1;
  do {
    ret =
        hb_vio_get_data(channel_id_, HB_VIO_PYM_FEEDBACK_SRC_DATA, src_vpm_buf);
    if (ret) {
      break;
    }

    if (src_vpm_buf->img_addr.height != nv12_frame->ctrl.height ||
        src_vpm_buf->img_addr.width != nv12_frame->ctrl.width) {
      LOGE << "chnl[" << channel_id_
           << "] PYM-conf is not match the mediafile, "
           << nv12_frame->ctrl.width << "*" << nv12_frame->ctrl.height
           << " is expected, and Now " << src_vpm_buf->img_addr.width << "*"
           << src_vpm_buf->img_addr.height
           << ". Conf-Path: " << input_cfg_->vpm_cfg_file;
      ret = -1;
      break;
    }

    auto y_len = nv12_frame->ctrl.height * nv12_frame->ctrl.width;
    memcpy(src_vpm_buf->img_addr.addr[0], nv12_frame->p_data, y_len);
    auto uv_len = y_len >> 1;
    memcpy(src_vpm_buf->img_addr.addr[1], nv12_frame->p_data + y_len, uv_len);
    ret = hb_vio_run_pym(channel_id_, src_vpm_buf);
    if (ret) {
      LOGE << "chnl[" << channel_id_ << "] hb_vio_run_pym failed, ret: " << ret;
      break;
    }
  } while (false);

  if (!ret) {
    // get pyramid data
    ret = GetPyramidFrame(src_vpm_buf, output_image);
    if (ret) {
      LOGE << "chnl[" << channel_id_
           << "] get pyramid frame failed, ret: " << ret;
    }
  }

  if (ret) {
    ret = hb_vio_free_pymbuf(channel_id_, HB_VIO_PYM_FEEDBACK_SRC_DATA,
                             src_vpm_buf);
    if (ret) {
      LOGE << "chnl[" << channel_id_
           << "] hb_vio_free_pymbuf src data failed, ret: " << ret;
    }
    std::free(src_vpm_buf);
    return ret;
  }

  return 0;
}

int FileInputModule::Free(std::shared_ptr<PyramidFrame> &input_image) {
  int ret = FreePyramidFrame(input_image);
  if (ret) {
    LOGE << "chnl[" << channel_id_
         << "] free pyramid frame failed, ret: " << ret;
    return ret;
  }
  return 0;
}

}  // namespace J5Sample
