/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/codec_module.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "message/vio_message.h"

namespace J5Sample {

int CodecModule::LoadConfigFile(
    const std::string &input_config_file,
    std::shared_ptr<JsonConfigWrapper> &output_json_cfg) {
  LOGI << "Load input module config file:" << input_config_file;
  std::ifstream ifs(input_config_file.c_str());
  if (!ifs) {
    LOGE << "Open config file " << input_config_file << " failed";
    return -1;
  }

  Json::Value cfg_jv;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;
  ifs >> cfg_jv;
  json_cfg.reset(new JsonConfigWrapper(cfg_jv));
  output_json_cfg = json_cfg;
  return 0;
}

int CodecModule::LoadConfig(const std::string &config_file) {
  std::shared_ptr<JsonConfigWrapper> cfg_json = nullptr;
  int ret = this->LoadConfigFile(config_file, cfg_json);
  if (ret) {
    LOGE << "load codec config file failed, ret: " << ret;
    return ret;
  }

  for (auto chnl_id : en_chnls_list_) {
    std::string chn_num = "chn_" + std::to_string(chnl_id);
    auto codec_cfg_json = cfg_json->GetSubConfig(chn_num);
    if (codec_cfg_json == nullptr) {
      LOGE << chn_num << " parameter is not exit!";
      return -1;
    }
    auto codec_cfg = std::make_shared<MediaCodecConfig>();
    codec_cfg->chn_id = chnl_id;
    if (codec_cfg_json->HasMember("codec_type")) {
      codec_cfg->codec_id = static_cast<media_codec_id_t>(
          codec_cfg_json->GetIntValue("codec_type"));
    } else {
      LOGE << chn_num << ": codec_type parameter is not exist";
      return -1;
    }
    if (codec_cfg_json->HasMember("image_width")) {
      codec_cfg->image_width = codec_cfg_json->GetIntValue("image_width");
    } else {
      LOGE << chn_num << ": image_width parameter is not exist";
      return -1;
    }
    if (codec_cfg_json->HasMember("image_height")) {
      codec_cfg->image_height = codec_cfg_json->GetIntValue("image_height");
    } else {
      LOGE << chn_num << ": image_height parameter is not exist";
      return -1;
    }
    if (codec_cfg_json->HasMember("frame_buf_count")) {
      codec_cfg->frame_buf_count =
          codec_cfg_json->GetIntValue("frame_buf_count");
    } else {
      LOGE << chn_num << ": frame_buf_count parameter is not exist";
      return -1;
    }
    if (codec_cfg_json->HasMember("jpeg_quality")) {
      codec_cfg->jpeg_quality = codec_cfg_json->GetIntValue("jpeg_quality");
    } else {
      LOGE << chn_num << ": jpeg_quality parameter is not exist";
      return -1;
    }
    chn_cfg_map_[codec_cfg->chn_id] = codec_cfg;
    LOGI << chn_num << " config info��";
    LOGI << "  codec_type: " << codec_cfg->codec_id;
    LOGI << "  image_width: " << codec_cfg->image_width;
    LOGI << "  image_height: " << codec_cfg->image_height;
    LOGI << "  frame_buf_count: " << codec_cfg->frame_buf_count;
    LOGI << "  jpeg_quality: " << codec_cfg->jpeg_quality;
  }
  return 0;
}

int CodecModule::ModuleInit(const std::shared_ptr<MediaCodecConfig> &chn_cfg) {
  if (chn_cfg == nullptr) {
    LOGE << "chn cfg is nullptr";
    return -1;
  }
  auto chn_id = chn_cfg->chn_id;
  auto codec_id = chn_cfg->codec_id;
  auto image_w = chn_cfg->image_width;
  auto image_h = chn_cfg->image_height;
  auto frame_buf_count = chn_cfg->frame_buf_count;
  auto jpeg_quality = chn_cfg->jpeg_quality;
  LOGI << "codec module init,"
       << " chn_id: " << chn_id << " codec_id: " << codec_id
       << " image_w: " << image_w << " image_h: " << image_h
       << " frame_buf_count: " << frame_buf_count
       << " jpeg_quality: " << jpeg_quality;

  if (chn_id >= MAX_CODEC_INSTANCE) {
    LOGE << "chn_id: " << chn_id
         << " is exceed max value: " << MAX_CODEC_INSTANCE;
    return -1;
  }

  const media_codec_descriptor_t *desc = NULL;
  desc = hb_mm_mc_get_descriptor(codec_id);
  LOGI << "codec description, "
       << " codec_id: " << desc->id << " mode: " << desc->mode
       << " name: " << desc->name;
  auto chn_ctx = std::make_shared<MediaCodecContext>();
  chn_ctx->chn_id_ = chn_id;
  chn_ctx_map_[chn_id] = chn_ctx;

  auto context = chn_ctx->context_;
  int ret = hb_mm_mc_get_default_context(codec_id, true, context);
  if (ret) {
    LOGE << "hb_mm_mc_get_default_context failed, ret:" << ret;
    return ret;
  }
  LOGI << "get default context: " << context;
  context->codec_id = MEDIA_CODEC_ID_JPEG;
  context->encoder = true;
  context->video_enc_params.width = image_w;
  context->video_enc_params.height = image_h;
  context->video_enc_params.pix_fmt = MC_PIXEL_FORMAT_NV12;
  context->video_enc_params.frame_buf_count = frame_buf_count;
  context->video_enc_params.external_frame_buf = true;
  context->video_enc_params.bitstream_buf_count = frame_buf_count;
  context->video_enc_params.mir_direction = MC_DIRECTION_NONE;
  context->video_enc_params.rot_degree = MC_CCW_0;
  context->video_enc_params.frame_cropping_flag = false;

  ret = hb_mm_mc_initialize(context);
  if (ret) {
    LOGE << "hb_mm_mc_initialize failed, ret: " << ret;
    return ret;
  }
  ret = hb_mm_mc_configure(context);
  if (ret) {
    LOGE << "hb_mm_mc_configure failed, ret: " << ret;
    return ret;
  }
  mc_jpeg_enc_params_t jpeg_params;
  memset(&jpeg_params, 0x00, sizeof(jpeg_params));
  ret = hb_mm_mc_get_jpeg_config(context, &jpeg_params);
  if (ret) {
    LOGE << "hb_mm_mc_get_jpeg_config failed, ret: " << ret;
    return ret;
  }
  jpeg_params.quality_factor = jpeg_quality;
  jpeg_params.crop_en = false;
  ret = hb_mm_mc_set_jpeg_config(context, &jpeg_params);
  if (ret) {
    LOGE << "hb_mm_mc_set_jpeg_config failed, ret: " << ret;
    return ret;
  }
  mc_av_codec_startup_params_t startup_params;
  startup_params.video_enc_startup_params.receive_frame_number = 0;
  ret = hb_mm_mc_start(context, &startup_params);
  if (ret) {
    LOGE << "hb_mm_mc_start failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int CodecModule::ModuleDeInit() {
  std::lock_guard<std::mutex> l(context_mutex_);
  for (auto &chn_ctx_item : chn_ctx_map_) {
    auto chn_ctx = chn_ctx_item.second;
    if (chn_ctx) {
      int ret = hb_mm_mc_stop(chn_ctx->context_);
      if (ret) {
        LOGE << "codec chn_id: " << chn_ctx_item.first
             << " stop failed, ret: " << ret;
        return ret;
      }
      ret = hb_mm_mc_release(chn_ctx->context_);
      if (ret) {
        LOGE << "codec chn_id: " << chn_ctx_item.first
             << " release failed, ret: " << ret;
        return ret;
      }
      chn_ctx->context_ = nullptr;
    }
  }
  return 0;
}

int CodecModule::Init(hobot::RunContext *context) {
  int ret = LoadConfig(config_file_);
  HOBOT_CHECK(ret == 0) << "load config file failed, ret: " << ret
                        << ", path: " << config_file_;

  for (auto chnl_id : en_chnls_list_) {
    auto chn_cfg_ite = chn_cfg_map_.find(chnl_id);
    if (chn_cfg_map_.end() == chn_cfg_ite) {
      continue;
    }
    auto chn_cfg = chn_cfg_ite->second;
    auto codec_id = chn_cfg->codec_id;
    // only test J5 jpeg encoder
    if (codec_id == MEDIA_CODEC_ID_JPEG || codec_id == MEDIA_CODEC_ID_MJPEG) {
      ret = ModuleInit(chn_cfg);
      HOBOT_CHECK(ret == 0) << "jpeg codec module init failed, ret: " << ret;
    } else {
      LOGE << "UnSupport codec id: " << codec_id;
      return -1;
    }
  }
  init_flag_ = true;
  return 0;
}

void CodecModule::Reset() {
  int ret = -1;
  if (init_flag_ == false) {
    return;
  }
  ret = this->ModuleDeInit();
  HOBOT_CHECK(ret == 0) << "module deinit failed, ret: " << ret;
}

FORWARD_DEFINE(CodecModule, 0) {
  auto msg_list = input[0];
  for (uint32_t msg_idx = 0; msg_idx < msg_list->size(); ++msg_idx) {
    hobot::spMessage spRequest = msg_list->at(msg_idx);
    VioMessage *vio_msg = static_cast<VioMessage *>(spRequest.get());
    HOBOT_CHECK(vio_msg);
    std::shared_ptr<PyramidFrame> pym_image = vio_msg->pym_image_;
    HOBOT_CHECK(pym_image);
    ModuleProfile profile("CodecModule");
    LOGI << "codec module receive vio msg,"
         << " msg_type: " << vio_msg->type_
         << " channel_id: " << vio_msg->channel_
         << " frame_id: " << vio_msg->frame_id_
         << " time_stamp: " << vio_msg->time_stamp_;
    HOBOT_CHECK(pym_image);
    DumpInputImage(pym_image);

    // hobot::spMessage output;
    std::shared_ptr<StreamFrame> stream_info = nullptr;
    if (GetStream(pym_image, stream_info) == 0 && stream_info != nullptr) {
      std::shared_ptr<CodecMessage> codec_msg(
          new CodecImageMessage(stream_info), [&, this](CodecImageMessage *p) {
            if (p) {
              LOGI << "begin delete CodecImageMessage";
              this->FreeStream(p->codec_image_);
              delete (p);
            }
            p = nullptr;
          });
      if (codec_msg) {
        profile.EndRecord();
        LOGD << profile;
        LOGD << "CodecModule.Forward return";
        workflow->Return(this, 0, codec_msg, context);
      }
    }
  }
}

bool CodecModule::FindAvailableImage(const int &image_width,
                                     const int &image_height,
                                     std::vector<ImageLevelInfo> &image_level,
                                     int &output_index) {
  int level_num = image_level.size();

  for (int i = 0; i < level_num; i++) {
    auto width = image_level[i].width;
    auto height = image_level[i].height;
    if (width == image_width) {
      if (height == image_height) {
        output_index = i;
        return true;
      }
    }
  }
  output_index = -1;
  return false;
}

bool CodecModule::FindPymLayer(const int &image_width, const int &image_height,
                               const std::shared_ptr<PyramidFrame> &input,
                               ImageLevelInfo &output) {
  bool bret = false;
  int output_index = -1;
  if (image_width <= 0 || image_height <= 0 || image_width > 4096 ||
      image_height > 4096 || input == nullptr) {
    LOGE << "input parameters is ileagal, "
         << " image_width: " << image_width << " image_height: " << image_height
         << " input: " << input;
    return false;
  }

  // 1. query src layer info
  std::vector<ImageLevelInfo> tmp;
  tmp.push_back(input->src_info_);
  bret = FindAvailableImage(image_width, image_height, tmp, output_index);
  if (bret == true) {
    output = input->src_info_;
    return bret;
  }
  // 2. query bilinear ds layer info
  bret = FindAvailableImage(image_width, image_height, input->bl_ds_,
                            output_index);
  if (bret == true) {
    output = input->bl_ds_[output_index];
    return bret;
  }
  // 3. query gauss ds layer info
  bret = FindAvailableImage(image_width, image_height, input->gs_ds_,
                            output_index);
  if (bret == true) {
    output = input->gs_ds_[output_index];
    return bret;
  }
  // 4. query roi ds layer info
  bret = FindAvailableImage(image_width, image_height, input->roi_ds_,
                            output_index);
  if (bret == true) {
    output = input->roi_ds_[output_index];
    return bret;
  }
  // 5. query roi us layer info
  bret = FindAvailableImage(image_width, image_height, input->roi_us_,
                            output_index);
  if (bret == true) {
    output = input->roi_us_[output_index];
    return bret;
  }
  return false;
}

int CodecModule::GetStream(const std::shared_ptr<PyramidFrame> &input,
                           std::shared_ptr<StreamFrame> &output) {
  if (input == nullptr) {
    LOGE << "input pyramid frame is nullptr";
    return -1;
  }
  if (init_flag_ == false) {
    LOGE << "codec module has not init";
    return -1;
  }
  int chn_id = input->channel_id_;
  auto chn_cfg_ite = chn_cfg_map_.find(chn_id);
  if (chn_cfg_map_.end() == chn_cfg_ite) {
    LOGE << "chn_id: " << chn_id << " is not in chn_cfg_map_";
    return -1;
  }
  auto chn_cfg = chn_cfg_ite->second;
  auto chn_ctx_ite = chn_ctx_map_.find(chn_id);
  if (chn_ctx_map_.end() == chn_ctx_ite) {
    LOGE << "chn_id: " << chn_id << " is not in chn_ctx_map_";
    return -1;
  }
  auto chn_ctx = chn_ctx_ite->second;
  auto context = chn_ctx->context_;
  if (context == nullptr) {
    LOGE << "chn_id: " << chn_id << " chn context is nullptr";
    return -1;
  }

  ImageLevelInfo src_info = {0};
  int image_width = chn_cfg->image_width;
  int image_height = chn_cfg->image_height;
  bool bret = FindPymLayer(image_width, image_height, input, src_info);
  if (bret == false) {
    LOGE << "not find pym layer, "
         << " codec_image_width: " << chn_cfg->image_width
         << " codec_image_height: " << chn_cfg->image_height;
    return -1;
  }
  LOGD << "find available pym layer,"
       << " width: " << src_info.width << " height: " << src_info.height
       << " image_width: " << image_width << " image_height: " << image_height
       << " stride: " << src_info.stride << " y_paddr: " << src_info.y_paddr
       << " c_paddr: " << src_info.c_paddr << " y_vaddr: " << src_info.y_vaddr
       << " c_vaddr: " << src_info.c_vaddr;

  // 1. dequeue input buffer
  media_codec_buffer_t frame_buffer;
  memset(&frame_buffer, 0x00, sizeof(media_codec_buffer_t));
  int ret = hb_mm_mc_dequeue_input_buffer(context, &frame_buffer, 2000);
  if (ret) {
    LOGE << "hb_mm_mc_dequeue_input_buffer failed, ret: " << ret;
    return ret;
  }
  // 2. queue input buffer
  frame_buffer.vframe_buf.phy_ptr[0] = src_info.y_paddr;
  frame_buffer.vframe_buf.vir_ptr[0] =
      reinterpret_cast<uint8_t *>(src_info.y_vaddr);
  frame_buffer.vframe_buf.phy_ptr[1] = src_info.c_paddr;
  frame_buffer.vframe_buf.vir_ptr[1] =
      reinterpret_cast<uint8_t *>(src_info.c_vaddr);
  frame_buffer.vframe_buf.size = image_width * image_height * 3 / 2;
  ret = hb_mm_mc_queue_input_buffer(context, &frame_buffer, 2000);
  if (ret) {
    std::cout << "hb_mm_mc_queue_input_buffer failed, ret: " << ret;
    return ret;
  }
  // 3. dequeue output buffer, get stream
  auto encoded_buffer = new media_codec_buffer_t;
  memset(encoded_buffer, 0x00, sizeof(media_codec_buffer_t));
  ret = hb_mm_mc_dequeue_output_buffer(context, encoded_buffer, nullptr, 2000);
  if (ret) {
    LOGE << "hb_mm_mc_dequeue_output_buffer failed, ret: " << ret;
    return ret;
  }

  // 4. StreamFrame output
  auto stream_frame = std::make_shared<StreamFrame>();
  if (chn_cfg->codec_id == MEDIA_CODEC_ID_JPEG) {
    stream_frame->pixel_format_ =
        HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG;
  } else if (chn_cfg->codec_id == MEDIA_CODEC_ID_MJPEG) {
    stream_frame->pixel_format_ =
        HorizonVisionPixelFormat::kHorizonVisionPixelFormatMJPEG;
  } else if (chn_cfg->codec_id == MEDIA_CODEC_ID_H265) {
    stream_frame->pixel_format_ =
        HorizonVisionPixelFormat::kHorizonVisionPixelFormatH265;
  } else {
    LOGE << "unSupport pixel format: " << chn_cfg->codec_id;
    return -1;
  }
  stream_frame->channel_id_ = input->channel_id_;
  stream_frame->time_stamp_ = input->time_stamp_;
  stream_frame->system_time_stamp_ = input->system_time_stamp_;
  stream_frame->frame_id_ = input->frame_id_;
  stream_frame->stream_context_ = reinterpret_cast<void *>(encoded_buffer);
  stream_frame->stream_info_.src_width = static_cast<uint16_t>(image_width);
  stream_frame->stream_info_.src_height = static_cast<uint16_t>(image_height);
  stream_frame->stream_info_.stream_size =
      static_cast<uint32_t>(encoded_buffer->vstream_buf.size);
  stream_frame->stream_info_.stream_pts =
      static_cast<uint64_t>(encoded_buffer->vstream_buf.pts);
  stream_frame->stream_info_.stream_paddr =
      static_cast<uint64_t>(encoded_buffer->vstream_buf.phy_ptr);
  stream_frame->stream_info_.stream_vaddr =
      reinterpret_cast<uint64_t>(encoded_buffer->vstream_buf.vir_ptr);
  output = stream_frame;

  // 5. test dump encoded image
  DumpOutputImage(output);
  return 0;
}

int CodecModule::FreeStream(const std::shared_ptr<StreamFrame> &input) {
  std::lock_guard<std::mutex> l(context_mutex_);
  int ret = -1;
  if (input == nullptr) {
    LOGE << "input stream frame is nullptr";
    return -1;
  }
  if (init_flag_ == false) {
    LOGE << "codec module has not init";
    return -1;
  }
  int chn_id = input->channel_id_;
  auto chn_ctx_ite = chn_ctx_map_.find(chn_id);
  if (chn_ctx_map_.end() == chn_ctx_ite) {
    LOGE << "chn_id: " << chn_id << " is not in chn_ctx_map_.";
    return -1;
  }
  auto chn_ctx = chn_ctx_ite->second;
  int chn_id_val = chn_ctx->chn_id_;
  HOBOT_CHECK(chn_id == chn_id_val)
      << "chn_id: " << chn_id << " chn_id_val: " << chn_id_val;
  auto context = chn_ctx->context_;
  if (context == nullptr) {
    LOGE << "chn_id: " << chn_id << "context is nullptr";
    return -1;
  }
  // dequeue output buffer, free stream
  media_codec_buffer_t *encoded_buffer =
      reinterpret_cast<media_codec_buffer_t *>(input->stream_context_);
  if (encoded_buffer == nullptr) {
    LOGE << "free stream frame failed, context is nullptr";
    return -1;
  }
  LOGD << "queue output buffer, context: " << context;
  ret = hb_mm_mc_queue_output_buffer(context, encoded_buffer, -1);
  if (ret) {
    LOGE << "hb_mm_mc_queue_output_buffer failed, ret: " << ret
         << " context: " << context;
    return ret;
  }
  if (encoded_buffer) delete encoded_buffer;

  return 0;
}

void CodecModule::GetDumpNum(const std::string &input, bool &dump_en,
                             int &dump_num) {
  if (0 == access(input.c_str(), F_OK)) {
    dump_en = true;
    std::string str;
    std::ifstream ifs(input);
    std::getline(ifs, str);
    if (str.empty()) {
      dump_num = -1;
    } else {
      // trim the spaces in the beginning
      str.erase(0, str.find_first_not_of(' '));
      // trim the spaces in the end
      str.erase(str.find_last_not_of(' ') + 1, std::string::npos);
      dump_num = std::stoi(str);
    }
  }
}

void CodecModule::DumpInputImage(const std::shared_ptr<PyramidFrame> &frame) {
  if (frame == nullptr) {
    return;
  }
  int chn_id = frame->channel_id_;
  int frame_id = frame->frame_id_;
  int ts = frame->time_stamp_;
  auto fmt = frame->pixel_format_;
  int width = frame->src_info_.width;
  int height = frame->src_info_.height;
  int stride = frame->src_info_.stride;
  int y_size = width * height;
  int c_size = y_size / 2;
  char *y_buf = reinterpret_cast<char *>(frame->src_info_.y_vaddr);
  char *c_buf = reinterpret_cast<char *>(frame->src_info_.c_vaddr);
  LOGD << "codec module input source image,"
       << " chn_id: " << chn_id << " frame_id: " << frame_id << " ts: " << ts
       << " fmt: " << fmt << " width: " << width << " height: " << height
       << " stride:" << stride << " y_paddr:" << frame->src_info_.y_paddr
       << " c_paddr:" << frame->src_info_.c_paddr
       << " y_vaddr:" << frame->src_info_.y_vaddr
       << " c_vaddr:" << frame->src_info_.c_vaddr;

  std::string file_name;
  bool dump_en = false;
  int dump_num = 0;
  GetDumpNum("./codec_input.txt", dump_en, dump_num);
  if (dump_en == true && (input_dump_count_++ < dump_num || dump_num < 0)) {
    input_dump_count_++;
    file_name = "pym_src_" + std::to_string(frame_id) + ".yuv";
    LOGW << "dump fname: " << file_name << " width: " << width
         << " height: " << height;
    std::ofstream ofs(file_name);
    ofs.write(y_buf, y_size);
    ofs.write(c_buf, c_size);
    ofs.flush();
    ofs.close();
  }
}

void CodecModule::DumpOutputImage(const std::shared_ptr<StreamFrame> &stream) {
  if (stream == nullptr) {
    return;
  }
  int chn_id = stream->channel_id_;
  int frame_id = stream->frame_id_;
  auto fmt = stream->pixel_format_;
  auto pts = stream->stream_info_.stream_pts;
  char *vaddr = reinterpret_cast<char *>(stream->stream_info_.stream_vaddr);
  uint32_t size = stream->stream_info_.stream_size;

  std::string file_name;
  bool dump_en = false;
  int dump_num = 0;
  GetDumpNum("./codec_output.txt", dump_en, dump_num);
  if (dump_en == true && (output_dump_count_++ < dump_num || dump_num < 0)) {
    if (fmt == HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG ||
        fmt == HorizonVisionPixelFormat::kHorizonVisionPixelFormatMJPEG) {
      file_name = "chn" + std::to_string(chn_id) + "_" +
                  std::to_string(frame_id) + ".jpg";
    } else if (fmt == HorizonVisionPixelFormat::kHorizonVisionPixelFormatH265) {
      if (dump_num < 0)
        file_name = "chn" + std::to_string(chn_id) + ".h264";
      else
        file_name =
            "chn" + std::to_string(chn_id) + std::to_string(dump_num) + ".h265";
    } else {
      LOGE << "unSupport codec format: " << fmt;
      return;
    }
    LOGW << "dump fname: " << file_name << " frame_id: " << frame_id
         << " pts: " << pts
         << " frame_addr: " << reinterpret_cast<void *>(vaddr)
         << " frame_size: " << size;
    std::ofstream ofs(file_name);
    ofs.write(vaddr, size);
    ofs.flush();
    ofs.close();
  }
}

}  // namespace J5Sample
