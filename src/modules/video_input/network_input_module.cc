/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/video_input/network_input_module.h"

#include <string>

#include "hobotlog/hobotlog.hpp"
#include "modules/vio_module.h"
#include "utils/protocol/test.pb.h"

namespace J5Sample {

std::once_flag NetworkInputModule::fb_init_flag_;

int NetworkInputModule::LoadConfig(const std::string &config_file) {
  std::shared_ptr<JsonConfigWrapper> cfg_json = nullptr;
  int ret = this->LoadConfigFile(config_file, cfg_json);
  if (ret) {
    LOGE << "chnl[" << channel_id_ << "] load config file failed, ret: " << ret;
    return ret;
  }

  std::string chn_num = "chn_" + std::to_string(channel_id_);
  auto network_cfg_json = cfg_json->GetSubConfig(chn_num);
  if (network_cfg_json == nullptr) {
    LOGE << chn_num << ": parameter is not exit!";
    return -1;
  }

  network_input_cfg_ = std::make_shared<NetworkInputConfig>();
  if (network_cfg_json->HasMember("ip_port")) {
    network_input_cfg_->ip_port =
        network_cfg_json->GetSTDStringValue("ip_port");
  } else {
    LOGE << chn_num << ": ip_port parameter is not exist";
    return -1;
  }
  if (network_cfg_json->HasMember("data_type")) {
    network_input_cfg_->data_type = static_cast<HorizonVisionPixelFormat>(
        network_cfg_json->GetIntValue("data_type"));
  } else {
    LOGE << chn_num << ": data type parameter is not exist";
    return -1;
  }
  if (network_cfg_json->HasMember("height")) {
    network_input_cfg_->height = network_cfg_json->GetIntValue("height");
  } else {
    LOGE << chn_num << ": height parameter is not exist";
    return -1;
  }
  if (network_cfg_json->HasMember("width")) {
    network_input_cfg_->width = network_cfg_json->GetIntValue("width");
  } else {
    LOGE << chn_num << ": width parameter is not exist";
    return -1;
  }
  if (network_cfg_json->HasMember("with_image_name")) {
    network_input_cfg_->with_image_name =
        network_cfg_json->GetBoolValue("with_image_name");
  } else {
    LOGE << chn_num << ": with_image_name parameter is not exist";
    return -1;
  }

  LOGI << "config info for " << chn_num;
  LOGI << "  ip_port: " << network_input_cfg_->ip_port;
  LOGI << "  data_type: " << network_input_cfg_->data_type;
  LOGI << "  width: " << network_input_cfg_->width;
  LOGI << "  height: " << network_input_cfg_->height;
  LOGI << "  with_image_name: " << network_input_cfg_->with_image_name;
  return 0;
}

int NetworkInputModule::ModuleInit() {
  HOBOT_CHECK(input_cfg_) << "chnl[" << channel_id_
                          << "] input cfg has not set";
  int ret = this->LoadConfig(input_cfg_->source_cfg_file);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] load network config failed, ret: " << ret
                        << ", path: " << input_cfg_->source_cfg_file;

  // init zmq
  if (!receiver_.Init(network_input_cfg_->ip_port.c_str())) {
    LOGE << "chnl[" << channel_id_ << "] init receiver failed, the ip port is "
         << network_input_cfg_->ip_port;
    return -1;
  }
  LOGW << "chnl[" << channel_id_
       << "] init receiver success, ip_port: " << network_input_cfg_->ip_port;

  std::call_once(fb_init_flag_, [&]() {
    // vio init all device
    ret = hb_vio_init(input_cfg_->vpm_cfg_file.c_str());
    HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                          << "] hb_vio_init failed, ret: " << ret
                          << " vpm cfg file: " << input_cfg_->vpm_cfg_file;
  });

  ret = hb_vio_start_pipeline(channel_id_);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] hb_vio_start_pipeline failed, ret: " << ret;

  init_flag_ = true;
  return 0;
}

int NetworkInputModule::ModuleDeInit() {
  if (init_flag_ == false) {
    LOGE << "chnl[" << channel_id_
         << "] module has not init, init_flag: " << init_flag_;
    return -1;
  }

  // vio stop pipeline
  int ret = hb_vio_stop_pipeline(channel_id_);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] hb_vio_stop_pipeline failed, ret: " << ret;

  if (vio_module_->GetTaskNum() == 0) {
    // vio deinit all pipeline
    ret = hb_vio_deinit();
    HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                          << "] hb_vio_deinit failed, ret: " << ret;
  }
  // receiver finish
  receiver_.Fini();

  init_flag_ = false;
  return 0;
}

bool NetworkInputModule::HasNext() { return !is_finish_; }

int NetworkInputModule::Next(std::shared_ptr<PyramidFrame> &output_image) {
  int ret = -1;
  int channel_id = channel_id_;
  std::shared_ptr<PyramidFrame> output = nullptr;
  char *nv12_data = nullptr;
  int width = network_input_cfg_->width;
  int height = network_input_cfg_->height;
  int y_size = width * height;
  int uv_size = y_size / 2;

  // only support nv12 formart
  HOBOT_CHECK(network_input_cfg_->data_type ==
              HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12);

  auto frame = receiver_.RecvFrame();
  if (nullptr == frame) return -1;

  // there have two different protocol here: eval/normal
  // in eval mode, the protocol contains many additional info
  // in normal mode, the protocol only contains image
  auto msg_num = network_input_cfg_->with_image_name ? 2 : 1;

  LOGD << "chnl[" << channel_id_ << "] network receiver msg num: " << msg_num;
  auto src_vpm_buf = reinterpret_cast<hb_vio_buffer_t *>(
      std::calloc(1, sizeof(hb_vio_buffer_t)));
  if (nullptr == src_vpm_buf) {
    LOGE << "chnl[" << channel_id << "] std::calloc src vpm buffer failed";
    return -1;
  }
  ret = hb_vio_get_data(channel_id, HB_VIO_PYM_FEEDBACK_SRC_DATA, src_vpm_buf);
  if (ret) {
    LOGE << "chnl[" << channel_id
         << "] get pym feedback src failed, ret: " << ret;
    return ret;
  }

  // normal model protocol
  if (msg_num == 1) {
    for (auto &image : frame->images) {
      nv12_data = reinterpret_cast<char *>(zmq_msg_data(image));
      auto size = zmq_msg_size(image);
      receiver_.sendRsp();
      if (size != static_cast<size_t>(height * width * 3 / 2)) {
        LOGE << "chnl[" << channel_id << "] nv12 image height: " << height
             << " nv12 image width: " << width << " nv12 image size: " << size;
        return -1;
      } else {
        memcpy(src_vpm_buf->img_addr.addr[0], nv12_data, y_size);
        memcpy(src_vpm_buf->img_addr.addr[1], nv12_data + y_size, uv_size);
      }
    }
  } else {  // in eval, each two msgs contain a complete frame info
    LOGD << "chnl[" << channel_id
         << "] frame image size: " << frame->images.size();
    for (size_t i = 0; i < frame->images.size(); i += msg_num) {
      static std::string last_file;
      // additional info in msg[0]
      ImageInfo::image_info img_info;
      int img_msg_size = zmq_msg_size(frame->images[i]);
      char *img_msg = reinterpret_cast<char *>(zmq_msg_data(frame->images[i]));
      std::string pb_msg(img_msg, img_msg_size);
      if (pb_msg == "transfer completed") {
        LOGW << "chnl[" << channel_id << "] transfer completed";
        is_finish_ = true;
        return -1;
      }
      img_info.ParseFromString(pb_msg);
      receiver_.sendRsp();

      int image_width = img_info.image_width();
      int image_height = img_info.image_height();
      int image_valid_width = img_info.image_valid_width();
      int image_valid_height = img_info.image_valid_height();
      std::string image_name = img_info.image_name();
      LOGD << "chnl[" << channel_id << "] image_name: " << image_name
           << " image_width: " << image_width
           << " image_height: " << image_height
           << " image_valid_width: " << image_valid_width
           << " image_valid_height: " << image_valid_height;
      HOBOT_CHECK(image_width == image_valid_width);
      HOBOT_CHECK(image_height == image_valid_height);

      if (last_file == image_name) {
        LOGW << "chnl[" << channel_id << "] receive msg aggain, skip it";
        delete frame;
        return false;
      }
      last_file = img_info.image_name();

      // image info in msg[1]
      nv12_data = reinterpret_cast<char *>(zmq_msg_data(frame->images[i + 1]));
      int y_size = image_width * image_height;
      int uv_size = y_size / 2;
      {
        // dump nv12 image
        bool dump_en = false;
        int dump_num = 0;
        GetDumpNum("./vin_input.txt", dump_en, dump_num);
        if (dump_en == true && (dump_count_++ < dump_num || dump_num < 0)) {
          std::string file_name = "network_" + std::to_string(image_width) +
                                  "_" + std::to_string(image_height) + ".nv12";
          DumpToFile2Plane(file_name.c_str(), nv12_data, nv12_data + y_size,
                           y_size, uv_size);
        }
      }
      memcpy(src_vpm_buf->img_addr.addr[0], nv12_data, y_size);
      memcpy(src_vpm_buf->img_addr.addr[1], nv12_data + y_size, uv_size);
    }
  }
  ret = hb_vio_run_pym(channel_id, src_vpm_buf);
  if (ret) {
    LOGE << "chnl[" << channel_id << "] hb_vio_run_pym failed, ret: " << ret;
    return ret;
  }
  delete frame;

  // get pyramid data
  ret = GetPyramidFrame(src_vpm_buf, output_image);
  if (ret) {
    LOGE << "chnl[" << channel_id << "] get pyramid frame failed, ret: " << ret;
    return ret;
  }

  return 0;
}

int NetworkInputModule::Free(std::shared_ptr<PyramidFrame> &input_image) {
  int ret = -1;

  ret = FreePyramidFrame(input_image);
  if (ret) {
    LOGE << "chnl[" << channel_id_
         << "] free pyramid frame failed, ret: " << ret;
    return ret;
  }
  return 0;
}

}  // namespace J5Sample
