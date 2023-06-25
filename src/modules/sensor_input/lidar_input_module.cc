/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/sensor_input/lidar_input_module.h"

#include <string>

#include "hobotlog/hobotlog.hpp"
#include "modules/sio_module.h"
#include "utils/protocol/test.pb.h"

namespace J5Sample {

int LidarInputModule::LoadConfig(const std::string &config_file) {
  std::shared_ptr<JsonConfigWrapper> cfg_json = nullptr;
  int ret = this->LoadConfigFile(config_file, cfg_json);
  if (ret) {
    LOGE << "chnl[" << channel_id_ << "] load config file failed, ret: " << ret;
    return ret;
  }

  std::string chn_num = "chn_" + std::to_string(channel_id_);
  auto lidar_cfg_json = cfg_json->GetSubConfig(chn_num);
  if (lidar_cfg_json == nullptr) {
    LOGE << chn_num << ": parameter is not exit!";
    return -1;
  }

  lidar_input_cfg_ = std::make_shared<LidarInputConfig>();
  if (lidar_cfg_json->HasMember("ip_port")) {
    lidar_input_cfg_->ip_port = lidar_cfg_json->GetSTDStringValue("ip_port");
  } else {
    LOGE << chn_num << ": ip_port parameter is not exist";
    return -1;
  }

  LOGI << "config info for " << chn_num;
  return 0;
}

int LidarInputModule::ModuleInit() {
  int ret = this->LoadConfig(input_cfg_[sensor_type_]->source_cfg_file);
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] load network config failed, ret: " << ret
                        << ", path: " << input_cfg_[sensor_type_]->source_cfg_file;

  // init zmq
  if (!receiver_.Init(lidar_input_cfg_->ip_port.c_str())) {
    LOGE << "chnl[" << channel_id_ << "] init receiver failed, the ip port is "
         << lidar_input_cfg_->ip_port;
    return -1;
  }
  LOGW << "chnl[" << channel_id_
       << "] init receiver success, ip_port: " << lidar_input_cfg_->ip_port;

  init_flag_ = true;
  return 0;
}

int LidarInputModule::ModuleDeInit() {
  if (init_flag_ == false) {
    LOGE << "chnl[" << channel_id_
         << "] module has not init, init_flag: " << init_flag_;
    return -1;
  }

  // receiver finish
  receiver_.Fini();

  init_flag_ = false;
  return 0;
}

bool LidarInputModule::HasNext() { return !is_finish_; }


int LidarInputModule::Next(std::shared_ptr<std::vector<uint8_t>> &sensor_data) {
  auto frame = receiver_.RecvFrame();
  if (nullptr == frame) return -1;

  for (auto &image : frame->images) {
    auto frame_data = reinterpret_cast<uint8_t *>(zmq_msg_data(image));
    auto frame_size = zmq_msg_size(image);
    receiver_.sendRsp();
    sensor_data->resize(frame_size);
    memcpy(sensor_data->data(), frame_data, frame_size);
  }

  return 0;
}

}  // namespace J5Sample
