/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/video_input/camera_input_module.h"

#include <memory>
#include <mutex>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "modules/vio_module.h"

namespace J5Sample {

std::once_flag CameraInputModule::cam_init_flag_;

int CameraInputModule::LoadConfig(const std::string &config_file) {
  std::shared_ptr<JsonConfigWrapper> cam_cfg_json = nullptr;
  int ret = this->LoadConfigFile(config_file, cam_cfg_json);
  if (ret) {
    LOGE << "load config file failed, ret: " << ret;
    return ret;
  }
  cam_input_cfg_ = std::make_shared<CameraInputConfig>();
  if (cam_cfg_json->HasMember("cam_index")) {
    cam_input_cfg_->cam_index = cam_cfg_json->GetIntValue("cam_index");
  } else {
    LOGE << "cam_index parameter is not exist";
    return -1;
  }
  if (cam_cfg_json->HasMember("cam_cfg_file")) {
    cam_input_cfg_->cam_cfg_file =
        cam_cfg_json->GetSTDStringValue("cam_cfg_file");
  } else {
    LOGE << "cam_cfg_file parameter is not exist";
    return -1;
  }
  if (cam_cfg_json->HasMember("frame_count")) {
    cam_input_cfg_->frame_count = cam_cfg_json->GetIntValue("frame_count");
  }

  LOGI << "cam_index: " << cam_input_cfg_->cam_index;
  LOGI << "cam_cfg_file: " << cam_input_cfg_->cam_cfg_file;
  LOGI << "frame_count: " << cam_input_cfg_->frame_count;
  return 0;
}

int CameraInputModule::ModuleInit() {
  HOBOT_CHECK(input_cfg_) << "input cfg has not set";
  int ret = this->LoadConfig(input_cfg_->source_cfg_file);
  HOBOT_CHECK(ret == 0) << "load camera config file failed, ret: " << ret
                        << ", path: " << input_cfg_->source_cfg_file;
  std::call_once(cam_init_flag_, [&]() {
    // vio and camera init all device
    ret = hb_vio_init(input_cfg_->vpm_cfg_file.c_str());
    HOBOT_CHECK(ret == 0) << "hb_vio_init failed, ret: " << ret
                          << ", config: " << input_cfg_->vpm_cfg_file;
    ret = hb_cam_init(cam_input_cfg_->cam_index,
                      cam_input_cfg_->cam_cfg_file.c_str());
    HOBOT_CHECK(ret == 0) << "hb_cam_init failed, ret: " << ret
                          << ", cam_id: " << cam_input_cfg_->cam_index
                          << ", config: " << cam_input_cfg_->cam_cfg_file;
  });

  ret = hb_vio_start_pipeline(channel_id_);
  HOBOT_CHECK(ret == 0) << "hb_vio_start_pipeline failed, ret: " << ret
                        << " channel_id: " << channel_id_;
  ret = hb_cam_start(channel_id_);
  HOBOT_CHECK(ret == 0) << "hb_cam_start failed, ret: " << ret
                        << " channel_id: " << channel_id_;

  init_flag_ = true;
  return 0;
}

int CameraInputModule::ModuleDeInit() {
  if (init_flag_ == false) {
    LOGE << "module has not init, init_flag: " << init_flag_;
    return -1;
  }
  int ret = hb_vio_stop_pipeline(channel_id_);
  HOBOT_CHECK(ret == 0) << "hb_vio_stop_pipeline failed, ret: " << ret
                        << " channel_id: " << channel_id_;
  ret = hb_cam_stop(channel_id_);
  HOBOT_CHECK(ret == 0) << "hb_cam_stop failed, ret: " << ret
                        << " channel_id: " << channel_id_;

  if (vio_module_->GetTaskNum() == 0) {
    // camera and vio deinit all pipeline
    ret = hb_cam_deinit(cam_input_cfg_->cam_index);
    HOBOT_CHECK(ret == 0) << "hb_cam_deinit failed, ret: " << ret
                          << " cam_index: " << cam_input_cfg_->cam_index;
    ret = hb_vio_deinit();
    HOBOT_CHECK(ret == 0) << "hb_vio_deinit failed, ret: " << ret;
  }
  init_flag_ = false;
  return 0;
}

bool CameraInputModule::HasNext() {
  int frame_count = cam_input_cfg_->frame_count;
  return (frame_count == -1 || count_ < frame_count);
}

int CameraInputModule::Next(std::shared_ptr<PyramidFrame> &output_image) {
  // get pyramid data
  void *src_buf = nullptr;
  int ret = GetPyramidFrame(src_buf, output_image);
  if (ret) {
    LOGE << "get pyramid frame failed, ret: " << ret;
    return ret;
  }
  count_++;
  return 0;
}

int CameraInputModule::Free(std::shared_ptr<PyramidFrame> &input_image) {
  int ret = FreePyramidFrame(input_image);
  if (ret) {
    LOGE << "free pyramid frame failed, ret: " << ret;
    return ret;
  }
  return 0;
}

}  // namespace J5Sample
