/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/vio_module.h"

#include <fstream>

#include "common/profile.h"
#include "hobotlog/hobotlog.hpp"
#include "modules/video_input/camera_input_module.h"
#include "modules/video_input/file_input_module.h"
#include "modules/video_input/network_input_module.h"
#include "modules/vio_module.h"

namespace J5Sample {

int VioModule::LoadConfig(std::vector<int> &en_chnls) {
  LOGI << "Load vio config file:" << config_file_;
  std::ifstream ifs(config_file_);
  if (!ifs) {
    LOGE << "error!!! vio config file is not exist";
    return -1;
  }
  Json::Value cfg_jv;
  std::shared_ptr<JsonConfigWrapper> vio_cfg_json = nullptr;
  ifs >> cfg_jv;
  vio_cfg_json.reset(new JsonConfigWrapper(cfg_jv));

  int config_index;
  input_cfg_ = std::make_shared<InputConfig>();
  if (vio_cfg_json->HasMember("config_index")) {
    config_index = vio_cfg_json->GetIntValue("config_index");
  } else {
    LOGE << "config index parameter is not exist";
    return -1;
  }
  if (vio_cfg_json->HasMember("board_name")) {
    input_cfg_->board_name = vio_cfg_json->GetSTDStringValue("board_name");
  }

  std::string config_index_name = "config_" + std::to_string(config_index);
  auto config_json = vio_cfg_json->GetSubConfig(config_index_name);
  HOBOT_CHECK(config_json);

  // parse input config
  if (config_json->HasMember("data_source")) {
    input_cfg_->data_source = config_json->GetSTDStringValue("data_source");
  }
  if (config_json->HasMember("chnl_ids")) {
    input_cfg_->channel_id = config_json->GetIntArray("chnl_ids");
  }
  input_cfg_->max_vio_buffer = 0;
  if (config_json->HasMember("max_vio_buffer")) {
    input_cfg_->max_vio_buffer = config_json->GetIntValue("max_vio_buffer");
  }
  if (0 == input_cfg_->max_vio_buffer) {
    input_cfg_->max_vio_buffer = 3;
  }
  if (config_json->HasMember("is_msg_package")) {
    input_cfg_->is_msg_package = config_json->GetIntValue("is_msg_package");
  }
  if (config_json->HasMember("source_cfg_file")) {
    input_cfg_->source_cfg_file =
        config_json->GetSTDStringValue("source_cfg_file");
  }
  if (config_json->HasMember("vpm_cfg_file")) {
    input_cfg_->vpm_cfg_file = config_json->GetSTDStringValue("vpm_cfg_file");
  }
  en_chnls = input_cfg_->channel_id;

  LOGI << "board_name: " << input_cfg_->board_name;
  LOGI << "data_source: " << input_cfg_->data_source;
  LOGI << "max_vio_buffer: " << input_cfg_->max_vio_buffer;
  LOGI << "source_cfg_file:" << input_cfg_->source_cfg_file;
  LOGI << "vpm_cfg_file:" << input_cfg_->vpm_cfg_file;

  return 0;
}

int VioModule::Init(hobot::RunContext *context) {
  LOGD << "Enter Init.";
  if (!input_module_map_.empty()) {
    LOGW << "DataSourcesInit has been executed!";
    return 0;
  }

  for (auto chnl_id : input_cfg_->channel_id) {
    auto input_module = InputModuleFactory(chnl_id);
    if (!input_module) {
      LOGE << "Create InputModule object for channel " << chnl_id << " failed";
      return -1;
    }
    int ret = input_module->Init();
    HOBOT_CHECK(ret == 0) << "ModuleInit failed, ret: " << ret;
    AddTask();
    input_module_map_[chnl_id] = input_module;
  }

  LOGD << "Init Suc.";
  return 0;
}

void VioModule::Reset() {
  for (auto &input_module_ite : input_module_map_) {
    auto input_module = input_module_ite.second;
    FreeTask();
    input_module->Reset();
  }
}


class Vec_vioMessage : public hobot::Message {
  public:
  std::vector<spVioMessage> vios_;
  Vec_vioMessage(){}
  explicit Vec_vioMessage(std::vector<spVioMessage> vios) : vios_(vios) {}
  //spVioMessage -> std::shared_ptr<VioMessage>
};


FORWARD_DEFINE(VioModule, 0) {
  LOGV << "Enter Forward0.";
  ModuleProfile profile("VioModule");
  std::vector<spVioMessage> vec_vio;//1 -> 6
  for (auto &input_module_ite : input_module_map_) {
    auto input_module = input_module_ite.second;
    auto vio_msg = input_module->GetVioMessage();
    vec_vio.push_back(vio_msg);
    //std::cout<<"size = "<<vec_vio.size()<<std::endl;
    if (vio_msg) {
      profile.EndRecord();
      LOGD << profile;
      LOGD << "Forward return";
    }
  }

  int pym_id = vec_vio[0]->pym_image_->frame_id_ /6; //(1 7 13 19 )/6 -> 0 1 2 3
  //
  if(pym_id % 3 == 0){
    workflow->Return(this, 0, hobot::spMessage(new Vec_vioMessage(vec_vio)), context);
  }
  //workflow->Return(this, 0, hobot::spMessage(new Vec_vioMessage(vec_vio)), context);
  
  // sleep 5 ms and reschedule
  workflow->Reschedule(this, 0, input, context, 5);
}

spInputModule VioModule::InputModuleFactory(int chnl_id) {
  spInputModule input_module = nullptr;
  if (input_cfg_->data_source == "mipi_camera") {
    input_module =
        std::make_shared<CameraInputModule>(chnl_id, input_cfg_, this);
  } else if (input_cfg_->data_source == "network_feedback") {
    input_module =
        std::make_shared<NetworkInputModule>(chnl_id, input_cfg_, this);
  } else if (input_cfg_->data_source == "file_feedback") {
    input_module = std::make_shared<FileInputModule>(chnl_id, input_cfg_, this);
  } else {
    LOGE << "Unsupport data source: " << input_cfg_->data_source;
  }

  return input_module;
}

}  // namespace J5Sample
