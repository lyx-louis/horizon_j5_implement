/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/sio_module.h"

#include <fstream>

#include "common/profile.h"
#include "hobotlog/hobotlog.hpp"
#include "modules/sio_module.h"
#include "modules/sensor_input/lidar_input_module.h"


namespace J5Sample {

int SioModule::LoadConfig() {
  LOGI << "Load sio config file:" << config_file_;
  std::ifstream ifs(config_file_);
  if (!ifs) {
    LOGE << "error!!! sio config file is not exist";
    return -1;
  }
  Json::Value cfg_jv;
  std::shared_ptr<JsonConfigWrapper> sio_cfg_json = nullptr;
  ifs >> cfg_jv;
  sio_cfg_json.reset(new JsonConfigWrapper(cfg_jv));

  std::vector<int> config_index;
  if (sio_cfg_json->HasMember("config_index")) {
    config_index = sio_cfg_json->GetIntArray("config_index");
  } else {
    LOGE << "config index parameter is not exist";
    return -1;
  }

  std::map<int, std::string> tmp_chnl_type_map;
  for (auto &conf_idx : config_index) {
    std::string config_index_name = "config_" + std::to_string(conf_idx);
    auto config_json = sio_cfg_json->GetSubConfig(config_index_name);
    HOBOT_CHECK(config_json);

    // parse sonsor config
    auto sensor_cfg = std::make_shared<SensorConfig>();
    if (config_json->HasMember("sensor_type")) {
      sensor_cfg->sensor_type = config_json->GetSTDStringValue("sensor_type");
    }

    if (config_json->HasMember("chnl_ids")) {
      sensor_cfg->chnl_ids = config_json->GetIntArray("chnl_ids");
    }
    for (auto chnl_id : sensor_cfg->chnl_ids) {
      auto chnl_type_map_ite = tmp_chnl_type_map.find(chnl_id);
      if (tmp_chnl_type_map.end() != chnl_type_map_ite) {
        LOGE << "chnl_id " << chnl_id << " conflict in sensor_type "
             << chnl_type_map_ite->second << " and " << sensor_cfg->sensor_type;
        return -1;
      } else {
        tmp_chnl_type_map[chnl_id] = sensor_cfg->sensor_type;
      }
    }

    if (config_json->HasMember("max_buf_count")) {
      sensor_cfg->max_buf_count = config_json->GetIntValue("max_buf_count");
    }

    if (config_json->HasMember("source_cfg_file")) {
      sensor_cfg->source_cfg_file =
          config_json->GetSTDStringValue("source_cfg_file");
    }

    if (!sensor_cfg->sensor_type.empty()) {
      input_cfg_[sensor_cfg->sensor_type] = sensor_cfg;
    }

    LOGI << "Sensor[" << sensor_cfg->sensor_type
         << "] source_cfg_file:" << sensor_cfg->source_cfg_file;
  }

  if (input_cfg_.empty()) {
    LOGF << "Sensors configuration list is empty!";
  }

  return 0;
}

int SioModule::Init(hobot::RunContext *context) {
  LOGD << "Enter Init.";
  if (!sensors_map_.empty()) {
    LOGW << "DataSourcesInit has been executed!";
    return 0;
  }

  for (auto &sensor_cfg_ite : input_cfg_) {
    auto sensor_type = sensor_cfg_ite.first;
    auto sensor_cfg = sensor_cfg_ite.second;
    for (auto chnl_id : sensor_cfg->chnl_ids) {
      auto input_module = SensorFactory(sensor_type, chnl_id);
      if (!input_module) {
        LOGE << "Create Sensor object for channel " << chnl_id << " failed";
        return -1;
      }
      int ret = input_module->Init();
      HOBOT_CHECK(ret == 0) << "ModuleInit failed, ret: " << ret;
      AddTask();
      sensors_map_[chnl_id] = input_module;
    }
  }

  LOGD << "Init Suc.";
  return 0;
}

void SioModule::Reset() {
  for (auto &input_module_ite : sensors_map_) {
    auto input_module = input_module_ite.second;
    FreeTask();
    input_module->Reset();
  }
}

FORWARD_DEFINE(SioModule, 0) {
  LOGV << "Enter Forward0.";
  ModuleProfile profile("SioModule");

  auto multi_sio_msg = std::make_shared<MultiSioMessage>();
  multi_sio_msg->multi_sio_msg_.resize(input_cfg_.size());
  for (auto &input_module_ite : sensors_map_) {
    auto input_module = input_module_ite.second;
    multi_sio_msg->multi_sio_msg_[input_module_ite.first] =
        input_module->GetSioMessage();
  }
  profile.EndRecord();
  LOGD << profile;
  LOGD << "Forward return";
  workflow->Return(this, 0, multi_sio_msg, context);

  // sleep 5 ms and reschedule
  workflow->Reschedule(this, 0, input, context, 5);
}

spSensor SioModule::SensorFactory(const std::string &sensor_type, int chnl_id) {
  spSensor input_module = nullptr;
  if (sensor_type == "lidar") {
    input_module = std::make_shared<LidarInputModule>(sensor_type, chnl_id,
                                                      input_cfg_, this);
  } else {
    LOGE << "Unsupport sensor type: " << sensor_type;
  }

  return input_module;
}

}  // namespace J5Sample
