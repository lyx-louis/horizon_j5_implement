/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef J5_SAMPLE_UTILS_JSON_CFG_WRAPPER_H_
#define J5_SAMPLE_UTILS_JSON_CFG_WRAPPER_H_
#include <string.h>
#include <memory>
#include <string>
#include <vector>
#include "json/json.h"

namespace J5Sample {

class JsonConfigWrapper {
 public:
  explicit JsonConfigWrapper(Json::Value config) : config_(config) {}

  int GetIntValue(std::string key, int default_value = 0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asInt();
  }

  bool GetBoolValue(std::string key, bool default_value = false) {
    auto value_int = GetIntValue(key, default_value);
    return value_int == 0 ? false : true;
  }

  float GetFloatValue(std::string key, float default_value = 0.0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asFloat();
  }

  std::string GetSTDStringValue(std::string key,
                                std::string default_value = "") {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asString();
  }

  std::vector<std::string> GetSTDStringArray(std::string key) {
    std::vector<std::string> result;
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      value_js = Json::Value("");
    }

    if (value_js.isString()) {
      auto tmp = value_js.asString();
      result.push_back(tmp);
      return result;
    }
    for (unsigned int i = 0; i < value_js.size(); i++) {
      result.push_back(value_js[i].asString());
    }
    return result;
  }

  std::vector<int> GetIntArray(std::string key) {
    auto value_js = config_[key.c_str()];
    std::vector<int> result;
    if (value_js.isNull()) {
      return result;
    }
    if (value_js.isInt()) {
      auto tmp = value_js.asInt();
      result.push_back(tmp);
      return result;
    }
    for (unsigned int i = 0; i < value_js.size(); i++) {
      result.push_back(value_js[i].asInt());
    }
    return result;
  }

  std::shared_ptr<JsonConfigWrapper> GetSubConfig(std::string key) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonConfigWrapper>(new JsonConfigWrapper(value_js));
  }

  std::shared_ptr<JsonConfigWrapper> GetSubConfig(int key) {
    auto value_js = config_[key];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonConfigWrapper>(new JsonConfigWrapper(value_js));
  }

  bool HasMember(std::string key) { return config_.isMember(key); }
  int ItemCount(void) { return config_.size(); }
  Json::Value GetJsonCfg(void) { return config_; }

  std::vector<std::string> GetJsonKeys() {
    std::vector<std::string> ret;
    for (auto sub = config_.begin(); sub != config_.end(); sub++) {
      ret.push_back(sub.name());
    }
    return ret;
  }

 protected:
  Json::Value config_;
};

}  // namespace J5Sample
#endif  // J5_SAMPLE_UTILS_JSON_CFG_WRAPPER_H_
