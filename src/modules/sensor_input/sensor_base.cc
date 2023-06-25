/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/sensor_input/sensor_base.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "common/profile.h"
#include "hobotlog/hobotlog.hpp"

namespace J5Sample {

int SensorBase::LoadConfigFile(
    const std::string &input_config_file,
    std::shared_ptr<JsonConfigWrapper> &output_json_cfg) {
  LOGI << "chnl[" << channel_id_
       << "] Load input module config file:" << input_config_file;
  std::ifstream ifs(input_config_file.c_str());
  if (!ifs) {
    LOGE << "chnl[" << channel_id_ << "] Open config file " << input_config_file
         << " failed";
    return -1;
  }

  Json::Value cfg_jv;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;
  ifs >> cfg_jv;
  json_cfg.reset(new JsonConfigWrapper(cfg_jv));
  output_json_cfg = json_cfg;
  return 0;
}

int SensorBase::Init() {
  int ret = -1;

  LOGD << "chnl[" << channel_id_ << "] Enter Init.";
  ret = this->ModuleInit();
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] init failed, ret: " << ret;
  LOGD << "chnl[" << channel_id_ << "]  Init Suc.";
  return 0;
}

void SensorBase::Reset() {
  int ret = -1;
  int wait_count = 100;

  while (GetConsumedBufferNum() > 0 && wait_count-- > 0) {
    // LOGD << "Wait Sio buffer free.";
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  ret = this->ModuleDeInit();
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] deinit failed, ret: " << ret;
}

std::shared_ptr<SioMessage> SensorBase::GetSioMessage() {
  ModuleProfile profile("SensorBase");

  bool buf_allocated(false);
  do {
    buf_allocated = AllocBuffer();
    if (!buf_allocated) break;
    if (!this->HasNext()) break;

    std::shared_ptr<std::vector<uint8_t>> sp_data =
        std::make_shared<std::vector<uint8_t>>();
    if (this->Next(sp_data) != 0) break;

    std::shared_ptr<SioMessage> sio_msg(new SioMessage(), [&, this](
                                                              SioMessage *p) {
      if (p) {
        LOGI << "chnl[" << channel_id_
             << "] begin delete vioMessage, sequnce_id_ = " << p->sequnce_id_;
        this->FreeBuffer();
        delete (p);
      }
      p = nullptr;
    });
    if (sio_msg == nullptr) {
      break;
    } else {
      sio_msg->data_buf_ = sp_data;
    }

    profile.EndRecord();
    LOGD << profile;
    LOGD << "chnl[" << channel_id_ << "] Forward return";
    static std::atomic_uint64_t last_frame_id_{0};
    sio_msg->sequnce_id_ = ++last_frame_id_;
    return sio_msg;
  } while (0);

  if (buf_allocated) {
    FreeBuffer();
  }

  return nullptr;
}

bool SensorBase::AllocBuffer() {
  std::lock_guard<std::mutex> lk(sio_buffer_mutex_);
  if (consumed_sio_buffers_ < input_cfg_[sensor_type_]->max_buf_count) {
    ++consumed_sio_buffers_;
    LOGV << "chnl[" << channel_id_
         << "] alloc buffer success, consumed_sio_buffers_: "
         << consumed_sio_buffers_;
    return true;
  }

  LOGV << "chnl[" << channel_id_ << "] no sio_buffer to use.";
  return false;
}

void SensorBase::FreeBuffer() {
  std::lock_guard<std::mutex> lk(sio_buffer_mutex_);
  if (0 >= consumed_sio_buffers_) {
    LOGE << "chnl[" << channel_id_
         << "] should not happen, consumed_sio_buffers: "
         << consumed_sio_buffers_;
    return;
  }
  --consumed_sio_buffers_;
  LOGV << "chnl[" << channel_id_
       << "] free buffer success, consumed_sio_buffers_: "
       << consumed_sio_buffers_;
}

int SensorBase::GetConsumedBufferNum() {
  std::lock_guard<std::mutex> lk(sio_buffer_mutex_);
  return consumed_sio_buffers_;
}

int SensorBase::DumpToFile(const char *filename, char *srcBuf,
                            unsigned int size) {
#if 0
  FILE *yuvFd = fopen(filename, "w+");
  if (yuvFd == NULL) {
    std::cout << "chnl[" << channel_id_ << "] open file: " << filename
              << " failed" << std::endl;
    return -1;
  }

  char *buffer = reinterpret_cast<char *>(malloc(size));

  if (buffer == NULL) {
    std::cout << "chnl[" << channel_id_ << "] malloc failed " << std::endl;
    fclose(yuvFd);
    return -1;
  }

  memcpy(buffer, srcBuf, size);
  fflush(stdout);
  fwrite(buffer, 1, size, yuvFd);
  fflush(yuvFd);

  if (yuvFd) fclose(yuvFd);
  if (buffer) free(buffer);

  LOGI << "chnl[" << channel_id_ << "] filedump: " << filename
       << " size: " << size << " is successed" << std::endl;
#endif
  return 0;
}

}  // namespace J5Sample
