/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "modules/video_input/input_module.h"

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

#define DOWN_SCALE_MAIN_MAX 5
#define DOWN_SCALE_ROI_MAX 6
#define UP_SCALE_MAIN_MAX 1

int InputModule::LoadConfigFile(
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

int InputModule::Init() {
  int ret = -1;

  LOGD << "chnl[" << channel_id_ << "] Enter Init.";
  ret = this->ModuleInit();
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] init failed, ret: " << ret;
  LOGD << "chnl[" << channel_id_ << "]  Init Suc.";
  return 0;
}

void InputModule::Reset() {
  int ret = -1;
  int wait_count = 100;

  while (GetConsumedBufferNum() > 0 && wait_count-- > 0) {
    // LOGD << "Wait vio buffer free.";
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  ret = this->ModuleDeInit();
  HOBOT_CHECK(ret == 0) << "chnl[" << channel_id_
                        << "] deinit failed, ret: " << ret;
}

std::shared_ptr<VioMessage> InputModule::GetVioMessage() {
  ModuleProfile profile("InputModule");
  // LOGD << "chnl[" << channel_id_ << "] Enter forword0";
  bool buf_allocated(false);
  do {
    std::shared_ptr<PyramidFrame> pym_image = nullptr;
    buf_allocated = AllocBuffer();
    if (!buf_allocated) break;
    if (!this->HasNext()) break;
    if (this->Next(pym_image) != 0) break;
    if (pym_image == nullptr) break;
    if (pym_image->frame_id_ % sample_freq_ != 0) {
      Free(pym_image);
      break;
    }

    std::shared_ptr<VioMessage> vio_msg(
        new ImageVioMessage(pym_image), [&, this](ImageVioMessage *p) {
          if (p) {
            LOGI << "chnl[" << channel_id_
                 << "] begin delete VioMessage, frame_id: " << p->frame_id_;
            this->Free(p->pym_image_);
            this->FreeBuffer();
            delete (p);
          }
          p = nullptr;
        });
    if (vio_msg == nullptr || 0 != pym_image->frame_id_ % sample_freq_) {
      vio_msg = nullptr;
      break;
    }

    profile.EndRecord();
    LOGD << profile;
    LOGD << "chnl[" << channel_id_ << "] Forward return";
    static std::atomic_uint64_t last_frame_id_{0};
    vio_msg->pym_image_->frame_id_ = ++last_frame_id_;
    return vio_msg;
  } while (0);

  if (buf_allocated) {
    FreeBuffer();
  }
  return nullptr;
}

bool InputModule::AllocBuffer() {
  std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
  // LOGD << "Enter AllocBuffer(), consumed_vio_buffers: "
  //   << consumed_vio_buffers_;
  if (consumed_vio_buffers_ < input_cfg_->max_vio_buffer) {
    ++consumed_vio_buffers_;
    LOGV << "chnl[" << channel_id_
         << "] alloc buffer success, consumed_vio_buffers_: "
         << consumed_vio_buffers_;
    return true;
  }

  LOGV << "chnl[" << channel_id_ << "] no vio_buffer to use.";
  return false;
}

void InputModule::FreeBuffer() {
  std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
  // LOGD << "Enter FreeBuffer(), consumed_vio_buffers: "
  //   << consumed_vio_buffers_;
  if (0 >= consumed_vio_buffers_) {
    LOGE << "chnl[" << channel_id_
         << "] should not happen, consumed_vio_buffers: "
         << consumed_vio_buffers_;
    return;
  }
  --consumed_vio_buffers_;
  LOGV << "chnl[" << channel_id_
       << "] free buffer success, consumed_vio_buffers_: "
       << consumed_vio_buffers_;
}

int InputModule::GetConsumedBufferNum() {
  std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
  return consumed_vio_buffers_;
}

void InputModule::GetDumpNum(const std::string &input, bool &dump_en,
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

int InputModule::GetPyramidFrame(void *src_buf,
                                 std::shared_ptr<PyramidFrame> &output_image) {
  std::shared_ptr<PyramidFrame> output = nullptr;

  void *pym_buf =
      reinterpret_cast<void *>(std::calloc(1, sizeof(pym_buffer_v2_t)));
  if (nullptr == pym_buf) {
    LOGE << "chnl[" << channel_id_ << "] std::calloc pym buffer failed";
    return -1;
  }

  int ret = hb_vio_get_data(channel_id_, HB_VIO_PYM_DATA_V2, pym_buf);
  if (ret) {
    LOGE << "chnl[" << channel_id_ << "] hb_vio_get_data failed, ret: " << ret;
    return ret;
  }
  LOGD << "chnl[" << channel_id_
       << "] get pym buffer success, channel_id: " << channel_id_
       << " pym_buf: " << pym_buf;

  {
    /* dump pym result */
    bool dump_en = false;
    int dump_num = 0;
    GetDumpNum("./vps_output.txt", dump_en, dump_num);
    if (dump_en == true && (dump_count_ < dump_num || dump_num < 0)) {
      DumpPymBuf(channel_id_, pym_id_, pym_buf, dump_count_);
      dump_count_++;
    }
  }

  output = std::make_shared<PyramidFrame>();
  ret = ConvertPymInfo(src_buf, pym_buf, output);
  if (ret) {
    LOGE << "chnl[" << channel_id_ << "] convert pym info failed, ret: " << ret;
    return ret;
  }
  output->frame_id_ = NextFrameId();
  output_image = output;
  return 0;
}

int InputModule::FreePyramidFrame(std::shared_ptr<PyramidFrame> &input_image) {
  void *src_buf = input_image->src_context_;
  if (src_buf) {
    // feedback mode
    int ret =
        hb_vio_free_pymbuf(channel_id_, HB_VIO_PYM_FEEDBACK_SRC_DATA, src_buf);
    if (ret) {
      LOGE << "chnl[" << channel_id_
           << "] hb_vio_free_pymbuf src data failed, ret: " << ret;
      return ret;
    }
    std::free(src_buf);
  }

  void *pym_buf = input_image->pym_context_;
  if (pym_buf == nullptr) {
    LOGE << "chnl[" << channel_id_ << "] pym addr is nullptr";
    return -1;
  }
  int ret = hb_vio_free_pymbuf(channel_id_, HB_VIO_PYM_DATA_V2, pym_buf);
  if (ret) {
    LOGE << "chnl[" << channel_id_
         << "] hb_vio_free_pymbuf pym data failed, ret: " << ret;
    return ret;
  }
  std::free(pym_buf);
  LOGD << "chnl[" << channel_id_
       << "] free pym buffer success, pym_buf: " << pym_buf;
  input_image->pym_context_ = nullptr;
  input_image = nullptr;
  return 0;
}

int InputModule::ConvertPymInfo(void *src_buf, void *pym_buf,
                                std::shared_ptr<PyramidFrame> pym_img) {
  if (nullptr == pym_buf) {
    LOGE << "chnl[" << channel_id_ << "] input pyramid buffer is nullptr";
    return -1;
  }
  if (pym_img == nullptr) {
    pym_img = std::make_shared<PyramidFrame>();
  }

  auto pym_addr = static_cast<pym_buffer_v2_t *>(pym_buf);
  pym_img->src_context_ = src_buf;
  pym_img->pym_context_ = pym_buf;
  pym_img->bl_ds_.resize(DOWN_SCALE_MAIN_MAX);
  pym_img->gs_ds_.resize(DOWN_SCALE_MAIN_MAX);
  pym_img->roi_ds_.resize(DOWN_SCALE_ROI_MAX);
  pym_img->roi_us_.resize(UP_SCALE_MAIN_MAX);
  pym_img->channel_id_ = channel_id_;
  pym_img->frame_id_ = pym_addr->pym_img_info.frame_id;
  pym_img->time_stamp_ = pym_addr->pym_img_info.time_stamp;
  LOGI << "chnl[" << channel_id_
       << "] j5 vio message, channel_id: " << pym_img->channel_id_
       << " frame_id: " << pym_img->frame_id_
       << " time_stamp: " << pym_img->time_stamp_;

  /* 1. src out */
  {
    pym_img->src_info_.width = pym_addr->src_out.width;
    pym_img->src_info_.height = pym_addr->src_out.height;
    pym_img->src_info_.stride = pym_addr->src_out.stride_size;
    pym_img->src_info_.y_paddr = pym_addr->src_out.paddr[0];
    pym_img->src_info_.c_paddr = pym_addr->src_out.paddr[1];
    pym_img->src_info_.y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->src_out.addr[0]);
    pym_img->src_info_.c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->src_out.addr[1]);
    LOGD << "chnl[" << channel_id_ << "] j5 channel_id:" << pym_img->channel_id_
         << " src layer index: -1 width:" << pym_img->src_info_.width
         << " height:" << pym_img->src_info_.height
         << " stride:" << pym_img->src_info_.stride
         << " y_paddr:" << pym_img->src_info_.y_paddr
         << " c_paddr:" << pym_img->src_info_.c_paddr
         << " y_vaddr:" << pym_img->src_info_.y_vaddr
         << " c_vaddr:" << pym_img->src_info_.c_vaddr;
  }

  /* 2. bl downscale pym convert */
  for (int i = 0; i < DOWN_SCALE_MAIN_MAX; ++i) {
    pym_img->bl_ds_[i].width = pym_addr->bl[i].width;
    pym_img->bl_ds_[i].height = pym_addr->bl[i].height;
    pym_img->bl_ds_[i].stride = pym_addr->bl[i].stride_size;
    pym_img->bl_ds_[i].y_paddr = pym_addr->bl[i].paddr[0];
    pym_img->bl_ds_[i].c_paddr = pym_addr->bl[i].paddr[1];
    pym_img->bl_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->bl[i].addr[0]);
    pym_img->bl_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->bl[i].addr[1]);

    LOGD << "chnl[" << channel_id_ << "] j5 channel_id:" << pym_img->channel_id_
         << " bl_ds layer index:" << i << " width:" << pym_img->bl_ds_[i].width
         << " height:" << pym_img->bl_ds_[i].height
         << " stride:" << pym_img->bl_ds_[i].stride
         << " y_paddr:" << pym_img->bl_ds_[i].y_paddr
         << " c_paddr:" << pym_img->bl_ds_[i].c_paddr
         << " y_vaddr:" << pym_img->bl_ds_[i].y_vaddr
         << " c_vaddr:" << pym_img->bl_ds_[i].c_vaddr;
  }

  /* 3. gs downscale pym convert */
  for (int i = 0; i < DOWN_SCALE_MAIN_MAX; ++i) {
    pym_img->gs_ds_[i].width = pym_addr->gs[i].width;
    pym_img->gs_ds_[i].height = pym_addr->gs[i].height;
    pym_img->gs_ds_[i].stride = pym_addr->gs[i].stride_size;
    pym_img->gs_ds_[i].y_paddr = pym_addr->gs[i].paddr[0];
    pym_img->gs_ds_[i].c_paddr = pym_addr->gs[i].paddr[1];
    pym_img->gs_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->gs[i].addr[0]);
    pym_img->gs_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->gs[i].addr[1]);

    LOGD << "chnl[" << channel_id_ << "] j5 channel_id:" << pym_img->channel_id_
         << " gs_ds layer index:" << i << " width:" << pym_img->gs_ds_[i].width
         << " height:" << pym_img->gs_ds_[i].height
         << " stride:" << pym_img->gs_ds_[i].stride
         << " y_paddr:" << pym_img->gs_ds_[i].y_paddr
         << " c_paddr:" << pym_img->gs_ds_[i].c_paddr
         << " y_vaddr:" << pym_img->gs_ds_[i].y_vaddr
         << " c_vaddr:" << pym_img->gs_ds_[i].c_vaddr;
  }

  /* 4. roi downscale convert */
  for (int i = 0; i < DOWN_SCALE_ROI_MAX; ++i) {
    pym_img->roi_ds_[i].width = pym_addr->ds_roi[i].width;
    pym_img->roi_ds_[i].height = pym_addr->ds_roi[i].height;
    pym_img->roi_ds_[i].stride = pym_addr->ds_roi[i].stride_size;
    pym_img->roi_ds_[i].y_paddr = pym_addr->ds_roi[i].paddr[0];
    pym_img->roi_ds_[i].c_paddr = pym_addr->ds_roi[i].paddr[1];
    pym_img->roi_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->ds_roi[i].addr[0]);
    pym_img->roi_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->ds_roi[i].addr[1]);

    LOGD << "chnl[" << channel_id_ << "] j5 channel_id:" << pym_img->channel_id_
         << " roi_ds layer index:" << i
         << " width:" << pym_img->roi_ds_[i].width
         << " height:" << pym_img->roi_ds_[i].height
         << " stride:" << pym_img->roi_ds_[i].stride
         << " y_paddr:" << pym_img->roi_ds_[i].y_paddr
         << " c_paddr:" << pym_img->roi_ds_[i].c_paddr
         << " y_vaddr:" << pym_img->roi_ds_[i].y_vaddr
         << " c_vaddr:" << pym_img->roi_ds_[i].c_vaddr;
  }

  /* 5. roi upscale convert */
  for (int i = 0; i < UP_SCALE_MAIN_MAX; ++i) {
    pym_img->roi_us_[i].width = pym_addr->us_roi.width;
    pym_img->roi_us_[i].height = pym_addr->us_roi.height;
    pym_img->roi_us_[i].stride = pym_addr->us_roi.stride_size;
    pym_img->roi_us_[i].y_paddr = pym_addr->us_roi.paddr[0];
    pym_img->roi_us_[i].c_paddr = pym_addr->us_roi.paddr[1];
    pym_img->roi_us_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->us_roi.addr[0]);
    pym_img->roi_us_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->us_roi.addr[1]);

    LOGD << "chnl[" << channel_id_ << "] j5 channel_id:" << pym_img->channel_id_
         << " roi_us layer index:" << i
         << " width:" << pym_img->roi_us_[i].width
         << " height:" << pym_img->roi_us_[i].height
         << " stride:" << pym_img->roi_us_[i].stride
         << " y_paddr:" << pym_img->roi_us_[i].y_paddr
         << " c_paddr:" << pym_img->roi_us_[i].c_paddr
         << " y_vaddr:" << pym_img->roi_us_[i].y_vaddr
         << " c_vaddr:" << pym_img->roi_us_[i].c_vaddr;
  }
  return 0;
}

int InputModule::DumpToFile2Plane(const char *filename, char *srcBuf,
                                  char *srcBuf1, unsigned int size,
                                  unsigned int size1) {
  FILE *yuvFd = fopen(filename, "w+");
  if (yuvFd == NULL) {
    std::cout << "chnl[" << channel_id_ << "] open file: " << filename
              << " failed" << std::endl;
    return -1;
  }

  char *buffer = reinterpret_cast<char *>(malloc(size + size1));
  if (buffer == NULL) {
    std::cout << "chnl[" << channel_id_ << "] malloc failed " << std::endl;
    fclose(yuvFd);
    return -1;
  }

  memcpy(buffer, srcBuf, size);
  memcpy(buffer + size, srcBuf1, size1);

  fflush(stdout);

  fwrite(buffer, 1, size + size1, yuvFd);

  fflush(yuvFd);

  if (yuvFd) fclose(yuvFd);
  if (buffer) free(buffer);

  std::cout << "chnl[" << channel_id_ << "] filedump: " << filename
            << " size: " << size << " is successed" << std::endl;

  return 0;
}

int InputModule::DumpToFile(const char *filename, char *srcBuf,
                            unsigned int size) {
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
  return 0;
}

void InputModule::DumpPymBuf(uint32_t pipe_id, uint32_t pym_id,
                             void *pym_buffer, int count) {
  pym_buffer_v2_t *pym_buf = reinterpret_cast<pym_buffer_v2_t *>(pym_buffer);
  if (pym_buf == NULL) {
    std::cout << "chnl[" << channel_id_ << "] dump_pym_buf buf null."
              << std::endl;
    return;
  }
  char file_name[100];
  int size;
  int32_t loop_count = count;
  snprintf(file_name, sizeof(file_name), "s%u_pym%u_src_%dx%d_out_%d.yuv",
           pipe_id, pym_id, pym_buf->src_out.stride_size,
           pym_buf->src_out.height, loop_count);
  LOGI << "chnl[" << channel_id_
       << "] src width:" << pym_buf->src_out.stride_size
       << " height:" << pym_buf->src_out.height << " file_name:" << file_name;

  size = pym_buf->src_out.height * pym_buf->src_out.stride_size;
  if (pym_buf->src_out.addr[0] != NULL) {
    if (pym_buf->src_out.addr[1] != NULL) {
      DumpToFile2Plane(file_name, pym_buf->src_out.addr[0],
                       pym_buf->src_out.addr[1], size, size / 2);
    } else {
      DumpToFile(file_name, pym_buf->src_out.addr[0], size);
    }
  }

  snprintf(file_name, sizeof(file_name), "s%u_pym%u_us_%dx%d_roi_%d.yuv",
           pipe_id, pym_id, pym_buf->us_roi.stride_size, pym_buf->us_roi.height,
           loop_count);

  size = pym_buf->us_roi.height * pym_buf->us_roi.stride_size;
  if (pym_buf->us_roi.addr[0] != NULL) {
    if (pym_buf->us_roi.addr[1] != NULL) {
      DumpToFile2Plane(file_name, pym_buf->us_roi.addr[0],
                       pym_buf->us_roi.addr[1], size, size / 2);
    } else {
      DumpToFile(file_name, pym_buf->us_roi.addr[0], size);
    }
    LOGI << "chnl[" << channel_id_
         << "] us width:" << pym_buf->src_out.stride_size
         << " height:" << pym_buf->src_out.height << " file_name:" << file_name;
  }

  for (int32_t i = 0; i < 6; i++) {
    snprintf(file_name, sizeof(file_name), "s%u_pym%u_ds%d_%dx%d_roi_%d.yuv",
             pipe_id, pym_id, i, pym_buf->ds_roi[i].stride_size,
             pym_buf->ds_roi[i].height, loop_count);
    size = pym_buf->ds_roi[i].height * pym_buf->ds_roi[i].stride_size;
    if (pym_buf->ds_roi[i].addr[0] != NULL) {
      if (pym_buf->ds_roi[i].addr[1] != NULL) {
        DumpToFile2Plane(file_name, pym_buf->ds_roi[i].addr[0],
                         pym_buf->ds_roi[i].addr[1], size, size / 2);
      } else {
        DumpToFile(file_name, pym_buf->ds_roi[i].addr[0], size);
      }
      LOGI << "chnl[" << channel_id_ << "] ds: " << i
           << " width:" << pym_buf->src_out.stride_size
           << " height:" << pym_buf->src_out.height
           << " file_name:" << file_name;
    }
  }

  for (int32_t i = 0; i < 5; i++) {
    snprintf(file_name, sizeof(file_name), "s%u_pym%u_gs%d_%dx%d_%d.yuv",
             pipe_id, pym_id, i, pym_buf->gs[i].stride_size,
             pym_buf->gs[i].height, loop_count);
    size = pym_buf->gs[i].height * pym_buf->gs[i].stride_size;
    if (pym_buf->gs[i].addr[0] != NULL) {
      if (pym_buf->gs[i].addr[1] != NULL) {
        DumpToFile2Plane(file_name, pym_buf->gs[i].addr[0],
                         pym_buf->gs[i].addr[1], size, size / 2);
      } else {
        DumpToFile(file_name, pym_buf->gs[i].addr[0], size);
      }
      LOGI << "chnl[" << channel_id_ << "] gs: " << i
           << " width:" << pym_buf->src_out.stride_size
           << " height:" << pym_buf->src_out.height
           << " file_name:" << file_name;
    }
  }

  for (int32_t i = 0; i < 5; i++) {
    snprintf(file_name, sizeof(file_name), "s%u_pym%u_bl%d_%dx%d_%d.yuv",
             pipe_id, pym_id, i, pym_buf->bl[i].stride_size,
             pym_buf->bl[i].height, loop_count);
    size = pym_buf->bl[i].height * pym_buf->bl[i].stride_size;
    if (pym_buf->bl[i].addr[0] != NULL) {
      if (pym_buf->bl[i].addr[1] != NULL) {
        DumpToFile2Plane(file_name, pym_buf->bl[i].addr[0],
                         pym_buf->bl[i].addr[1], size, size / 2);
      } else {
        DumpToFile(file_name, pym_buf->bl[i].addr[0], size);
      }
      LOGI << "chnl[" << channel_id_ << "] gs: " << i
           << " width:" << pym_buf->src_out.stride_size
           << " height:" << pym_buf->src_out.height
           << " file_name:" << file_name;
    }
  }
}

}  // namespace J5Sample
