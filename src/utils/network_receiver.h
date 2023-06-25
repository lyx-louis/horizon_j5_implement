/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef _J5_SAMPLE_INCLUDE_MODULES_INPUT_NETWORK_RECEIVER_H_
#define _J5_SAMPLE_INCLUDE_MODULES_INPUT_NETWORK_RECEIVER_H_
#include <zmq.h>
#include <condition_variable>
#include <list>
#include <memory>
#include <string>
#include <vector>

namespace J5Sample {

struct PlaybackRecvFrame {
  PlaybackRecvFrame() { images.clear(); }

  ~PlaybackRecvFrame() {
    for (size_t i = 0; i < images.size(); i++) {
      zmq_msg_close(images[i]);
      delete images[i];
    }
    images.clear();
  }

  std::vector<zmq_msg_t *> images;
};

class NetworkReceiver {
 public:
  NetworkReceiver();

  ~NetworkReceiver();

  bool Init(const char *end_point);

  void sendRsp();

  void Fini();

  PlaybackRecvFrame *RecvFrame();

 private:
  bool inited_;
  void *socket_recv_;
  void *zmq_context_;

  std::string end_point_;
};

}  // namespace J5Sample
#endif  // _J5_SAMPLE_INCLUDE_MODULES_INPUT_NETWORK_RECEIVER_H_
