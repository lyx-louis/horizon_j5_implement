// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.
#include "network_receiver.h"

#include <cstring>
#include <iostream>
#include <thread>

#include "hobotlog/hobotlog.hpp"

#define RECV_QUEUE_SIZE 2
#define RECV_BUF_SIZE (10 * 1024 * 1024)

namespace J5Sample {

NetworkReceiver::NetworkReceiver()
    : socket_recv_(nullptr), zmq_context_(nullptr) {
  inited_ = false;
}

NetworkReceiver::~NetworkReceiver() { Fini(); }

bool NetworkReceiver::Init(const char *end_point) {
  int ret = -1;
  if (inited_) {
    return false;
  }

  end_point_ = end_point;

  zmq_context_ = zmq_ctx_new();

  // Socket to talk to server
  // socket_recv_ = zmq_socket(zmq_context_, ZMQ_PULL);
  socket_recv_ = zmq_socket(zmq_context_, ZMQ_REP);

  int hwm = RECV_QUEUE_SIZE;
  ret = zmq_setsockopt(socket_recv_, ZMQ_RCVHWM, &hwm, sizeof(int));
  if (ret) {
    LOGE << "zmq_setsockopt error, ret: " << ret
         << "system error: " << strerror(errno);
  }

  int recvbuf = RECV_BUF_SIZE;
  ret = zmq_setsockopt(socket_recv_, ZMQ_RCVBUF, &recvbuf, sizeof(int));
  if (ret) {
    LOGE << "zmq_setsockopt error, ret: " << ret
         << "system error: " << strerror(errno);
  }

  int timeout = 1;
  ret = zmq_setsockopt(socket_recv_, ZMQ_RCVTIMEO, &timeout, sizeof(int));
  if (ret) {
    LOGE << "zmq_setsockopt error, ret: " << ret
         << "system error: " << strerror(errno);
  }

  ret = zmq_bind(socket_recv_, end_point);
  if (ret) {
    LOGE << "zmq bind failed, ret: " << ret
         << " system_error: " << strerror(errno)
         << " socket_recv: " << socket_recv_ << " end_point: " << end_point;
    return false;
  }

  inited_ = true;
  return true;
}

void NetworkReceiver::Fini() {
  if (!inited_) {
    return;
  }

  zmq_close(socket_recv_);
  zmq_ctx_destroy(zmq_context_);
  inited_ = false;
}

PlaybackRecvFrame *NetworkReceiver::RecvFrame() {
  PlaybackRecvFrame *frame = new PlaybackRecvFrame;

  // int recv_to = 10000;  // 10s
  // // image data
  // zmq_setsockopt(socket_recv_, ZMQ_RCVTIMEO, &recv_to, sizeof(int));

  while (true) {
    auto image_data = new zmq_msg_t;
    zmq_msg_init(image_data);

    int rc = zmq_msg_recv(image_data, socket_recv_, 0);
    if (rc == -1) {
      zmq_msg_close(image_data);
      delete image_data;
      delete frame;
      return nullptr;
    }
#if 0
    auto data = reinterpret_cast<char*>(image_data);
    LOGI << " image data: " << data;
    for (int i = 0 ; i < 32; i++) {
      LOGI << "index: " << i << " data: " << data[i];
    }
#endif
    frame->images.push_back(image_data);
    if (!zmq_msg_more(image_data)) break;
  }

  return frame;
}

void NetworkReceiver::sendRsp() {
  zmq_msg_t msg;
  char done[5] = "done";
  zmq_msg_init_data(&msg, done, 4, 0, nullptr);
  zmq_msg_send(&msg, socket_recv_, 0);
}

}  // namespace J5Sample
