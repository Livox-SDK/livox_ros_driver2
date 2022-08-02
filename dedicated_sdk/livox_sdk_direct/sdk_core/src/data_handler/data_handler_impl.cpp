//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "data_handler_impl.h"
#include <base/logging.h>
#include <mutex>
#include "base/network/network_util.h"

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <unistd.h>
#endif // WIN32

using std::lock_guard;
using std::mutex;
using std::shared_ptr;

namespace livox {

namespace direct {

DataHandlerImpl::DataHandlerImpl(DataHandler* handler) : handler_(handler) {}

bool DataHandlerImpl::Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr) {
  if (direct_host_ipinfo_ptr == nullptr && direct_lidars_info_ptr == nullptr) {
    LOG_ERROR("Data handler impl init failed, pointers to input parameters are null");
    return false;
  }
  
  direct_host_ipinfo_ptr_ = direct_host_ipinfo_ptr;
  direct_lidars_info_ptr_ = direct_lidars_info_ptr;

  if (!CreateIOThread()) {
    return false;
  }

  if (!CreateDataChannel()) {
    return false;
  }
  LOG_INFO("Data Handler Impl Init Succ.");
  return true;
}

bool DataHandlerImpl::Start() {
  return thread_->Start();
}

bool DataHandlerImpl::CreateIOThread() {
  thread_ = std::make_shared<IOThread>();
  if (thread_ == nullptr || !(thread_->Init(false, false))) {
    LOG_ERROR("Create data io thread failed or thread init failed.");
    return false;
  }
  return true;
}

bool DataHandlerImpl::CreateDataChannel() {
  if (direct_host_ipinfo_ptr_ == nullptr && direct_lidars_info_ptr_ == nullptr) {
    LOG_ERROR("Create data channel failed, pointers to input parameters are null.");
    return false;
  }

  if (direct_host_ipinfo_ptr_) {
    if(!CreateSocketAndAddDelegate(direct_host_ipinfo_ptr_->host_point_data_ip, direct_host_ipinfo_ptr_->host_point_data_port)) {
      return false;
    }

    if (!CreateSocketAndAddDelegate(direct_host_ipinfo_ptr_->host_imu_data_ip, direct_host_ipinfo_ptr_->host_imu_data_port)) {
      return false;
    }
    return true;
  } else {
    for (auto it = direct_lidars_info_ptr_->begin(); it != direct_lidars_info_ptr_->end(); ++it) {
      DirectLidarInfo& direct_lidar_info = *it;
      if (!CreateSocketAndAddDelegate(direct_lidar_info.host_ipinfo_ptr->host_point_data_ip,
          direct_lidar_info.host_ipinfo_ptr->host_point_data_port)) {
        return false;
      }
      if (!CreateSocketAndAddDelegate(direct_lidar_info.host_ipinfo_ptr->host_imu_data_ip,
          direct_lidar_info.host_ipinfo_ptr->host_imu_data_port)) {
        return false;
      }
    }
  }
  return true;
}

bool DataHandlerImpl::CreateSocketAndAddDelegate(const std::string& host_ip, const uint16_t port) {
  if (host_ip.empty() || port == 0) {
    return false;
  }

  std::string key = host_ip + ":" + std::to_string(port);
  if (data_channel_.find(key) != data_channel_.end()) {
    return true;
  }

  socket_t sock = util::CreateSocket(port, true, true, true, host_ip);
  if (sock < 0) {
    LOG_ERROR("Add data channel faileld, can not create socket, the ip {} port {} ", host_ip.c_str(), port);
    return false;
  }
  thread_->loop().lock()->AddDelegate(sock, this, nullptr);
  data_channel_[key] = sock;

  return true;
}

void DataHandlerImpl::OnData(socket_t sock, void *client_data) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);

  std::unique_ptr<char[]> &buf = data_buffers_;
  if (buf.get() == NULL) {
    buf.reset(new char[kMaxBufferSize]);
  }

  int size = kMaxBufferSize;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(buf.get()), kMaxBufferSize, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }

  // std::string device_ip = std::string(inet_ntoa(((struct sockaddr_in *)&addr)->sin_addr));
  // uint32_t s_addr = inet_addr("255.255.255.255");
  uint32_t handle = ((struct sockaddr_in *)&addr)->sin_addr.s_addr;

  if (handler_) {
      handler_->OnDataCallback(handle, buf.get(), size);
  }
}

DataHandlerImpl::~DataHandlerImpl() {
  for (auto it = data_channel_.begin(); it != data_channel_.end(); ++it) {
    socket_t& sock = it->second;
    if (sock > 0) {
      thread_->loop().lock()->RemoveDelegate(sock, this);
      util::CloseSock(sock);
    }
  }

  data_channel_.clear();

  if (thread_) {
    thread_->Quit();
    thread_->Join();
    thread_->Uninit();
  }
}

}  // namespace livox
} //namespace direct 
