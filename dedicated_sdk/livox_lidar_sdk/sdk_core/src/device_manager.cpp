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

#include "device_manager.h"

#include "comm/define.h"
#include "comm/generate_seq.h"
#include "base/logging.h"

#include "command_handler/general_command_handler.h"
#include "data_handler/data_handler.h"

namespace livox {
namespace lidar {

DeviceManager::DeviceManager()  {
}

DeviceManager& DeviceManager::GetInstance() {
  static DeviceManager device_manager;
  return device_manager;
}

bool DeviceManager::Init(const std::string& host_ip) {
  is_view_ = true;
  host_ip_ = host_ip;
  comm_port_.reset(new CommPort());

  if (!GeneralCommandHandler::GetInstance().Init(host_ip ,is_view_, this)) {
    LOG_ERROR("General command handle init failed.");
    return false;
  }

  if (!DataHandler::GetInstance().Init()) {
    LOG_ERROR("Data handle init failed.");
    return false;
  }

  if (!CreateIOThread()) {
    LOG_ERROR("Create IO thread failed.");
    return false;
  }

  if (!CreateDetectionChannel()) {
    LOG_ERROR("Create detection channel failed.");
    return false;
  }

  detection_thread_ = std::make_shared<std::thread>(&DeviceManager::DetectionLidars, this);
  is_stop_detection_.store(false);
  return true;
}

bool DeviceManager::Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {
  lidars_cfg_ptr_ = lidars_cfg_ptr;
  custom_lidars_cfg_ptr_ = custom_lidars_cfg_ptr;
  if (lidars_cfg_ptr_ && !(lidars_cfg_ptr_->empty())) {
    host_ip_ = lidars_cfg_ptr_->at(0).host_net_info.cmd_data_ip;
  } else if (custom_lidars_cfg_ptr && !(custom_lidars_cfg_ptr->empty())) {
    host_ip_ = custom_lidars_cfg_ptr->at(0).host_net_info.cmd_data_ip;
  } else {
    LOG_ERROR("Device manager init failed, can not find cmd host ip.");
    return false;
  }
  comm_port_.reset(new CommPort());

  if (!GeneralCommandHandler::GetInstance().Init(lidars_cfg_ptr, custom_lidars_cfg_ptr, this)) {
    LOG_ERROR("General command handle init failed.");
    return false;
  }

  if (!DataHandler::GetInstance().Init()) {
    LOG_ERROR("Data handle init failed.");
    return false;
  }

  GenerateDevTypeTable();

  if (!CreateIOThread()) {
    LOG_ERROR("Create IO thread failed.");
    return false;
  }

  if (!CreateChannel()) {
    LOG_ERROR("Create channel failed.");
    return false;
  }

  if (!(lidars_cfg_ptr_->empty())) {
    detection_thread_ = std::make_shared<std::thread>(&DeviceManager::DetectionLidars, this);
    is_stop_detection_.store(false);
  }

  return true;
}

void DeviceManager::GenerateDevTypeTable() {
  std::map<uint8_t, uint16_t> dev_type_cmd_port_;
  std::map<uint8_t, uint16_t> dev_type_data_port_;
  for (auto it = lidars_cfg_ptr_->begin(); it != lidars_cfg_ptr_->end(); ++it) {
    const LivoxLidarCfg& lidar_cfg = *it;
    lidars_cmd_port_[lidar_cfg.device_type].insert(lidar_cfg.lidar_net_info.cmd_data_port);
    lidars_cmd_port_[lidar_cfg.device_type].insert(lidar_cfg.lidar_net_info.push_msg_port);
    lidars_cmd_port_[lidar_cfg.device_type].insert(lidar_cfg.lidar_net_info.log_data_port);

    lidars_data_port_[lidar_cfg.device_type].insert(lidar_cfg.lidar_net_info.point_data_port);
    lidars_data_port_[lidar_cfg.device_type].insert(lidar_cfg.lidar_net_info.imu_data_port);
  }

  for (auto it = custom_lidars_cfg_ptr_->begin(); it != custom_lidars_cfg_ptr_->end(); ++it) {
    const LivoxLidarCfg& lidar_cfg = *it;
    uint32_t lidar_ip = inet_addr(lidar_cfg.lidar_net_info.lidar_ipaddr.c_str());
    custom_lidars_cfg_map_[lidar_ip] = lidar_cfg;
  }
}


bool DeviceManager::CreateIOThread() {
  if (!CreateDetectionIOThread()) {
    LOG_ERROR("Device manager init failed, create detection io thread failed.");
    return false;
  }

  if (!CreateCommandIOThread()) {
    LOG_ERROR("Device manager init failed, create comman io thread failed.");
    return false;
  }
  
  if (!CreateDataIOThread()) {
    LOG_ERROR("Device manager init failed, create data io thread failed.");
    return false;
  }
  return true;
}

bool DeviceManager::CreateDetectionIOThread() {
  detection_io_thread_ = std::make_shared<IOThread>();
  if (detection_io_thread_ == nullptr || !(detection_io_thread_->Init(true, false))) {
    LOG_ERROR("Create command io thread failed, thread_ptr is nullptr or thread init failed");
    return false;
  }
  return detection_io_thread_->Start();
}

bool DeviceManager::CreateCommandIOThread() {
  cmd_io_thread_ = std::make_shared<IOThread>();
  if (cmd_io_thread_ == nullptr || !(cmd_io_thread_->Init(true, false))) {
    LOG_ERROR("Create command io thread failed, thread_ptr is nullptr or thread init failed");
    return false;
  }
  return cmd_io_thread_->Start();
}

bool DeviceManager::CreateDataIOThread() {
  data_io_thread_ = std::make_shared<IOThread>();
  if (data_io_thread_ == nullptr || !(data_io_thread_->Init(true, false))) {
    LOG_ERROR("Create command io thread failed, thread_ptr is nullptr or thread init failed");
    return false;
  }
  return data_io_thread_->Start();
}

bool DeviceManager::CreateChannel() {
  if (!CreateDetectionChannel()) {
    LOG_ERROR("Create detection channel failed.");
    return false;
  }

  for (auto it = lidars_cfg_ptr_->begin(); it != lidars_cfg_ptr_->end(); ++it) {
    const HostNetInfo& host_net_info = it->host_net_info;
    if (!CreateDataChannel(host_net_info)) {
      LOG_ERROR("Create data channel failed.");
      return false;
    }

    if (!CreateCommandChannel(it->device_type, host_net_info, false)) {
      LOG_ERROR("Create command channel failed.");
      return false;
    }
  }

  for (auto it = custom_lidars_cfg_ptr_->begin(); it != custom_lidars_cfg_ptr_->end(); ++it) {
    const HostNetInfo& host_net_info = it->host_net_info;
    if (!CreateDataChannel(host_net_info)) {
      LOG_ERROR("Create data channel failed.");
      return false;
    }

    if (!CreateCommandChannel(it->device_type, host_net_info, true)) {
      LOG_ERROR("Create command channel failed.");
      return false;
    }
  }
  return true;
}

bool DeviceManager::CreateDetectionChannel() {
#ifdef WIN32
#else
  detection_broadcast_socket_ = util::CreateSocket(kDetectionPort, true, true, true, "255.255.255.255");
  LOG_INFO("Create detection channel detection socket:{}", detection_broadcast_socket_);
  if (detection_broadcast_socket_ < 0) {
    LOG_ERROR("Create detection broadcast socket failed.");
    return false;
  }
  detection_io_thread_->loop().lock()->AddDelegate(detection_broadcast_socket_, this, nullptr);
#endif

  detection_socket_ = util::CreateSocket(kDetectionPort, true, true, true, host_ip_);
  LOG_INFO("Create detection channel detection socket:{}", detection_broadcast_socket_);
  if (detection_socket_ < 0) {
    LOG_ERROR("Create detection socket failed.");
    return false;
  }
  detection_io_thread_->loop().lock()->AddDelegate(detection_socket_, this, nullptr);

  std::string key = host_ip_ + ":" + std::to_string(kDetectionListenPort);
  channel_info_[key] = detection_socket_;
  if (custom_command_channel_.find(key) == custom_command_channel_.end()) {
    custom_command_channel_[key] = detection_socket_;
  }
  
  return true;
}

bool DeviceManager::CreateDataChannel(const HostNetInfo& host_net_info) {
  if (!CreateDataSocketAndAddDelegate(host_net_info.point_data_ip, host_net_info.point_data_port)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }

  if (!CreateDataSocketAndAddDelegate(host_net_info.imu_data_ip, host_net_info.imu_data_port)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }
  return true;
}

bool DeviceManager::CreateCommandChannel(const uint8_t dev_type, const HostNetInfo& host_net_info, bool is_custom) {
  if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.cmd_data_ip, host_net_info.cmd_data_port, is_custom)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }

  if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.push_msg_ip, host_net_info.push_msg_port, is_custom)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }

  if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.log_data_ip, host_net_info.log_data_port, is_custom)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }
  return true;
}

bool DeviceManager::CreateCmdSocketAndAddDelegate(const uint8_t dev_type, const std::string& host_ip, const uint16_t port, bool is_custom) {
  if (host_ip.empty() || port == 0 || port == kLogPort) {
    return true;
  }

  std::string key = host_ip + ":" + std::to_string(port);
  if (port == kDetectionListenPort) {
    if (is_custom) {
      if (custom_command_channel_.find(key) == custom_command_channel_.end()) {
        custom_command_channel_[key] = detection_socket_;
      }
    } else {
      if (general_command_channel_.find(dev_type) == general_command_channel_.end()) {
        general_command_channel_[dev_type] = detection_socket_;
      }
    }
    return true;
  }

  if (channel_info_.find(key) != channel_info_.end()) {
    if (is_custom) {
      if (custom_command_channel_.find(key) == custom_command_channel_.end()) {
        custom_command_channel_[key] = channel_info_[key];
      }
    } else {
      if (general_command_channel_.find(dev_type) == general_command_channel_.end()) {
        general_command_channel_[dev_type] = channel_info_[key];
      }
    }
    return true;
  }

  socket_t sock = -1;
  if (host_ip == "local") {
    sock = util::CreateSocket(port, true, true, true, "");
  } else {
    sock = util::CreateSocket(port, true, true, true, host_ip);
  }
  if (sock < 0) {
    LOG_ERROR("Add command channel faileld, can not create socket, the ip {} port {} ", host_ip.c_str(), port);
    return false;
  }
  socket_vec_.push_back(sock);
  channel_info_[key] = sock;
  command_channel_.insert(sock);

  
  if (is_custom) {    
    custom_command_channel_[key] = sock;
  } else {
    general_command_channel_[dev_type] = sock;
  }

  cmd_io_thread_->loop().lock()->AddDelegate(sock, this, nullptr);
  return true;
}

bool DeviceManager::CreateDataSocketAndAddDelegate(const std::string& host_ip, const uint16_t port) {
  if (host_ip.empty() || port == 0 || port == kLogPort || port == kDetectionPort) {
    return true;
  }

  std::string key = host_ip + ":" + std::to_string(port);
  if (channel_info_.find(key) != channel_info_.end()) {
    return true;
  }

  socket_t sock = -1;
  if (host_ip == "local") {
    sock = util::CreateSocket(port, true, true, true, "");
  } else {
    sock = util::CreateSocket(port, true, true, true, host_ip);
  }
  if (sock < 0) {
    LOG_ERROR("Add command channel faileld, can not create socket, the ip {} port {} ", host_ip.c_str(), port);
    return false;
  }
  socket_vec_.push_back(sock);
  channel_info_[key] = sock;  

  data_channel_.insert(sock);
  data_io_thread_->loop().lock()->AddDelegate(sock, this, nullptr);
  return true;
}

void DeviceManager::DetectionLidars() {
  while (!is_stop_detection_) {
    Detection();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void DeviceManager::Detection() {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = sizeof(uint8_t);

  CommPacket packet;
  packet.protocol = kLidarSdk;
  packet.version = kSdkVer;
  packet.seq_num = GenerateSeq::GetSeq();
  packet.cmd_id = kCommandIDLidarSearch;
  packet.cmd_type = kCommandTypeCmd;
  packet.sender_type = kHostSend;
  packet.data = req_buff;
  packet.data_len = req_len;

  //LOG_INFO("Detection lidars, the seq:{}", packet.seq_num);

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
  servaddr.sin_port = htons(kDetectionPort);

  int byte_send = sendto(detection_socket_, (const char*)buf.data(), size, 0,
      (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
    LOG_INFO("Detection lidars failed, Send to lidar failed.");
  }
}

void DeviceManager::OnData(socket_t sock, void *client_data) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);

  std::unique_ptr<char[]> buf = nullptr;
  if (buf.get() == NULL) {
    buf.reset(new char[kMaxBufferSize]);
  }

  int size = util::RecvFrom(sock, reinterpret_cast<char *>(buf.get()), kMaxBufferSize, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }

  uint32_t handle = ((struct sockaddr_in *)&addr)->sin_addr.s_addr;
  uint16_t port = ntohs(((struct sockaddr_in *)&addr)->sin_port);

  struct in_addr log_addr;
  log_addr.s_addr = handle;
  std::string lidar_ip = inet_ntoa(log_addr);
  if (lidar_ip == host_ip_) {
    return;
  }

  if (is_view_) {
    std::shared_ptr<ViewLidarIpInfo> view_lidar_info_ptr = nullptr;
    {
      std::lock_guard<std::mutex> lock(view_lidars_info_mutex_);
      if (view_lidars_info_.find(handle) != view_lidars_info_.end()) {
        view_lidar_info_ptr = view_lidars_info_[handle];
      }
    }

    if (view_lidar_info_ptr != nullptr) {
      if (port == view_lidar_info_ptr->lidar_point_port || port == view_lidar_info_ptr->lidar_imu_data_port) {
        DataHandler::GetInstance().Handle(view_lidar_info_ptr->dev_type, handle, (uint8_t*)(buf.get()), size);
      } else {
        GeneralCommandHandler::GetInstance().Handler(view_lidar_info_ptr->dev_type, handle, port, (uint8_t*)(buf.get()), size);
      }
    } else {
      GeneralCommandHandler::GetInstance().Handler(handle, port, (uint8_t*)(buf.get()), size);
    }
    return;
  }

  if (custom_lidars_cfg_map_.find(handle) != custom_lidars_cfg_map_.end()) {
    const LivoxLidarCfg& lidar_cfg = custom_lidars_cfg_map_[handle];
    if (port == lidar_cfg.lidar_net_info.imu_data_port || port == lidar_cfg.lidar_net_info.point_data_port) {
      DataHandler::GetInstance().Handle(lidar_cfg.device_type, handle, (uint8_t*)(buf.get()), size);
      return;
    }
    if (port == lidar_cfg.lidar_net_info.cmd_data_port || port == lidar_cfg.lidar_net_info.push_msg_port ||
        port == lidar_cfg.lidar_net_info.log_data_port) {
      GeneralCommandHandler::GetInstance().Handler(lidar_cfg.device_type, handle, port, (uint8_t*)(buf.get()), size);
      return;
    }
  }

  uint8_t dev_type = GetDeviceType(handle);
  if (dev_type != 0) {
    if (lidars_data_port_.find(dev_type) != lidars_data_port_.end()) {
      if (lidars_data_port_[dev_type].find(port) != lidars_data_port_[dev_type].end()) {
        DataHandler::GetInstance().Handle(dev_type, handle, (uint8_t*)(buf.get()), size);
        return;
      }
    }

    if (lidars_cmd_port_.find(dev_type) != lidars_cmd_port_.end()) {
      if (lidars_cmd_port_[dev_type].find(port) != lidars_cmd_port_[dev_type].end()) {
        GeneralCommandHandler::GetInstance().Handler(dev_type, handle, port, (uint8_t*)(buf.get()), size);
        return;
      }
    }
  }

  if (port == kDetectionPort) {
    GeneralCommandHandler::GetInstance().Handler(handle, port, (uint8_t*)(buf.get()), size);
    return;
  }
}

void DeviceManager::HandleDetectionData(uint32_t handle, DetectionData* detection_data) {
  if (handle == 0 || detection_data == nullptr) {
    return;
  }

  if (is_view_) {
    {
      std::lock_guard<std::mutex> lock(view_device_mutex_);
      if (view_devices_.find(handle) == view_devices_.end()) {
        ViewDevice& view_device = view_devices_[handle];
        view_device.handle = handle;
        view_device.dev_type = detection_data->dev_type;
        view_device.cmd_port = detection_data->cmd_port;
        view_device.is_get.store(false);
        view_device.is_set.store(false);
      }
    }

    const ViewDevice& view_device = view_devices_[handle];
    if (!view_device.is_get.load()) {
      GetLivoxLidarInternalInfo(handle);
    } else {
      if (!view_device.is_set.load()) {
        std::shared_ptr<ViewLidarIpInfo> view_lidar_info_ptr = nullptr;
        {
          std::lock_guard<std::mutex> lock(view_lidars_info_mutex_);
          view_lidar_info_ptr = view_lidars_info_[handle];
        }
        if (view_lidar_info_ptr == nullptr) {
          LOG_ERROR("Update view lidar cfg failed, can not find lidar info, the handle:{}", handle);
          return;
        }
        GeneralCommandHandler::GetInstance().UpdateLidarCfg(*view_lidar_info_ptr);
      }
    }
    return;
  }

  std::lock_guard<std::mutex> lock(lidars_dev_type_mutex_);
  if (lidars_dev_type_.find(handle) != lidars_dev_type_.end()) {
    uint16_t dev_type = lidars_dev_type_[handle];
    if (dev_type != detection_data->dev_type) {
      LOG_ERROR("The lidar of handle:{} dev_type is error, the dev_type1:{}, the dev_type2:{}",
          handle, dev_type, detection_data->dev_type);
      return;
    }
    return;
  } 
  
  lidars_dev_type_[handle] = detection_data->dev_type; 
}

void DeviceManager::GetLivoxLidarInternalInfo(const uint32_t handle) {
  GeneralCommandHandler::GetInstance().QueryLivoxLidarInternalInfo(handle, 
      DeviceManager::GetLivoxLidarInternalInfoCallback, this);
}

void DeviceManager::GetLivoxLidarInternalInfoCallback(livox_status status, uint32_t handle,
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (client_data == nullptr) {
    LOG_ERROR("Get livox lidar internal info failed, client data is nullptr.");
    return;
  }

  if (status != kLivoxLidarStatusSuccess) {
    LOG_ERROR("Get livox lidar internal info failed, the status:{}", status);
    return;
  }

  DeviceManager* self = (DeviceManager*)(client_data);
  self->AddViewLidar(handle, response);
}

void DeviceManager::AddViewLidar(const uint32_t handle, LivoxLidarDiagInternalInfoResponse* response) {
  if (response == nullptr) {
    return;
  }
  if (response->ret_code != 0) {
    LOG_ERROR("Get livox lidar internal info failed, the ret_code:{}", response->ret_code);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(view_device_mutex_);
    if (view_devices_.find(handle) == view_devices_.end()) {
      LOG_ERROR("Add view lidar failed, can not get cmd port, the handle:{}", handle);
      return;
    }
  }

  ViewDevice& view_device = view_devices_[handle];
  if (view_device.is_get.load()) {
    return;
  }

  std::shared_ptr<ViewLidarIpInfo> view_lidar_info_ptr(new ViewLidarIpInfo());
  view_lidar_info_ptr->handle = handle;
  view_lidar_info_ptr->dev_type = view_device.dev_type;
  view_lidar_info_ptr->lidar_cmd_port = view_device.cmd_port;
  view_lidar_info_ptr->host_ip = host_ip_;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIPCfg) {
      memcpy(&(view_lidar_info_ptr->host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(view_lidar_info_ptr->lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIPCfg) {
      memcpy(&(view_lidar_info_ptr->host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(view_lidar_info_ptr->lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  LOG_INFO("host_point_port:{}, lidar_point_port:{}, host_imu_data_port:{}, lidar_imu_data_port:{}",
      view_lidar_info_ptr->host_point_port, view_lidar_info_ptr->lidar_point_port,
      view_lidar_info_ptr->host_imu_data_port,  view_lidar_info_ptr->lidar_imu_data_port);

  CreateViewDataChannel(*view_lidar_info_ptr);
  {
    std::lock_guard<std::mutex> lock(view_lidars_info_mutex_);
    view_lidars_info_[handle] = view_lidar_info_ptr;
  }
  view_device.is_get.store(true);
  GeneralCommandHandler::GetInstance().UpdateLidarCfg(*view_lidar_info_ptr);
}

void DeviceManager::CreateViewDataChannel(const ViewLidarIpInfo& view_lidar_info) {
  std::string point_key = view_lidar_info.host_ip + ":" + std::to_string(view_lidar_info.host_point_port);
  if (channel_info_.find(point_key) == channel_info_.end()) {
    socket_t sock = -1;
    sock = util::CreateSocket(view_lidar_info.host_point_port, true, true, true, view_lidar_info.host_ip);
    if (sock < 0) {
      LOG_ERROR("Create View point data channel faileld, can not create socket, the ip {} port {} ",
          view_lidar_info.host_ip.c_str(), view_lidar_info.host_point_port);
      return;
    }
    socket_vec_.push_back(sock);
    channel_info_[point_key] = sock;
    data_io_thread_->loop().lock()->AddDelegate(sock, this, nullptr);
  }

  std::string imu_key = view_lidar_info.host_ip + ":" + std::to_string(view_lidar_info.host_imu_data_port);
  if (channel_info_.find(imu_key) == channel_info_.end()) {
    socket_t sock = -1;
    sock = util::CreateSocket(view_lidar_info.host_imu_data_port, true, true, true, view_lidar_info.host_ip);
    if (sock < 0) {
      LOG_ERROR("Create View point data channel faileld, can not create socket, the ip {} port {} ",
          view_lidar_info.host_ip.c_str(), view_lidar_info.host_imu_data_port);
      return;
    }
    socket_vec_.push_back(sock);
    channel_info_[imu_key] = sock;
    data_io_thread_->loop().lock()->AddDelegate(sock, this, nullptr);
  }
}

void DeviceManager::UpdateViewLidarCfgCallback(const uint32_t handle) {
  if (is_view_) {
    std::lock_guard<std::mutex> lock(view_device_mutex_);
    if (view_devices_.find(handle) == view_devices_.end()) {
      LOG_ERROR("Device manager change livox lidar faield, can not find the view device info, the handle:{}", handle);
      return;
    }
    view_devices_[handle].is_set.store(true);
  }
}

uint8_t DeviceManager::GetDeviceType(const uint32_t handle) {
  uint8_t dev_type = 0;
  std::lock_guard<std::mutex> lock(lidars_dev_type_mutex_);
  if (lidars_dev_type_.find(handle) != lidars_dev_type_.end()) {
    dev_type = lidars_dev_type_[handle];
  }
  return dev_type;
}

void DeviceManager::OnTimer(TimePoint now) {
  GeneralCommandHandler::GetInstance().CommandsHandle(now);
}

int DeviceManager::SendTo(const uint8_t dev_type, const uint32_t handle, const std::vector<uint8_t>& buf, 
    const int16_t size, const struct sockaddr *addr, socklen_t addrlen) {
  socket_t sock = -1;
  if (!GetCmdChannel(dev_type, handle, sock)) {
    sock = detection_socket_;
  }
  std::lock_guard<std::mutex> lock(mutex_cmd_channel_);
  size_t byte =  sendto(sock, (const char*)buf.data(), size, 0, addr, addrlen);    
	return byte;
}

bool DeviceManager::GetCmdChannel(const uint8_t dev_type, const uint32_t handle, socket_t& sock) {
  if (custom_lidars_cfg_map_.find(handle) != custom_lidars_cfg_map_.end()) {
    const LivoxLidarCfg& lidar_cfg = custom_lidars_cfg_map_[handle];
    std::string key = lidar_cfg.host_net_info.cmd_data_ip + ":" + std::to_string(lidar_cfg.host_net_info.cmd_data_port);
    if (custom_command_channel_.find(key) != custom_command_channel_.end()) {
      sock = custom_command_channel_[key];
      return true;
    }
    return false;
  }

  if (general_command_channel_.find(dev_type) != general_command_channel_.end()) {
    if (general_command_channel_.find(dev_type) != general_command_channel_.end()) {
      sock = general_command_channel_[dev_type];
      return true;
    }
    return false;
  }
  return false;
}


void DeviceManager::Destory() {
  host_ip_ = "";

  if (detection_socket_ > 0) {
    detection_io_thread_->loop().lock()->RemoveDelegate(detection_socket_, this);
  }

  if (detection_broadcast_socket_ > 0) {
    detection_io_thread_->loop().lock()->RemoveDelegate(detection_broadcast_socket_, this);
  }

  for (auto it = command_channel_.begin(); it != command_channel_.end(); ++it) {
    socket_t sock = *it;
    if (sock > 0) {
      cmd_io_thread_->loop().lock()->RemoveDelegate(sock, this);
    }
  }
  
  for (auto it = data_channel_.begin(); it != data_channel_.end(); ++it) {
    socket_t sock = *it;
    if (sock > 0) {
      data_io_thread_->loop().lock()->RemoveDelegate(sock, this);
    }
  }

  for (socket_t& sock : socket_vec_) {
    util::CloseSock(sock);
    sock = -1;
  }
  socket_vec_.clear();

  if (detection_thread_) {
    is_stop_detection_.store(true);
    detection_thread_->join();
    detection_thread_ = nullptr;

    if (detection_socket_ > 0) {
      util::CloseSock(detection_socket_);
      detection_socket_ = -1;
    }

    if (detection_broadcast_socket_ > 0) {
      util::CloseSock(detection_broadcast_socket_);
      detection_broadcast_socket_ = -1;
    }
  }

  if (detection_io_thread_) {
    detection_io_thread_->Quit();
    detection_io_thread_->Join();
    detection_io_thread_->Uninit();
    detection_io_thread_ = nullptr;
  }

  if (cmd_io_thread_) {
    cmd_io_thread_->Quit();
    cmd_io_thread_->Join();
    cmd_io_thread_->Uninit();
    cmd_io_thread_ = nullptr;
  }

  if (data_io_thread_) {
    data_io_thread_->Quit();
    data_io_thread_->Join();
    data_io_thread_->Uninit();
    data_io_thread_ = nullptr;
  }

  lidars_cfg_ptr_ = nullptr;
  custom_lidars_cfg_ptr_ = nullptr;

  lidars_cmd_port_.clear();
  lidars_data_port_.clear();

  custom_lidars_.clear();
  custom_lidars_cfg_map_.clear();

  channel_info_.clear();
  general_command_channel_.clear();
  custom_command_channel_.clear();


  command_channel_.clear();
  data_channel_.clear();


  comm_port_.reset(nullptr);

  is_stop_detection_.store(true);

  {
    std::lock_guard<std::mutex> lock(lidars_dev_type_mutex_);
    lidars_dev_type_.clear();
  }

  is_view_ = false;
  host_ip_ = "";
  
  {
    std::lock_guard<std::mutex> lock(view_device_mutex_);
    view_devices_.clear();
  }

  {
    std::lock_guard<std::mutex> lock(view_lidars_info_mutex_);
    view_lidars_info_.clear();
  }
}

DeviceManager::~DeviceManager() {
  Destory();
}

} // namespace lidar
} // namespace livox

