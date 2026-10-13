//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif
#include "device_manager.h"

#include <iostream>

#include "comm/define.h"
#include "comm/generate_seq.h"
#include "base/logging.h"
#include "command_handler/command_impl.h"
#include "command_handler/general_command_handler.h"
#include "data_handler/data_handler.h"
#include "logger_handler/logger_manager.h"
#include "debug_point_cloud_handler/debug_point_cloud_manager.h"

namespace livox {
namespace lidar {

DeviceManager::DeviceManager()
    : sdk_framework_cfg_ptr_(nullptr),
      lidars_cfg_ptr_(nullptr),
      custom_lidars_cfg_ptr_(nullptr),
      lidar_logger_cfg_ptr_(nullptr),
      detection_socket_(0),
      detection_broadcast_socket_(0),
      cmd_io_thread_(nullptr),
      data_io_thread_(nullptr),
      detection_io_thread_(nullptr),
      comm_port_(nullptr),
      is_stop_detection_(false),
      detection_thread_(nullptr),
      is_view_(false),
      detection_host_ip_(""),
      enable_save_log_(false) {
}

DeviceManager& DeviceManager::GetInstance() {
  static DeviceManager device_manager;
  return device_manager;
}

bool DeviceManager::Init(const std::string& host_ip, const LivoxLidarLoggerCfgInfo* log_cfg_info) {
  is_view_ = true;
  detection_host_ip_ = host_ip;
  comm_port_.reset(new CommPort());

  std::shared_ptr<LivoxLidarLoggerCfg> lidar_logger_cfg_ptr(new LivoxLidarLoggerCfg());
  if (log_cfg_info != nullptr) {
    lidar_logger_cfg_ptr->lidar_log_enable = log_cfg_info->lidar_log_enable;
    lidar_logger_cfg_ptr->lidar_log_cache_size = log_cfg_info->lidar_log_cache_size;
    lidar_logger_cfg_ptr->lidar_log_path = log_cfg_info->lidar_log_path;
    
    if(!DebugPointCloudManager::GetInstance().SetStorePath(lidar_logger_cfg_ptr->lidar_log_path)) {
      LOG_ERROR("Set the debug point cloud file storage path failed.");
      return false;
    }
  }

  if (!LoggerManager::GetInstance().Init(lidar_logger_cfg_ptr)) {
    LOG_ERROR("Logger manager init failed.");
    return false;
  }

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
                         std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr,
                         std::shared_ptr<LivoxLidarLoggerCfg> lidar_logger_cfg_ptr,
                         std::shared_ptr<LivoxLidarSdkFrameworkCfg>& sdk_framework_cfg_ptr) {
  is_view_ = false;
  lidars_cfg_ptr_ = lidars_cfg_ptr;
  custom_lidars_cfg_ptr_ = custom_lidars_cfg_ptr;
  lidar_logger_cfg_ptr_ = lidar_logger_cfg_ptr;
  sdk_framework_cfg_ptr_ = sdk_framework_cfg_ptr;

  if (lidars_cfg_ptr_ && !(lidars_cfg_ptr_->empty())) {
    detection_host_ip_ = lidars_cfg_ptr_->at(0).host_net_info.host_ip;
  } else if (custom_lidars_cfg_ptr && !(custom_lidars_cfg_ptr->empty())) {
    detection_host_ip_ = custom_lidars_cfg_ptr->at(0).host_net_info.host_ip;
  } else {
    LOG_ERROR("Device manager init failed, can not find cmd host ip.");
    return false;
  }
  comm_port_.reset(new CommPort());
  
  if (!lidar_logger_cfg_ptr) {
    LOG_ERROR("lidar_logger_cfg_ptr is nullptr.");
    return false;
  }

  if (!DebugPointCloudManager::GetInstance().SetStorePath(lidar_logger_cfg_ptr->lidar_log_path)) {
    LOG_ERROR("Set the debug point cloud file storage path failed.");
    return false;
  }

  if (!LoggerManager::GetInstance().Init(lidar_logger_cfg_ptr)) {
    LOG_ERROR("Logger manager init failed.");
    return false;
  }

  if (!GeneralCommandHandler::GetInstance().Init(custom_lidars_cfg_ptr, this)) {
    LOG_ERROR("General command handle init failed.");
    return false;
  }

  if (!DataHandler::GetInstance().Init()) {
    LOG_ERROR("Data handle init failed.");
    return false;
  }

  GetLidarConfigMap();

  if (!CreateIOThread()) {
    LOG_ERROR("Create IO thread failed.");
    return false;
  }

  if (!CreateChannel()) {
    LOG_ERROR("Create channel failed.");
    return false;
  }

  if (!(lidars_cfg_ptr_->empty()) || !(custom_lidars_cfg_ptr->empty())) {
    detection_thread_ = std::make_shared<std::thread>(&DeviceManager::DetectionLidars, this);
    is_stop_detection_.store(false);
  }

  LOG_INFO("Init livox lidars succ.");
  return true;
}

void DeviceManager::GetLidarConfigMap() {
  for (auto it = lidars_cfg_ptr_->begin(); it != lidars_cfg_ptr_->end(); ++it) {
    const LivoxLidarCfg& lidar_cfg = *it;
    type_lidars_cfg_map_[lidar_cfg.device_type] = lidar_cfg;
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

  for (auto it = custom_lidars_cfg_ptr_->begin(); it != custom_lidars_cfg_ptr_->end(); ++it) {
    const HostNetInfo& host_net_info = it->host_net_info;
    if (!CreateDataChannel(host_net_info)) {
      LOG_ERROR("Create data channel failed.");
      return false;
    }

    if (!CreateCommandChannel(it->device_type, host_net_info)) {
      LOG_ERROR("Create command channel failed.");
      return false;
    }
  }
  return true;
}

bool DeviceManager::CreateDetectionChannel() {
#ifdef WIN32
#else
  detection_broadcast_socket_ = util::CreateSocket(kDetectionPort, true, true, true, "255.255.255.255", "");
  if (detection_broadcast_socket_ < 0) {
    LOG_ERROR("Create detection broadcast socket failed.");
    return false;
  }
  detection_io_thread_->GetLoop().lock()->AddDelegate(detection_broadcast_socket_, this, nullptr);
#endif

  std::string key = detection_host_ip_ + ":" + std::to_string(kDetectionPort);
  detection_socket_ = util::CreateSocket(kDetectionPort, true, true, true, detection_host_ip_, "");
  if (detection_socket_ < 0) {
    LOG_ERROR("Create detection socket failed.");
    return false;
  }
  detection_io_thread_->GetLoop().lock()->AddDelegate(detection_socket_, this, nullptr);

  channel_info_[key] = detection_socket_;
  if (custom_command_channel_.find(key) == custom_command_channel_.end()) {
    custom_command_channel_[key] = detection_socket_;
  }

  return true;
}

bool DeviceManager::CreateDataChannel(const HostNetInfo& host_net_info) {
  if (!CreateDataSocketAndAddDelegate(host_net_info.host_ip, host_net_info.point_data_port, host_net_info.multicast_ip)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }

  if (!CreateDataSocketAndAddDelegate(host_net_info.host_ip, host_net_info.imu_data_port, host_net_info.multicast_ip)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }

  if (!CreateDataSocketAndAddDelegate(host_net_info.host_ip, kHostDebugPointCloudPort, host_net_info.multicast_ip)) {
    LOG_ERROR("Create debug point cloud socket and add delegate failed.");
    return false;
  }
  return true;
}

bool DeviceManager::CreateCommandChannel(const uint8_t dev_type, const HostNetInfo& host_net_info) {
  if (sdk_framework_cfg_ptr_->master_sdk) {
    if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.host_ip, host_net_info.cmd_data_port, kCmd)) {
      LOG_ERROR("Create socket and add delegate failed.");
      return false;
    }
  }

  if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.host_ip, host_net_info.push_msg_port, kPush)) {
    LOG_ERROR("Create socket and add delegate failed.");
    return false;
  }

  if (dev_type == kLivoxLidarTypePA) {
    //if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.push_msg_ip, kPaHostFaultPort, is_custom)) {
    if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.host_ip, kPaHostFaultPort, kFault)) {
      LOG_ERROR("Create socket and add delegate failed.");
      return false;
    }
  }

#ifdef WIN32
#else
  if (dev_type == kLivoxLidarTypeMid360 || dev_type == kLivoxLidarTypeMid360s || dev_type == kLivoxLidarTypeAvia2 || dev_type == kLivoxLidarTypeMid360l) {
    socket_t broadcast_socket = util::CreateSocket(host_net_info.push_msg_port, true, true, true, "255.255.255.255", "");
    if (broadcast_socket < 0) {
      LOG_ERROR("Create broadcast socket failed.");
      return false;
    }
    vec_broadcast_socket_.push_back(broadcast_socket);
    cmd_io_thread_->GetLoop().lock()->AddDelegate(broadcast_socket, this, nullptr);
  }
#endif

  if (LoggerManager::GetInstance().GetLogEnable()) {
    if (!CreateCmdSocketAndAddDelegate(dev_type, host_net_info.host_ip, host_net_info.log_data_port, kLog)) {
      LOG_ERROR("Create socket and add delegate failed.");
      return false;
    }
  }

  return true;
}

bool DeviceManager::CreateCmdSocketAndAddDelegate(const uint8_t dev_type, const std::string& host_ip,
                                                  const uint16_t port, const HostSocketType type) {
  if (host_ip.empty() || port == 0 || port == kLogPort) {
    return true;
  }

  std::string key = host_ip + ":" + std::to_string(port);

  if (channel_info_.find(key) != channel_info_.end()) {
    if (custom_command_channel_.find(key) == custom_command_channel_.end()) {
      custom_command_channel_[key] = channel_info_[key];
    }
    return true;
  }

  socket_t sock = -1;
  if (host_ip == "local") {
    sock = util::CreateSocket(port, true, true, true, "", "");
  } else {
    sock = util::CreateSocket(port, true, true, true, host_ip, "");
  }

  if (sock < 0) {
    LOG_ERROR("Add command channel faileld, can not create socket, dev_type:{}, the ip {} port {} ",
        dev_type, host_ip.c_str(), port);
    return false;
  }

  socket_vec_.push_back(sock);
  channel_info_[key] = sock;
  command_channel_.insert(sock); 
  custom_command_channel_[key] = sock;

  cmd_io_thread_->GetLoop().lock()->AddDelegate(sock, this, nullptr);
  return true;
}

bool DeviceManager::CreateDataSocketAndAddDelegate(const std::string& host_ip, const uint16_t port, const std::string& multicast_ip) {
  if (host_ip.empty() || port == 0 || port == kLogPort || port == kDetectionPort) {
    return true;
  }

  std::string key = host_ip + ":" + std::to_string(port);
  if (channel_info_.find(key) != channel_info_.end()) {
    return true;
  }

  socket_t sock = -1;
  if (host_ip == "local") {
    sock = util::CreateSocket(port, true, true, false, "", multicast_ip);
  } else {
    sock = util::CreateSocket(port, true, true, false, host_ip, multicast_ip);
  }
  if (sock < 0) {
    LOG_ERROR("Add command channel faileld, can not create socket, the ip {} port {} ", host_ip.c_str(), port);
    return false;
  }
  socket_vec_.push_back(sock);
  channel_info_[key] = sock;  

  data_channel_.insert(sock);
  data_io_thread_->GetLoop().lock()->AddDelegate(sock, this, nullptr);
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

  CommPacket packet;
  packet.protocol = kLidarSdk;
  packet.version = kSdkVer;
  packet.seq_num = GenerateSeq::GetSeq();
  packet.cmd_id = kCommandIDLidarSearch;
  packet.cmd_type = kCommandTypeCmd;
  packet.sender_type = kHostSend;
  packet.data = req_buff;
  packet.data_len = 0;

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

  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;
  std::string lidar_ip = inet_ntoa(tmp_addr);

  if (lidar_ip == detection_host_ip_) {
    return;
  }

  if (port == kMid360LidarDebugPointCloudPort || port == kHAPDebugPointCloudPort) {
    DebugPointCloudManager::GetInstance().Handler(handle, port, (uint8_t*)(buf.get()), size);
  }

  if (port == kHAPLogPort || port == kPaLidarLogPort || port == kMid360LidarLogPort) {
    LoggerManager::GetInstance().Handler(handle, port, (uint8_t*)(buf.get()), size);
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
    if (port == kDetectionPort || port == lidar_cfg.lidar_net_info.cmd_data_port || port == lidar_cfg.lidar_net_info.push_msg_port ||
        port == lidar_cfg.lidar_net_info.log_data_port || port == kPaLidarFaultPort) {
      GeneralCommandHandler::GetInstance().Handler(lidar_cfg.device_type, handle, port, (uint8_t*)(buf.get()), size);
      return;
    }
    return;
  }

  // parse the device type info from config_ptr and add to custom_lidars_cfg_map_
  if (port != kDetectionPort) {
    return;
  }

  CommPacket packet;
  memset(&packet, 0, sizeof(packet));
  if (!(comm_port_->ParseCommStream((uint8_t*)(buf.get()), size, &packet))) {
    LOG_INFO("Parse Command Stream failed.");
    return;
  }
  if (packet.cmd_id != kCommandIDLidarSearch) {
    return;
  }
  if (packet.data == nullptr || packet.data_len == 0) {
    return;
  }
  DetectionData* detection_data = (DetectionData*)(packet.data);
  if (detection_data->ret_code != 0) {
    return;
  }

  if (type_lidars_cfg_map_.find(detection_data->dev_type) == type_lidars_cfg_map_.end()) {
    return;
  }

  LivoxLidarCfg& lidar_cfg = type_lidars_cfg_map_[detection_data->dev_type];
  struct in_addr binary_ip;
  binary_ip.s_addr = handle;
  lidar_cfg.lidar_net_info.lidar_ipaddr = inet_ntoa(binary_ip);
  custom_lidars_cfg_map_[handle] = lidar_cfg;
  custom_lidars_cfg_ptr_->push_back(lidar_cfg);
  GeneralCommandHandler::GetInstance().Init(custom_lidars_cfg_ptr_, this);
  GeneralCommandHandler::GetInstance().CreateCommandHandler(detection_data->dev_type);

  for (auto it = custom_lidars_cfg_ptr_->begin(); it != custom_lidars_cfg_ptr_->end(); ++it) {
    const HostNetInfo& host_net_info = it->host_net_info;
    if (!CreateDataChannel(host_net_info)) {
      LOG_ERROR("Create data channel failed.");
      return;
    }

    if (!CreateCommandChannel(it->device_type, host_net_info)) {
      LOG_ERROR("Create command channel failed.");
      return;
    }
  }
  return;
}

void DeviceManager::HandleDetectionData(uint32_t handle, DetectionData* detection_data, bool is_get_loader_mode, bool is_load_mode) {
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
    if (!view_device.is_get.load() && is_get_loader_mode && !is_load_mode) {
      GetLivoxLidarInternalInfo(handle);
    } else if (is_get_loader_mode && !is_load_mode) {
      if (!view_device.is_set.load() && is_get_loader_mode && !is_load_mode) {
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

    if (is_get_loader_mode && is_load_mode) {
      GeneralCommandHandler::GetInstance().LivoxLidarInfoChange(handle);
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
    if (is_get_loader_mode && is_load_mode) {
      GeneralCommandHandler::GetInstance().LivoxLidarInfoChange(handle);
    }
    return;
  }
  lidars_dev_type_[handle] = detection_data->dev_type;

  if (is_get_loader_mode && is_load_mode) {
    GeneralCommandHandler::GetInstance().LivoxLidarInfoChange(handle);
  }
}

void DeviceManager::GetLivoxLidarInternalInfo(const uint32_t handle) {
  CommandImpl::QueryLivoxLidarInternalInfo(handle, DeviceManager::GetLivoxLidarInternalInfoCallback, this);
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
  view_lidar_info_ptr->host_ip = detection_host_ip_;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(&(view_lidar_info_ptr->host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(view_lidar_info_ptr->lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(&(view_lidar_info_ptr->host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(view_lidar_info_ptr->lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  if (view_lidar_info_ptr->dev_type == kLivoxLidarTypeMid360) {
    view_lidar_info_ptr->lidar_point_port = kMid360LidarPointCloudPort;
    view_lidar_info_ptr->lidar_imu_data_port = kMid360LidarImuDataPort;
  }

  if (view_lidar_info_ptr->dev_type == kLivoxLidarTypeMid360s) {
    view_lidar_info_ptr->lidar_point_port = kMid360sLidarPointCloudPort;
    view_lidar_info_ptr->lidar_imu_data_port = kMid360sLidarImuDataPort;
  }

  if (view_lidar_info_ptr->dev_type == kLivoxLidarTypeAvia2) {
    view_lidar_info_ptr->lidar_point_port = kAvia2LidarPointCloudPort;
    view_lidar_info_ptr->lidar_imu_data_port = kAvia2LidarImuDataPort;
  }

  if (view_lidar_info_ptr->dev_type == kLivoxLidarTypeMid360l) {
    view_lidar_info_ptr->lidar_point_port = kMid360lLidarPointCloudPort;
    view_lidar_info_ptr->lidar_imu_data_port = kMid360lLidarImuDataPort;
  }


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
    sock = util::CreateSocket(view_lidar_info.host_point_port, true, true, true, view_lidar_info.host_ip, "");
    if (sock < 0) {
      LOG_ERROR("Create View point data channel faileld, can not create socket, the ip {} port {} ",
          view_lidar_info.host_ip.c_str(), view_lidar_info.host_point_port);
      return;
    }
    socket_vec_.push_back(sock);
    channel_info_[point_key] = sock;
    data_io_thread_->GetLoop().lock()->AddDelegate(sock, this, nullptr);
  }

  std::string imu_key = view_lidar_info.host_ip + ":" + std::to_string(view_lidar_info.host_imu_data_port);
  if (channel_info_.find(imu_key) == channel_info_.end()) {
    socket_t sock = -1;
    sock = util::CreateSocket(view_lidar_info.host_imu_data_port, true, true, true, view_lidar_info.host_ip, "");
    if (sock < 0) {
      LOG_ERROR("Create View point data channel faileld, can not create socket, the ip {} port {} ",
          view_lidar_info.host_ip.c_str(), view_lidar_info.host_imu_data_port);
      return;
    }
    socket_vec_.push_back(sock);
    channel_info_[imu_key] = sock;
    data_io_thread_->GetLoop().lock()->AddDelegate(sock, this, nullptr);
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

int DeviceManager::SendCommand(const uint8_t dev_type, const uint32_t handle, const std::vector<uint8_t>& buf, 
    const int16_t size, const struct sockaddr *addr, socklen_t addrlen) {
  socket_t sock = -1;
  if (!GetCmdChannel(dev_type, handle, sock)) {
    LOG_WARN("Get cmd channel faileld, the lidar handle: {}", handle);
    sock = detection_socket_;
  }
  std::lock_guard<std::mutex> lock(mutex_cmd_channel_);
  return sendto(sock, (const char*)buf.data(), size, 0, addr, addrlen);
}

bool DeviceManager::GetCmdChannel(const uint8_t dev_type, const uint32_t handle, socket_t& sock) {
  if (custom_lidars_cfg_map_.find(handle) != custom_lidars_cfg_map_.end()) {
    const LivoxLidarCfg& lidar_cfg = custom_lidars_cfg_map_[handle];
    std::string key = lidar_cfg.host_net_info.host_ip + ":" + std::to_string(lidar_cfg.host_net_info.cmd_data_port);
    if (custom_command_channel_.find(key) != custom_command_channel_.end()) {
      sock = custom_command_channel_[key];
      return true;
    }
    return false;
  }
  return false;
}

int DeviceManager::SendLoggerCommand(const uint8_t dev_type, const uint32_t handle, const std::vector<uint8_t>& buf, 
    const int16_t size, const struct sockaddr *addr, socklen_t addrlen) {
  socket_t sock = -1;
  if (!GetLoggerCmdChannel(dev_type, handle, sock)) {
    sock = detection_socket_;
  }
  std::lock_guard<std::mutex> lock(mutex_logger_cmd_channel_);
  return sendto(sock, (const char*)buf.data(), size, 0, addr, addrlen);
}

bool DeviceManager::GetLoggerCmdChannel(const uint8_t dev_type, const uint32_t handle, socket_t& sock) {
  if (custom_lidars_cfg_map_.find(handle) != custom_lidars_cfg_map_.end()) {
    const LivoxLidarCfg& lidar_cfg = custom_lidars_cfg_map_[handle];
    std::string key = lidar_cfg.host_net_info.host_ip + ":" + std::to_string(lidar_cfg.host_net_info.log_data_port);
    if (custom_command_channel_.find(key) != custom_command_channel_.end()) {
      sock = custom_command_channel_[key];
      return true;
    }
    return false;
  }
  return false;
}

void DeviceManager::Destory() {
  detection_host_ip_ = "";

  if (detection_socket_ > 0) {
    detection_io_thread_->GetLoop().lock()->RemoveDelegate(detection_socket_, this);
  }

  if (detection_broadcast_socket_ > 0) {
    detection_io_thread_->GetLoop().lock()->RemoveDelegate(detection_broadcast_socket_, this);
  }

  for (auto it = command_channel_.begin(); it != command_channel_.end(); ++it) {
    socket_t sock = *it;
    if (sock > 0) {
      cmd_io_thread_->GetLoop().lock()->RemoveDelegate(sock, this);
    }
  }

  for (auto it = vec_broadcast_socket_.begin(); it != vec_broadcast_socket_.end(); ++it) {
    socket_t sock = *it;
    if (sock > 0) {
      cmd_io_thread_->GetLoop().lock()->RemoveDelegate(sock, this);
    }
  }
  
  for (auto it = data_channel_.begin(); it != data_channel_.end(); ++it) {
    socket_t sock = *it;
    if (sock > 0) {
      data_io_thread_->GetLoop().lock()->RemoveDelegate(sock, this);
    }
  }

  for (socket_t& sock : socket_vec_) {
    util::CloseSock(sock);
    sock = -1;
  }
  socket_vec_.clear();

  for (socket_t & sock : vec_broadcast_socket_) {
    util::CloseSock(sock);
    sock = -1;
  }
  vec_broadcast_socket_.clear();

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

  lidars_cfg_ptr_ = nullptr;
  custom_lidars_cfg_ptr_ = nullptr;

  type_lidars_cfg_map_.clear();
  custom_lidars_cfg_map_.clear();

  channel_info_.clear();
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
  detection_host_ip_ = "";
  
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
