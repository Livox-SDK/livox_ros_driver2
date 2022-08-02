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

#include "general_command_handler.h"

#include "livox_lidar_def.h"
#include "command_handler/command_handler.h"
#include "command_handler/hap_command_handler.h"

#include "base/logging.h"
#include "comm/protocol.h"
#include "comm/generate_seq.h"

#include "build_request.h"

namespace livox {
namespace lidar {
  
GeneralCommandHandler& GeneralCommandHandler::GetInstance() {
  static GeneralCommandHandler general_command_handler;
  return general_command_handler;
}

bool GeneralCommandHandler::Init(const std::string& host_ip, const bool is_view, DeviceManager* device_manager) {
  is_view_ = is_view;
  host_ip_ = host_ip;
  device_manager_ = device_manager;
  comm_port_.reset(new CommPort());
  return true; 
}



bool GeneralCommandHandler::Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr, DeviceManager* device_manager) {
  comm_port_.reset(new CommPort());
  for (auto it = lidars_cfg_ptr->begin(); it != lidars_cfg_ptr->end(); ++it) {
    const uint8_t dev_type = it->device_type;
    if (lidars_command_handler_.find(dev_type) == lidars_command_handler_.end()) {
      if (dev_type == kLivoxLidarTypeIndustrialHAP) {
        std::shared_ptr<HapCommandHandler> hap_command_handler_ptr(new HapCommandHandler(device_manager));
        lidars_command_handler_[dev_type] = hap_command_handler_ptr;        
        if (!(lidars_command_handler_[dev_type]->Init(lidars_cfg_ptr, custom_lidars_cfg_ptr))) {
          LOG_ERROR("General command handler init failed, the lidar of type:{} command init failed.", dev_type);
          return false;
        }
      }
    }
  }

  for (auto it = custom_lidars_cfg_ptr->begin(); it != custom_lidars_cfg_ptr->end(); ++it) {
    const uint8_t dev_type = it->device_type;
    if (lidars_command_handler_.find(dev_type) == lidars_command_handler_.end()) {
      if (dev_type == kLivoxLidarTypeIndustrialHAP) {
        lidars_command_handler_[dev_type].reset(new HapCommandHandler(device_manager));
        if (!(lidars_command_handler_[dev_type]->Init(lidars_cfg_ptr, custom_lidars_cfg_ptr))) {
          LOG_ERROR("General command handler init failed, the custom lidar of type:{} command init failed.", dev_type);
          return false;
        }
      }
    }
  }
  device_manager_ = device_manager;
  return true;
}

  
void GeneralCommandHandler::Destory() {
  device_manager_ = nullptr;
  comm_port_.reset(nullptr);
  {
    std::lock_guard<std::mutex> lock(dev_type_mutex_);
    device_dev_type_.clear();
  }

  {
    std::lock_guard<std::mutex> lock(devices_mutex_);
    devices_.clear();
  }

  lidars_command_handler_.clear();

  {
    std::mutex commands_mutex_;
    std::map<uint32_t, std::pair<Command, TimePoint> > commands_;
  }

  livox_lidar_info_change_cb_ = nullptr;
  livox_lidar_info_change_client_data_ = nullptr;

  host_ip_ = "";
  is_view_ = false;
}

GeneralCommandHandler::~GeneralCommandHandler() { 
  Destory();
}

void GeneralCommandHandler::Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size) {
  if (buf == nullptr || buf_size == 0) {
    return;
  }

  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  if (!(comm_port_->ParseCommStream((uint8_t*)buf, buf_size, &packet))) {
    LOG_INFO("Parse Command Stream failed.");
    return;
  }

  if (lidar_port == kDetectionPort && packet.cmd_id == kCommandIDLidarSearch) {
    if (packet.cmd_type == kCommandTypeCmd) {
      return;
    }

    HandleDetectionData(handle, lidar_port, packet);
  } else if (packet.cmd_id == kCommandIDLidarGetInternalInfo) {
    uint32_t seq = packet.seq_num;
    Command command;
    {
      std::lock_guard<std::mutex> lock(commands_mutex_);
      if (commands_.find(seq) == commands_.end()) {
        return;
      }

      command = commands_[seq].first;
      command.packet = packet;
      commands_.erase(seq);
    }

    if (command.cb) {
      (*command.cb)(kLivoxLidarStatusSuccess, handle, command.packet.data);
    }
  }
}

void GeneralCommandHandler::AddCommand(const Command& command) {
  if (command.packet.cmd_type == kCommandTypeAck) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(commands_mutex_);
  commands_[command.packet.seq_num] = std::make_pair(command, std::chrono::steady_clock::now() + std::chrono::milliseconds(command.time_out));
  Command &cmd = commands_[command.packet.seq_num].first;
  if (cmd.packet.data != NULL) {
    cmd.packet.data = NULL;
    cmd.packet.data_len = 0;
  }
 
}

void GeneralCommandHandler::Handler(const uint8_t dev_type, const uint32_t handle, const uint16_t lidar_port,
    uint8_t *buf, uint32_t buf_size) {
  if (buf == nullptr || buf_size == 0) {
    return;
  }

  CommPacket packet;
  memset(&packet, 0, sizeof(packet));
  if (!(comm_port_->ParseCommStream((uint8_t*)buf, buf_size, &packet))) {
    LOG_INFO("Parse Command Stream failed.");
    return;
  }
  
  if (lidar_port == kDetectionPort && packet.cmd_id == kCommandIDLidarSearch) {
    if (packet.cmd_type == kCommandTypeCmd) {
      LOG_INFO("GeneralCommandHandler::Handler Recv detection buf_size:{}", buf_size);
      return;
    }
    
    HandleDetectionData(handle, lidar_port, packet);
    return;
  }


  LOG_INFO("GeneralCommandHandler::Handler enter hap command handle, cmd_type:{}", packet.cmd_type);
  std::shared_ptr<CommandHandler> cmd_handler = GetLidarCommandHandler(dev_type);
  if (cmd_handler == nullptr) {
    LOG_ERROR("GeneralCommandHandler::Handler get cmd handler failed");
    return;
  }

  Command command;
  if (packet.cmd_type == kCommandTypeAck) {
    uint16_t seq = packet.seq_num;
    std::lock_guard<std::mutex> lock(commands_mutex_);
    if (commands_.find(seq) != commands_.end()) {
      command = commands_[seq].first;
      command.packet = packet;
      commands_.erase(seq);
    }
  } else if (packet.cmd_type == kCommandTypeCmd) {
    command.packet = packet;
    command.handle = handle;
  }
  cmd_handler->Handle(handle, command);
}

bool GeneralCommandHandler::VerifyNetSegment(const DetectionData* detection_data) {
  if (is_view_) {
    if (host_ip_.empty()) {
      LOG_ERROR("Verify net segment faield, the host ip is empty.");
      return false;
    }

    std::vector<uint8_t> host_ip_vec;
    if (!BuildRequest::IpToU8(host_ip_, ".", host_ip_vec)) {
      return false;
    }
    
    if (host_ip_vec[0] == detection_data->lidar_ip[0] &&
        host_ip_vec[1] == detection_data->lidar_ip[1] &&
        host_ip_vec[2] == detection_data->lidar_ip[2]) {
      LOG_INFO("Host ip:{}, lidar ip:{}.{}.{}.{}", host_ip_.c_str(), detection_data->lidar_ip[0],
          detection_data->lidar_ip[1], detection_data->lidar_ip[2], detection_data->lidar_ip[3]);
      return true;
    }
    std::string lidar_ip = std::to_string(detection_data->lidar_ip[0]) + "." +
      std::to_string(detection_data->lidar_ip[1]) + "." +
      std::to_string(detection_data->lidar_ip[2]) + "." +
      std::to_string(detection_data->lidar_ip[3]);
    LOG_ERROR("The host address and lidar address are on different network segments, the host_ip:{}, lidar_ip:{}",
        host_ip_.c_str(), lidar_ip.c_str());
    return false;
  }
  return true;
}

void GeneralCommandHandler::CreateCommandHandle(const uint8_t dev_type) {
  if (!is_view_) {
    return;
  }
  if (lidars_command_handler_.find(dev_type) == lidars_command_handler_.end()) {
    if (dev_type == kLivoxLidarTypeIndustrialHAP) {
      std::shared_ptr<HapCommandHandler> hap_command_handler_ptr(new HapCommandHandler(device_manager_));
      lidars_command_handler_[dev_type] = hap_command_handler_ptr;        
      if (!(lidars_command_handler_[dev_type]->Init(is_view_))) {
        LOG_ERROR("General command handler init failed, the lidar of type:{} command init failed.", dev_type);
      }
    }
  }
}

void GeneralCommandHandler::HandleDetectionData(uint32_t handle, uint16_t lidar_port, const CommPacket& packet) {
  if (packet.data == nullptr || packet.data_len == 0) {
    return;
  }

  DetectionData* detection_data = (DetectionData*)(packet.data);
  if (detection_data->ret_code != 0) {
    LOG_ERROR("Detection lidar faield, the handle:{}, lidar_port:{}, ret_code:{}",
        handle, lidar_port, detection_data->ret_code);
    return;
  }

  if (!VerifyNetSegment(detection_data)) {
    return;
  }

  CreateCommandHandle(detection_data->dev_type);

  std::string lidar_ip = std::to_string(detection_data->lidar_ip[0]) + "." +
      std::to_string(detection_data->lidar_ip[1]) + "." +
      std::to_string(detection_data->lidar_ip[2]) + "." +
      std::to_string(detection_data->lidar_ip[3]);

  
  if (devices_.find(handle) != devices_.end()) {
    DeviceInfo& device_info = devices_[handle];
    if (!(device_info.is_update_cfg.load())) {
      if (!is_view_) {
        UpdateLidarCfg(detection_data->dev_type, handle, detection_data->cmd_port);
      }
    }

    if (strcmp(device_info.sn.c_str(), detection_data->sn) != 0) {
      LOG_ERROR("Lidar ip conflic, the lidar ip:{}, the sn1:{}, the sn2:{}", lidar_ip.c_str(),
          device_info.sn.c_str(), detection_data->sn);
    }

    if (device_manager_) {
      device_manager_->HandleDetectionData(handle, detection_data);
    }

    return;
  }
  
  DeviceInfo& device_info = devices_[handle];
  device_info.sn = detection_data->sn;
  device_info.lidar_ip = lidar_ip;
  device_info.dev_type = detection_data->dev_type;
  device_info.is_update_cfg.store(false);
  //devices_[handle] = std::move(device_info);
  

  LOG_INFO("Handle detection data, handle:{}, dev_type:{}, sn:{}, lidar_ip:{}, cmd_port:{}",
      handle, detection_data->dev_type, detection_data->sn, lidar_ip.c_str(), detection_data->cmd_port);
  {
    std::lock_guard<std::mutex> lock(dev_type_mutex_);
    if (device_dev_type_.find(handle) != device_dev_type_.end()) {
      if (device_dev_type_[handle] != detection_data->dev_type) {
        LOG_ERROR("Lidar dev type conflic, the lidar ip:{}, the dev_type1:{}, the dev_type2:{}",
            lidar_ip.c_str(), device_dev_type_[handle], detection_data->dev_type);
      }
    } else {
      device_dev_type_[handle] = detection_data->dev_type;
      LOG_INFO("Handle detection data, set buff handle:{}, dev_type:{}", handle, detection_data->dev_type);
    }
  }

  if (device_manager_) {
    device_manager_->HandleDetectionData(handle, detection_data);
  }

  if (!is_view_) {
    UpdateLidarCfg(detection_data->dev_type, handle, detection_data->cmd_port);
  }
}

void GeneralCommandHandler::UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info) {
  std::shared_ptr<CommandHandler> cmd_handler = GetLidarCommandHandler(view_lidar_info.dev_type);
  if (cmd_handler != nullptr) {
    cmd_handler->UpdateLidarCfg(view_lidar_info);
  }
}

void GeneralCommandHandler::UpdateLidarCfg(const uint8_t dev_type, const uint32_t handle, const uint16_t lidar_cmd_port) {
  std::shared_ptr<CommandHandler> cmd_handler = GetLidarCommandHandler(dev_type);
  if (cmd_handler != nullptr) {
    cmd_handler->UpdateLidarCfg(handle, lidar_cmd_port);
  } 
}


void GeneralCommandHandler::LivoxLidarInfoChange(const uint32_t handle) {
  LivoxLidarInfo lidar_info;
  {
    devices_[handle].is_update_cfg.store(true);
    if (devices_.find(handle) == devices_.end()) {
      LOG_ERROR("Lidar info change failed, can not found device, the handle:{}", handle);
      return;
    }

    const DeviceInfo& device_info = devices_[handle];
    strcpy(lidar_info.sn, device_info.sn.c_str());
    strcpy(lidar_info.lidar_ip, device_info.lidar_ip.c_str());
    lidar_info.dev_type = device_info.dev_type;
  }

  if (device_manager_ && is_view_) {
    device_manager_->UpdateViewLidarCfgCallback(handle);
  }

  if (livox_lidar_info_change_cb_) {
    livox_lidar_info_change_cb_(handle, &lidar_info, livox_lidar_info_change_client_data_);
  }
}

std::shared_ptr<CommandHandler> GeneralCommandHandler::GetLidarCommandHandler(const uint8_t dev_type) {
  if (lidars_command_handler_.find(dev_type) != lidars_command_handler_.end()) {
    return lidars_command_handler_[dev_type];
  }
  return nullptr;
}

livox_status GeneralCommandHandler::QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  std::set<ParamKeyName> key_sets {
    kKeyPclDataType,
    kKeyPatternMode,
    kKeyDualEmitEnable,
    kKeyPointSendEnable,
    kKeyLidarIPCfg,
    kKeyLidarPointDataHostIPCfg,
    kKeyLidarImuHostIPCfg,
    kKeyInstallAttitude,
    kKeyBlindSpotSet,
    kKeyWorkMode,
    kKeyGlassHeat,
    kKeyImuDataEn,
    kKeyFusaEn,
    kKeySn,
    kKeyProductInfo,
    kKeyVersionApp,
    kKeyVersionLoader,
    kKeyVersionHardware,
    kKeyMac,
    kKeyCurWorkState,
    kKeyStatusCode,
    kKeyLidarDiagStatus,
    kKeyLidarFlashStatus
  };

  uint16_t key_num = key_sets.size();
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  for (const auto &key : key_sets) {
    LivoxLidarKeyValueParam* kList = (LivoxLidarKeyValueParam*)&req_buff[req_len];
    kList->key = static_cast<uint16_t>(key);
    req_len += sizeof(uint16_t);
  }

  return SendCommand(handle,
                    kCommandIDLidarGetInternalInfo,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarDiagInternalInfoResponse>(cb, client_data));
}


livox_status GeneralCommandHandler::SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPclDataType);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(data_type);
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


// livox_status GeneralCommandHandler::EnableLivoxLidarHighResolutionPointType(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
//   uint8_t req_buff[kMaxCommandBufferSize] = {0};
//   uint16_t req_len = 0;

//   uint16_t key_num = 1;
//   memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
//   req_len = sizeof(key_num) + sizeof(uint16_t);

//   LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
//   kv->key = static_cast<uint16_t>(kKeyPclDataType);
//   kv->length = sizeof(uint8_t);
//   kv->value[0] = static_cast<uint8_t>(kLivoxLidarCartesianCoordinateHighData); //32bit point type
//   req_len += sizeof(LivoxLidarKeyValueParam);

//   return SendCommand(handle,
//                      kCommandIDLidarWorkModeControl,
//                     req_buff,
//                     req_len,
//                     MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
// }

// livox_status GeneralCommandHandler::DisableLivoxLidarHighResolutionPointType(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
//   uint8_t req_buff[kMaxCommandBufferSize] = {0};
//   uint16_t req_len = 0;

//   uint16_t key_num = 1;
//   memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
//   req_len = sizeof(key_num) + sizeof(uint16_t);

//   LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
//   kv->key = static_cast<uint16_t>(kKeyPclDataType);
//   kv->length = sizeof(uint8_t);
//   kv->value[0] = static_cast<uint8_t>(kLivoxLidarCartesianCoordinateLowData); //16bit point type
//   req_len += sizeof(LivoxLidarKeyValueParam);

//   return SendCommand(handle,
//                     kCommandIDLidarWorkModeControl,
//                     req_buff,
//                     req_len,
//                     MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
// }

livox_status GeneralCommandHandler::SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPatternMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(scan_pattern);
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyDualEmitEnable);
  kv->length = sizeof(uint8_t);
  if (enable) {
    kv->value[0] = 0x01;
  } else {
    kv->value[0] = 0x00;
  }
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPointSendEnable);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPointSendEnable);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarIp(uint32_t handle, const LivoxLidarIpInfo* ip_config,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetLidarIPInfoRequest(*ip_config, req_buff, req_len)) {
    return -1;
  }

  return SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarPointDataHostIPCfg(uint32_t handle, const HostPointIPInfo& host_point_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetHostPointDataIPInfoRequest(host_point_ipcfg, req_buff, req_len)) {
    return -1;
  }

  return SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarImuDataHostIPCfg(uint32_t handle, const HostImuDataIPInfo& host_imu_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetHostImuDataIPInfoRequest(host_imu_ipcfg, req_buff, req_len)) {
    return -1;
  }

  return SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status GeneralCommandHandler::SetLivoxLidarInstallAttitude(uint32_t handle, const LivoxLidarInstallAttitude& install_attitude,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyInstallAttitude);
  kv->length = sizeof(LivoxLidarInstallAttitude);

  LivoxLidarInstallAttitude* install_attitude_val = (LivoxLidarInstallAttitude*)&kv->value;
  memcpy(install_attitude_val, &install_attitude, sizeof(LivoxLidarInstallAttitude));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(LivoxLidarInstallAttitude);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //blind spot
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyBlindSpotSet);
  kv->length = sizeof(uint32_t);
  uint32_t* blind_spot_set = reinterpret_cast<uint32_t*>(&kv->value[0]);
  *blind_spot_set = blind_spot;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint32_t);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  
  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyWorkMode);
  kv->length = sizeof(uint8_t);
  uint8_t* val_work_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_work_mode = work_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status GeneralCommandHandler::EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t); 

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01; //enable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00; //disable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(glass_heat);
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status GeneralCommandHandler::EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyImuDataEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyImuDataEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFusaEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01; //enable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFusaEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00; //disable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::SetLivoxLidarLogParam(uint32_t handle, const LivoxLidarLogParam& log_param, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyLogParamSet);
  kv->length = sizeof(LivoxLidarLogParam);
  LivoxLidarLogParam* log_param_val = (LivoxLidarLogParam*)&kv->value;
  memcpy(log_param_val, &log_param, sizeof(LivoxLidarLogParam));

  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(LivoxLidarLogParam);

  return SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status GeneralCommandHandler::LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data) {
  LivoxLidarResetRequest reset_request;
  return SendCommand(handle, kCommandIDLidarRebootDevice, (uint8_t*)&reset_request, sizeof(LivoxLidarResetRequest),
      MakeCommandCallback<LivoxLidarResetResponse>(cb, client_data));
}

// Upgrade
livox_status GeneralCommandHandler::LivoxLidarStartUpgrade(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarStartUpgradeCallback cb, void* client_data) {
  return SendCommand(handle, 
                    kCommandIDGeneralRequestUpgrade,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarStartUpgradeResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::LivoxLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarXferFirmwareCallback cb, void* client_data) {
  return SendCommand(handle, 
                    kCommandIDGeneralXferFirmware,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarXferFirmwareResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::LivoxLidarCompleteXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarCompleteXferFirmwareCallback cb, void* client_data) {
  return SendCommand(handle,
                    kCommandIDGeneralCompleteXferFirmware,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarCompleteXferFirmwareResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::LivoxLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
    uint16_t length, LivoxLidarGetUpgradeProgressCallback cb, void* client_data) {
  return SendCommand(handle,
                    kCommandIDGeneralRequestUpgradeProgress,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarGetUpgradeProgressResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::LivoxLidarRequestFirmwareInfo(uint32_t handle,
    LivoxLidarRequestFirmwareInfoCallback cb, void* client_data) {
  return SendCommand(handle,
                    kCommandIDGeneralRequestFirmwareInfo, 
                    nullptr, 
                    0,
                    MakeCommandCallback<LivoxLidarRequestFirmwareInfoResponse>(cb, client_data));
}

livox_status GeneralCommandHandler::LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb,
    void* client_data) {
  LivoxLidarRebootRequest reboot_request;
  reboot_request.timeout = 100;
  return SendCommand(handle,
      kCommandIDLidarRebootDevice, (uint8_t *)&reboot_request,
      sizeof(reboot_request), MakeCommandCallback<LivoxLidarRebootResponse>(cb,
      client_data));
}

livox_status GeneralCommandHandler::SendCommand(uint32_t handle,
                                            uint16_t command_id,
                                            uint8_t *data,
                                            uint16_t length,
                                            const std::shared_ptr<CommandCallback> &cb) {
  struct in_addr addr;
  addr.s_addr = handle;
  std::string lidar_ip = inet_ntoa(addr);
  uint16_t seq = GenerateSeq::GetSeq();
  Command command(seq, command_id, kCommandTypeCmd, kHostSend, data, length, handle, lidar_ip, cb);

  LOG_INFO("Send command, the seq:{}, command_id:{}, lidar_ip:{}", seq, command_id, lidar_ip.c_str());

  if (device_dev_type_.find(handle) != device_dev_type_.end()) {
    uint8_t dev_type = device_dev_type_[handle];
    std::shared_ptr<CommandHandler> cmd_handler = GetLidarCommandHandler(dev_type);
    if (cmd_handler != nullptr) {
      cmd_handler->SendCommand(command);
    }
  }

  {
    std::lock_guard<std::mutex> lock(commands_mutex_);  
    commands_[command.packet.seq_num] = std::make_pair(command, std::chrono::steady_clock::now() + std::chrono::milliseconds(command.time_out));
    Command &cmd = commands_[command.packet.seq_num].first;
    LOG_INFO("Command_ Command_id:{}, seq:{}", command_id, command.packet.seq_num);

    if (cmd.packet.data != NULL) {
      cmd.packet.data = NULL;
      cmd.packet.data_len = 0;
    }
  }
 
  return kLivoxLidarStatusSuccess;
}

void GeneralCommandHandler::CommandsHandle(TimePoint now) {
  std::list<Command> timeout_commands;

  
  {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    std::map<uint32_t, std::pair<Command, TimePoint> >::iterator ite = commands_.begin();
    while (ite != commands_.end()) {
      std::pair<Command, TimePoint> &command_pair = ite->second;
      if (now > command_pair.second) {
        timeout_commands.push_back(command_pair.first);
        uint32_t seq = ite->first;
        ++ite;
        commands_.erase(seq);
      } else {
        ++ite;
      }
    }
  }
  for (auto& timeout_command : timeout_commands) {
    if (timeout_command.cb) {
      (*timeout_command.cb)(kLivoxLidarStatusTimeout, timeout_command.handle, timeout_command.packet.data);
    }
  }
}

}  // namespace livox
} // namespace lidar

