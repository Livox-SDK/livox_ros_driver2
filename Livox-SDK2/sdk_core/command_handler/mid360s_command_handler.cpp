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

#include "mid360s_command_handler.h"
#include "livox_lidar_def.h"

#include "base/command_callback.h"

#include "base/logging.h"
#include "comm/protocol.h"
#include "comm/generate_seq.h"

#include "build_request.h"
#include "general_command_handler.h"

#include "parse_lidar_state_info.h"

namespace livox {
namespace lidar {

Mid360sCommandHandler::Mid360sCommandHandler(DeviceManager* device_manager) 
    : CommandHandler(device_manager),
      comm_port_(new CommPort),
      is_view_(false) {
}

bool Mid360sCommandHandler::Init(bool is_view) {
  is_view_ = is_view;
  return true;
}

bool Mid360sCommandHandler::Init(const std::map<uint32_t, LivoxLidarCfg>& custom_lidars_cfg_map) {
  for (const auto& it : custom_lidars_cfg_map) {
    if (it.second.device_type == kLivoxLidarTypeMid360s && custom_lidars_.find(it.first) == custom_lidars_.end()) {
      custom_lidars_[it.first] = it.second;
    }
  }
  return true;
}

void Mid360sCommandHandler::Handle(const uint32_t handle, uint16_t lidar_port, const Command& command) {
  if (command.packet.cmd_type == kCommandTypeAck) {
    LOG_INFO(" Receive Ack: Id {} Seq {}", command.packet.cmd_id, command.packet.seq_num);
    OnCommandAck(handle, command);
  } else if (command.packet.cmd_type == kCommandTypeCmd) {
    LOG_INFO(" Receive Command: Id {} Seq {}", command.packet.cmd_id, command.packet.seq_num);
    OnCommandCmd(handle, lidar_port, command);
  }
}

void Mid360sCommandHandler::OnCommandAck(uint32_t handle, const Command &command) {
  if (command.cb == nullptr) {
    return;
  }

  if (command.packet.data == nullptr) {
    (*command.cb)(kLivoxLidarStatusTimeout, handle, command.packet.data);
    return;
  }

  (*command.cb)(kLivoxLidarStatusSuccess, handle, command.packet.data);
}

void Mid360sCommandHandler::OnCommandCmd(uint32_t handle, const uint16_t lidar_port, const Command &command) {
  if (command.packet.cmd_id == kCommandIDLidarPushMsg && lidar_port == kMid360sLidarPushMsgPort) {
    std::string info; 
    ParseLidarStateInfo::Parse(command.packet, info);
    GeneralCommandHandler::GetInstance().PushLivoxLidarInfo(handle, info);
  }
}

void Mid360sCommandHandler::UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info) {
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (devices_.find(view_lidar_info.handle) != devices_.end()) {
      return;
    }
  }
  SetViewLidar(view_lidar_info);
}

void Mid360sCommandHandler::UpdateLidarCfg(const uint32_t handle, const uint16_t lidar_cmd_port) {
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (devices_.find(handle) != devices_.end()) {
      return;
    }
  }
  
  if (custom_lidars_.find(handle) != custom_lidars_.end()) {
    const LivoxLidarCfg& lidar_cfg = custom_lidars_[handle];
    SetCustomLidar(handle, lidar_cmd_port, lidar_cfg);
    return;
  }
}

void Mid360sCommandHandler::SetViewLidar(const ViewLidarIpInfo& view_lidar_info) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  if (!BuildRequest::BuildUpdateViewLidarCfgRequest(view_lidar_info, req_buff, req_len)) {
    LOG_ERROR("Build update view lidar cfg request failed.");
    return;
  }

  struct in_addr addr;
  addr.s_addr = view_lidar_info.handle;
  std::string lidar_ip = inet_ntoa(addr);
  uint16_t seq = GenerateSeq::GetSeq();
  Command command(seq, kCommandIDLidarWorkModeControl, kCommandTypeCmd, kHostSend, req_buff, req_len, view_lidar_info.handle,
      lidar_ip, MakeCommandCallback<LivoxLidarAsyncControlResponse>(Mid360sCommandHandler::UpdateLidarCallback, this));
  SendCommand(command, view_lidar_info.lidar_cmd_port);
}

void Mid360sCommandHandler::SetCustomLidar(const uint32_t handle, const uint16_t lidar_cmd_port, const LivoxLidarCfg& lidar_cfg) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  if (!BuildRequest::BuildUpdateMid360LidarCfgRequest(lidar_cfg, req_buff, req_len)) {
    LOG_ERROR("Build update lidar cfg request failed.");
    return;
  }

  struct in_addr addr;
  addr.s_addr = handle;
  std::string lidar_ip = inet_ntoa(addr);
  uint16_t seq = GenerateSeq::GetSeq();
  Command command(seq, kCommandIDLidarWorkModeControl, kCommandTypeCmd, kHostSend, req_buff, req_len, handle,
      lidar_ip, MakeCommandCallback<LivoxLidarAsyncControlResponse>(Mid360sCommandHandler::UpdateLidarCallback, this));
  SendCommand(command, lidar_cmd_port);
}

void Mid360sCommandHandler::UpdateLidarCallback(livox_status status, uint32_t handle,
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    LOG_INFO("Update lidar failed, the status:{}", status);
    return;
  }

  if (response == nullptr) {
    LOG_ERROR("Update lidar failed, the handle:{}, status:{}, response is nullptr.", handle, status);
    return;
  }

  if (response->ret_code == 0 && response->error_key == 0) {
    if (client_data != nullptr) {
      Mid360sCommandHandler* self = (Mid360sCommandHandler*)client_data;
      self->AddDevice(handle);
    }
    LOG_INFO("Update lidar:{} succ.", handle);
    GeneralCommandHandler::GetInstance().LivoxLidarInfoChange(handle);
  } else {
    //GeneralCommandHandler::GetInstance().LivoxLidarInfoChange(handle);
    LOG_ERROR("Update lidar failed, the ret_code:{}, error_key:{}", response->ret_code, response->error_key);
  }
}

void Mid360sCommandHandler::AddDevice(const uint32_t handle) {
  std::lock_guard<std::mutex> lock(device_mutex_);
  devices_.insert(handle);
}

bool Mid360sCommandHandler::IsStatusException(const Command &command) {
  if (!command.packet.data) {
    return false;
  }

  if (command.packet.cmd_id != kCommandIDLidarWorkModeControl) {
    return false;
  }

  LivoxLidarAsyncControlResponse* data = (LivoxLidarAsyncControlResponse*)(command.packet.data);
  if (data->ret_code != 0) {
    return false;
  }
  return true;
}

livox_status Mid360sCommandHandler::SendCommand(const Command &command, const uint16_t lidar_cmd_port) {
  if (command.packet.cmd_type == kCommandTypeAck) {
    return kLivoxLidarStatusFailure;
  }

  GeneralCommandHandler::GetInstance().AddCommand(command);

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);


  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(command.lidar_ip.c_str());
  servaddr.sin_port = htons(lidar_cmd_port);

  int byte_send = device_manager_->SendCommand(kLivoxLidarTypeMid360s, command.handle, buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("Sent cmd to lidar failed, the send_byte:{}, cmd_id:{}, seq:{}, lidar_ip:{}",
        byte_send, command.packet.cmd_id, command.packet.seq_num, command.lidar_ip.c_str());
    if (command.cb) {
      (*command.cb)(kLivoxLidarStatusSendFailed, command.handle, nullptr);
    }
    return kLivoxLidarStatusSendFailed;
  }
  return kLivoxLidarStatusSuccess;
}

livox_status Mid360sCommandHandler::SendCommand(const Command &command) {
  if (command.packet.cmd_type == kCommandTypeAck) {
    return kLivoxLidarStatusFailure;
  }

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(command.lidar_ip.c_str());
  servaddr.sin_port = htons(kMid360sLidarCmdPort);

  int byte_send = device_manager_->SendCommand(kLivoxLidarTypeMid360s, command.handle, buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("Sent cmd to lidar failed, the send_byte:{}, cmd_id:{}, seq:{}, lidar_ip:{}",
        byte_send, command.packet.cmd_id, command.packet.seq_num, command.lidar_ip.c_str());
    if (command.cb) {
      (*command.cb)(kLivoxLidarStatusSendFailed, command.handle, nullptr);
    }
    return kLivoxLidarStatusSendFailed;
  }
  return kLivoxLidarStatusSuccess;
}

livox_status Mid360sCommandHandler::SendLoggerCommand(const Command &command) {
  if (command.packet.cmd_type == kCommandTypeAck) {
    return kLivoxLidarStatusFailure;
  }

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(command.lidar_ip.c_str());
  servaddr.sin_port = htons(kMid360sLidarLogPort);

  int byte_send = device_manager_->SendLoggerCommand(kLivoxLidarTypeMid360s, command.handle, buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("Sent cmd to lidar failed, the send_byte:{}, cmd_id:{}, seq:{}, lidar_ip:{}",
        byte_send, command.packet.cmd_id, command.packet.seq_num, command.lidar_ip.c_str());
    if (command.cb) {
      (*command.cb)(kLivoxLidarStatusSendFailed, command.handle, nullptr);
    }
    return kLivoxLidarStatusSendFailed;
  }
  return kLivoxLidarStatusSuccess;
}


bool Mid360sCommandHandler::GetHostInfo(const uint32_t handle, std::string& host_ip, uint16_t& cmd_port) {
  return true;
}


}  // namespace livox
} // namespace lidar

