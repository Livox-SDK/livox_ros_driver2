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

#include "command_handler.h"
#include "base/command_callback.h"
#include <mutex>
#include "base/logging.h"
#include "command_impl.h"

using std::shared_ptr;
using std::list;
using std::make_pair;
using std::multimap;
using std::pair;

namespace livox {
namespace vehicle {

CommandHandler &command_handler() {
  static CommandHandler handler;
  return handler;
}

std::string CommandHandler::GetRemoteIp(uint8_t slot) {
  std::string remote_ip = "";
  if (slot_ip_.find(slot) != slot_ip_.end()) {
    remote_ip = slot_ip_[slot];
  }
  return remote_ip;
}

bool IsBroadcastSearchResponse(const Command &command) {
  if (!command.packet.data) {
    return false;
  }
  if (command.packet.cmd_id != kCommandIDLidarSearch) {
    return false;
  }
  return true;
}

void CommandHandler::AddDevice(const std::vector<LidarRegisterInfo>& lidars_info) {
  for (const auto& lidar_info : lidars_info) {
    slot_ip_[lidar_info.slot] = std::string(lidar_info.ip_addr);
  }
}

bool CommandHandler::Init(const std::string& net_if) {
  thread_.reset(new IOThread());
  thread_->Init(true, true);
  sock_ = util::CreateSocket(kCommandPort, true, true, true, net_if);
  if (sock_ == -1) {
    return false;
  }
  channel_.reset(new CommandChannel(sock_, this));
  channel_->Bind(thread_->loop());
  return thread_->Start();
}

void CommandHandler::Uninit() {
  if (channel_) {
    channel_->Uninit();
  }
  thread_->Quit();
  thread_->Join();
  thread_->Uninit();
  if (sock_ != -1) {
    util::CloseSock(sock_);
    sock_ = -1;
  }
  
  slot_ip_.clear();
  lidar_info_.clear();

  exception_detail_callback_ = nullptr;
  exception_detail_client_data_ = nullptr;

  exception_info_callback_ = nullptr;
  exception_info_client_data_ = nullptr;

  info_callback_ = nullptr;
  info_client_data_ = nullptr;

  log_callback_ = nullptr;
  log_client_data_ = nullptr;

  safety_callback_ = nullptr;
  safety_client_data_ = nullptr;
}

void CommandHandler::OnCommand(uint8_t slot, const Command &command) {
  if (command.packet.cmd_type == kCommandTypeAck) {
    LOG_INFO(" Receive Ack: Id {} Seq {}", command.packet.cmd_id, command.packet.seq_num);
    OnCommandAck(slot, command);
  } else if (command.packet.cmd_type == kCommandTypeCmd) {
    LOG_INFO(" Receive Command: Id {} Seq {}", command.packet.cmd_id, command.packet.seq_num);
    OnCommandCmd(slot, command);
  }
}

void CommandHandler::SendCommandAck(uint8_t *req, uint16_t length, const Command& command) {
  LOG_INFO("Send Command Ack: ID {} Seq: {}", command.packet.cmd_id, command.packet.cmd_type);
  Command cmd(command.slot,
              kCommandTypeAck,
              command.packet.cmd_id,
              command.packet.seq_num,
              req,
              length,
              0,
              GetRemoteIp(command.slot),
              std::shared_ptr<CommandCallback>());
  channel_->SendAsync(cmd);
  return;
}

livox_vehicle_status CommandHandler::SendCommand(uint8_t slot,
                                         uint8_t command_id,
                                         uint8_t *data,
                                         uint16_t length,
                                         const shared_ptr<CommandCallback> &cb) {
  if (GetRemoteIp(slot).empty()) {
    return kVehicleStatusChannelNotExist;
  }
  LOG_INFO("Send Command: ID {} Slot: {}", command_id, slot);
  Command cmd(slot,
              kCommandTypeCmd,
              command_id,
              CommandChannel::GenerateSeq(),
              data,
              length,
              KDefaultTimeOut,
              GetRemoteIp(slot),
              cb);
  channel_->SendAsync(cmd);
  return kVehicleStatusSuccess;
}

bool IsStatusException(const Command &command) {
  if (!command.packet.data) {
    return false;
  }
  if (command.packet.cmd_id != kCommandIDLidarWorkModeControl) {
    return false;
  }
  auto data = (LidarSyncControlResponse*)(command.packet.data);
  LidarStatusCode status_code = {0};
  if (std::memcmp((void*) &data->status_code, (void*)&status_code, sizeof(LidarStatusCode)) == 0) {
    return false;
  }
  return true;
}

void CommandHandler::QueryDiagnosisInfoCallback(livox_vehicle_status status, uint8_t slot, LivoxDetailExceptionInfo* info) {
  if (status == kVehicleStatusSuccess) {
    if (exception_detail_callback_) {
      exception_detail_callback_(slot, info, exception_detail_client_data_);
    }
  } else if (status == kVehicleStatusTimeout) {
   QueryDiagnosisInfo(slot);
  }
}

void CommandHandler::QueryDiagnosisInfo(uint8_t slot) {
  LOG_ERROR("Query Diagosise Info");
  SendCommand(slot,
              kCommandIDLidarGetDiagnosisInfo,
              nullptr,
              0,
              MakeCommandCallback<CommandHandler, LivoxDetailExceptionInfo>(
                this, &CommandHandler::QueryDiagnosisInfoCallback));
}

void CommandHandler::OnLidarInfoChange(const Command &command) {
  if (!command.packet.data) {
    return;
  }
  LOG_INFO("lidar info change");
  uint8_t slot = command.slot;
  auto data = static_cast<SearchLidarResponse*>((void *)command.packet.data);
  if (lidar_info_.find(slot) != lidar_info_.end()) {
    if (strncmp((const char*)lidar_info_[slot].broadcast_code, (const char*)data->broadcast_code, 16) == 0) {
      return;
    }
  }
  VehicleDeviceInfo info;
  info.slot = command.slot;
  memcpy(info.broadcast_code, data->broadcast_code, 16);
  info.ip_addr = data->ip_addr;
  lidar_info_[slot] = info;
  if (info_callback_) {
    info_callback_(&info, info_client_data_);
  }
}

void CommandHandler::OnCommandAck(uint8_t slot, const Command &command) {

  if (IsStatusException(command)) {
    //QueryDiagnosisInfo(slot);
  }

  if (IsBroadcastSearchResponse(command)) {
    OnLidarInfoChange(command);
    return;
  }

  if (!command.cb) {
    return;
  }

  if (!command.packet.data) {
    (*command.cb)(kVehicleStatusTimeout, slot, command.packet.data);
    return;
  }
  (*command.cb)(kVehicleStatusSuccess, slot, command.packet.data);
}

void CommandHandler::OnCommandCmd(uint8_t slot, const Command &command) {
  if (command.packet.cmd_id == kCommandIDLidarGetExceptionInfo) {
    // send ack
    if (exception_info_callback_) {
      LidarStatusCode* status = (LidarStatusCode*)command.packet.data;
      exception_info_callback_(slot, status, exception_info_client_data_);
    }
    uint8_t ret_code = 0x00;
    SendCommandAck(&ret_code, sizeof(uint8_t), command);
    //query diagnosis
    //Todo 先注释掉，pa 上还未实现
    //QueryDiagnosisInfo(slot);
  } else if (command.packet.cmd_id == kCommandIDLidarLogInfo) {
    LidarLogInfo* log_info = (LidarLogInfo*)command.packet.data;
    if (!log_callback_) {
      return;
    }
    log_callback_(slot, log_info, log_client_data_);

    LidarLogInfoAck ack;
    ack.ret_code = 0x00;
    ack.log_type = log_info->log_type;
    ack.file_index = log_info->file_index;
    ack.trans_index = log_info->trans_index;
    // send ack
    SendCommandAck((uint8_t*)&ack, sizeof(LidarLogInfoAck), command);
  } else if (command.packet.cmd_id == kCommandIDLidarSafetyInfo) {
    if (safety_callback_) {
      LidarSafetyInfo* safety_info = (LidarSafetyInfo*)command.packet.data;
      safety_callback_(slot, safety_info, safety_client_data_);
    }
  }
}

}
}  // namespace livox
