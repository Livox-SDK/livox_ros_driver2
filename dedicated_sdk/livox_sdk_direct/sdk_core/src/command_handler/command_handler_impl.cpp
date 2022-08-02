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

#include "command_handler_impl.h"

#include "base/network/network_util.h"
#include "base/logging.h"

#include "build_request.h"
#include "parse_lidar_state_info.h"

namespace livox {

namespace direct {

CommandHandlerImpl::CommandHandlerImpl(CommandHandler* handler) : handler_(handler), comm_port_(new CommPort) {}

bool CommandHandlerImpl::Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr,
    const uint8_t lidars_num) {
  if (direct_host_ipinfo_ptr == nullptr && direct_lidars_info_ptr == nullptr) {
    LOG_ERROR("Command handler impl init failed, pointers to input parameters are null");
    return false;
  }
  if (lidars_num == 0 || lidars_num >= kMaxLidarCount) {
    LOG_ERROR("Command handler impl init failed, lidar_num is zero or exceeds the maximum 32");
    return false;
  }

  direct_host_ipinfo_ptr_ = direct_host_ipinfo_ptr;
  direct_lidars_info_ptr_ = direct_lidars_info_ptr;

  if (!CreateIOThread()) {
    LOG_ERROR("Create command io thread failed.");
    return false;
  }

  if (!CreateCommandChannel()) {
    LOG_ERROR("Create command channel failed.");
    return false;
  }

  if (broadcast_thread_ == nullptr) {
    broadcast_thread_ = std::make_shared<std::thread>(&CommandHandlerImpl::SetUpLidars, this);
  }
  return true;
}

bool CommandHandlerImpl::Start() {
  return true;
}

bool CommandHandlerImpl::CreateIOThread() {
  io_thread_ptr = std::make_shared<IOThread>();
  if (io_thread_ptr == nullptr || !(io_thread_ptr->Init(false, false))) {
    LOG_ERROR("Create command io thread failed, thread_ptr is nullptr or thread init failed");
    return false;
  }
  return io_thread_ptr->Start();
}

bool CommandHandlerImpl::CreateCommandChannel() {
  if (direct_host_ipinfo_ptr_) {
    if (!CreateChannel(direct_host_ipinfo_ptr_)) {
      return false;
    }
    return true;
  }

  for (auto it = direct_lidars_info_ptr_->begin(); it != direct_lidars_info_ptr_->end(); ++it) {
    std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr = it->host_ipinfo_ptr;
    if (!CreateChannel(direct_host_ipinfo_ptr)) {
      return false;
    }
  }
  return true;
}

bool CommandHandlerImpl::CreateChannel(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr) {
  if (direct_host_ipinfo_ptr == nullptr) {
    LOG_ERROR("Create command channel failed, direct_host_ipinfo_ptr is nullptr.");
    return false;
  }

  if (!CreateSocketAndAddDelegate(direct_host_ipinfo_ptr->host_push_msg_ip,
      direct_host_ipinfo_ptr->host_cmd_port)) {
    return false;
  }

  if (!CreateSocketAndAddDelegate(direct_host_ipinfo_ptr->host_push_msg_ip,
      direct_host_ipinfo_ptr->host_log_port)) {
    return false;
  }

  if (!CreateSocketAndAddDelegate(direct_host_ipinfo_ptr->host_push_msg_ip,
      direct_host_ipinfo_ptr->host_push_cmd_port)) {
    return false;
  }
  return true;
}

bool CommandHandlerImpl::CreateSocketAndAddDelegate(const std::string& host_ip, const uint16_t port) {
  if (host_ip.empty() || port == 0) {
    return false;
  }
  std::string key = host_ip + ":" + std::to_string(port);
  if (devices_channel_.find(key) != devices_channel_.end()) {
    return true;
  }

  socket_t sock = util::CreateSocket(port, true, true, true, host_ip);
  if (sock < 0) {
    LOG_ERROR("Add command channel faileld, can not create socket, the ip {} port {} ", host_ip.c_str(), port);
    return false;
  }
  io_thread_ptr->loop().lock()->AddDelegate(sock, this, nullptr);

  devices_channel_[key] = sock;
  return true;
}

void CommandHandlerImpl::SetUpLidars() {
  while (!is_quite_) {
    if (direct_host_ipinfo_ptr_) {
      BroadcastHostInfo();
    } else {
      for (auto it = direct_lidars_info_ptr_->begin(); it != direct_lidars_info_ptr_->end(); ++it) {
        DirectLidarInfo& direct_lidar_info = *it;
        SetUpLidar(direct_lidar_info, false);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void CommandHandlerImpl::BroadcastHostInfo() {
  if (direct_host_ipinfo_ptr_ == nullptr) {
    return;
  }

  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest().BuildBroadcastRequest(direct_host_ipinfo_ptr_, req_buff, req_len)) {
    LOG_ERROR("Broad cast host info failed, build broadcast request failed.");
    return;
  }

  Command cmd(BuildRequest().GenerateSeq(), kKeyCmdID, kCommandTypeCmd, 255,
      std::string(15, '\0'), req_buff, req_len);

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, cmd.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(kLidarCmdPort);
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");

  int byte_send = SendTo(direct_host_ipinfo_ptr_->host_push_msg_ip, direct_host_ipinfo_ptr_->host_cmd_port,
      buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
     LOG_INFO("Broadcast host info failed, Send to lidar failed.");
  }
}

bool CommandHandlerImpl::SetUpLidar(const DirectLidarInfo& direct_lidar_info, bool is_update) {
  if (is_update) {
    StopBroadcast();
    LOG_INFO("StopBroadcast");
  }

  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest().BuildSetUpLidarRequest(direct_lidar_info, req_buff, req_len)) {
    return false;
  }


  uint16_t seq = BuildRequest().GenerateSeq();
  Command cmd(seq, kKeyCmdID, kCommandTypeCmd, direct_lidar_info.lidar_id,
      direct_lidar_info.sn, req_buff, req_len);

  LOG_INFO("Set Up Lidar, the seq:{}, sn:{}, lidar_id:{}", seq, cmd.packet.sn, cmd.packet.lidar_id);

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, cmd.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
  servaddr.sin_port = htons(direct_lidar_info.lidar_ipinfo_ptr->lidar_cmd_port);

  int byte_send = 0;
  if (is_update) {
    if (direct_host_ipinfo_ptr_ == nullptr) {
      return false;
    }
    byte_send = SendTo(direct_host_ipinfo_ptr_->host_push_msg_ip, direct_host_ipinfo_ptr_->host_cmd_port,
      buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  } else {
    byte_send = SendTo(direct_lidar_info.host_ipinfo_ptr->host_push_msg_ip, direct_lidar_info.host_ipinfo_ptr->host_cmd_port,
      buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  }

  if (byte_send < 0) {
     LOG_INFO("Set up lidar failed, Send to lidar failed.");
    return false;
  }
  return true;
}

int CommandHandlerImpl::SendTo(const std::string& host_ip, const uint16_t host_cmd_port, 
    const std::vector<uint8_t>& buf, const int16_t size, const struct sockaddr *addr, socklen_t addrlen) {
  socket_t sock = -1;
  if (!GetCmdChannel(host_ip, host_cmd_port, sock)) {
    LOG_INFO("Send to lidar failed, can not get cmd channel, the ip {}, port {}",
        host_ip.c_str(), host_cmd_port);
    return -1;
  }
  

  std::lock_guard<std::mutex> lock(mutex_channel_);
  return sendto(sock, (const char*)buf.data(), size, 0, addr, addrlen);
}

void CommandHandlerImpl::OnData(socket_t sock, void *) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  
  std::unique_ptr<char[]> buf;
  buf.reset(new char[kCacheSize]);  

  int size = util::RecvFrom(sock, reinterpret_cast<char *>(buf.get()), kCacheSize, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }

  uint32_t handle = ((struct sockaddr_in *)&addr)->sin_addr.s_addr;

  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  if (!(comm_port_->ParseCommStream((uint8_t*)(buf.get()), size, &packet))) {
    LOG_INFO("Parse COmmand Stream failed.");
    return;
  }

  if (packet.msg_type == kCommandTypeAck) {
    LOG_INFO("Recieve Ack: Seq {} Cmd_Id {} Msg_Type {} Lidar_Id {} SN {}", packet.seq_num, packet.cmd_id,
        packet.msg_type, packet.lidar_id, packet.sn);
    OnCommandAck(handle, packet);
  } else if (packet.msg_type == kCommandTypeCmd) {
    LOG_INFO("Recieve Cmd: Seq {} Cmd_Id {} Msg_Type {} Lidar_Id {} SN {}", packet.seq_num, packet.cmd_id,
        packet.msg_type, packet.lidar_id, packet.sn);
    OnCommand(handle, packet);
  } else {
    LOG_INFO("Command handler impl get data failed, unknown msg_type {}", packet.msg_type);
  }
}

void CommandHandlerImpl::OnTimer(TimePoint now) {
  if (now - last_heartbeat_ > std::chrono::seconds(3)) {
    last_heartbeat_ =  std::chrono::steady_clock::now();
  }

  std::list<Command> timeout_commands;
  std::map<uint16_t, std::pair<Command, TimePoint> >::iterator ite = commands_.begin();
  while (ite != commands_.end()) {
    std::pair<Command, TimePoint> &command_pair = ite->second;
    if (now > command_pair.second) {
      timeout_commands.push_back(command_pair.first);
      commands_.erase(ite++);
    } else {
      ++ite;
    }
  }

  for (auto& timeout_command : timeout_commands) {
    LOG_WARN("Command Timeout: Id {}, Seq {}", 
        timeout_command.packet.cmd_id, timeout_command.packet.seq_num);
    if (timeout_command.cb_) {
      (*timeout_command.cb_)(kDirectStatusTimeout, timeout_command.slot_, timeout_command.packet.data);
    }
  }
}

void CommandHandlerImpl::OnCommandAck(const uint32_t handle, const CommPacket& packet) {
  uint16_t seq = packet.seq_num;
  if (commands_.find(seq) != commands_.end()) {
    Command command = commands_[seq].first;
    command.packet = packet;
    if (command.cb_) {
      (*command.cb_)(kDirectStatusSuccess, handle, command.packet.data);
    } else {
      LOG_ERROR("Command callback failed, the callback is nullptr.");
    }
    commands_.erase(seq);
    return;
  } 

  if (packet.cmd_id == kKeyCmdID) {
    DirectLidarCmdResInfo res_info;
    strcpy(res_info.sn, packet.sn);
    res_info.lidar_id = packet.lidar_id;
    memcpy(&res_info.res, packet.data, packet.data_len);
    if (res_info.res.ret_code != 0) {
      LOG_ERROR("Set lidar failed, the sn {}, the lidar id {} the ret_code {}, the error_key {}",
          packet.sn, res_info.lidar_id, res_info.res.ret_code, res_info.res.error_key);
    }

    if (!IsExist(handle) || is_quite_) {
      if (handler_) {
        handler_->DirectLidarCfgUpdateCallback(handle, &res_info);
      }
    }    
  } 
}

bool CommandHandlerImpl::IsExist(const uint32_t handle) {
  std::lock_guard<std::mutex> lock(mutex_lidars_);
  if (direct_lidars_.find(handle) == direct_lidars_.end()) {
    direct_lidars_.insert(handle);
    return false;
  }
  return true;
}

void CommandHandlerImpl::OnCommand(const uint32_t handle, const CommPacket& packet) {
  if (packet.cmd_id == kKeyPushCmdID) {
    DirectLidarStateInfo info;
    if (!ParseLidarStateInfo::Parse(packet, info)) {
      LOG_ERROR("OnCommand Parse Lidar State Info failed.");
      return;
    }

    if (handler_) {
      handler_->LidarStateInfoCallback(handle, &info);
    }
  } else if (packet.cmd_id >= kKeyLogStartID && packet.cmd_id <= kKeyLogEndID) {
    // write to file
  } else {
    LOG_INFO("On command failed, unknown command id, the command id {}", packet.cmd_id);
  }
}

bool CommandHandlerImpl::GetCmdChannel(const std::string& ip, const uint16_t port, socket_t& sock) {
  std::string key = ip + ":" + std::to_string(port);
  if (devices_channel_.find(key) == devices_channel_.end()) {
    return false;
  }
  sock = devices_channel_[key];
  return true;
}

bool CommandHandlerImpl::IsResetDevice(const Command& command) {
  return command.packet.cmd_id == kKeyReSetDevice;
}


livox_direct_status CommandHandlerImpl::UpdateDirectLidarCfg(
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr) {
  if (direct_lidars_info_ptr->size() != 1) {
    return false;
  }

  const DirectLidarInfo& direct_lidar_info = direct_lidars_info_ptr->at(0);
  LOG_INFO("Update direct lidar cfg begin.\n");
  SetUpLidar(direct_lidar_info, true);
  LOG_INFO("Update direct lidar cfg end.\n");

  return 0;
}

void CommandHandlerImpl::SendAsync(const Command &command) {
  if (IsResetDevice(command)) {
    StopBroadcast();
  }

  LOG_INFO("Send Command: Id {} Seq {}", command.packet.cmd_id,  command.packet.seq_num);
  SendInternal(command);
  if (command.packet.msg_type == kCommandTypeAck) {
    return;
  }
  commands_[command.packet.seq_num] = std::make_pair(command, std::chrono::steady_clock::now() + std::chrono::milliseconds(command.time_out_));
  Command &cmd = commands_[command.packet.seq_num].first;
  if (cmd.packet.data != NULL) {
    cmd.packet.data = NULL;
    cmd.packet.data_len = 0;
  }
}

void CommandHandlerImpl::SendInternal(const Command &command) {
  if (direct_host_ipinfo_ptr_ == nullptr) {
    if (command.cb_) {
      (*command.cb_)(kDirectStatusSendFailed, command.slot_, nullptr);
    }
  }

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);


  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(command.lidar_ip_.c_str());
  servaddr.sin_port = htons(kLidarCmdPort);
  
  int byte_send = SendTo(direct_host_ipinfo_ptr_->host_push_msg_ip, direct_host_ipinfo_ptr_->host_cmd_port,
      buf, size, (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("Sent cmd to lidar failed, the send_byte:{}, cmd_id:{}, seq:{}, lidar_ip:{}, lidar_port:{}",
        byte_send, command.packet.cmd_id, command.packet.seq_num, command.lidar_ip_.c_str(), kLidarCmdPort);
    if (command.cb_) {
      (*command.cb_)(kDirectStatusSendFailed, command.slot_, nullptr);
    }
  }
}


void CommandHandlerImpl::StopBroadcast() {
  if (broadcast_thread_) {
    is_quite_.store(true);
    broadcast_thread_->join();
    broadcast_thread_ = nullptr;
  }
}

CommandHandlerImpl::~CommandHandlerImpl() {
  StopBroadcast();

  for (const auto& device_channel : devices_channel_) {
    const socket_t& sock = device_channel.second;
    if (sock > 0) {
      io_thread_ptr->loop().lock()->RemoveDelegate(sock, this);
      util::CloseSock(sock);
    }
  }

  if (!devices_channel_.empty()) {
    devices_channel_.clear();
  }

  if (io_thread_ptr) {
    io_thread_ptr->Quit();
    io_thread_ptr->Join();
    io_thread_ptr->Uninit();
  }
}

}  // namespace livox
} // namespace direct

