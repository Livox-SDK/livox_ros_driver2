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

#include "command_channel.h"
#include <functional>
#include <atomic>
#include "base/logging.h"
#include "base/network/network_util.h"
#include "command_impl.h"
#include "livox_def_vehicle.h"
#include <stdio.h>

using std::bind;
using std::list;
using std::make_pair;
using std::map;
using std::pair;
using std::string;
using std::chrono::steady_clock;

namespace livox {
namespace vehicle {

CommandChannel::CommandChannel(socket_t sock,
                               CommandChannelDelegate *cb)
    : sock_(sock),
      callback_(cb),
      comm_port_(new CommPort) {}

bool CommandChannel::Bind(std::weak_ptr<IOLoop> loop) {
  if (loop.expired()) {
    return false;
  }
  loop_ = loop;

  log_sock_ = util::CreateSocket(kLogPort);
  if (log_sock_ == -1) {
    return false;
  }
  loop_.lock()->AddDelegate(sock_, this);
  loop_.lock()->AddDelegate(log_sock_, this);
  return true;
}

void CommandChannel::OnData(socket_t sock, void *) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  uint32_t buf_size = 0;
  uint8_t *cache_buf = comm_port_->FetchCacheFreeSpace(&buf_size);
  int size = buf_size;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(cache_buf), buf_size, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }
  comm_port_->UpdateCacheWrIdx(size);
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  while ((kParseSuccess == comm_port_->ParseCommStream(&packet))) {
    if (packet.cmd_type == kCommandTypeAck) {
      uint16_t seq = packet.seq_num;
      if (commands_.find(seq) != commands_.end()) {
        Command command = commands_[seq].first;
        command.packet = packet;
        if (callback_) {
          callback_->OnCommand(command.slot, command);
        }
        commands_.erase(seq);
      } else if (packet.cmd_id == kCommandIDLidarSearch) {
        if (callback_) {
          Command command;
          command.packet = packet;
          command.slot = packet.sender_id;
          callback_->OnCommand(command.slot, command);
        }
      }
    } else if (packet.cmd_type == kCommandTypeCmd) {
        if (callback_) {
          Command command;
          command.packet = packet;
          command.slot = packet.sender_id;
          callback_->OnCommand(command.slot, command);
        }
    }
  }
}

void CommandChannel::SendAsync(const Command &command) {
  Command cmd = DeepCopy(command);
  if (!loop_.expired()) {
    auto w_ptr = WeakProtector(protector_);
    loop_.lock()->PostTask([this, w_ptr, cmd]() {
      if(w_ptr.expired()) {
        return;
      }
      Send(cmd);
    });
  }
}

void CommandChannel::LidarSearchBroadcastRequest() {
  LOG_INFO("Send Search Broadcast Request");
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));
  packet.cmd_type = kCommandTypeCmd;
  packet.seq_num = CommandChannel::GenerateSeq();
  packet.cmd_id = kCommandIDLidarSearch;
  packet.data_len = 0;
  packet.data = nullptr;
  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(kCommandPort);
  // broadcast udp
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, packet);
 
  int byte_send = sendto(sock_, (const char*)buf.data(), size, 0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("LidarDiscovery Send Failed");
  }
}

void CommandChannel::OnTimer(TimePoint now) {
  if (now - last_heartbeat_ > std::chrono::seconds(3)) {
    last_heartbeat_ =  std::chrono::steady_clock::now();
    LidarSearchBroadcastRequest();
  }

  list<Command> timeout_commands;
  map<uint16_t, pair<Command, TimePoint> >::iterator ite = commands_.begin();
  while (ite != commands_.end()) {
    pair<Command, TimePoint> &command_pair = ite->second;
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
    if (callback_) {
      timeout_command.packet.cmd_type = kCommandTypeAck;
      callback_->OnCommand(timeout_command.slot, timeout_command);
    }
  }
}

void CommandChannel::Uninit() {
  if (!loop_.expired()) {
    loop_.lock()->RemoveDelegate(sock_, this);
    loop_.lock()->RemoveDelegate(log_sock_, this);
  }
  if (log_sock_ != -1) {
    util::CloseSock(log_sock_);
    log_sock_ = -1;
  }
  callback_ = nullptr;
  if (comm_port_) {
    comm_port_.reset(nullptr);
  }
  commands_.clear();
}

void CommandChannel::SendInternal(const Command &command) {
  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  socket_t sock = -1;
  //log command correspond to kLogPort
  if (command.packet.cmd_id == kCommandIDLidarLogInfo) {
    servaddr.sin_port = htons(kLogPort);
    sock = log_sock_;
  } else {
    servaddr.sin_port = htons(kCommandPort);
    sock = sock_;
  }
  servaddr.sin_addr.s_addr = inet_addr(command.remote_ip.c_str());

  int byte_send = sendto(sock, (const char*)buf.data(), size, 0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));
  if (byte_send < 0) {
    if (command.cb) {
      (*command.cb)(kVehicleStatusSendFailed, command.slot, nullptr);
    }
  }
}

uint16_t CommandChannel::GenerateSeq() {
  static std::atomic<std::uint16_t> seq(1);
  uint16_t value = seq.load();
  uint16_t desired = 0;
  do {
    if (value == UINT16_MAX) {
      desired = 1;
    } else {
      desired = value + 1;
    }
  } while (!seq.compare_exchange_weak(value, desired));
  return desired;
}

Command CommandChannel::DeepCopy(const Command &cmd) {
  Command result_cmd(cmd);
  if (result_cmd.packet.data != NULL) {
    result_cmd.packet.data = new uint8_t[result_cmd.packet.data_len];
    memcpy(result_cmd.packet.data, cmd.packet.data, result_cmd.packet.data_len);
  }
  return result_cmd;
}

void CommandChannel::Send(const Command &command) {
  LOG_INFO(" Send Command: Id {} Seq {}", command.packet.cmd_id, command.packet.seq_num);
  SendInternal(command);
  if (command.packet.cmd_type == kCommandTypeAck) {
    return;
  }
  commands_[command.packet.seq_num] = make_pair(command, steady_clock::now() + std::chrono::milliseconds(command.time_out));
  Command &cmd = commands_[command.packet.seq_num].first;
  if (cmd.packet.data != NULL) {
    delete[] cmd.packet.data;
    cmd.packet.data = NULL;
    cmd.packet.data_len = 0;
  }
}
}  // namespace vehicle
}  // namespace livox
