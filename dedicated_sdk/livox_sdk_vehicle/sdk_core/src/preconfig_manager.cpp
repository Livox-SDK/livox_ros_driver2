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

#include "preconfig_manager.h"
#include "command_handler/command_impl.h"
#include "command_handler/command_channel.h"
#include <set>
#include <list>
#include "base/logging.h"
#include "base/network/network_util.h"

using std::string;

namespace livox {
namespace vehicle {

bool PreConfigManager::Init(const std::string& net_if, LidarPreConfigParam& param) {
  param_ = param;
  if (comm_port_ == NULL) {
    comm_port_.reset(new CommPort());
  }
  uint16_t port = kCommandPort;
  sock_ = util::CreateSocket(port, true, true, true, net_if);
  if (sock_ < 0) {
    LOG_ERROR("Create Socket Failed, net_if: {}", net_if);
    return false;
  }
  thread_ = std::make_shared<IOThread>();
  thread_->Init(true, false);
  loop_ = thread_->loop();
  last_heartbeat_ =  TimePoint();
  loop_.lock()->AddDelegate(sock_, this);
  return thread_->Start();
}

void PreConfigManager::Uninit() {
  if (sock_ > 0) {
    if (!loop_.expired()) {
      loop_.lock()->RemoveDelegate(sock_, this);
    }
  }

  if (thread_) {
    thread_->Quit();
    thread_->Join();
    thread_->Uninit();
  }

  if (sock_ > 0) {
    util::CloseSock(sock_);
    sock_ = -1;
  }

  if (comm_port_) {
    comm_port_.reset(NULL);
  }

  commands_.clear();
  lidar_info_.clear();
}

void PreConfigManager::SetLidarPreConfigParamCallback(PreConfigCallback cb, void* client_data) {
  client_data_ = client_data;
  preconfig_call_back_ = cb;
}

void PreConfigManager::OnData(socket_t sock, void *) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  uint32_t buf_size = 0;
  uint8_t *cache_buf = comm_port_->FetchCacheFreeSpace(&buf_size);
  int size = buf_size;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(cache_buf), buf_size, 0, &addr, &addrlen);
  if (size < 0) {
    return;
  }
  LOG_INFO("On Data size: {}", size);
  std::string device_ip = std::string(inet_ntoa(((struct sockaddr_in *)&addr)->sin_addr));

  comm_port_->UpdateCacheWrIdx(size);
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  while (kParseSuccess == comm_port_->ParseCommStream(&packet)) {
    if (packet.cmd_id == kCommandIDLidarSearch) {
      LOG_INFO("On broadcast Ack");
      OnBroadcastAck(packet, device_ip);
      } else if (packet.cmd_id == kCommandIDLidarPreconfig) {
      LOG_INFO("On preconfig Ack");
      OnPreconfigAck(packet);
    }
  }
}

void PreConfigManager::OnTimer(TimePoint now) {
  if (now - last_heartbeat_ > std::chrono::seconds(5)) {
    last_heartbeat_ =  std::chrono::steady_clock::now();
    LidarSearchBroadcastRequest();
  }
  std::list<PreConfigCommand> timeout_commands;
  auto ite = commands_.begin();
  while (ite != commands_.end()) {
    auto &command_pair = ite->second;
    if (now > command_pair.second) {
      LOG_INFO("timeout seq: {}",ite->first);
      timeout_commands.push_back(command_pair.first);
      commands_.erase(ite++);
    } else {
      ++ite;
    }
  }

  for (auto & command : timeout_commands) {
    LOG_WARN("Command Timeout");
    // retry to config
    LidarPreConfig(command);
  }
}

void PreConfigManager::OnPreconfigAck(const CommPacket &packet) {
  if (!packet.data) {
    LOG_INFO("Empty preconfig ack");
    return;
  }
  uint16_t seq = packet.seq_num;
  if (commands_.find(seq) == commands_.end()) {
    LOG_INFO("Can't Find Seq: {}", seq);
    return;
  }
  std::string broadcast_code = commands_[seq].first.broadcast_code;
  commands_.erase(seq);
  uint8_t ret_code = *packet.data;
  if (ret_code == 0) {
    preconfig_call_back_(kVehicleStatusSuccess, broadcast_code.c_str(), client_data_);
  } else {
    preconfig_call_back_(kVehicleStatusFailure, broadcast_code.c_str(), client_data_);
  } 
}

void PreConfigManager::OnBroadcastAck(const CommPacket &packet,  const std::string& device_ip) {
  if (!packet.data) {
    return;
  }
  auto data = static_cast<SearchLidarResponse*>((void *)packet.data);
  std::string broadcast_code = std::string((char *)data->broadcast_code);
  if (lidar_info_.find(broadcast_code) != lidar_info_.end()) {
    return;
  }
  lidar_info_.insert(broadcast_code);
  PreConfigCommand command = {
    device_ip,
    broadcast_code
  };
  LidarPreConfig(command);
}

void PreConfigManager::LidarPreConfig(PreConfigCommand& command) {
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));
  packet.cmd_type = kCommandTypeCmd;
  packet.seq_num = CommandChannel::GenerateSeq();
  packet.cmd_id = kCommandIDLidarPreconfig;
  packet.data_len = sizeof(LidarPreConfigParam);
  packet.data = (uint8_t*)&param_;
  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(kCommandPort);
  // broadcast udp
  servaddr.sin_addr.s_addr = inet_addr(command.device_ip.c_str());
  LOG_INFO("Lidar PreConfig seq: {}", packet.seq_num);
  int byte_send = sendto(sock_, (const char*)buf.data(), size, 0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("LidarDiscovery Send Failed");
  }

  //record command
  // 1s timeout
  commands_[packet.seq_num] = 
        std::make_pair(command, std::chrono::steady_clock::now() + std::chrono::seconds(2));
}

void PreConfigManager::LidarSearchBroadcastRequest() {
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
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(kCommandPort);
  // broadcast udp
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");

  int byte_send = sendto(sock_, (const char*)buf.data(), size, 0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));
  if (byte_send < 0) {
    LOG_ERROR("LidarDiscovery Send Failed");
  }
}

PreConfigManager &preconfig_manager() {
  static PreConfigManager preconfig_manager;
  return preconfig_manager;
}

}
}  // namespace livox
