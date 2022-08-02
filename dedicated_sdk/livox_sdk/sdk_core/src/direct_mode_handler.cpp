//
// The MIT License (MIT)
//
// Copyright (c) 2021 Livox. All rights reserved.
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

#include "direct_mode_handler.h"
#include <algorithm>
#include <mutex>
#include <iostream>
#include <vector>
#include "base/logging.h"
#include "base/network/network_util.h"
#include "command_handler/command_impl.h"
#include "device_manager.h"
#include "livox_def.h"

using std::tuple;
using std::string;
using std::vector;
using std::chrono::steady_clock;

namespace livox {

uint16_t DirectModeHandler::GetCommandTimeout(uint8_t command_set, uint8_t command_id) {
  return KDefaultTimeOut;
}

bool DirectModeHandler::Init(std::weak_ptr<IOLoop> loop) {
  LOG_INFO("Create Direct Mode Init");
  if (comm_port_ == NULL) {
    comm_port_.reset(new CommPort());
  }
  if (loop.expired()) {
    return false;
  }
  loop_ = loop;

  sock_ = util::CreateSocket(ANY_PORT, true, true, true);
  if (sock_ < 0) {
    LOG_INFO("Create Direct Mode Socket Failed");
    return false;
  }
  loop_.lock()->AddDelegate(sock_, this);
  return true;
}

void DirectModeHandler::OnData(socket_t sock, void *) {
  if (sock == sock_
      || status_map_.find(sock) != status_map_.end()) {
    OnCmdData(sock);
  } else {
    OnPacketData(sock);
  }
}

void DirectModeHandler::OnCmdData(socket_t sock) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  uint32_t buf_size = 0;
  uint8_t *cache_buf = comm_port_->FetchCacheFreeSpace(&buf_size);
  int size = buf_size;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(cache_buf), buf_size, 0, &addr, &addrlen);
  if (size < 0) {
    return;
  }

  comm_port_->UpdateCacheWrIdx(size);
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  while ((kParseSuccess == comm_port_->ParseCommStream(&packet))) {
    if (packet.cmd_set == kCommandSetLidar && packet.cmd_code == kCommandIDLidarBroadcast) {
      OnBroadcastAck(packet);
    } else if (packet.cmd_set == kCommandSetLidar && packet.cmd_code == kCommandIDLidarMsg) {
      OnStatusInfoCallback(sock, packet.data);
    } else if (packet.packet_type == kCommandTypeAck) {
      uint16_t seq = packet.seq_num;
      if (commands_.find(seq) != commands_.end()) {
        Command command = commands_[seq].first;
        command.packet = packet;
        if (command.cb) {
          (*command.cb)(kStatusSuccess, command.handle, command.packet.data);
        }
        commands_.erase(seq);
      }
    } else {
      LOG_INFO("Unknown Command");
    }
  }
}

void DirectModeHandler::OnPacketData(socket_t sock) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  if (data_buffers_.get() == NULL) {
    data_buffers_.reset(new char[kMaxBufferSize]);
  }

  int size = kMaxBufferSize;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(data_buffers_.get()), kMaxBufferSize, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }

  LivoxEthPacket *lidar_data = (LivoxEthPacket *)data_buffers_.get();
  if (lidar_data == NULL) {
    return;
  }
  uint16_t data_size = GetDataSize(size, lidar_data->data_type);

  if (point_map_.find(sock) != point_map_.end()) {
    uint8_t slot = point_map_[sock];
    PointDataCallback cb = std::get<1>(point_data_cb_[slot]);
    if (cb) {
      cb(slot, lidar_data, data_size, std::get<2>(point_data_cb_[slot]));
    }
  } else if (imu_map_.find(sock) != imu_map_.end()) {
    uint8_t slot = imu_map_[sock];
    ImuDataCallback cb = std::get<1>(imu_data_cb_[slot]);
    if (cb) {
      cb(slot, lidar_data, data_size, std::get<2>(imu_data_cb_[slot]));
    }
  }
}

uint16_t DirectModeHandler::GetDataSize(uint16_t size, uint8_t data_type) {
  uint16_t real_size = size;
  switch (data_type) {
    case kCartesian:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxRawPoint);
      break;
    case kSpherical:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxSpherPoint);
      break;
    case kExtendCartesian:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxExtendRawPoint);
      break;
    case kExtendSpherical:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxExtendSpherPoint);
      break;
    case kDualExtendCartesian:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxDualExtendRawPoint);
      break;
    case kDualExtendSpherical:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxDualExtendSpherPoint);
      break;
    case kImu:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxImuPoint);
      break;
    case kTripleExtendCartesian:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxTripleExtendRawPoint);
      break;
    case kTripleExtendSpherical:
      real_size = (size - kPrefixDataSize) / sizeof(LivoxTripleExtendSpherPoint);
      break;
    default:
      break;
  }
  return real_size;
}

void DirectModeHandler::OnTimer(TimePoint now) {
  if (now - last_heartbeat_ > std::chrono::seconds(5)) {
    last_heartbeat_ =  std::chrono::steady_clock::now();
    //LOG_INFO("Lidar Search broadcast request");
    LidarSearchBroadcastRequest();
  }

  std::list<Command> timeout_commands;
  auto ite = commands_.begin();
  while (ite != commands_.end()) {
    auto &command_pair = ite->second;
    if (now > command_pair.second) {
      timeout_commands.push_back(command_pair.first);
      commands_.erase(ite++);
    } else {
      ++ite;
    }
  }

  for (std::list<Command>::iterator ite = timeout_commands.begin(); ite != timeout_commands.end(); ++ite) {
    LOG_WARN("Command Timeout: Set {}, Id {}, Seq {}",
        (uint16_t)ite->packet.cmd_set, ite->packet.cmd_code, ite->packet.seq_num);
    if (ite->cb) {
      (*ite->cb)(kStatusTimeout, ite->handle, ite->packet.data);
    }
  }
}

void DirectModeHandler::Uninit() {
  if (sock_ > 0) {
    if (!loop_.expired()) {
      loop_.lock()->RemoveDelegate(sock_, this);
    }
    util::CloseSock(sock_);
    sock_ = -1;
  }
  // close imu sock
  for (const auto& imu : imu_map_) {
    socket_t sock = imu.first;
    loop_.lock()->RemoveDelegate(sock, this);
    util::CloseSock(sock_);
  }
  imu_map_.clear();
  // close point sock
  for (const auto& point : point_map_) {
    socket_t sock = point.first;
    loop_.lock()->RemoveDelegate(sock, this);
    util::CloseSock(sock);
  }
  point_map_.clear();
  // close status sock
  for (const auto& status : status_map_) {
    socket_t sock = status.first;
    loop_.lock()->RemoveDelegate(sock, this);
    util::CloseSock(sock);
  }
  status_map_.clear();

  if (comm_port_) {
    comm_port_.reset(NULL);
  }
}

livox_status DirectModeHandler::SwitchToNormalMode(uint8_t slot, SwithToNormalModeCallback cb, void *client_data) {
  if (lidars_info_.find(slot) == lidars_info_.end()) {
    return kStatusFailure;
  }
  LOG_INFO("Lidar SN: {}", std::string((char*)lidars_info_[slot].broadcast_code, kBroadcastCodeSize));
  Command cmd(slot,
              kCommandTypeCmd,
              kCommandSetLidar,
              kCommandIDLidarSwitchMode,
              CommandChannel::GenerateSeq(),
              lidars_info_[slot].broadcast_code,
              sizeof(lidars_info_[slot].broadcast_code),
              DirectModeHandler::GetCommandTimeout(kCommandSetGeneral, kCommandIDLidarSwitchMode),
              MakeCommandCallback<SwithToNormalModeResponse>(cb, client_data));
  LOG_INFO("Switch To Normal Mode");
  SendAsync(cmd);
  return kStatusSuccess;
}

Command DirectModeHandler::DeepCopy(const Command &cmd) {
  Command result_cmd(cmd);
  if (result_cmd.packet.data != NULL) {
    result_cmd.packet.data = new uint8_t[result_cmd.packet.data_len];
    memcpy(result_cmd.packet.data, cmd.packet.data, result_cmd.packet.data_len);
  }
  return result_cmd;
}

void DirectModeHandler::SendAsync(const Command &command) {
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

void DirectModeHandler::Send(const Command &command) {
  SendInternal(command);
  commands_[command.packet.seq_num] = make_pair(command, steady_clock::now() + std::chrono::milliseconds(command.time_out));
  Command &cmd = commands_[command.packet.seq_num].first;
  if (cmd.packet.data != NULL) {
    delete[] cmd.packet.data;
    cmd.packet.data = NULL;
    cmd.packet.data_len = 0;
  }
}

void DirectModeHandler::SendInternal(const Command &command) {
  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(53000);
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");

  int byte_send = sendto(sock_, (const char*)buf.data(), size, 0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));
  if (byte_send < 0) {
    //LOG_INFO("DirectModeHandler Send Internal Failed");
    if (command.cb) {
      (*command.cb)(kStatusSendFailed, command.handle, NULL);
    }
  }
}

void DirectModeHandler::OnStatusInfoCallback(socket_t sock, void * data) {
  if (status_map_.find(sock) == status_map_.end()) {
    return;
  }
  uint8_t slot = status_map_[sock];
  auto cb = std::get<1>(status_data_cb_[slot]);
  if (cb) {
    cb(slot, (LidarStatusInfo*)data, std::get<2>(status_data_cb_[slot]));
  }
  return;
}

void DirectModeHandler::OnBroadcastAck(const CommPacket & pack) {
  if (!pack.data) {
    return;
  }
  auto data = static_cast<LidarBroadcastInfo*>((void*)pack.data);
  // LOG_INFO("On Broadcast Info");
  // LOG_INFO("Slot: {}", data->slot);
  // LOG_INFO("Connect State: {}", data->connected_state);
  // LOG_INFO("Broadcastcode: {}", std::string((char *)data->broadcast_code, 16));
  // uint8_t unicast_ip[4] = {};
  // memcpy(unicast_ip, (void *)&data->unicast_ip_addr, 4);
  // LOG_INFO("Unicast IP Addr: {}.{}.{}.{}", unicast_ip[0],unicast_ip[1],unicast_ip[2],unicast_ip[3]);
  // LOG_INFO("Unicast Data Port: {}", data->unicast_data_port);
  // LOG_INFO("Unicast IMU Port: {}", data->unicast_imu_port);
  // LOG_INFO("Unicast Status Port: {}", data->unicast_status_port);
  // LOG_INFO("-------------------------------------------");
  uint8_t slot = data->slot;
  lidars_info_[slot] = *data;
  memcpy(lidars_info_[slot].broadcast_code, data->broadcast_code, kBroadcastCodeSize);
  CheckAndCreateChannel(slot);
}

void DirectModeHandler::CheckAndCreateChannel(uint8_t slot) {
  if (point_data_cb_.find(slot) != point_data_cb_.end()) {
    if (std::get<0>(point_data_cb_[slot]) == false) {
      std::get<0>(point_data_cb_[slot]) = true;
      CreatePointChannel(slot);
    }
  }
  if (imu_data_cb_.find(slot) != imu_data_cb_.end()) {
    if (std::get<0>(imu_data_cb_[slot]) == false) {
      std::get<0>(imu_data_cb_[slot]) = true;
      CreateImuChannel(slot);
    }
  }
  if (status_data_cb_.find(slot) != status_data_cb_.end()) {
    if (std::get<0>(status_data_cb_[slot]) == false) {
      std::get<0>(status_data_cb_[slot]) = true;
      CreateStatusChannel(slot);
    }
  }
}

void DirectModeHandler::CreatePointChannel(uint8_t slot) {
  uint16_t port = lidars_info_[slot].unicast_data_port;
  socket_t sock = util::CreateSocket(port);
  if (sock < 0) {
    return;
  }
  if (!loop_.expired()) {
    loop_.lock()->AddDelegate(sock, this);
  }
  point_map_[sock] = slot;
}

void DirectModeHandler::CreateImuChannel(uint8_t slot) {
  uint16_t port = lidars_info_[slot].unicast_imu_port;
  socket_t sock = util::CreateSocket(port);
  if (sock < 0) {
    return;
  }
  if (!loop_.expired()) {
    loop_.lock()->AddDelegate(sock, this);
  }
  imu_map_[sock] = slot;
}

void DirectModeHandler::CreateStatusChannel(uint8_t slot) {
  uint16_t port = lidars_info_[slot].unicast_status_port;
  socket_t sock = util::CreateSocket(port);
  if (sock < 0) {
    return;
  }
  if (!loop_.expired()) {
    loop_.lock()->AddDelegate(sock, this);
  }
  status_map_[sock] = slot;
}

void DirectModeHandler::LidarSearchBroadcastRequest() {
  Command command(0,
                  kCommandTypeCmd,
                  kCommandSetLidar,
                  kCommandIDLidarBroadcast,
                  CommandChannel::GenerateSeq(),
                  NULL,
                  0,
                  0,
                  std::shared_ptr<CommandCallback>());
  SendInternal(command);
}

void DirectModeHandler::SetPointDataCallback(uint8_t slot, PointDataCallback cb, void *client_data) {
  point_data_cb_[slot] = std::make_tuple(false, cb, client_data);
}

void DirectModeHandler::SetImuDataCallback(uint8_t slot, ImuDataCallback cb, void *client_data) {
  imu_data_cb_[slot] = std::make_tuple(false, cb, client_data);
}

void DirectModeHandler::SetStatusDataCallback(uint8_t slot, StatusDataCallback cb, void *client_data) {
  status_data_cb_[slot] = std::make_tuple(false, cb, client_data);
}

DirectModeHandler &direct_mode_handler() {
  static DirectModeHandler direct_mode;
  return direct_mode;
}

}  // namespace livox
