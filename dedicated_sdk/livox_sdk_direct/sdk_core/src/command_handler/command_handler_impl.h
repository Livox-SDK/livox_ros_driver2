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


#ifndef LIVOX_COMMAND_HANDLER_IMPL_H_
#define LIVOX_COMMAND_HANDLER_IMPL_H_

#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <ws2def.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#endif // WIN32

#include <memory>
#include <set>
#include <array>
#include <chrono>
#include <string.h>

#include "base/io_thread.h"
#include "comm/define.h"
#include "comm/comm_port.h"

#include "livox_def_direct.h"

#include "command_handler.h"

namespace livox {

namespace direct {

class CommandHandler;

const uint8_t kSdkVer = 3;
static const uint8_t kMaxConnectedDeviceNum = 32;

struct Command {
  CommPacket packet;
  uint32_t slot_;
  std::string lidar_ip_;
  uint32_t time_out_;
  std::shared_ptr<CommandCallback> cb_;
  Command() : packet() {}
  Command (uint32_t seq_num, 
           uint16_t cmd_id,
           uint8_t msg_type,
           uint8_t lidar_id,
           const std::string& sn,
           uint8_t* data,
           uint16_t data_len,
           uint32_t slot = 0,
           std::string lidar_ip = "",
           uint16_t lidar_cmd_port = 0,
           std::shared_ptr<CommandCallback> cb = nullptr)
      : packet(), slot_(slot), lidar_ip_(lidar_ip), time_out_(KDefaultTimeOut), cb_(cb) {
    memset(&packet, 0, sizeof(packet));
    packet.protocol = kLidarSdk;
    packet.version = kSdkVer;
    packet.seq_num = seq_num;
    packet.cmd_id = cmd_id;
    packet.msg_type = msg_type;
    packet.lidar_id = lidar_id;
    strcpy(packet.sn, sn.c_str());
    packet.data = data;
    packet.data_len = data_len;
  }
};

class CommandHandlerImpl : public IOLoop::IOLoopDelegate{
 public:
  CommandHandlerImpl(CommandHandler* handler);
  ~CommandHandlerImpl();
  bool Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
      std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr,
      const uint8_t lidars_num);
  bool Start();
  void OnCommand(uint8_t handle, const Command &command);
  void OnData(socket_t sock, void *);
  void OnTimer(TimePoint now);
  livox_direct_status UpdateDirectLidarCfg(std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr);
  void SendAsync(const Command &command);
 private:
  bool CreateIOThread();
  bool CreateCommandChannel();
  bool CreateChannel(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr);
  bool CreateSocketAndAddDelegate(const std::string& host_ip, const uint16_t port);

  void SetUpLidars();
  void BroadcastHostInfo();
  bool SetUpLidar(const DirectLidarInfo& direct_lidar_info, bool is_update);
  
  int SendTo(const std::string& host_ip, const uint16_t host_cmd_port, 
      const std::vector<uint8_t>& buf, const int16_t size, const struct sockaddr *addr, socklen_t addrlen);

  bool GetCmdChannel(const std::string& ip, const uint16_t port, socket_t& sock);

  void OnCommandAck(const uint32_t handle, const CommPacket& packet);
  void OnCommand(const uint32_t handle, const CommPacket& packet);
  bool ParseLidarStateInfo(const CommPacket& packet, DirectLidarStateInfo& info);
  void StopBroadcast();

  void SendInternal(const Command &command);

  bool IsResetDevice(const Command& command);
  bool IsExist(const uint32_t handle);

 private:
  CommandHandler *handler_;
  std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr_;
  std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr_;
  uint8_t lidars_num_;
  std::atomic<bool> is_quite_{false};
  std::shared_ptr<std::thread> broadcast_thread_;
  std::mutex mutex_lidars_;
  std::set<uint32_t> direct_lidars_;
  //std::array<DirectLidarStateInfo, kMaxConnectedDeviceNum> direct_lidars_;

  std::shared_ptr<IOThread> io_thread_ptr;

  std::mutex mutex_channel_;
  std::map<std::string, socket_t> devices_channel_;

  
  std::map<uint16_t, std::pair<Command, TimePoint> > commands_;
  TimePoint last_heartbeat_ = TimePoint();

  std::unique_ptr<CommPort> comm_port_;
};

}  // namespace livox
} // namespace direct

#endif  // LIVOX_COMMAND_HANDLER_IMPL_H_









