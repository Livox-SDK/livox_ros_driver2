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

#ifndef LIVOX_DEVICE_MANAGER_H_
#define LIVOX_DEVICE_MANAGER_H_

#include "livox_lidar_def.h"

#include "comm/define.h"
#include "comm/comm_port.h"
#include "base/io_thread.h"
#include "base/network/network_util.h"

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <map>
#include <mutex>
#include <string.h>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

namespace livox {
namespace lidar {

static const size_t kMaxBufferSize = 8192;
const uint8_t kSdkVer = 3;

class Protector {};

struct Command {
  CommPacket packet;
  uint32_t handle;
  std::string lidar_ip;
  uint32_t time_out;
  std::shared_ptr<CommandCallback> cb;
  Command() : packet() {}
  Command (
           uint32_t seq_num, 
           uint16_t cmd_id,
           uint8_t cmd_type,
           uint8_t sender_type,
           uint8_t* data,
           uint16_t data_len,
           uint32_t handle = 0,
           std::string lidar_ip = "",
           std::shared_ptr<CommandCallback> cb = nullptr)
      : packet(), handle(handle), lidar_ip(lidar_ip), time_out(KDefaultTimeOut), cb(cb) {
    memset(&packet, 0, sizeof(packet));
    packet.protocol = kLidarSdk;
    packet.version = kSdkVer;
    packet.seq_num = seq_num;
    packet.cmd_id = cmd_id;
    packet.cmd_type = cmd_type;
    packet.sender_type = sender_type;
    packet.data = data;
    packet.data_len = data_len;
  }
};

class DeviceManager : public IOLoop::IOLoopDelegate {
 private:
  DeviceManager();
  DeviceManager(const DeviceManager& other) = delete;
  DeviceManager& operator=(const DeviceManager& other) = delete;
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;
  void Destory();
  ~DeviceManager();
  static DeviceManager& GetInstance();

  bool Init(const std::string& host_ip);

  bool Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
      std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);

  void HandleDetectionData(uint32_t handle, DetectionData* detection_data);

  int SendTo(const uint8_t dev_type, const uint32_t handle, const std::vector<uint8_t>& buf, 
    const int16_t size, const struct sockaddr *addr, socklen_t addrlen);
  
  static void GetLivoxLidarInternalInfoCallback(livox_status status, uint32_t handle,
        LivoxLidarDiagInternalInfoResponse* response, void* client_data);
  void UpdateViewLidarCfgCallback(const uint32_t handle);

  void OnData(socket_t sock, void *);
  void OnTimer(TimePoint now);
 private:
  void GetLivoxLidarInternalInfo(const uint32_t handle);
  void AddViewLidar(const uint32_t handle, LivoxLidarDiagInternalInfoResponse* response);
  void CreateViewDataChannel(const ViewLidarIpInfo& view_lidar_info);

  void GenerateDevTypeTable();
  void InitDevTypeTable(const LivoxLidarCfg& lidar_cfg);

  bool CreateIOThread();
  bool CreateDetectionIOThread();
  bool CreateCommandIOThread();
  bool CreateDataIOThread();

  bool CreateChannel();
  bool CreateDetectionChannel();
  bool CreateDataChannel(const HostNetInfo& host_net_info);
  bool CreateCommandChannel(const uint8_t dev_type, const HostNetInfo& host_net_info, bool is_custom);
  bool CreateCmdSocketAndAddDelegate(const uint8_t dev_type, const std::string& host_ip, const uint16_t port, bool is_custom);
  bool CreateDataSocketAndAddDelegate(const std::string& host_ip, const uint16_t port);

  void DetectionLidars();
  void Detection();

  uint8_t GetDeviceType(const uint32_t handle);
  void IsLidarData(const uint32_t handle, const uint16_t lidar_port, uint8_t& dev_type);
 private:
  bool GetCmdChannel(const uint8_t dev_type, const uint32_t handle, socket_t& sock);
 private:
  std::shared_ptr<std::vector<LivoxLidarCfg>> lidars_cfg_ptr_;
  std::shared_ptr<std::vector<LivoxLidarCfg>> custom_lidars_cfg_ptr_;

  std::map<uint16_t, std::set<uint16_t>> lidars_cmd_port_;
  std::map<uint16_t, std::set<uint16_t>> lidars_data_port_;

  std::set<uint32_t> custom_lidars_;
  std::map<uint32_t, LivoxLidarCfg> custom_lidars_cfg_map_;

  socket_t detection_socket_;
  socket_t detection_broadcast_socket_;

  //socket_t detection_socket2_;

  std::vector<socket_t> socket_vec_;
  std::mutex mutex_cmd_channel_;

  std::map<std::string, socket_t> channel_info_;
  std::map<uint8_t, socket_t> general_command_channel_;
  std::map<std::string, socket_t> custom_command_channel_;

  std::set<socket_t> command_channel_;
  std::set<socket_t> data_channel_;
  std::vector<socket_t> vec_broadcast_socket_;

  std::shared_ptr<IOThread> cmd_io_thread_;
  std::shared_ptr<IOThread> data_io_thread_;
  std::shared_ptr<IOThread> detection_io_thread_;

  std::unique_ptr<CommPort> comm_port_;

  std::atomic<bool> is_stop_detection_{false};
  std::shared_ptr<std::thread> detection_thread_;

  std::mutex lidars_dev_type_mutex_;
  std::map<uint32_t, uint16_t> lidars_dev_type_;

  bool is_view_;
  std::string host_ip_;
  
  std::mutex view_device_mutex_;
  std::map<uint32_t, ViewDevice> view_devices_;

  std::mutex view_lidars_info_mutex_;
  std::map<uint32_t, std::shared_ptr<ViewLidarIpInfo>> view_lidars_info_;
};

} // namespace lidar
} // namespace livox

#endif // LIVOX_DEVICE_MANAGER_H_

