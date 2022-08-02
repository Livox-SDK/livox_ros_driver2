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

#ifndef LIVOX_DIRECT_MODE_HANDLER
#define LIVOX_DIRECT_MODE_HANDLER

#include <mutex>
#include <string>
#include <algorithm>
#include "base/io_thread.h"
#include "base/noncopyable.h"
#include "comm/comm_port.h"
#include "command_handler/command_channel.h"
#include <stdio.h>

namespace livox {

const uint16_t ANY_PORT = 0;
static const size_t kPrefixDataSize = 18;


class DirectModeHandler : public noncopyable, IOLoop::IOLoopDelegate {
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;

  using PointDataCallback = std::function<void(uint8_t slot, LivoxEthPacket *data, uint32_t data_num, void *client_data)> ;
  using ImuDataCallback = std::function<void(uint8_t slot, LivoxEthPacket *data, uint32_t data_num, void *client_data)> ;
  using StatusInfoCallback = std::function<void(uint8_t slot, LidarStatusInfo* status_info, void *client_data)> ;

  bool Init(std::weak_ptr<IOLoop> loop);
  void Uninit();

  void SetPointDataCallback(uint8_t slot, PointDataCallback cb, void *client_data);
  void SetImuDataCallback(uint8_t slot, ImuDataCallback cb, void *client_data);
  void SetStatusDataCallback(uint8_t slot, StatusDataCallback cb, void *client_data);

  void OnData(socket_t, void *client_data);
  void OnTimer(TimePoint now);

  livox_status SwitchToNormalMode(uint8_t slot, SwithToNormalModeCallback cb, void *client_data);

 private:
  Command DeepCopy(const Command &cmd);
  void SendAsync(const Command &command);
  void Send(const Command &command);
  void SendInternal(const Command &command);

  void OnBroadcastAck(const CommPacket & pack);
  void CheckAndCreateChannel(uint8_t slot);
  void CreatePointChannel(uint8_t slot);

  void CreateImuChannel(uint8_t slot);
  void CreateStatusChannel(uint8_t slot);
  void LidarSearchBroadcastRequest();

  void OnPacketData(socket_t sock);
  void OnCmdData(socket_t sock);
  void OnStatusInfoCallback(socket_t sock, void * data);

  uint16_t GetDataSize(uint16_t size, uint8_t data_type);
  uint16_t GetCommandTimeout(uint8_t command_set, uint8_t command_id);

 private:
  TimePoint last_heartbeat_;
  socket_t sock_ = -1;
  std::weak_ptr<IOLoop> loop_;
  std::unique_ptr<CommPort> comm_port_ = nullptr;
  std::map<uint16_t, std::pair<Command, TimePoint> > commands_;

  using PointCallbackMap = std::tuple<bool, PointDataCallback, void*>;
  using ImuCallbackMap = std::tuple<bool, ImuDataCallback, void*>;
  using StateCallbackMap = std::tuple<bool, StatusInfoCallback, void*>;

  std::map<uint8_t, LidarBroadcastInfo> lidars_info_;
  std::map<uint8_t, PointCallbackMap> point_data_cb_;
  std::map<uint8_t, ImuCallbackMap> imu_data_cb_;
  std::map<uint8_t, StateCallbackMap> status_data_cb_;

  std::map<socket_t, uint8_t> point_map_;
  std::map<socket_t, uint8_t> imu_map_;
  std::map<socket_t, uint8_t> status_map_;

  static const size_t kMaxBufferSize = 8192;
  std::unique_ptr<char[]> data_buffers_;

  using SharedProtecotr = std::shared_ptr<Protector>;
  using WeakProtector = std::weak_ptr<Protector>;
  SharedProtecotr protector_ = std::make_shared<Protector>();
};

DirectModeHandler &direct_mode_handler();

}  // namespace livox

#endif  // LIVOX_DIRECT_MODE_HANDLER
