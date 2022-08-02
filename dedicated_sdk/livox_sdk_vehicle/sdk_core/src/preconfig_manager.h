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

#ifndef LIVOX_PRECONFIG_MANAGER_H_
#define LIVOX_PRECONFIG_MANAGER_H_

#include <functional>
#include <string>
#include <string.h>
#include <set>
#include "preconfig_manager.h"
#include "base/io_thread.h"
#include "base/noncopyable.h"
#include "../include/comm/protocol.h"
#include "../include/comm/comm_port.h"
#include "livox_sdk_vehicle.h"

namespace livox {
namespace vehicle {

class PreConfigManager : public noncopyable, IOLoop::IOLoopDelegate {
 public:
  using PreConfigCallback = std::function<void(livox_vehicle_status, const char*, void*)>;
  typedef std::chrono::steady_clock::time_point TimePoint;
  typedef struct {
    std::string device_ip;
    std::string broadcast_code;
  } PreConfigCommand;

  PreConfigManager() = default;
  ~PreConfigManager() = default;

  bool Init(const std::string& net_if, LidarPreConfigParam& param);
  void Uninit();

  void OnData(socket_t sock, void* client_data);
  void OnTimer(TimePoint now);

  void SetLidarPreConfigParamCallback(PreConfigCallback cb, void* client_data);

 private:
  void OnPreconfigAck(const CommPacket &packet);
  void OnBroadcastAck(const CommPacket &packet,  const std::string& device_ip);
  void LidarPreConfig(PreConfigCommand& command);
  void LidarSearchBroadcastRequest();

 private:
  socket_t sock_ = -1;
  std::shared_ptr<IOThread> thread_;
  std::weak_ptr<IOLoop> loop_;
  std::unique_ptr<CommPort> comm_port_;

  LidarPreConfigParam param_;
  void* client_data_ = nullptr;
  PreConfigCallback preconfig_call_back_;
  std::set<std::string> lidar_info_;
  std::map<uint16_t, std::pair<PreConfigCommand, TimePoint> > commands_;
  TimePoint last_heartbeat_;

};

PreConfigManager &preconfig_manager();

}
}  // namespace livox

#endif  // LIVOX_DEVICE_MANAGER_H_
