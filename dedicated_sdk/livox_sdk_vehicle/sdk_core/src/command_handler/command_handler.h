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

#ifndef LIVOX_COMMAND_HANDLER_H_
#define LIVOX_COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include "base/command_callback.h"
#include "base/io_thread.h"
#include "base/network/network_util.h"
#include "command_channel.h"
#include "livox_sdk_vehicle.h"
#include "livox_def_vehicle.h"

namespace livox {
namespace vehicle {

class CommandHandler : public noncopyable, public CommandChannelDelegate {
 public:
  using ExceptionDetailCallback = std::function<void(uint8_t, LivoxDetailExceptionInfo* , void*)>;
  using ExceptionInfoCallback = std::function<void(uint8_t, LidarStatusCode*, void*)>;
  using LidarInfoChangeCallback = std::function<void(VehicleDeviceInfo*, void*)>;
  using LidarLogCallback = std::function<void(uint8_t, LidarLogInfo*, void*)>;
  using LidarSafetyInfoCallback = std::function<void(uint8_t, LidarSafetyInfo*, void*)>;

  explicit CommandHandler() {}

  bool Init(const std::string& net_if);
  void Uninit();
  void AddDevice(const std::vector<LidarRegisterInfo>& lidars_info);
  void SetExceptionDetailCallback(ExceptionDetailCallback cb, void* client_data) { 
    exception_detail_callback_ = cb;
    exception_detail_client_data_ = client_data;
  }
  void SetExceptionInfoCallback(ExceptionInfoCallback cb, void* client_data) {
    exception_info_callback_ = cb;
    exception_info_client_data_ = client_data;
  }
  void SetLidarInfoChangeCallback(LidarInfoChangeCallback cb, void* client_data) {
    info_callback_ = cb;
    info_client_data_ = client_data;
  }
  void SetLidarLogCallback(LidarLogCallback cb, void* client_data) {
    log_callback_ = cb;
    log_client_data_ = client_data;
  }
  void SetLidarSafetyInfoCallback(LidarSafetyInfoCallback cb, void* client_data) {
    safety_callback_ = cb;
    safety_client_data_ = client_data;
  }

  livox_vehicle_status SendCommand(uint8_t slot,
                           uint8_t command_id,
                           uint8_t *data,
                           uint16_t length,
                           const std::shared_ptr<CommandCallback> &cb);

  void OnCommand(uint8_t slot, const Command &command) override;

 private:
  void SendCommandAck(uint8_t* req, uint16_t length, const Command& command);
  std::string GetRemoteIp(uint8_t slot);
  void OnCommandAck(uint8_t slot, const Command &command);
  void OnCommandCmd(uint8_t slot, const Command &command);
  void QueryDiagnosisInfoCallback(livox_vehicle_status status, uint8_t slot, LivoxDetailExceptionInfo* info);
  void QueryDiagnosisInfo(uint8_t slot);
  void OnLidarInfoChange(const Command &command);

 private:
  std::shared_ptr<CommandChannel> channel_;
  std::map<uint8_t, std::string> slot_ip_;
  std::map<uint8_t, VehicleDeviceInfo> lidar_info_;
  socket_t sock_;
  std::shared_ptr<IOThread> thread_;

  ExceptionDetailCallback exception_detail_callback_;
  void* exception_detail_client_data_ = nullptr;

  ExceptionInfoCallback exception_info_callback_;
  void* exception_info_client_data_ = nullptr;

  LidarInfoChangeCallback info_callback_;
  void* info_client_data_ = nullptr;

  LidarLogCallback log_callback_;
  void* log_client_data_ = nullptr;

  LidarSafetyInfoCallback safety_callback_;
  void* safety_client_data_ = nullptr;
};

CommandHandler &command_handler();

}
}  // namespace livox

#endif  // LIVOX_COMMAND_HANDLER_H_
