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

#ifndef HAP_COMMAND_HANDLER_H_
#define HAP_COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <list>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/comm_port.h"
#include "comm/define.h"

#include "livox_lidar_def.h"
#include "device_manager.h"

#include "command_handler.h"

namespace livox {
namespace lidar {  

class HapCommandHandler : public CommandHandler {
 public:
  HapCommandHandler(DeviceManager* device_manager);
  ~HapCommandHandler() {}
  static HapCommandHandler& GetInstance();
  virtual bool Init(bool is_view);

  virtual bool Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);
  virtual void Handle(const uint32_t handle, uint16_t lidar_port, const Command& command);
  virtual void UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info);
  virtual void UpdateLidarCfg(const uint32_t handle, const uint16_t lidar_cmd_port);
  virtual livox_status SendCommand(const Command& command);  
  static void UpdateLidarCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
  void AddDevice(const uint32_t handle);
 private:
  void SetCustomLidar(const uint32_t handle, const uint16_t lidar_cmd_port, const LivoxLidarCfg& lidar_cfg);
  void SetGeneralLidar(const uint32_t handle, const uint16_t lidar_cmd_port);
  void SetViewLidar(const ViewLidarIpInfo& view_lidar_info);
  livox_status SendCommand(const Command &command, const uint16_t lidar_cmd_port);

  
  bool GetHostInfo(const uint32_t handle, std::string& host_ip, uint16_t& cmd_port);

  void CommandsHandle(TimePoint now);

  void OnCommand(uint32_t handle, const Command &command);
  void OnCommandAck(uint32_t handle, const Command &command);
  void OnCommandCmd(uint32_t handle, const Command &command);
  bool IsStatusException(const Command &command);
  void QueryDiagnosisInfo(uint32_t handle);
  void OnLidarInfoChange(const Command &command);
 private:
  std::unique_ptr<CommPort> comm_port_;
  std::mutex device_mutex_;
  std::set<uint32_t> devices_;
  std::map<uint32_t, LivoxLidarCfg> lidars_custom_;
  LivoxLidarCfg general_lidar_cfg_;
  bool is_view_;
};

}  // namespace livox
} // namespace lidar

#endif  // HAP_COMMAND_HANDLER_H_
