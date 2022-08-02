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

#ifndef COMMAND_HANDLER_H_
#define COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/define.h"

#include "livox_lidar_def.h"
#include "device_manager.h"

namespace livox {
namespace lidar {

class CommandHandler {
 public:
  CommandHandler(DeviceManager* device_manager) : device_manager_(device_manager) {}
  ~CommandHandler() {}

  virtual bool Init(bool is_view) = 0;
  virtual bool Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) = 0;

  virtual void Handle(const uint32_t handle, const Command& command) = 0;
  virtual void UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info) = 0;
  virtual void UpdateLidarCfg(const uint32_t handle, const uint16_t lidar_cmd_port) = 0;
  virtual livox_status SendCommand(const Command& command) = 0;
 protected:
  DeviceManager* device_manager_;
};

}  // namespace livox
} // namespace lidar

#endif  // COMMAND_HANDLER_H_
