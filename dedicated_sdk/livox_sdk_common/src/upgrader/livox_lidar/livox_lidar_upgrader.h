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

#ifndef LIVOX_LIDAR_UPGRADER_H_
#define LIVOX_LIDAR_UPGRADER_H_

#include <fstream>
#include <ios>
#include <memory>
#include <thread>
#include <functional>

#include <livox_def_common.h>
#include <livox_lidar_def.h>
#include <livox_lidar_api.h>

#include "../firmware/firmware.h"

namespace livox {
namespace common {

class LivoxLidarUpgrader {
 public:  
  using UpgradeProgressCallback = std::function<void(uint32_t handle, LidarUpgradeState state)>;

  LivoxLidarUpgrader(const Firmware& firmware, const uint32_t handle);
  ~LivoxLidarUpgrader();

  void AddUpgradeProgressObserver(UpgradeProgressCallback observer);
  bool StartUpgradeLivoxLidar();

  livox_status StartUpgrade();
  livox_status XferFirmware();
  livox_status CompleteXferFirmware();
  livox_status GetUpgradeProgress();
  livox_status UpgradeComplete();

  static void StartUpgradeResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarStartUpgradeResponse* response, void* client_data);
  static void XferFirmwareResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarXferFirmwareResponse* response, void* client_data);
  static void CompleteXferFirmwareResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarCompleteXferFirmwareResponse* response, void* client_data);
  static void GetProgressResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarGetUpgradeProgressResponse* response, void* client_data);
  static void UpgradeCompleteResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarRebootResponse* response, void* client_data);

  int32_t FsmStateChange(FsmEvent event);
  void FsmEventHandler(FsmEvent event, uint8_t progress);

  bool IsUpgradeComplete() { return (fsm_state_ == kUpgradeIdle); }
  bool IsUpgradeError() { return (fsm_state_ == kUpgradeTimeout) || (fsm_state_ == kUpgradeErr); }

 private:
  const Firmware& firmware_;
  uint32_t read_offset_;
  uint32_t read_length_;
  uint32_t handle_;
  uint32_t fsm_state_;

  uint32_t upgrade_error_;
  uint8_t progress_;

  uint32_t try_count_;
  std::shared_ptr<std::thread> upgrade_thread_;
  UpgradeProgressCallback observer_;

};

} // namespace comm
} // namespace LIVOX_LIDAR_UPGRADER_H_

#endif
