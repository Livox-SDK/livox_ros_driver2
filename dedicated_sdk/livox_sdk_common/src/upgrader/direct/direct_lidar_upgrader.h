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

#ifndef LIVOX_DIRECT_LIDAR_UPGRADER_H_
#define LIVOX_DIRECT_LIDAR_UPGRADER_H_

#include <fstream>
#include <ios>
#include <memory>
#include <thread>
#include <functional>

#include <livox_def_common.h>
#include <livox_def_direct.h>
#include <livox_sdk_direct.h>

#include "../firmware/firmware.h"

namespace livox {
namespace common {

class DirectLidarUpgrader {
 public:  
  using DirectLidarUpgradeProgressCallback = std::function<void(uint32_t slot, LidarUpgradeState state)>;


  DirectLidarUpgrader(const Firmware& firmware, const uint32_t slot);
  ~DirectLidarUpgrader();

  void AddUpgradeProgressObserver(DirectLidarUpgradeProgressCallback observer);
  bool StartUpgradeDirectLidar();

  livox_direct_status StartUpgrade();
  livox_direct_status XferFirmware();
  livox_direct_status CompleteXferFirmware();
  livox_direct_status GetUpgradeProgress();
  livox_direct_status UpgradeComplete();

  static void StartUpgradeResponseHandler(livox_direct_status status, uint32_t slot,
      DirectStartUpgradeResponse* response, void* client_data);
  static void XferFirmwareResponseHandler(livox_direct_status status, uint32_t slot,
      DirectXferFirmwareResponse* response, void* client_data);
  static void CompleteXferFirmwareResponseHandler(livox_direct_status status, uint32_t slot,
      DirectCompleteXferFirmwareResponse* response, void* client_data);
  static void GetProgressResponseHandler(livox_direct_status status, uint32_t slot,
      DirectGetUpgradeProgressResponse* response, void* client_data);
  static void UpgradeCompleteResponseHandler(livox_direct_status status, uint32_t slot,
      DirectRebootResponse* response, void* client_data);

  int32_t FsmStateChange(FsmEvent event);
  void FsmEventHandler(FsmEvent event, uint8_t progress);

  bool IsUpgradeComplete() { return (fsm_state_ == kUpgradeIdle); }
  bool IsUpgradeError() { return (fsm_state_ == kUpgradeTimeout) || (fsm_state_ == kUpgradeErr); }

 private:
  const Firmware& firmware_;
  uint32_t read_offset_;
  uint32_t read_length_;
  uint32_t slot_;
  uint32_t fsm_state_;

  uint32_t upgrade_error_;
  uint8_t progress_;

  uint32_t try_count_;
  std::shared_ptr<std::thread> upgrade_thread_;
  DirectLidarUpgradeProgressCallback observer_;

};

} // namespace comm
} // namespace LIVOX_VEHICLE_LIDAR_UPGRADER_H_

#endif
