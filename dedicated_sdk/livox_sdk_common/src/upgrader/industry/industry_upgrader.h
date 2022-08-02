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

#ifndef LIVOX_INDUSTRY_UPGRADER_H_
#define LIVOX_INDUSTRY_UPGRADER_H_

#include <livox_def_common.h>
#include <livox_def.h>
#include <livox_sdk.h>
#include <functional>
#include <memory>
#include <fstream>
#include <vector>
#include <string>

#include "../firmware/firmware.h"
#include <thread>

namespace livox {
namespace common {

class IndustryUpgrader {
public:
  using UpgradeProgressCallback = std::function<void(uint8_t slot, LidarUpgradeState state)>;
  IndustryUpgrader() =  default;
  ~IndustryUpgrader() = default;
  void AddUpgradeProgressObserver(std::function<void(uint8_t slot, LidarUpgradeState state)> observer);
  void StopUpgradeLidar(uint8_t handle);
  void StartUpgradeLidar(uint8_t handle, std::shared_ptr<Firmware> firmware);

  static void StartUpgradeResponseHandler(livox_status status, uint8_t slot,
      StartUpgradeResponse* response, void* client_data);
  static void XferFirmwareResponseHandler(livox_status status, uint8_t slot,
      XferFirmwareResponse* response, void* client_data);
  static void CompleteXferFirmwareResponseHandler(livox_status status, uint8_t slot,
      CompleteXferFirmwareResponse* response, void* client_data);
  static void GetProgressResponseHandler(livox_status status, uint8_t slot,
      GetUpgradeProgressResponse* response, void* client_data);
  static void UpgradeCompleteResponseHandler(livox_status status, uint8_t slot,
      uint8_t response, void* client_data);

  livox_status StartUpgrade();
  livox_status XferFirmware();
  livox_status CompleteXferFirmware();
  livox_status GetUpgradeProgress();
  livox_status UpgradeComplete();
  int32_t FsmStateChange(FsmEvent event);
  void FsmEventHandler(FsmEvent event, uint8_t progress);
  bool IsUpgradeComplete() { return (fsm_state_ == kUpgradeIdle); }
  bool IsUpgradeError() {
    return (fsm_state_ == kUpgradeTimeout) || (fsm_state_ == kUpgradeErr);
  }

 private:
  std::shared_ptr<Firmware> firmware_;

  uint8_t slot_ = 0;
  uint32_t read_offset_ = 0;
  uint32_t read_length_ = 1024;
  uint32_t fsm_state_ = 0;

  uint32_t upgrade_error_ = 0;
  uint8_t progress_ = 0;

  uint32_t try_count_;

  std::shared_ptr<std::thread> upgrade_thread_;
  UpgradeProgressCallback observer_;
};

}
}  // namespace livox

#endif  // LIVOX_INDUSTRY_UPGRADER_H_
