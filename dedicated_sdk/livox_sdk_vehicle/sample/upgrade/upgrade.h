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

#ifndef UPGRADE_UPGRADE_H_
#define UPGRADE_UPGRADE_H_

#include <fstream>
#include <ios>
#include <memory>

#include "firmware.h"
#include "lidar.h"
#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"

namespace livox {

class Upgrade;

typedef enum {
  kUpgradeIdle = 0,
  kUpgradeRequest = 1,
  kUpgradeXferFirmware = 2,
  kUpgradeCompleteXferFirmware = 3,
  kUpgradeGetUpgradeProgress = 4,
  kUpgradeComplete = 5,
  kUpgradeTimeout = 6,
  kUpgradeErr = 7,
  kUpgradeUndef = 8
} FsmState;

typedef enum {
  kEventRequestUpgrade = 0,
  kEventXferFirmware = 1,
  kEventCompleteXferFirmware = 2,
  kEventGetUpgradeProgress = 3,
  kEventComplete = 4,
  kEventReinit = 5,
  kEventTimeout = 6,
  kEventErr = 7,
  kEventUndef = 8
} FsmEvent;

/** Lidar feature. */
typedef enum {
  kEverythingIsOk = 0,
  kFirmwareOutOfLength = 1,
  kSystemIsNotReady = 2,
  kFirmwareTypeMismatch = 3,
  kUpgradeStateMismatch = 4,
} RequestUpgradeReturnCode;

const uint32_t kGeneralTryCountLimit = 3;
const uint32_t kGetProgressTryCountLimit = 10;

#pragma pack(1)
/*
typedef struct {

} ; */

#pragma pack()

// typedef int32_t (*fn_fsm_state_service)(void* private_data);
// typedef int32_t (*fn_fsm_event_process)(void* private_data);
// typedef int32_t (*fn_get_upgrade_progress)(void* private_data);

class Upgrade {
 public:
  Upgrade(std::shared_ptr<Firmware> &firmware);
  ~Upgrade() = default;

  static void StartUpgradeResponseHandler(livox_vehicle_status status, uint8_t slot,
      StartUpgradeResponse* response, void* client_data);
  static void XferFirmwareResponseHandler(livox_vehicle_status status, uint8_t slot,
      XferFirmwareResponse* response, void* client_data);
  static void CompleteXferFirmwareResponseHandler(livox_vehicle_status status, uint8_t slot,
      CompleteXferFirmwareResponse* response, void* client_data);
  static void GetProgressResponseHandler(livox_vehicle_status status, uint8_t slot,
      GetUpgradeProgressResponse* response, void* client_data);
  static void UpgradeCompleteResponseHandler(livox_vehicle_status status, uint8_t slot,
      RebootResponse* response, void* client_data);

  livox_vehicle_status StartUpgrade();
  livox_vehicle_status XferFirmware();
  livox_vehicle_status CompleteXferFirmware();
  livox_vehicle_status GetUpgradeProgress();
  livox_vehicle_status UpgradeComplete();
  int32_t FsmStateChange(FsmEvent event);
  void FsmEventHandler(FsmEvent event);
  void SetLidar(std::shared_ptr<Lidar> &lidar) {
    lidar_ = lidar;
    slot_ = lidar_->dev_info_.slot;
  }
  bool IsUpgradeComplete() { return (fsm_state_ == kUpgradeIdle); }
  bool IsUpgradeError() {
    return (fsm_state_ == kUpgradeTimeout) || (fsm_state_ == kUpgradeErr);
  }

 private:
  std::shared_ptr<Firmware> firmware_;
  uint32_t read_offset_;
  uint32_t read_length_;
  std::shared_ptr<Lidar> lidar_;
  uint8_t slot_;
  uint32_t fsm_state_;

  uint32_t upgrade_error_;
  uint8_t progress_;

  uint32_t try_count_;
};
}

#endif
