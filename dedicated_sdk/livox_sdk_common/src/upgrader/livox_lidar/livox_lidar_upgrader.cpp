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

#include <string.h>

#include "livox_lidar_upgrader.h"

namespace livox {
namespace common {

typedef int32_t (LivoxLidarUpgrader::*FnFsmEvent)();

typedef struct {
  uint32_t state;
  uint32_t event;
  FnFsmEvent event_handler;
  uint32_t next_state;
} FsmEventTable;

const FsmEventTable upgrade_event_table[] = {
  {kUpgradeIdle, kEventRequestUpgrade, &LivoxLidarUpgrader::StartUpgrade, kUpgradeRequest},
  {kUpgradeRequest, kEventRequestUpgrade, &LivoxLidarUpgrader::StartUpgrade, kUpgradeRequest},
  {kUpgradeRequest, kEventXferFirmware, &LivoxLidarUpgrader::XferFirmware, kUpgradeXferFirmware},
  {kUpgradeXferFirmware, kEventXferFirmware, &LivoxLidarUpgrader::XferFirmware, kUpgradeXferFirmware},
  {kUpgradeXferFirmware, kEventCompleteXferFirmware, &LivoxLidarUpgrader::CompleteXferFirmware, kUpgradeCompleteXferFirmware},
  {kUpgradeCompleteXferFirmware, kEventCompleteXferFirmware, &LivoxLidarUpgrader::CompleteXferFirmware, kUpgradeCompleteXferFirmware},
  {kUpgradeCompleteXferFirmware, kEventGetUpgradeProgress, &LivoxLidarUpgrader::GetUpgradeProgress, kUpgradeGetUpgradeProgress},
  {kUpgradeGetUpgradeProgress, kEventGetUpgradeProgress, &LivoxLidarUpgrader::GetUpgradeProgress, kUpgradeGetUpgradeProgress},
  {kUpgradeGetUpgradeProgress, kEventComplete, &LivoxLidarUpgrader::UpgradeComplete, kUpgradeComplete},
  {kUpgradeComplete, kEventComplete, &LivoxLidarUpgrader::UpgradeComplete, kUpgradeComplete},
  {kUpgradeComplete, kEventReinit, nullptr, kUpgradeIdle}
  // {kUpgradeRebootDevice, kEventReinit, nullptr, kUpgradeIdle},
};

LivoxLidarUpgrader::LivoxLidarUpgrader(const Firmware& firmware, const uint32_t handle)
    : firmware_(firmware), read_offset_(0), read_length_(1024), handle_(handle), fsm_state_(0),
      upgrade_error_(0), progress_(0), try_count_(0) {}

LivoxLidarUpgrader::~LivoxLidarUpgrader() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (IsUpgradeError()) {
      printf("LivoxLidar lidar[%d] upgrade error, try again please!\r\n", handle_);
      break;
    }

    if (IsUpgradeComplete()) {
      printf("LivoxLidar lidar[%d] upgrade successfully.\r\n", handle_);
      break;
    }
  }

  if (upgrade_thread_ && upgrade_thread_->joinable()) {
    upgrade_thread_->join();
    upgrade_thread_ = nullptr;
  }
}


void LivoxLidarUpgrader::AddUpgradeProgressObserver(UpgradeProgressCallback observer) {
    observer_ = observer;
    return;
}

bool LivoxLidarUpgrader::StartUpgradeLivoxLidar() {
  upgrade_thread_ = std::make_shared<std::thread>([this](){
      this->FsmEventHandler(kEventRequestUpgrade, 10);
  });
  return true;
}


void LivoxLidarUpgrader::FsmEventHandler(FsmEvent event, uint8_t progress) {
  FnFsmEvent event_handler = nullptr;
  if ((event == kEventTimeout) || (event == kEventErr)) {
    FsmStateChange(event);
  }
  printf("Fsm event handler, the livox_lidar[%d] State[%d] | Event[%d]\r\n", handle_, fsm_state_, event);
  for (uint32_t i = 0; i < sizeof(upgrade_event_table) / sizeof(upgrade_event_table[0]); i++) {
    if ((fsm_state_ == upgrade_event_table[i].state) && (event == upgrade_event_table[i].event)) {
      event_handler = upgrade_event_table[i].event_handler;
      fsm_state_ = upgrade_event_table[i].next_state;
      printf("Fsm event handler, the livox_lidar[%d] New State[%d] | Event[%d]\r\n", handle_, fsm_state_, event);
      break;
    }
  }

  if (event_handler) {
    (this->*event_handler)();
  }

  if (observer_) {
      LidarUpgradeState upgrade_state = {event, progress};
      observer_(handle_, upgrade_state);
  }
}

livox_status LivoxLidarUpgrader::StartUpgrade() {
  read_offset_ = 0;
  upgrade_error_ = 0;
  progress_ = 0;

  uint8_t request_buf[1024] = { 0 };
  LivoxLidarStartUpgradeRequest *request = (LivoxLidarStartUpgradeRequest *)request_buf;
  request->firmware_type = firmware_.header_.firmware_type;
  request->firmware_length = firmware_.header_.firmware_length;
  request->encrypt_type = firmware_.header_.encrypt_type;
  request->dev_type = firmware_.header_.device_type;
  printf("Start upgrade, the livox_lidar[%d] device type [%d]\r\n", handle_, request->dev_type);

  if (kEnlFileVersionV3 == firmware_.FirmwarePackageVersion()) {
    LivoxLidarStartUpgradeRequestV3 *request_v3 = (LivoxLidarStartUpgradeRequestV3 *)request_buf;
    request_v3->firmware_version = firmware_.header_.firmware_version;
    request_v3->firmware_buildtime = firmware_.header_.modify_time;
    memcpy(request_v3->hw_whitelist, firmware_.header_.hw_whitelist, sizeof(request_v3->hw_whitelist));
    return LivoxLidarStartUpgrade(handle_, request_buf, sizeof(LivoxLidarStartUpgradeRequestV3), StartUpgradeResponseHandler, this);
  } else {
    return LivoxLidarStartUpgrade(handle_, request_buf, sizeof(LivoxLidarStartUpgradeRequest), StartUpgradeResponseHandler, this);  
  }
}

livox_status LivoxLidarUpgrader::XferFirmware() {
  uint8_t send_bufer[2048];
  LivoxLidarXferFirmwareResquest* request = (LivoxLidarXferFirmwareResquest*)send_bufer;
  uint32_t firmware_length = firmware_.header_.firmware_length;
  uint32_t read_length = read_length_;

  if (read_offset_ < firmware_length) {
    if (read_length > (firmware_length - read_offset_)) {
      read_length = firmware_length - read_offset_;
    }
  } else {
    printf("The livox_lidar[%d] xfer firmware failed, Read_offset is err, firmware_length[%d], read_offset[%d].\r\n",
        handle_, firmware_length, read_offset_);
    return -1;
  }

  memcpy(request->data, &firmware_.data_[read_offset_], read_length);
  request->offset = read_offset_;
  request->length = read_length;
  request->encrypt_type = firmware_.header_.encrypt_type;
  // read_offset_ += read_length;
  printf("The livox_lidar[%d] xfer firmware read offset %d\r\n", handle_, request->offset);

  return LivoxLidarXferFirmware(handle_, (uint8_t *)request,
      sizeof(LivoxLidarXferFirmwareResquest) + read_length - sizeof(request->data),
      XferFirmwareResponseHandler, this);
}

livox_status LivoxLidarUpgrader::CompleteXferFirmware() {
  uint8_t send_bufer[2048];
  LivoxLidarCompleteXferFirmwareResquest* request = (LivoxLidarCompleteXferFirmwareResquest*)send_bufer;

  request->checksum_type = firmware_.header_.checksum_type;
  request->checksum_length = firmware_.header_.checksum_length;
  memcpy(request->checksum, firmware_.header_.checksum, request->checksum_length);

  return LivoxLidarCompleteXferFirmware(handle_, (uint8_t *)request,
      sizeof(LivoxLidarCompleteXferFirmwareResquest) +
      request->checksum_length - sizeof(request->checksum),
      CompleteXferFirmwareResponseHandler, this);
}

livox_status LivoxLidarUpgrader::GetUpgradeProgress() {
  return LivoxLidarGetUpgradeProgress(handle_, nullptr, 0, GetProgressResponseHandler, this);
}

livox_status LivoxLidarUpgrader::UpgradeComplete() {
  return LivoxLidarRequestReboot(handle_, UpgradeCompleteResponseHandler, this);
}

int32_t LivoxLidarUpgrader::FsmStateChange(FsmEvent event) {
  if (event < kEventUndef) {
    fsm_state_ = event;
  }
  return 0;
}

/**
 * Upgrade Callback handler
 */
void LivoxLidarUpgrader::StartUpgradeResponseHandler(livox_status status, uint32_t handle,
    LivoxLidarStartUpgradeResponse* response, void* client_data) {
  LivoxLidarUpgrader* upgrade = static_cast<LivoxLidarUpgrader *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    upgrade->try_count_ = 0;
    printf("Start upgrade the livox_lidar[%d], ret_code[%d]!\r\n", handle, response->ret_code);
    if (response->ret_code) {
      if (response->ret_code == kSystemIsNotReady) {
        printf("Start upgrade failed, the livox_lidar[%d] is busy, try again!\r\n", handle);
        upgrade->FsmEventHandler(kEventRequestUpgrade, 10);
      } else {
        printf("Start upgrade failed, the livox_lidar[%d] ret_code[%d], try again!\r\n", handle, response->ret_code);
        upgrade->FsmEventHandler(kEventErr, 100);
      }
    } else {
      printf("Start upgrade succ, the livox_lidar[%d] start to xfer data!\r\n", handle);
      upgrade->FsmEventHandler(kEventXferFirmware, 20);
    }
  } else {
    printf("Start upgrade failed, the livox_lidar[%d] start upgrade is timeout[%d], try again!\r\n", handle, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventRequestUpgrade, 10);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Start upgrade failed, the livox_lidar[%d] start upgrade exceed limit! exit!\r\n", handle);
    }
  }
}

void LivoxLidarUpgrader::XferFirmwareResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarXferFirmwareResponse* response, void* client_data) {
  LivoxLidarUpgrader* upgrade = static_cast<LivoxLidarUpgrader *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("The livox_lidar[%d] Xfer firmware fail[%d]\r\n", handle, response->ret_code);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      upgrade->read_offset_ += upgrade->read_length_;
      if (upgrade->read_offset_ < upgrade->firmware_.header_.firmware_length) {
        upgrade->FsmEventHandler(kEventXferFirmware, 20);
      } else {
        printf("Xfer firmware succ, the livox_lidar[%d] last offset[%d]\r\n", handle, upgrade->read_offset_);
        upgrade->FsmEventHandler(kEventCompleteXferFirmware, 40);
      }
    }
  } else {
    printf("Xfer firmware failed, the livox_lidar[%d] xfer firmware timeout, try_count:%d, try again!\r\n", handle, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventXferFirmware, 20);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Xfer firmware failed, the livox_lidar[%d] exceed limit! exit!\r\n", handle);
    }
  }
}

void LivoxLidarUpgrader::CompleteXferFirmwareResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarCompleteXferFirmwareResponse* response, void* client_data) {
  LivoxLidarUpgrader* upgrade = static_cast<LivoxLidarUpgrader *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Complete xfer failed, the livox_lidar[%d] ret_code:%d.\r\n", handle, response->ret_code);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      printf("The livox_lidar[%d] complete xfer succ.\n", handle);
      upgrade->FsmEventHandler(kEventGetUpgradeProgress, 50);
    }
  } else {
    printf("Complete xfer failed, the livox_lidar[%d] complete xfer is timeout, try_count:%d, try again!\r\n", handle, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
    upgrade->FsmEventHandler(kEventCompleteXferFirmware, 50);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Complete xfer failed, the livox_lidar[%d] complete xfer exceed limit! exit!\r\n", handle);
    }
  }
}

void LivoxLidarUpgrader::GetProgressResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarGetUpgradeProgressResponse* response, void* client_data) {
  LivoxLidarUpgrader* upgrade = static_cast<LivoxLidarUpgrader *>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Get progress failed, the livox_lidar[%d] ret_code:%d.\r\n", handle, response->ret_code);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      if (response->progress < 100) {
        printf("The livox_lidar[%d] get progress[%d]\r\n", handle, response->progress);
        upgrade->FsmEventHandler(kEventGetUpgradeProgress, response->progress/2 + 50);
      } else {
        printf("The livox_lidar[%d] Get progress[%d]\r\n", handle, response->progress);
        upgrade->FsmEventHandler(kEventComplete, 100);
      }
    }
  } else {
    //printf("Get progress failed, the livox_lidar[%d] get progress timeout, try_count:%d, try again!\r\n", handle, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGetProcessTryCountLimit) {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress, upgrade->progress_/2 + 50);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Get progress failed, the livox_lidar[%d] get progress exceed limit! exit!\r\n", handle);
    }
  }
}

void LivoxLidarUpgrader::UpgradeCompleteResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarRebootResponse* response, void* client_data) {
  LivoxLidarUpgrader* upgrade = static_cast<LivoxLidarUpgrader *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Upgrade complete failed, the livox_lidar[%d] reboot device fail!\r\n", handle);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      printf("The livox_lidar[%d] upgrade complete succ.\n", handle);
      upgrade->FsmEventHandler(kEventReinit, 100);
    }
  } else {
    printf("Upgrade complete failed, the livox_lidar[%d] reboot device timeout, try_count:%d, try again!\r\n",
        handle, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventComplete, 100);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventReinit, 100);
      printf("Upgrade complete failed, the livox_lidar[%d] reboot device exceed limit! exit!\r\n", handle);
    }
  }
}

} // namespace comm
} // namespace livox
