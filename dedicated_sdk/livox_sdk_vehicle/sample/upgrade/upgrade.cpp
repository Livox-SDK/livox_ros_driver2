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

#include "upgrade.h"

namespace livox {

typedef int32_t (Upgrade::*FnFsmEvent)();

typedef struct {
  uint32_t state;
  uint32_t event;
  FnFsmEvent event_handler;
  uint32_t next_state;
} FsmEventTable;

const FsmEventTable upgrade_event_table[] = {
    {kUpgradeIdle, kEventRequestUpgrade, &Upgrade::StartUpgrade,
     kUpgradeRequest},
    {kUpgradeRequest, kEventRequestUpgrade, &Upgrade::StartUpgrade,
     kUpgradeXferFirmware},
    {kUpgradeRequest, kEventXferFirmware, &Upgrade::XferFirmware,
     kUpgradeXferFirmware},
    {kUpgradeXferFirmware, kEventXferFirmware, &Upgrade::XferFirmware,
     kUpgradeXferFirmware},
    {kUpgradeXferFirmware, kEventCompleteXferFirmware,
     &Upgrade::CompleteXferFirmware, kUpgradeCompleteXferFirmware},
    {kUpgradeCompleteXferFirmware, kEventGetUpgradeProgress,
     &Upgrade::GetUpgradeProgress, kUpgradeGetUpgradeProgress},
    {kUpgradeGetUpgradeProgress, kEventGetUpgradeProgress,
     &Upgrade::GetUpgradeProgress, kUpgradeGetUpgradeProgress},
    {kUpgradeGetUpgradeProgress, kEventComplete, &Upgrade::UpgradeComplete,
     kUpgradeComplete},
    {kUpgradeComplete, kEventReinit, nullptr, kUpgradeIdle}
    // {kUpgradeRebootDevice, kEventReinit, nullptr, kUpgradeIdle},
};

Upgrade::Upgrade(std::shared_ptr<Firmware>& firmware)
    : firmware_(firmware),
      read_offset_(0),
      read_length_(1024),
      slot_(0),
      fsm_state_(0),
      upgrade_error_(0),
      progress_(0),
      try_count_(0) {}

livox_vehicle_status Upgrade::StartUpgrade() {
  read_offset_ = 0;
  // fsm_state_   = 0;
  upgrade_error_ = 0;
  progress_ = 0;

  StartUpgradeRequest request = {0};
  request.firmware_type = firmware_->header_.firmware_type;
  request.firmware_length = firmware_->header_.firmware_length;
  request.encrypt_type = firmware_->header_.encrypt_type;
  request.dev_type = firmware_->header_.device_type;
  printf("Device type [%d]\r\n", request.dev_type);
  return LidarStartUpgrade(slot_, (uint8_t *)&request, sizeof(request),
      StartUpgradeResponseHandler, this);
}

livox_vehicle_status Upgrade::XferFirmware() {
  uint8_t send_bufer[2048];
  XferFirmwareResquest* request = (XferFirmwareResquest*)send_bufer;
  uint32_t firmware_length = firmware_->header_.firmware_length;
  uint32_t read_length = read_length_;

  if (read_offset_ < firmware_length) {
    if (read_length > (firmware_length - read_offset_)) {
      read_length = firmware_length - read_offset_;
    }
  } else {
    printf("Read_offset is err, firmware_length[%d]\r\n", firmware_length);
    return -1;
  }

  memcpy(request->data, &firmware_->data_[read_offset_], read_length);
  request->offset = read_offset_;
  request->length = read_length;
  request->encrypt_type = firmware_->header_.encrypt_type;
  read_offset_ += read_length;
  printf("read offset %d\r\n", request->offset);
  return LidarXferFirmware(slot_, (uint8_t *)request,
      sizeof(XferFirmwareResquest) + read_length - sizeof(request->data),
      XferFirmwareResponseHandler, this);
}

livox_vehicle_status Upgrade::CompleteXferFirmware() {
  uint8_t send_bufer[2048];
  CompleteXferFirmwareResquest* request =
      (CompleteXferFirmwareResquest*)send_bufer;

  request->checksum_type = firmware_->header_.checksum_type;
  request->checksum_length = firmware_->header_.checksum_length;
  memcpy(request->checksum, firmware_->header_.checksum,
         request->checksum_length);

  return LidarCompleteXferFirmware(slot_, (uint8_t *)request,
      sizeof(CompleteXferFirmwareResquest) +
      request->checksum_length - sizeof(request->checksum),
      CompleteXferFirmwareResponseHandler, this);
}

livox_vehicle_status Upgrade::GetUpgradeProgress() {
  return LidarGetUpgradeProgress(slot_, nullptr, 0, GetProgressResponseHandler,
      this);
}

livox_vehicle_status Upgrade::UpgradeComplete() {
  return RequestReboot(slot_, UpgradeCompleteResponseHandler, this);
}

int32_t Upgrade::FsmStateChange(FsmEvent event) {
  if (event < kEventUndef) {
    fsm_state_ = event;
  }

  return 0;
}

void Upgrade::FsmEventHandler(FsmEvent event) {
  FnFsmEvent event_handler = nullptr;
  if ((event == kEventTimeout) || (event == kEventErr)) {
    FsmStateChange(event);
  }

  // printf("State[%d] | Event[%d]\r\n", fsm_state_, event);
  for (uint32_t i = 0;
      i < sizeof(upgrade_event_table) / sizeof(upgrade_event_table[0]); i++) {
    if ((fsm_state_ == upgrade_event_table[i].state) &&
        (event == upgrade_event_table[i].event)) {
      event_handler = upgrade_event_table[i].event_handler;
      fsm_state_ = upgrade_event_table[i].next_state;
      printf("New State[%d] | Event[%d]\r\n", fsm_state_, event);
      break;
    }
  }

  if (event_handler) {
    (this->*event_handler)();
  }
}

/**
 * Upgrade Callback handler
 */
void Upgrade::StartUpgradeResponseHandler(livox_vehicle_status status, uint8_t slot,
    StartUpgradeResponse* response, void* client_data) {
  Upgrade* upgrade = static_cast<Upgrade *>(client_data);

  if (status == kStatusSuccess) {
    upgrade->try_count_ = 0;
    printf("Start upgrade ret_code[%d]!\r\n", response->ret_code);
    if (response->ret_code) {
      if (response->ret_code == kSystemIsNotReady) {
        printf("Lidar is busy, try again!\r\n");
        upgrade->FsmEventHandler(kEventRequestUpgrade);
      } else {
        printf("Start upgrade fail[%d]!\r\n", response->ret_code);
        upgrade->FsmEventHandler(kEventErr);
      }
    } else {
      printf("Start to xfer data!\r\n");
      upgrade->FsmEventHandler(kEventXferFirmware);
    }
  } else {
    printf("Start upgrade is timeout[%d], try again!\r\n", upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventRequestUpgrade);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr);
      printf("Start upgrade exceed limit! exit!\r\n");
    }
  }
}

void Upgrade::XferFirmwareResponseHandler(livox_vehicle_status status, uint8_t slot,
      XferFirmwareResponse* response, void* client_data) {
  Upgrade* upgrade = static_cast<Upgrade *>(client_data);

  if (status == kStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Xfer firmware fail[%d]\r\n", response->ret_code);
      upgrade->FsmEventHandler(kEventErr);
    } else {
      if (upgrade->read_offset_ < upgrade->firmware_->header_.firmware_length) {
        upgrade->FsmEventHandler(kEventXferFirmware);
      } else {
        printf("Last offset[%d]\r\n", upgrade->read_offset_);
        upgrade->FsmEventHandler(kEventCompleteXferFirmware);
      }
    }
  } else {
    printf("Xfer firmware is timeout[%d], try again!\r\n", upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventXferFirmware);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr);
      printf("Xfer firmware exceed limit! exit!\r\n");
    }
  }
}

void Upgrade::CompleteXferFirmwareResponseHandler(livox_vehicle_status status, uint8_t slot,
      CompleteXferFirmwareResponse* response, void* client_data) {
  Upgrade* upgrade = static_cast<Upgrade *>(client_data);

  if (status == kStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Complete xfer firmware[%d]\r\n", response->ret_code);
      upgrade->FsmEventHandler(kEventErr);
    } else {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress);
    }
  } else {
    printf("Complete xfer is timeout[%d], try again!\r\n", upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
    upgrade->FsmEventHandler(kEventGetUpgradeProgress);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr);
      printf("Complete xfer exceed limit! exit!\r\n");
    }
  }
}

void Upgrade::GetProgressResponseHandler(livox_vehicle_status status, uint8_t slot,
      GetUpgradeProgressResponse* response, void* client_data) {
  Upgrade* upgrade = static_cast<Upgrade *>(client_data);

  if (status == kStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Get progress fail[%d]\r\n", response->ret_code);
      upgrade->FsmEventHandler(kEventErr);
    } else {
      if (response->progress < 100) {
        printf("Get progress[%d]\r\n", response->progress);
        upgrade->FsmEventHandler(kEventGetUpgradeProgress);
      } else {
        printf("Get progress[%d]\r\n", response->progress);
        upgrade->FsmEventHandler(kEventComplete);
      }
    }
  } else {
    printf("Get progress timeout[%d]! try again!\r\n", upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr);
      printf("Get progress exceed limit! exit!\r\n");
    }
  }
}

void Upgrade::UpgradeCompleteResponseHandler(livox_vehicle_status status, uint8_t slot,
      RebootResponse* response, void* client_data) {
  Upgrade* upgrade = static_cast<Upgrade *>(client_data);

  if (status == kStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Reboot device fail!\r\n");
      upgrade->FsmEventHandler(kEventErr);
    } else {
      upgrade->FsmEventHandler(kEventReinit);
    }
  } else {
    printf("Reboot device timeout[%d]! try again!\r\n", upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventReinit);
      printf("Reboot device exceed limit! exit!\r\n");
    }
  }
}

}
