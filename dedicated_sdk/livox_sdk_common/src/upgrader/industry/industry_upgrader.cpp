#include "industry_upgrader.h"
#include <chrono>
#include <thread>
#include <string.h>


namespace livox {
namespace common {

typedef int32_t (IndustryUpgrader::*FnFsmEvent)();

typedef struct {
  uint32_t state;
  uint32_t event;
  FnFsmEvent event_handler;
  uint32_t next_state;
} FsmEventTable;

const FsmEventTable upgrade_event_table[] = {
    {kUpgradeIdle, kEventRequestUpgrade, &IndustryUpgrader::StartUpgrade,
        kUpgradeRequest},
    {kUpgradeRequest, kEventRequestUpgrade, &IndustryUpgrader::StartUpgrade,
        kUpgradeXferFirmware},
    {kUpgradeRequest, kEventXferFirmware, &IndustryUpgrader::XferFirmware,
        kUpgradeXferFirmware},
    {kUpgradeXferFirmware, kEventXferFirmware, &IndustryUpgrader::XferFirmware,
        kUpgradeXferFirmware},
    {kUpgradeXferFirmware, kEventCompleteXferFirmware,
        &IndustryUpgrader::CompleteXferFirmware, kUpgradeCompleteXferFirmware},
    {kUpgradeCompleteXferFirmware, kEventGetUpgradeProgress,
        &IndustryUpgrader::GetUpgradeProgress, kUpgradeGetUpgradeProgress},
    {kUpgradeGetUpgradeProgress, kEventGetUpgradeProgress,
        &IndustryUpgrader::GetUpgradeProgress, kUpgradeGetUpgradeProgress},
    {kUpgradeGetUpgradeProgress, kEventComplete, &IndustryUpgrader::UpgradeComplete,
        kUpgradeComplete},
    {kUpgradeComplete, kEventReinit, nullptr, kUpgradeIdle}
    // {kUpgradeRebootDevice, kEventReinit, nullptr, kUpgradeIdle},
};

void IndustryUpgrader::AddUpgradeProgressObserver(UpgradeProgressCallback observer) {
    observer_ = observer;
    return;
}

void IndustryUpgrader::StartUpgradeLidar(uint8_t slot, std::shared_ptr<Firmware> firmware) {
    firmware_ = firmware;
    slot_ = slot;

    read_offset_ = 0;
    fsm_state_ = 0;
    upgrade_error_ = 0;
    progress_ = 0;
    try_count_ = 0;

    upgrade_thread_ = std::make_shared<std::thread>([this](){
        this->FsmEventHandler(kEventRequestUpgrade, 10);
    });
}

void IndustryUpgrader::StopUpgradeLidar(uint8_t slot) {
    if (upgrade_thread_ && upgrade_thread_->joinable()) {
       upgrade_thread_->join();
       upgrade_thread_ = nullptr;
    }
    return;
}

livox_status IndustryUpgrader::StartUpgrade() {
  StartUpgradeRequest request = {0};
  request.firmware_type = firmware_->header_.firmware_type;
  request.firmware_length = firmware_->header_.firmware_length;
  request.encrypt_type = firmware_->header_.encrypt_type;
  request.dev_type = firmware_->header_.device_type;
  printf("Start upgrade, the industry_lidar[%d] device type [%d]\r\n", slot_, request.dev_type);
  return LidarStartUpgrade(slot_, (uint8_t *)&request, sizeof(request),
      StartUpgradeResponseHandler, this);
}

livox_vehicle_status IndustryUpgrader::XferFirmware() {
  uint8_t send_bufer[2048];
  XferFirmwareResquest* request = (XferFirmwareResquest*)send_bufer;
  uint32_t firmware_length = firmware_->header_.firmware_length;
  uint32_t read_length = read_length_;

  if (read_offset_ < firmware_length) {
    if (read_length > (firmware_length - read_offset_)) {
      read_length = firmware_length - read_offset_;
    }
  } else {
    printf("The industry_lidar[%d] xfer firmware failed, Read_offset is err, firmware_length[%d], read_offset[%d].\r\n",
        slot_, firmware_length, read_offset_);
    return -1;
  }

  memcpy(request->data, &firmware_->data_[read_offset_], read_length);
  request->offset = read_offset_;
  request->length = read_length;
  request->encrypt_type = firmware_->header_.encrypt_type;
  read_offset_ += read_length;
  printf("The industry_lidar[%d] xfer firmware read offset %d\r\n", slot_, request->offset);
  return LidarXferFirmware(slot_, (uint8_t *)request,
      sizeof(XferFirmwareResquest) + read_length - sizeof(request->data),
      XferFirmwareResponseHandler, this);
}

livox_status IndustryUpgrader::CompleteXferFirmware() {
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

livox_status IndustryUpgrader::GetUpgradeProgress() {
  return LidarGetUpgradeProgress(slot_, nullptr, 0, GetProgressResponseHandler,
      this);
}

livox_status IndustryUpgrader::UpgradeComplete() {
  return RebootDevice(slot_, 100, UpgradeCompleteResponseHandler, this);
}

int32_t IndustryUpgrader::FsmStateChange(FsmEvent event) {
  if (event < kEventUndef) {
    fsm_state_ = event;
  }
  return 0;
}

void IndustryUpgrader::FsmEventHandler(FsmEvent event, uint8_t progress) {
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
      //printf("New State[%d] | Event[%d]\r\n", fsm_state_, event);
      break;
    }
  }

  if (event_handler) {
    (this->*event_handler)();
  }

  if (observer_) {
      LidarUpgradeState upgrade_state = {event, progress};
      observer_(slot_, upgrade_state);
  }
}

/**
 * Upgrade Callback handler
 */
void IndustryUpgrader::StartUpgradeResponseHandler(livox_status status, uint8_t slot,
    StartUpgradeResponse* response, void* client_data) {
  IndustryUpgrader* upgrade = static_cast<IndustryUpgrader *>(client_data);

  if (status == kVehicleStatusSuccess) {
    upgrade->try_count_ = 0;
    printf("Start upgrade the industry_lidar[%d], ret_code[%d]!\r\n", slot, response->ret_code);
    if (response->ret_code) {
      if (response->ret_code == kSystemIsNotReady) {
        printf("Start upgrade failed, the industry_lidar[%d] is busy, try again!\r\n", slot);
        upgrade->FsmEventHandler(kEventRequestUpgrade, 10);
      } else {
        printf("Start upgrade failed, the industry_lidar[%d] ret_code[%d], try again!\r\n", slot, response->ret_code);
        upgrade->FsmEventHandler(kEventErr, 100);
      }
    } else {
      printf("Start upgrade succ, the industry_lidar[%d] start to xfer data!\r\n", slot);
      upgrade->FsmEventHandler(kEventXferFirmware, 20);
    }
  } else {
    printf("Start upgrade failed, the industry_lidar[%d] start upgrade is timeout[%d], try again!\r\n", slot, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventRequestUpgrade, 10);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Start upgrade failed, the industry_lidar[%d] start upgrade exceed limit! exit!\r\n", slot);
    }
  }
}

void IndustryUpgrader::XferFirmwareResponseHandler(livox_status status, uint8_t slot,
      XferFirmwareResponse* response, void* client_data) {
  IndustryUpgrader* upgrade = static_cast<IndustryUpgrader *>(client_data);

  if (status == kVehicleStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("The industry_lidar[%d] Xfer firmware fail[%d]\r\n", slot, response->ret_code);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      if (upgrade->read_offset_ < upgrade->firmware_->header_.firmware_length) {
        upgrade->FsmEventHandler(kEventXferFirmware, 20);
      } else {
        printf("Xfer firmware succ, the industry_lidar[%d] last offset[%d]\r\n", slot, upgrade->read_offset_);
        upgrade->FsmEventHandler(kEventCompleteXferFirmware, 40);
      }
    }
  } else {
    printf("Xfer firmware failed, the industry_lidar[%d] xfer firmware timeout, try_count:%d, try again!\r\n", slot, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventXferFirmware, 20);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Xfer firmware failed, the industry_lidar[%d] exceed limit! exit!\r\n", slot);
    }
  }
}

void IndustryUpgrader::CompleteXferFirmwareResponseHandler(livox_status status, uint8_t slot,
      CompleteXferFirmwareResponse* response, void* client_data) {
  IndustryUpgrader* upgrade = static_cast<IndustryUpgrader *>(client_data);

  if (status == kVehicleStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Complete xfer failed, the industry_lidar[%d] ret_code:%d.\r\n", slot, response->ret_code);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      printf("The industry_lidar[%d] complete xfer succ.\n", slot);
      upgrade->FsmEventHandler(kEventGetUpgradeProgress, 50);
    }
  } else {
    printf("Complete xfer failed, the industry_lidar[%d] complete xfer is timeout, try_count:%d, try again!\r\n", slot, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress, 50);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Complete xfer failed, the industry_lidar[%d] complete xfer exceed limit! exit!\r\n", slot);
    }
  }
}

void IndustryUpgrader::GetProgressResponseHandler(livox_status status, uint8_t slot,
      GetUpgradeProgressResponse* response, void* client_data) {
  IndustryUpgrader* upgrade = static_cast<IndustryUpgrader *>(client_data);

  if (status == kVehicleStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response->ret_code) {
      printf("Get progress failed, the industry_lidar[%d] ret_code:%d.\r\n", slot, response->ret_code);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      if (response->progress < 100) {
        printf("The industry_lidar[%d] get progress[%d]\r\n", slot, response->progress);
        upgrade->FsmEventHandler(kEventGetUpgradeProgress, response->progress/2 + 50);
      } else {
        printf("The industry_lidar[%d] Get progress[%d]\r\n", slot, response->progress);
        upgrade->FsmEventHandler(kEventComplete, 100);
      }
    }
    upgrade->progress_ = response->progress;
  } else {
    printf("Get progress failed, the industry_lidar[%d] get progress timeout, try_count:%d, try again!\r\n", slot, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress, upgrade->progress_/2 + 50);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventErr, 100);
      printf("Get progress failed, the industry_lidar[%d] get progress exceed limit! exit!\r\n", slot);
    }
  }
}

void IndustryUpgrader::UpgradeCompleteResponseHandler(livox_status status, uint8_t slot,
      uint8_t response, void* client_data) {
  IndustryUpgrader* upgrade = static_cast<IndustryUpgrader *>(client_data);

  if (status == kVehicleStatusSuccess) {
    upgrade->try_count_ = 0;
    if (response != 0) {
      printf("Upgrade complete failed, the industry_lidar[%d] reboot device fail!\r\n", slot);
      upgrade->FsmEventHandler(kEventErr, 100);
    } else {
      printf("The industry_lidar[%d] upgrade complete succ.\n", slot);
      upgrade->FsmEventHandler(kEventReinit, 100);
    }
  } else {
    printf("Upgrade complete failed, the industry_lidar[%d] reboot device timeout, try_count:%d, try again!\r\n",
        slot, upgrade->try_count_);
    ++upgrade->try_count_;
    if (upgrade->try_count_ < kGeneralTryCountLimit) {
      upgrade->FsmEventHandler(kEventGetUpgradeProgress, 100);
    } else {
      upgrade->try_count_ = 0;
      upgrade->FsmEventHandler(kEventReinit, 100);
      printf("Upgrade complete failed, the industry_lidar[%d] reboot device exceed limit! exit!\r\n", slot);
    }
  }
}

} // namespace comm
} // namespace livox
