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

#include "upgrade_manager.h"
#include <livox_sdk_common.h>
#include <thread>
#include "direct/direct_lidar_upgrader.h"

namespace livox {
namespace common {

UpgradeManager &upgrade_manager() {
  static UpgradeManager manager;
  return manager;
}

UpgradeManager::UpgradeManager() : info_cb_(nullptr), client_data_(nullptr) {}

bool UpgradeManager::SetUpgradeFirmwarePath(const std::string& path) {
  firmware_ptr_ = std::make_shared<Firmware>();
  if (!firmware_ptr_->Open(path.c_str())) {
    printf("Open firmware_path fail\r\n");
    return false;
  }
  return true;
}

void UpgradeManager::SetUpgradeProgressCallback(OnUpgradeProgressCallbackCallback cb, void* client_data) {
  info_cb_ = cb;
  client_data_ = client_data;
  vehicle_upgrader_->AddUpgradeProgressObserver([cb, client_data](uint8_t slot, LidarUpgradeState state) {
    if (cb) {
      cb(LivoxLidarType::kVehicleLidarType, slot, state, client_data);
    }
  });

  industry_upgrader_->AddUpgradeProgressObserver([cb, client_data](uint8_t slot, LidarUpgradeState state) {
    if (cb) {
      cb(LivoxLidarType::kIndustryLidarType, slot, state, client_data);
    }
  });
  return;
}

void UpgradeManager::StartUpgradeLidar(LivoxLidarType type, uint8_t slot) {
  if (type == LivoxLidarType::kVehicleLidarType) {
    vehicle_upgrader_->StartUpgradeLidar(slot, firmware_ptr_);
  } else {
    industry_upgrader_->StartUpgradeLidar(slot, firmware_ptr_);
  }
}

void UpgradeManager::StopUpgradeLidar(LivoxLidarType type, uint8_t slot) {
  if (type == LivoxLidarType::kVehicleLidarType) {
    vehicle_upgrader_->StopUpgradeLidar(slot);
  } else {
    industry_upgrader_->StopUpgradeLidar(slot);
  }
}

void UpgradeManager::UpgradeVehicleLidar(const uint8_t slot) {
  VehicleLidarUpgrader upgrade(vehicle_firmware_, slot);
  upgrade.StartUpgradeVehicleLidar();
}

//vehicle lidar upgrader
bool UpgradeManager::SetVehicleUpgradeFirmwarePath(const char* firmware_path) {
  if (!vehicle_firmware_.Open(firmware_path)) {
    printf("Open firmware_path fail\r\n");
    return false;
  }

  return true;
}

void UpgradeManager::UpgradeVehicleLidars(const uint8_t* slot, const uint8_t lidar_num) {
  std::vector<VehicleLidarUpgrader> upgrader_vec;
  upgrader_vec.reserve(lidar_num);
  for (size_t i = 0; i < lidar_num; ++i) {   
    VehicleLidarUpgrader upgrader(vehicle_firmware_, slot[i]);

    OnUpgradeProgressCallbackCallback cb = info_cb_;
    void* client_data = client_data_;
    upgrader.AddUpgradeProgressObserver([cb, client_data](uint8_t slot, LidarUpgradeState state) {
      if (cb) {
        cb(LivoxLidarType::kVehicleLidarType, slot, state, client_data);
      }
    });
 
    upgrader_vec.emplace_back(std::move(upgrader));
  }

  for (size_t i = 0; i < upgrader_vec.size(); ++i) {
    VehicleLidarUpgrader& upgrader = upgrader_vec[i];
    upgrader.StartUpgradeVehicleLidar();
  }

  CloseVehicleLidarFirmwareFile();
}

void UpgradeManager::CloseVehicleLidarFirmwareFile() {
  vehicle_firmware_.Close();
}

//Direct lidar upgrader
bool UpgradeManager::SetDirectUpgradeFirmwarePath(const char* firmware_path) {
  if (!direct_firmware_.Open(firmware_path)) {
    printf("Open firmware_path fail\r\n");
    return false;
  }

  return true;
}

void UpgradeManager::SetDirectLidarUpgradeProgressCallback(OnDirectLidarUpgradeProgressCallback cb, void* client_data) {
  direct_lidar_info_cb_ = cb;
  direct_lidar_client_data_ = client_data;
}

void UpgradeManager::UpgradeDirectLidars(const uint32_t* handle, const uint8_t lidar_num) {
  std::vector<DirectLidarUpgrader> upgrader_vec;
  upgrader_vec.reserve(lidar_num);
  for (size_t i = 0; i < lidar_num; ++i) {   
    DirectLidarUpgrader upgrader(direct_firmware_, handle[i]);

    OnDirectLidarUpgradeProgressCallback cb = direct_lidar_info_cb_;
    void* client_data = direct_lidar_client_data_;
    upgrader.AddUpgradeProgressObserver([cb, client_data](uint32_t slot, LidarUpgradeState state) {
      if (cb) {
        cb(LivoxLidarType::kDirectLidarType, slot, state, client_data);
      }
    });
 
    upgrader_vec.emplace_back(std::move(upgrader));
  }

  for (size_t i = 0; i < upgrader_vec.size(); ++i) {
    DirectLidarUpgrader& upgrader = upgrader_vec[i];
    upgrader.StartUpgradeDirectLidar();
  }

  CloseDirectLidarFirmwareFile();
}

void UpgradeManager::CloseDirectLidarFirmwareFile() {
  direct_firmware_.Close();
}

//livoix lidar upgrader
bool UpgradeManager::SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path) {
  if (!livox_lidar_firmware_.Open(firmware_path)) {
    printf("Open firmware_path fail\r\n");
    return false;
  }
  return true;
}

void UpgradeManager::SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data) {
  livox_lidar_info_cb_ = cb;
  livox_lidar_client_data_ = client_data;
}

void UpgradeManager::UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num) {
  std::vector<LivoxLidarUpgrader> upgrader_vec;
  upgrader_vec.reserve(lidar_num);
  for (size_t i = 0; i < lidar_num; ++i) {   
    LivoxLidarUpgrader upgrader(livox_lidar_firmware_, handle[i]);

    OnLivoxLidarUpgradeProgressCallback cb = livox_lidar_info_cb_;
    void* client_data = livox_lidar_client_data_;
    upgrader.AddUpgradeProgressObserver([cb, client_data](uint32_t handle, LidarUpgradeState state) {
      if (cb) {
        cb(LivoxLidarType::kLivoxLidarType, handle, state, client_data);
      }
    });
 
    upgrader_vec.emplace_back(std::move(upgrader));
  }

  for (size_t i = 0; i < upgrader_vec.size(); ++i) {
    LivoxLidarUpgrader& upgrader = upgrader_vec[i];
    upgrader.StartUpgradeLivoxLidar();
  }

  CloseLivoxLidarFirmwareFile();
}

void UpgradeManager::CloseLivoxLidarFirmwareFile() {
  livox_lidar_firmware_.Close();
}

}  // namespace comm
}  // namespace livox
