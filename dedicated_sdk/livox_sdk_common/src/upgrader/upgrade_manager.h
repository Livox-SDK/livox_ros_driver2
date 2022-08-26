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

#ifndef LIVOX_UPGRADE_MANAGER_H_
#define LIVOX_UPGRADE_MANAGER_H_

#include <livox_def_common.h>
#include <livox_def_vehicle.h>
#include <functional>
#include <memory>
//#include <mutex>
//#include <condition_variable>
//#include <thread>
#include <fstream>
//#include <list>
#include <vector>
#include <string>
//#include <atomic>

#include "vehicle/vehicle_upgrader.h"
#include "vehicle/vehicle_lidar_upgrader.h"

#include "industry/industry_upgrader.h"

#include "livox_lidar/livox_lidar_upgrader.h"


#include "firmware/firmware.h"


namespace livox {
namespace common {

class UpgradeManager {
public:
  using OnUpgradeProgressCallbackCallback = 
    std::function<void(LivoxLidarType type, uint8_t slot, LidarUpgradeState state, void *client_data)>;

  using OnDirectLidarUpgradeProgressCallback = 
    std::function<void(LivoxLidarType type, uint32_t slot, LidarUpgradeState state, void *client_data)>;

  using OnLivoxLidarUpgradeProgressCallback = 
    std::function<void(LivoxLidarType type, uint32_t handle, LidarUpgradeState state, void *client_data)>;

  UpgradeManager();
  ~UpgradeManager() = default;
 
  bool SetUpgradeFirmwarePath(const std::string& path);
  void SetUpgradeProgressCallback(OnUpgradeProgressCallbackCallback cb, void* client_data);

  void StartUpgradeLidar(LivoxLidarType type, uint8_t slot);
  void StopUpgradeLidar(LivoxLidarType type, uint8_t slot);

  // Vehicle lidar upgrader
  bool SetVehicleUpgradeFirmwarePath(const char* firmware_path);
  void UpgradeVehicleLidar(const uint8_t slot);
  void UpgradeVehicleLidars(const uint8_t* slot, const uint8_t lidar_num);

  // Direct lidar upgrade
  bool SetDirectUpgradeFirmwarePath(const char* firmware_path);
  void SetDirectLidarUpgradeProgressCallback(OnDirectLidarUpgradeProgressCallback cb, void* client_data);
  void UpgradeDirectLidars(const uint32_t* handle, const uint8_t lidar_num);

  // Livox lidar upgrade
  bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path);
  void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data);
  void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num);
  void CloseLivoxLidarFirmwareFile();
  
private:
 void CloseVehicleLidarFirmwareFile();
 void CloseDirectLidarFirmwareFile();

private:
  std::shared_ptr<VehicleUpgrader> vehicle_upgrader_ = std::make_shared<VehicleUpgrader>();
  std::shared_ptr<IndustryUpgrader> industry_upgrader_ = std::make_shared<IndustryUpgrader>();

  std::shared_ptr<Firmware> firmware_ptr_;

  // for vehicle 
  Firmware vehicle_firmware_;
  Firmware livox_lidar_firmware_;

  OnUpgradeProgressCallbackCallback info_cb_;
  void *client_data_;

  OnLivoxLidarUpgradeProgressCallback livox_lidar_info_cb_;
  void *livox_lidar_client_data_;  
};

UpgradeManager &upgrade_manager();

}  // namespace comm
}  // namespace livox

#endif  // LIVOX_UPGRADE_MANAGER_H_
