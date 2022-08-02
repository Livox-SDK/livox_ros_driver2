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

#include "livox_sdk_vehicle.h"
#include "livox_def_vehicle.h"
#include "livox_sdk_common.h"
#include "livox_def_common.h"
#include "livox_def.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <mutex>

// Lidar Number
const int kLidarNumber = 1;
const char* kNetIf = "172.20.1.100"; //local netcard's address
// Lidars' slot and ip
const LidarRegisterInfo lidar_info[kLidarNumber] = {
  { 1, "172.20.1.52" }
};

std::condition_variable cv;
std::mutex mtx;

void LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data) {
  printf("Lidar Slot: %d SN: %s\n", info->slot, info->broadcast_code);
  cv.notify_one();
}

void OnLidarUpgradeProgressCallbackCallback(LivoxLidarType type, uint8_t slot, LidarUpgradeState state, void* client_data) {
  printf("Upgrade Progress Callback, ldiar type: %d, slot: %d, state: %d progress: %d\n", type, slot, state.state, state.progress);
  if (state.progress == 100) {
    cv.notify_one();
  }
}

int main(int argc, const char *argv[]) {
  /** Init with host's network card's ip address. */
  if (!LivoxVehicleInit(kNetIf)) {
    printf("Livox Init Failed\n");
    LivoxVehicleUninit();
    return -1;
  }
  /** Set Lidar Inforamtion Change Callback. */
  SetVehicleLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  /** Register lidars slot and ip address to listen. */
  RegisterLidarInfo(lidar_info, kLidarNumber);

  //Lidar Upgrade
  if (!SetUpgradeFirmwarePath("/Users/jerry.lin/protocol/build/sample/lidar_upgrade/LIVOX_HAP_FW_15.02.0101.bin")) {
    LivoxVehicleUninit();
    return -1;
  }

  SetUpgradeProgressCallback(OnLidarUpgradeProgressCallbackCallback, nullptr);

  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock);
  }

  StartUpgradeLidar(LivoxLidarType::kVehicleLidarType, lidar_info[0].slot);

  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock);
  }

  StopUpgradeLidar(LivoxLidarType::kVehicleLidarType, lidar_info[0].slot);

  LivoxVehicleUninit();
  printf("Livox Quick Start Demo End!\n");
}

