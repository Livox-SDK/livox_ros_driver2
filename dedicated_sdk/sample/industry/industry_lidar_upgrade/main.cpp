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

#include "livox_sdk_common.h"
#include "livox_def_common.h"
#include "livox_def.h"
#include "livox_sdk.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <mutex>

std::condition_variable cv;
std::mutex mtx;

uint8_t lidar_handle = 0;

void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (!info) {
    return;
  }
  printf("Lidar Handle: %d SN: %s\n", info->handle, info->broadcast_code);
  lidar_handle = info->handle;
  if (info->state == kLidarStateNormal) {
    cv.notify_one();
  }
}

void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL || info->dev_type == kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  bool result = false;
  uint8_t handle = 0;
  AddLidarToConnect(info->broadcast_code, &handle);
}

void OnLidarUpgradeProgressCallbackCallback(LivoxLidarType type, uint8_t slot, LidarUpgradeState state, void* client_data) {
  printf("Upgrade Progress Callback, ldiar type: %d, slot: %d, state: %d progress: %d\n", type, slot, state.state, state.progress);
  if (state.progress == 100) {
    cv.notify_one();
  }
}

int main(int argc, const char *argv[]) {
  /** Init with host's network card's ip address. */
  if (!Init()) {
    printf("Livox Init Failed\n");
    Uninit();
    return -1;
  }

  /** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

  /** Set the callback function called when device state change,
    * which means connection/disconnection and changing of LiDAR state.
  */
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);

  if (!Start()) {
    Uninit();
    return -1;
  }

  //Lidar Upgrade
  if (!SetUpgradeFirmwarePath("/Users/jerry.lin/protocol/build/sample/industry_lidar_upgrade/LIVOX_CF_FW_13.08.0044.bin")) {
    Uninit();
    return -1;
  }

  SetUpgradeProgressCallback(OnLidarUpgradeProgressCallbackCallback, nullptr);

  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock);
  }

  StartUpgradeLidar(LivoxLidarType::kIndustryLidarType, lidar_handle);

  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock);
  }

  StopUpgradeLidar(LivoxLidarType::kIndustryLidarType, lidar_handle);

  Uninit();
  printf("Industry LiDAR Upgrade Demo End!\n");
}

