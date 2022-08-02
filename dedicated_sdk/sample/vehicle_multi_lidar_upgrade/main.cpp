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
#include <stdint.h>
#include <chrono>
#include <thread>
#include <map>
#include <vector>

#include "livox_sdk_vehicle.h"
#include "livox_def_vehicle.h"
#include "livox_sdk_common.h"
#include "livox_def_common.h"

#include <condition_variable>
#include <mutex>

using namespace std;

typedef struct {
  uint8_t WorkMode;
  uint8_t SlotId;
  uint8_t AddressData[18];
  uint8_t VersionInfo[12];
  uint8_t ProductInfo[19];
} LidarDiagInternalInfo;

const int kLidarNumber = 2;
const char* kNetIf = "192.168.1.100"; //local netcard's address
const LidarRegisterInfo lidar_info[kLidarNumber] = {
  { 1, "192.168.1.56" },
  { 2, "192.168.1.59" }
};

std::condition_variable cv;
std::mutex mtx;

void LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data) {
  printf("Lidar Slot: %d SN: %s\n", info->slot, info->broadcast_code);
  cv.notify_one();
}

void LidarExceptionDetailCallback(uint8_t slot,
  LivoxDetailExceptionInfo* info, void* client_data) {
    printf("exception detail callback\n");
}

void OnLidarUpgradeProgressCallbackCallback(LivoxLidarType type, uint8_t slot, LidarUpgradeState state, void* client_data) {
  printf("Upgrade Progress Callback, ldiar type: %d, slot: %d, state: %d progress: %d\n", type, slot, state.state, state.progress);
  if (state.progress == 100) {
    cv.notify_one();
  }
}

int main(int argc, char** argv) {
  printf("Commandline input %d args : \n", argc);
  if (argc != 2) {
    printf("Please input firmware path!\n");
  }

  for (int i = 0; i < argc; i++) {
    printf("Index:%d, param:%s\n", i, argv[i]);
  }

  /** Init with host's network card's ip address. */
  if (!LivoxVehicleInit(kNetIf)) {
    printf("Livox Init Failed\n");
    LivoxVehicleUninit();
    return -1;
  }

  SetVehicleLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr); /** Set Lidar Inforamtion Change Callback. */
  RegisterLidarInfo(lidar_info, kLidarNumber); /** Register lidars slot and ip address to listen. */

  //Lidar Upgrade
  if (!SetVehicleUpgradeFirmwarePath(argv[1])) {
    LivoxVehicleUninit();
    return -1;
  }

  SetUpgradeProgressCallback(OnLidarUpgradeProgressCallbackCallback, nullptr);

  while (true) {
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock);
    }
		std::vector<uint8_t> slots{1, 2};
    UpgradeVehicleLidars(slots.data(), 2);
    break;
  }

  LivoxVehicleUninit();
  exit(0);
  return 0;
}
