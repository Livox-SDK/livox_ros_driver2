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

#include "livox_lidar_def.h"
#include "livox_lidar_sdk.h"

#include "livox_sdk_common.h"
#include "livox_def_common.h"

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <atomic>
#include <ctime>

constexpr int kLidarNumber = 1;
constexpr int kRecordingTime = 17; // record 10 second point cloud data, hap lidar need 7 second for starting

std::atomic<bool> in_normal_status{false};
std::atomic<bool> is_working{false};

LvxDeviceInfo lvx_info[kLidarNumber];
int lidar_count = 0;

     
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (status != kVehicleStatusSuccess) {
    printf("Set lidar work mode failed.\n");
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
    return;
  }
  if (response == nullptr) {
    return;
  }
  printf("Work_Mode_Call_Back, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
  if (lidar_count >= kLidarNumber) {
    is_working.store(true);
  }
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    return;
  }
  printf("LidarInfoChangeCallback Lidar handle: %d SN: %s\n", handle, info->sn);
  if (lidar_count > kLidarNumber) {
    return;
  }
  
  memcpy(lvx_info[lidar_count].lidar_broadcast_code, info->sn, sizeof(info->sn));
  lvx_info[lidar_count].device_type = 10; // 10: HAP industrial
  lvx_info[lidar_count].lidar_id = handle;
  lvx_info[lidar_count].extrinsic_enable = 1; //enable extrinsic calculate
  lvx_info[lidar_count].roll = 0.0;
  lvx_info[lidar_count].pitch = 0.0;
  lvx_info[lidar_count].yaw = 0.0;
  lvx_info[lidar_count].x = 0.0;
  lvx_info[lidar_count].y = 0.0;
  lvx_info[lidar_count].z = 0.0;

  lvx_info[lidar_count].lidar_type = static_cast<uint8_t>(LivoxLidarType::kLivoxLidarType);
  ++lidar_count;
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  while (1) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (is_working.load()) {
      break;
    }
  }

  SetLvxRecordDir("./");

  AddDeviceInfo(lvx_info, kLidarNumber);

  printf("Start Record File\n");
  StartRecordFile(LIVOX_FILE_LVX2);
  
  std::this_thread::sleep_for(std::chrono::seconds(kRecordingTime));

  printf("Stop Record File\n");
  StopRecordLvxFile();

  LivoxLidarSdkUninit();
  printf("Livox Lvx File Demo End!\n");
}
