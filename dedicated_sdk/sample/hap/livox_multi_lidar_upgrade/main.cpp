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
#include "livox_lidar_api.h"
#include "livox_sdk_common.h"
#include "livox_def_common.h"

#include <condition_variable>
#include <mutex>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <set>

std::condition_variable cv;
std::mutex mtx;

std::vector<uint32_t> handles;

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("Work_Mode_Call_Back, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
  cv.notify_one();
}


void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("QueryInternalInfoCallback, status:%u, handle:%u, ret_code:%u.\n",
        status, handle, response->ret_code);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    return;
  }
  printf("LidarInfoChangeCallback Lidar handle: %d SN: %s\n", handle, info->sn);
  SetLivoxLidarWorkMode(handle, kLivoxLidarUpgrade, WorkModeCallback, nullptr);
  handles.push_back(handle);
}


void LivoxLidarUpgradeProgressCallback(LivoxLidarType type, uint32_t handle, LidarUpgradeState state, void* client_data) {
  printf("Upgrade Progress Callback, ldiar type: %d, handle: %d, state: %d progress: %d\n",
      type, handle, state.state, state.progress);
  if (state.progress == 100) {
    cv.notify_one();
  }
}

int main(int argc, const char *argv[]) {
  if (argc != 3) {
    printf("Params Invalid, must input config path and firmware path.\n");
    return -1;
  }
  const std::string path = argv[1];
  const std::string firmware_path = argv[2];

  printf("path:%s.\n", path.c_str());
  printf("firmware_path:%s.\n", firmware_path.c_str());

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    //Lidar Upgrade
  if (!SetLivoxLidarUpgradeFirmwarePath(firmware_path.c_str())) {
    LivoxLidarSdkUninit();
    return -1;
  }
  SetLivoxLidarUpgradeProgressCallback(LivoxLidarUpgradeProgressCallback, nullptr);

  while(handles.empty()) {}

  UpgradeLivoxLidars(handles.data(), handles.size());

#ifdef WIN32
  Sleep(3000000);
#else
  sleep(3000000);
#endif
  LivoxLidarSdkUninit();
	printf("Livox Lidars Update Demo End!\n");
  return 0;
}
