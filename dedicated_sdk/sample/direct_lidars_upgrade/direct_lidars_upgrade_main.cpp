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

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "livox_sdk_direct.h"

#include "parse_cfg_file.h"
#include <iostream>

#include "livox_sdk_common.h"
#include "livox_def_common.h"

#include <thread>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <mutex>

std::condition_variable cv;
std::mutex mtx;

std::mutex handle_mtx;
std::vector<uint32_t> handle_vec;

void DirectLidarInfo(const uint32_t handle, DirectLidarStateInfo* info, void* client_data) {
  if (info == NULL) {
    printf("Direct lidar info callback failed, the info is null.\n");
  }

  // printf("Direct_lidar_info_callback, handle:%u, sn:%s, product_info:%s, version_app:%s, version_loader[%d, %d, %d, %d], "
  //        "version_hardware[%d, %d, %d, %d], lidar_mac:%s.\n", handle, info->sn, info->product_info, info->version_app,
  //        info->version_loader[0], info->version_loader[1], info->version_loader[2], info->version_loader[3],
  //        info->version_hardware[0], info->version_hardware[1], info->version_hardware[2], info->version_hardware[3],
  //        info->lidar_mac);

  // printf("Direct_lidar_info_callback, lidar_id:%d, lidar_ipmode:%d, lidar_ip:%s, lidar_submask:%s, lidar_gateway:%s.\n",
  //         info->lidar_id, info->lidar_ipmode, info->lidar_ip, info->lidar_submask, info->lidar_gateway);

  // printf("Direct_lidar_info_callback, host_push_msg_ip:%s, host_point_data_ip:%s, host_imu_data_ip:%s, "
  //        "host_push_msg_port:%u, host_point_data_port:%u, host_imu_data_port:%u.\n", info->host_push_msg_ip,
  //        info->host_point_data_ip, info->host_imu_data_ip, info->host_push_msg_port, info->host_point_data_port, 
  //        info->host_imu_data_port);

  // printf("Direct_lidar_info_callback, sample_mode:%u, pattern_mode:%u, pcl_data_type:%u, imu_data_en:%u, work_mode:%u, "
  //        "work_state:%u, core_temp:%d.\n", info->sample_mode, info->pattern_mode, info->pcl_data_type, info->imu_data_en,
  //        info->work_mode, info->work_state, info->core_temp);

  // printf("Direct_lidar_info_callback, roll_deg:%f, pitch_deg:%f, yaw_deg:%f, x:%d, y:%d, z:%d.\n",
  //        info->install_attitude.roll_deg, info->install_attitude.pitch_deg, info->install_attitude.yaw_deg,
  //        info->install_attitude.x, info->install_attitude.y, info->install_attitude.z);
  std::lock_guard<std::mutex> mutex(handle_mtx);
  handle_vec.push_back(handle);
  cv.notify_one();
}

void OnLidarUpgradeProgressCallbackCallback(LivoxLidarType type, uint32_t handle, LidarUpgradeState state, void* client_data) {
  printf("Direct lidar Upgrade Progress Callback, ldiar type: %d, handle: %u, state: %d progress: %d\n",
      type, handle, state.state, state.progress);
  if (state.progress == 100) {
    cv.notify_one();
  }
}

int main(int argc, const char *argv[]) {
  if (argc != 3) {
    printf("The number of parameters entered is incorrect, must enter the json file path.\n");
    return -1;
  }

  std::string json_path = argv[1];
  std::string firmware_path = argv[2];

  DirectLidarHostCfg direct_host_cfg;
  uint8_t lidar_count;
  if (!ParseCfgFile(json_path).ParseHostConfig(direct_host_cfg)) {
    std::cout<<"Parse cfg file failed."<<std::endl;
    return -1;
  }

  printf("Livox SDK initializing.\n");

  /** Initialize Livox-SDK. */
  if (!LivoxDirectInit(&direct_host_cfg, NULL)) {
    printf("Livox direct init failed.\n");
    LivoxDirectUninit();
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  SetDirectLidarInfoCallback(DirectLidarInfo, NULL);

  //Lidar Upgrade
  if (!SetDirectUpgradeFirmwarePath(firmware_path.c_str())) {
    LivoxDirectUninit();
    return -1;
  }

  SetDirectLidarUpgradeProgressCallback(OnLidarUpgradeProgressCallbackCallback, nullptr);

  while (true) {
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock);
    }
    UpgradeDirectLidars(handle_vec.data(), handle_vec.size());
    break;
  }


#ifdef WIN32
  Sleep(30000);
#else
  sleep(300);
#endif

  return 0;
}
