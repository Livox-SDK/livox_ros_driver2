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
#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"

#include <atomic>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <ctime>

// Lidar Number
constexpr int kLidarNumber = 1;
const char* kNetIf = "192.168.1.50"; //local netcard's address
// Lidars' slot and ip
const LidarRegisterInfo lidar_info[kLidarNumber] = {
  { 1, "192.168.1.56" }
};
constexpr int kRecordingTime = 8; // seconds

std::atomic<bool> in_normal_status{false};
std::atomic<bool> is_working{false};

LvxDeviceInfo lvx_info[kLidarNumber];

void LidarWorkModeControlCallback(livox_vehicle_status status, uint8_t slot,
  LidarSyncControlResponse *rsp, void* client_data) {
  if (status == kVehicleStatusTimeout) {
    printf("lidar work mode control timeout\n");
  }
  if (status == kVehicleStatusSuccess) {
    in_normal_status.store(true);
  }
}

int point_count = 0;
void PointCloudCallback(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data) {
  if (++point_count % 2000 == 0) {
    printf("point cloud slot: %d, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
      slot, data_num, data->data_type, data->length, data->frame_counter);
    std::chrono::time_point<std::chrono::system_clock> time_point;
    time_point = std::chrono::system_clock::now();
    std::time_t ttp = std::chrono::system_clock::to_time_t(time_point);
    std::cout << "time: " << std::ctime(&ttp);
  }
  if (!data) {
    return;
  }
  if (data->data_type == kHighResolutionPointData) {
    LivoxVehicleExtendRawPoint *p_point_data = (LivoxVehicleExtendRawPoint *)data->data;
    for (uint32_t i = 0; i < data_num; i++) {
      //p_point_data[i].x;
      //p_point_data[i].y;
      //p_point_data[i].z;
    }
  }
  else if (data->data_type == kLowResolutionPointData) {
    LivoxVehicleExtendHalfRawPoint *p_point_data = (LivoxVehicleExtendHalfRawPoint *)data->data;
  }

}

// 1Hz
void LidarWorkModeControl() {
  while (1) {
    printf("lidar work mode control\n");
    SyncControlInfo info = {
      0x01, //normal mode
      11, // vehicle_speed
      30,  // temperature
      0xff
    };
    for (int i = 0; i < kLidarNumber; i++) {
      uint8_t slot = lidar_info[i].slot;
      livox_vehicle_status status = VehicleLidarSyncControl(slot, &info, LidarWorkModeControlCallback, nullptr);
      printf("livox status: %d\n", status);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (in_normal_status.load()) {
      break;
    }
  }
}

int lidar_count = 0;
void LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data) {
  printf("Lidar Slot: %d SN: %s\n", info->slot, info->broadcast_code);
  if (lidar_count > kLidarNumber) {
    return;
  }
  memcpy(lvx_info[lidar_count].lidar_broadcast_code, info->broadcast_code, sizeof(info->broadcast_code));
  lvx_info[lidar_count].device_type = 16; // 16: PA
  //车规雷达
  lvx_info[lidar_count].lidar_id = info->slot;
  lvx_info[lidar_count].extrinsic_enable = 1; //enable extrinsic calculate
  lvx_info[lidar_count].roll = 0.0;
  lvx_info[lidar_count].pitch = 0.0;
  lvx_info[lidar_count].yaw = 0.0;
  lvx_info[lidar_count].x = 0.0;
  lvx_info[lidar_count].y = 0.0;
  lvx_info[lidar_count].z = 0.0;

  lvx_info[lidar_count].lidar_type = static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType);

  if (++lidar_count >= kLidarNumber) {
    is_working.store(true);
  }
}

int main(int argc, const char *argv[]) {
  /** Init with host's network card's ip address. */
  if (!LivoxVehicleInit(kNetIf)) {
    printf("Livox Init Failed\n");
    LivoxVehicleUninit();
    return -1;
  }
  // SetVehicleLidarPointCloudCallback(PointCloudCallback, nullptr);
  /** Set Lidar Inforamtion Change Callback. */
  SetVehicleLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  /** Register lidars slot and ip address to listen. */
  RegisterLidarInfo(lidar_info, kLidarNumber);

    /** Sync work mode to lidars by 1 Hz. */
  std::thread control_thread(LidarWorkModeControl);

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

  control_thread.join();

  LivoxVehicleUninit();
  printf("Livox Lvx File Demo End!\n");
}
