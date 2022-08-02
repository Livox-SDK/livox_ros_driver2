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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>

// Lidar Number
const int kLidarNumber = 1;
const char* kNetIf = "192.168.1.111"; //local netcard's address
// Lidars' slot and ip
const LidarRegisterInfo lidar_info[kLidarNumber] = {
  { 1, "192.168.1.100" },
  //{ 3, "172.20.1.56" }
};

void PointCloudCallback(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data) {
  printf("point cloud slot: %d, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n", slot, data_num, data->data_type, data->length, data->frame_counter);
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


void ImuDataCallback(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data) {
  printf("Imu data callback slot:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      slot, data_num, data->data_type, data->length, data->frame_counter);
}


void LidarWorkModeControlCallback(livox_vehicle_status status, uint8_t slot,
  LidarSyncControlResponse *rsp, void* client_data) {
  if (status == kVehicleStatusSuccess) {
    printf("lidar work mode control status: %d, slot: %d, ret_code: %d error_key: %d\n",
      status, slot, rsp->ret_code, rsp->error_key);
  }
  else if (status == kVehicleStatusTimeout) {
    printf("lidar work mode control timeout\n");
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
  }
}

void OnLidarSetIpCallback(livox_vehicle_status status, uint8_t slot, uint8_t ret_code, void*) {
  if (status == kVehicleStatusSuccess) {
    printf("lidar set ip slot: %d, ret_code: %d\n",
      slot, ret_code);
  } else if (status == kVehicleStatusTimeout) {
    printf("lidar set ip number timeout\n");
  }
}

void OnLidarSetSlotNumberCallback(livox_vehicle_status status, uint8_t slot,
  LidarSyncControlResponse *rsp, void* client_data) {
  if (status == kVehicleStatusSuccess) {
    printf("lidar set slot number status: %d, slot: %d, ret_code: %d error_key: %d\n",
      status, slot, rsp->ret_code, rsp->error_key);
  } else if (status == kVehicleStatusTimeout) {
    printf("lidar set slot number timeout\n");
  }
}

void LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data) {
  printf("LidarInfoChangeCallback Lidar Slot: %d SN: %s\n", info->slot, info->broadcast_code);

  if (info->slot == 1) {
    LidarIpConfig ip_config;
    uint8_t ip_addr[4] = {192, 168, 1, 100};
    memcpy(&ip_config.ip_addr, ip_addr, 4);
    uint8_t gw_addr[4] = {192, 168, 1, 1};
    memcpy(&ip_config.gw_addr, gw_addr, 4);
    uint8_t net_mask[4] = {255, 255, 255, 0};
    memcpy(&ip_config.net_mask, net_mask, 4);
    VehicleLidarSetIp(info->slot, ip_config, OnLidarSetIpCallback, nullptr);

    uint8_t dst_slot = 1;
    VehicleLidarSetSlotNumber(info->slot, dst_slot, OnLidarSetSlotNumberCallback, nullptr);
  }
}

int main(int argc, const char *argv[]) {
  /** Init with host's network card's ip address. */
  if (!LivoxVehicleInit(kNetIf)) {
    printf("Livox Init Failed\n");
    LivoxVehicleUninit();
    return -1;
  }
  /** Set point cloud callback. */
  SetVehicleLidarPointCloudCallback(PointCloudCallback, nullptr);
  SetVehicleLidarImuDataCallback(ImuDataCallback, nullptr);
  /** Set Lidar Inforamtion Change Callback. */
  SetVehicleLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  /** Register lidars slot and ip address to listen. */
  RegisterLidarInfo(lidar_info, kLidarNumber);

  /** Sync work mode to lidars by 1 Hz. */
  std::thread control_thread(LidarWorkModeControl);
  control_thread.join();

  LivoxVehicleUninit();
  printf("Livox Quick Start Demo End!\n");
}
