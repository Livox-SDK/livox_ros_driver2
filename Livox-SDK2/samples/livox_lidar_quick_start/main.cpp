//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  // printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, udp_counter: %d\n",
  //     handle, data->dot_num, data->data_type, data->length, data->udp_cnt);

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      //p_point_data[i].x;
      //p_point_data[i].y;
      //p_point_data[i].z;
    }
  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
  } else if (data->data_type == kLivoxLidarDoubleEchoData) {
    LivoxLidarDoubleEchoRawPoint* p_point_data = (LivoxLidarDoubleEchoRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      //p_point_data[i].x1;
      //p_point_data[i].y1;
      //p_point_data[i].z1;
      //p_point_data[i].reflectivity1;
      //p_point_data[i].tag1;
      //p_point_data[i].x2;
      //p_point_data[i].y2;
      //p_point_data[i].z2;
      //p_point_data[i].reflectivity2;
      //p_point_data[i].tag2;
    }
  }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
  //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }

void EscModeSetCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  printf("set lidar esc mode!!!!\n");
  if (response == nullptr) {
    return;
  }
  printf("EscModeSetCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void PclFreqSetCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  printf("set lidar pcl freq mode!!!!\n");
  if (response == nullptr) {
    return;
  }
  printf("PclFreqSetCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void TimeFilterModeSetCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  printf("set lidar time filter mode!!!!\n");
  if (response == nullptr) {
    return;
  }
  printf("TimeFilterModeSetCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void FovModeSetCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  printf("set lidar fov mode!!!!\n");
  if (response == nullptr) {
    return;
  }
  printf("FovModeSetCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void PclTypeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("PclTypeCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}



// 1. HANDLES RESPONSE AFTER GATHERING INTERNAL INFO (CORRECTED TYPE NAME)
void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
                               LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess || response == nullptr) {
    printf("[ERROR] Failed to query LiDAR internal information. Status: %d\n", status);
    return;
  }

  printf("[INFO] Handshake complete. Pushing IP modification payload...\n");

  //!!!!!!!!!!!!!!!!!!!!! READ THIS DATA FROM ARGUMENTS OR CONFIG FILE !!!!!!!!!!!!!!!!!!!!
  static LivoxLidarIpInfo target_ip_payload;
  strcpy(target_ip_payload.ip_addr, "192.168.0.11");   // Target IP
  strcpy(target_ip_payload.net_mask, "255.255.255.0");  // Subnet
  strcpy(target_ip_payload.gw_addr, "192.168.0.1");     // Gateway

  SetLivoxLidarIp(handle, &target_ip_payload, [](livox_status status, uint32_t handle, 
                                                 LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (status == kLivoxLidarStatusSuccess && response && response->ret_code == 0) {
      printf("\n======================================================\n");
      printf("[SUCCESS] LiDAR IP permanently changed to %s", target_ip_payload.ip_addr);
      printf("======================================================\n\n");
    } else {
      printf("\n[ERROR] LiDAR rejected the IP rewrite. Return code: %d\n\n", response ? response->ret_code : -1);
    }
  }, NULL);
}

// 2. TRIGGERED AUTOMATICALLY WHEN THE DEVICE DETECTS AN ACTIVE PC LINK
void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    return;
  }
  
  static bool command_sent = false;
  if (!command_sent) {
    printf("[INFO] Found LiDAR (SN: %s). Initiating handshake hook...\n", info->sn);
    
    // Call QueryLivoxLidarInternalInfo first to force an active control session
    livox_status status = QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, NULL);
    if (status != kLivoxLidarStatusSuccess) {
        printf("[ERROR] Failed to dispatch internal info query task. Code: %d\n", status);
    } else {
        command_sent = true; // Prevent looping the command
    }
  }
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  return;
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  
  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  
  // OPTIONAL, to get imu data via 'ImuDataCallback'
  // some lidar types DO NOT contain an imu component
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  

#ifdef WIN32
  Sleep(300000);
#else
  sleep(300);
#endif
  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}
