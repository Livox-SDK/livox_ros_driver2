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
#include <iostream>

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  printf("point cloud handle: %d, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

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
  }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data = nullptr) {
    return;
  }
  printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }
     
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("Work_Mode_Call_Back, status:%u, handle:%u, ret_code:%u, error_key:%u.\n",
      status, handle, response->ret_code, response->error_key);

}


void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u.\n",
      status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
 
  printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u.\n",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void SetLivoxLidarPointDataHostIPCfgCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }

  printf("SetLivoxLidarPointDataHostIPCfgCallback, status:%u, handle:%u, ret_code:%u, error_key:%u.\n",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}


void SetLivoxLidarImuDataHostIPCfgCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetLivoxLidarImuDataHostIPCfgCallback, status:%u, handle:%u, ret_code:%u, error_key:%u.\n",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("QueryInternalInfoCallback, status:%u, handle:%u, ret_code:%u, param_num:%u.\n",
      status, handle, response->ret_code, response->param_num);
}


void EnableLivoxLidarGlassHeatCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("EnableLivoxLidarGlassHeatCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void DisableLivoxLidarGlassHeatCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("DisableLivoxLidarGlassHeatCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void SetLivoxLidarGlassHeatCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetLivoxLidarGlassHeatCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);

}
void SetPclDataTypeCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetPclDataTypeCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void DisableLivoxLidarHighResolutionPointTypeCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("DisableLivoxLidarHighResolutionPointTypeCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void LivoxLidarScanPatternCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("LivoxLidarScanPatternCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void SetLivoxLidarDualEmitCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetLivoxLidarDualEmitCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void EnableLivoxLidarPointSendCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("EnableLivoxLidarPointSendCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void DisableLivoxLidarPointSendCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("DisableLivoxLidarPointSendCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void SetLivoxLidarInstallAttitudeCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetLivoxLidarInstallAttitudeCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}


void SetLivoxLidarBlindSpotCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetLivoxLidarBlindSpotCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("EnableLivoxLidarImuDataCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}


void DisableLivoxLidarImuDataCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("DisableLivoxLidarImuDataCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}



void EnableLivoxLidarFusaFunciontCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("EnableLivoxLidarFusaFunciontCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}


void DisableLivoxLidarFusaFunciontCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("DisableLivoxLidarFusaFunciontCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}


void SetLivoxLidarLogParamCallback(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("SetLivoxLidarLogParamCallback status:%u, ret_code:%u, error_key:%u.\n", status, response->ret_code, response->error_key);
}

void LivoxLidarRequestResetCallback(livox_status status, uint32_t handle, 
    LivoxLidarResetResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("LivoxLidarRequestResetCallback status:%u, ret_code:%u.\n", status, response->ret_code);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("Lidar info change call back failed, the info is nullptr.\n");
    return;
  }
  printf("LidarInfoChangeCallback Lidar handle: %d SN: %s\n", handle, info->sn);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  EnableLivoxLidarGlassHeat(handle, EnableLivoxLidarGlassHeatCallback, nullptr);
  
  SetLivoxLidarGlassHeat(handle, kLivoxLidarStopPowerOnHeatingOrDiagnosticHeating, SetLivoxLidarGlassHeatCallback, nullptr);
  
  DisableLivoxLidarGlassHeat(handle, DisableLivoxLidarGlassHeatCallback, nullptr);

  SetLivoxLidarPclDataType(handle, kLivoxLidarCartesianCoordinateHighData, SetPclDataTypeCallback, nullptr);

  //EnableLivoxLidarHighResolutionPointType(handle, EnableLivoxLidarHighResolutionPointTypeCallback, nullptr);
  //DisableLivoxLidarHighResolutionPointType(handle, DisableLivoxLidarHighResolutionPointTypeCallback, nullptr);


  EnableLivoxLidarFusaFunciont(handle, EnableLivoxLidarFusaFunciontCallback, nullptr);
  DisableLivoxLidarFusaFunciont(handle, DisableLivoxLidarFusaFunciontCallback, nullptr);

  SetLivoxLidarScanPattern(handle, kLivoxLidarScanPatternRepetive, LivoxLidarScanPatternCallback, nullptr);
  
  SetLivoxLidarDualEmit(handle, 1, SetLivoxLidarDualEmitCallback, nullptr);

  EnableLivoxLidarPointSend(handle, EnableLivoxLidarPointSendCallback, nullptr);
  DisableLivoxLidarPointSend(handle, DisableLivoxLidarPointSendCallback, nullptr);

  // LivoxLidarIpInfo lidar_ip_info;
  // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
  // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
  // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
  // SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);

  // HostPointIPInfo lidar_host_pt_ip_info;
  // strcpy(lidar_host_pt_ip_info.host_ip_addr, "192.168.1.5");
  // lidar_host_pt_ip_info.host_point_data_port = 57000;
  // lidar_host_pt_ip_info.lidar_point_data_port = 57000;
  // SetLivoxLidarPointDataHostIPCfg( handle, &lidar_host_pt_ip_info, SetLivoxLidarPointDataHostIPCfgCallback, nullptr);

  // HostImuDataIPInfo lidar_host_imu_ip_info;
  // strcpy(lidar_host_imu_ip_info.host_ip_addr, "192.168.1.5");
  // lidar_host_imu_ip_info.host_imu_data_port = 58000;
  // lidar_host_imu_ip_info.lidar_imu_data_port = 58000;
  // SetLivoxLidarImuDataHostIPCfg( handle, &lidar_host_imu_ip_info, SetLivoxLidarImuDataHostIPCfgCallback, nullptr);


  EnableLivoxLidarImuData(handle, EnableLivoxLidarImuDataCallback, nullptr);
  DisableLivoxLidarImuData(handle, DisableLivoxLidarImuDataCallback, nullptr);

  LivoxLidarInstallAttitude install_attitude;
  install_attitude.roll_deg = 1.1;
  install_attitude.pitch_deg = 1.1;
  install_attitude.yaw_deg = 1.1;
  install_attitude.x = 1;
  install_attitude.y = 1;
  install_attitude.z = 1;
  SetLivoxLidarInstallAttitude(handle, &install_attitude, SetLivoxLidarInstallAttitudeCallback, nullptr);

  SetLivoxLidarBlindSpot(handle, 50, SetLivoxLidarBlindSpotCallback, nullptr);

  LivoxLidarRequestReset(handle, LivoxLidarRequestResetCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
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
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

#ifdef WIN32
  Sleep(30000);
#else
  sleep(8);
#endif
  LivoxLidarSdkUninit();
	printf("Livox Quick Start Demo End!\n");
  return 0;
}
