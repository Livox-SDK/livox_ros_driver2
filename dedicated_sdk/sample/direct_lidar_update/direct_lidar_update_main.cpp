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


uint32_t lidar_handle = 0;

void LidarCfgUpdateCallabck(const uint32_t handle, DirectLidarCmdResInfo* response, void* client_data) {
  printf("Lidar update cfg callback, handle[%u], ret_code:%u, ret_error_key:%u.\n",
      handle, response->res.ret_code, response->res.error_key);
}


void DirectLidarInfo(const uint32_t handle, DirectLidarStateInfo* info, void* client_data) {
  if (info == NULL) {
    printf("Direct lidar info callback failed, the info is null.\n");
  }

  lidar_handle = handle;
  printf("Direct_lidar_info_callback, handle:%u, sn:%s, product_info:%s, version_app:%s, version_loader[%d, %d, %d, %d], "
         "version_hardware[%d, %d, %d, %d], lidar_mac:%s.\n", handle, info->sn, info->product_info, info->version_app,
         info->version_loader[0], info->version_loader[1], info->version_loader[2], info->version_loader[3],
         info->version_hardware[0], info->version_hardware[1], info->version_hardware[2], info->version_hardware[3],
         info->lidar_mac);

  printf("Direct_lidar_info_callback, lidar_id:%d, lidar_ipmode:%d, lidar_ip:%s, lidar_submask:%s, lidar_gateway:%s.\n",
          info->lidar_id, info->lidar_ipmode, info->lidar_ip, info->lidar_submask, info->lidar_gateway);

  printf("Direct_lidar_info_callback, host_push_msg_ip:%s, host_point_data_ip:%s, host_imu_data_ip:%s, "
         "host_push_msg_port:%u, host_point_data_port:%u, host_imu_data_port:%u.\n", info->host_push_msg_ip,
         info->host_point_data_ip, info->host_imu_data_ip, info->host_push_msg_port, info->host_point_data_port, 
         info->host_imu_data_port);

  printf("Direct_lidar_info_callback, sample_mode:%u, pattern_mode:%u, pcl_data_type:%u, imu_data_en:%u, work_mode:%u, "
         "work_state:%u, core_temp:%d.\n", info->sample_mode, info->pattern_mode, info->pcl_data_type, info->imu_data_en,
         info->work_mode, info->work_state, info->core_temp);

  printf("Direct_lidar_info_callback, roll_deg:%f, pitch_deg:%f, yaw_deg:%f, x:%d, y:%d, z:%d.\n",
         info->install_attitude.roll_deg, info->install_attitude.pitch_deg, info->install_attitude.yaw_deg,
         info->install_attitude.x, info->install_attitude.y, info->install_attitude.z);
}

void PointCallback(const uint32_t handle, LivoxDirectEthPacket *lidar_data, uint32_t data_num, void *client_data) {
  if (lidar_data == NULL) {
    printf("Point data callback failed, the data is null\n");
  }

  // uint32_t point_data_length = lidar_data->length - sizeof(LivoxDirectEthPacket) + 1;
  // uint16_t size = 0;
  // if (lidar_data->data_type == kCartesianCoordinateHighData) {
  //   size = point_data_length / sizeof(LivoxDirectCartesianHighRawPoint);
  //   LivoxDirectCartesianHighRawPoint* high_point = (LivoxDirectCartesianHighRawPoint*)(&lidar_data->data[0]);
  //   for (size_t i = 0; i < size; ++ i) {
  //     printf("PointCallback high point data, size:%u, data_num:%u, x:%d, y:%d, z:%d, reflectivity:%u, tag:%u.\n",
  //         size, data_num, high_point[i].x, high_point[i].y, high_point[i].z, high_point[i].reflectivity, high_point[i].tag);
  //   }
  // } else if (lidar_data->data_type == kCartesianCoordinateLowData) {
  //   size = point_data_length / sizeof(LivoxDirectCartesianLowRawPoint);
  //   LivoxDirectCartesianLowRawPoint* low_point = (LivoxDirectCartesianLowRawPoint*)(&lidar_data->data[0]);
  //   for (size_t i = 0; i < size; ++ i) {
  //     printf("PointCallback low point data, size:%u, data_num:%u, x:%d, y:%d, z:%d, reflectivity:%u, tag:%u.\n",
  //            size, data_num, low_point[i].x, low_point[i].y, low_point[i].z, low_point[i].reflectivity, low_point[i].tag);
  //   }
  // } else if (lidar_data->data_type == kSphericalCoordinateData) {
  //   size = point_data_length / sizeof(LivoxDirectSpherPoint);
  //   LivoxDirectSpherPoint* point = (LivoxDirectSpherPoint*)(&lidar_data->data[0]);
  //   for (size_t i = 0; i < size; ++ i) {
  //     printf("PointCallback spher point data, size:%u, data_num:%u, depth:%d, theta:%d, phi:%d, reflectivity:%u, tag:%u.\n",
  //           size, data_num, point[i].depth, point[i].theta, point[i].phi, point[i].reflectivity, point[i].tag);
  //   }
  // } else {
  //   printf("PointCallback Unknow_DATA data_type:%u.\n", lidar_data->data_type);
  // }
}

void ImuCallback(const uint32_t handle, LivoxDirectEthPacket* data, uint32_t data_num, void* client_data) {
  if (data == NULL || data->data_type != 0) {
    printf("Imu data callback failed, the data is null.\n");
  }

  // uint32_t imu_data_length = data->length - sizeof(LivoxDirectEthPacket) + 1;
  // uint16_t size = imu_data_length / sizeof(LivoxDirectImuRawPoint);
  // LivoxDirectImuRawPoint* imu_point = (LivoxDirectImuRawPoint*)(&data->data[0]);
  // for (size_t i = 0; i < size; ++i) {
  //   printf("IMUCallback IMU_DATA, data_num:%u, index:%lu, gyro x:%f, y:%f, z:%f; acc x:%f, y:%f, z:%f.\n",
  //       data_num, i, imu_point[i].gyro_x, imu_point[i].gyro_y, imu_point[i].gyro_z,
  //       imu_point[i].acc_x, imu_point[i].acc_y, imu_point[i].acc_z);
  // }
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("The number of parameters entered is incorrect, must enter the json file path.\n");
    return -1;
  }

  std::string json_path = argv[1];
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
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  SetDirectLidarCfgUpdateCallback(LidarCfgUpdateCallabck, NULL);
  SetDirectLidarInfoCallback(DirectLidarInfo, NULL);
  SetDirectLidarPointDataCallback(PointCallback, NULL);
  SetDirectLidarImuDataCallback(ImuCallback, NULL);

  if (!LivoxDirectStart()) {
    return -1;
  }

#ifdef WIN32
  Sleep(5000);
#else
  sleep(5);
#endif

  DirectLidarCfg direct_lidar_cfg;
  strcpy(direct_lidar_cfg.sn, "47MDK150010051");
  direct_lidar_cfg.lidar_id = 40;
  direct_lidar_cfg.lidar_ipmode = 1;

  strcpy(direct_lidar_cfg.lidar_ipinfo_cfg.lidar_ipaddr, "192.168.1.40");
  strcpy(direct_lidar_cfg.lidar_ipinfo_cfg.lidar_subnet_mask, "255.255.255.0");
  strcpy(direct_lidar_cfg.lidar_ipinfo_cfg.lidar_gateway, "192.168.1.1");

  strcpy(direct_lidar_cfg.host_cfg.host_push_cmd_ip, "192.168.1.100");
  direct_lidar_cfg.host_cfg.host_push_cmd_port = 56201;

  strcpy(direct_lidar_cfg.host_cfg.host_point_data_ip, "192.168.1.100");
  direct_lidar_cfg.host_cfg.host_point_data_port = 57301;

  strcpy(direct_lidar_cfg.host_cfg.host_imu_data_ip, "192.168.1.100");
  direct_lidar_cfg.host_cfg.host_imu_data_port = 57401;

  direct_lidar_cfg.sample_mode = 1;
  direct_lidar_cfg.pattern_mode = 1;
  direct_lidar_cfg.pcl_data_type = 2;
  direct_lidar_cfg.imu_data_en = 0;
  direct_lidar_cfg.work_mode = 3;

  direct_lidar_cfg.install_attitude.roll_deg = 2.1;
  direct_lidar_cfg.install_attitude.pitch_deg = 2.1;
  direct_lidar_cfg.install_attitude.yaw_deg = 2.1;
  direct_lidar_cfg.install_attitude.x = 12;
  direct_lidar_cfg.install_attitude.y = 22;
  direct_lidar_cfg.install_attitude.z = 32;

  UpdateDirectLidarCfg(lidar_handle, &direct_lidar_cfg);

#ifdef WIN32
  Sleep(30000);
#else
  sleep(3000);
#endif

  return 0;
}
