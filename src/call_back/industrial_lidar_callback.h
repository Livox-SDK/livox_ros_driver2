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
#ifndef LIVOX_ROS_DRIVER_INDUSTRIAL_LIDAR_CALLBACK_H_
#define LIVOX_ROS_DRIVER_INDUSTRIAL_LIDAR_CALLBACK_H_	

#include "../comm/comm.h"

#include "../lds.h"
#include "../lds_lidar.h"

#include "livox_sdk.h"
#include "livox_def_common.h"

namespace livox_ros {
/**
 * LiDAR data source, data from dependent lidar.
 */
class IndustrialLidarCallback {
 public:
  static void OnLidarDataCb(PointCloudFrame* frame, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  
  static void StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);
  static void StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);

  static void DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *client_data);
  
  static void LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message);
  static void ControlFanCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);
  
  static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);
  static void SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);
  static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);
  static void SetRmcSyncTimeCb(livox_status status, uint8_t handle, uint8_t response, void *client_data);
  
  static void ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length, void *client_data);

  static void GetLidarExtrinsicParameterCb(livox_status status, uint8_t handle,
      LidarGetExtrinsicParameterResponse *response, void *client_data);

  static void SetHighSensitivityCb(livox_status status, uint8_t handle, DeviceParameterResponse *response,
                                   void *client_data);
 private:
  static LidarDevice* GetLidarDevice(const uint8_t handle);
  static LidarDevice* GetLidarDevice(const uint8_t handle, void *client_data);
};

} // namespace

#endif
