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

#ifndef LIVOX_DEVICE_LIDAR_H_
#define LIVOX_DEVICE_LIDAR_H_

#include <stdint.h>
#include <chrono>

#include "livox_sdk_vehicle.h"
#include "livox_def_vehicle.h"

namespace livox {

const uint32_t kMaxSupportLidarNum = 32;
const uint32_t kMaxFirmwareInfoItems = 8;
const uint32_t kMaxFirmwareInfoItemSize = 48;

typedef enum {
  kConnectStateOff = 0,
  kConnectStateHandshake,
  kConnectStateOn,
  kConnectStateGetLidarInfo,
  kConnectStateConfig,
  kConnectStateSampling
} LidarConnectState;

#pragma pack(1)
/** The port group of lidar */
// typedef struct {
//   char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code,
//                                               null-terminated string, 15
//                                               characters at most. */
//   int32_t handle;                          /**< Device handle. */
//   uint16_t data_port;                      /**< Point cloud data UDP port. */
//   uint16_t cmd_port;                       /**< Control command UDP port. */
//   uint16_t sensor_port;                    /**< IMU data UDP port. */
//   struct sockaddr_in client_addr;
//   uint32_t local_ip;
// } LidarConfig;

#pragma pack()

struct XPengStyleVersion {
    std::string fw_type; /**< App or Loader*/
    std::string hw_ver;  /**< H.x x(0~9,A~Z) */
    std::string app_ver; /**< Vx.y.z (0~9,A~Z) */
    std::string loader_ver; /**< Vx.y.z xyz(0~9,A~Z) */
};

struct Lidar {
  DeviceInfo dev_info_;
  uint32_t connect_state_;
  uint32_t local_ip_;
  uint32_t firmware_type; /**< in loader or application, get from 0xff cmd */
  uint32_t app_ver;
  uint32_t loader_ver;
  uint32_t hw_ver;

  struct XPengStyleVersion xpeng_ver;
};

}  // namespace livox
#endif  // LIVOX_DEVICE_LIDAR_H_
