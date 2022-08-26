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

#ifndef LIVOX_DEFINE_H_
#define LIVOX_DEFINE_H_

#include <stdio.h>
#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <atomic>

#include "livox_lidar_def.h"

namespace livox {
namespace lidar {

const uint16_t KDefaultTimeOut = 500;
static const uint32_t kMaxCommandBufferSize = 1400;

typedef struct {
  std::string lidar_ipaddr;
  std::string lidar_subnet_mask;
  std::string lidar_gateway;

  uint16_t cmd_data_port;
  uint16_t push_msg_port;
  uint16_t point_data_port;
  uint16_t imu_data_port;
  uint16_t log_data_port;
} LivoxLidarNetInfo;

typedef struct {
  std::string cmd_data_ip;
  uint16_t cmd_data_port;

  std::string push_msg_ip;
  uint16_t push_msg_port;

  std::string point_data_ip;
  uint16_t point_data_port;

  std::string imu_data_ip;
  uint16_t imu_data_port;

  std::string log_data_ip;
  uint16_t log_data_port;
} HostNetInfo;

typedef struct {
  std::vector<uint16_t> cmd_key_set;
} GeneralCfgInfo; 

typedef struct {
  uint8_t device_type;
  LivoxLidarNetInfo lidar_net_info;
  HostNetInfo host_net_info;
  GeneralCfgInfo general_cfg_info;
} LivoxLidarCfg;

typedef enum {
  /**
   * Lidar command set, set the working mode and sub working mode of a LiDAR.
   */
  kCommandIDLidarSearch = 0x0000,
  // kCommandIDLidarPreconfig = 0x01,
  
  kCommandIDLidarWorkModeControl = 0x0100,
  kCommandIDLidarGetInternalInfo = 0x0101,
  kCommandIDLidarPushMsg = 0x0102,

  kCommandIDLidarRebootDevice = 0x0200,
  kCommandIDLidarResetDevice = 0x0201,
  kCommandIDLidarSetPPSSync = 0x0202,

  kCommandIDLidarPushLog = 0x0300,
  kCommandIDLidarCollectionLog = 0x0301,
  kCommandIDLidarLogSysTimeSync = 0x0302,


  kCommandIDGeneralRequestUpgrade = 0x0400,
  kCommandIDGeneralXferFirmware = 0x0401,
  kCommandIDGeneralCompleteXferFirmware = 0x0402,
  kCommandIDGeneralRequestUpgradeProgress = 0x0403,
  kCommandIDGeneralRequestFirmwareInfo = 0xFF,
  kCommandIDLidarCommandCount
} LidarCommandID;

 typedef enum {
  /** command type, which requires response from the receiver. */
  kCommandTypeCmd = 0,
  /** acknowledge type, which is the response of command type. */
  kCommandTypeAck = 1,
} CommandType;

 typedef enum {
  /** command type, which requires response from the receiver. */
  kHostSend = 0,
  /** acknowledge type, which is the response of command type. */
  kLidarSend = 1,
} SendType;

typedef struct {
  uint8_t ret_code;
  uint8_t dev_type;
  char sn[16];
  uint8_t lidar_ip[4];
  uint16_t cmd_port;
} DetectionData;

typedef struct {
  uint32_t handle;
  uint16_t cmd_port;
  uint8_t dev_type;
  std::atomic<bool> is_get={false}; 
  std::atomic<bool> is_set={false};
} ViewDevice;


typedef struct {
  uint32_t handle;
  uint8_t dev_type;
  std::string host_ip;
  uint16_t lidar_cmd_port;

  uint16_t lidar_point_port;
  uint16_t lidar_imu_data_port;

  uint16_t host_point_port;
  uint16_t host_imu_data_port;
} ViewLidarIpInfo;


typedef struct {
  uint8_t lidar_ipaddr[4];
  uint8_t lidar_subnet_mask[4];
  uint8_t lidar_gateway[4];
} LivoxLidarIpInfoValue;

typedef struct {
  uint8_t host_ip[4];
  uint16_t host_port;
  uint16_t lidar_port;
} HostIpInfoValue;

static const uint16_t kDetectionPort = 56000;
static const uint16_t kDetectionListenPort = 56001;

static const uint16_t kHAPCmdPort = 56000;
static const uint16_t kHAPPointDataPort = 57000;
static const uint16_t kHAPIMUPort = 58000;
static const uint16_t kHAPLogPort = 59000;

static const uint16_t kLogPort = 59000;

static const uint16_t kMid360LidarCmdPort = 56100;
static const uint16_t kMid360LidarPushMsgPort = 56200;
static const uint16_t kMid360LidarPointCloudPort = 56300;
static const uint16_t kMid360LidarImuDataPort = 56400;
static const uint16_t kMid360LidarLogPort = 56500;

static const uint16_t kMid360HostCmdPort = 56101;
static const uint16_t kMid360HostPushMsgPort = 56201;
static const uint16_t kMid360HostPointCloudPort = 56301;
static const uint16_t kMid360HostImuDataPort = 56401;
static const uint16_t kMid360HostLogPort = 56501;


static const uint16_t kPaLidarCmdPort = 9347;
static const uint16_t kPaLidarPointCloudPort = 10000;
static const uint16_t kPaLidarFaultPort = 10001;
static const uint16_t kPaLidarLogPort = 1002;


typedef enum {
  kLidarCmdPort = 56100,
  kLidarPushCmdPort = 56200,
  kLidarPointDataPort = 56300,
  kLidarImuDataPort = 56400,
  kLidarLogPort = 56500
} LidarPort;

using DataCallback = std::function<void(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)>;
using LidarInfoCallback = std::function<void(const uint32_t, const uint8_t, const char*, void*)>;

} // namespace lidar
} // namespace livox

# endif // DEFINE_H_
