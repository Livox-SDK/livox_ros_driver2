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

#include "livox_def_direct.h"

namespace livox {

namespace direct{

const uint16_t KDefaultTimeOut = 500;
static const uint32_t kMaxCommandBufferSize = 1400;

typedef struct {
  std::string host_push_msg_ip;
  uint16_t host_push_cmd_port;
  uint16_t host_cmd_port;
  //uint16_t host_timesync_port;

  std::string host_point_data_ip;
  uint16_t host_point_data_port;
  
  std::string host_imu_data_ip;
  uint16_t host_imu_data_port;
  uint16_t host_log_port;
} DirectHostIpInfo;

typedef struct {
  std::string lidar_ipaddr;
  std::string lidar_subnet_mask;
  std::string lidar_gateway;

  uint16_t lidar_push_cmd_port;
  uint16_t lidar_cmd_port;
  //uint16_t lidar_timesync_port;
  uint16_t lidar_point_data_port;
  uint16_t lidar_imu_data_port;
  uint16_t lidar_log_port;
} DirectLidarIpInfo;

// typedef struct {
//   float roll_deg;
//   float pitch_deg;
//   float yaw_deg;
//   int x;
//   int y;
//   int z;
// } InstallAttitude;

typedef struct {
  std::string sn;
  uint8_t lidar_id;
  uint8_t lidar_ipmode;
  std::shared_ptr<DirectLidarIpInfo> lidar_ipinfo_ptr;
  std::shared_ptr<DirectHostIpInfo> host_ipinfo_ptr;
  uint8_t sample_mode;
  uint8_t pattern_mode;
  uint8_t pcl_data_type;
  uint8_t imu_data_en;
  InstallAttitude install_attitude;
  uint8_t work_mode;
} DirectLidarInfo;

typedef enum {
  kKeySn = 0x0000,
  kKeyProductInfo = 0x0001,
  kKeyVersionApp = 0x0002,
  kKeyVersionLoader = 0x0003,
  kKeyVersionHardWare = 0x0004,
  kKeyLidarMac = 0x0005,

  kKeyLidarID = 0x0100,
  kKeyLidarIpMode = 0x0101,
  kKeyLidarIpAddr = 0x0102,
  kKyeHostIpAddr = 0x0103,
  kKeySampleMode = 0x0104,
  kKeyPatternMode = 0x0105,
  kKeyPclDataType = 0x0106,
  kKeyImuDataEn = 0x0107,
  kKeyInstallAttitude = 0x0108,
  kKeyWorkMode = 0x0109,

  kKeyRoiEn = 0x0110,
  

  kKeyWorkState = 0x0301,
  kKeyCoreTemp = 0x0302,

  // for upgrader
  kKeyRequestUpgrade = 0x0200,
  kKeyXferFirmware = 0x0201,
  kKeyCompleteXferFirmware = 0x0202,
  kKeyRequestUpgradeProgress = 0x0203,
  kKeyRequestFirmwareInfo = 0xFF
} DirectLidarParamKeyName;

typedef enum {
  kKeyPushCmdID = 0x001,
  kKeyCmdID = 0x101,
  kKeyRebootDevice = 0x10E,
  kKeyReSetDevice = 0x10F,
  kKeyTimeSyncID = 0x201,
  kKeyLogStartID = 0x301,
  kKeyLogEndID = 0x305
} DirectLidarCmdID;


typedef enum {
  kHostCmdPort = 56101,
  kHostPushCmdPort = 56201,
  kHostPointDataPort = 56301,
  kHostImuDataPort = 56401,
  kHostLogPort = 56501
} HostPort;

typedef enum {
  kLidarCmdPort = 56100,
  kLidarPushCmdPort = 56200,
  kLidarPointDataPort = 56300,
  kLidarImuDataPort = 56400,
  kLidarLogPort = 56500
} LidarPort;

 typedef enum {
  /** command type, which requires response from the receiver. */
  kCommandTypeCmd = 0,
  /** acknowledge type, which is the response of command type. */
  kCommandTypeAck = 1,
} CommandType;

typedef struct {
  uint8_t ret_code;
  uint16_t error_key;
} DirectLidarRes;

typedef struct {
  uint16_t list_len;
  uint16_t rsvd;
} DirectLidarStateInfoHead;

typedef struct {
  uint8_t host_id[32];
} SearchDeviceLoggerRequest;

typedef struct {
  uint8_t ret_code;
  uint8_t broadcast_code[16];
  uint8_t device_state;   // 0 for idle 1 for busy 2 for occupy 3 for error.
  uint8_t host_id[32];
} SearchDeviceLoggerResponse;

typedef struct {
  uint8_t log_type;
  uint32_t time_s;
  uint8_t rsvd[32];
} EnableDeviceLoggerRequest;

typedef struct {
  uint8_t ret_code;
  uint8_t log_type;
} EnableDeviceLoggerResponse;

typedef struct {
  uint8_t log_type;              // 0
  uint8_t file_index;            // file index
  uint8_t file_name[32];         // file_name + utc_time.log
  uint16_t header_length;
  uint32_t header_data[1];       // file_header
} CreateDeviceLoggerRequest;

typedef struct {
  uint8_t ret_code;
  uint8_t log_type;              // 0
  uint8_t file_index;            // file index
} CreateDeviceLoggerResponse;

typedef struct {
  uint8_t log_type;         // 0
  uint8_t file_index;       // file index
  uint8_t trans_state;      // 0 for cfg, 1 for log
  uint32_t trans_index;     //sequence of trans file
  uint16_t data_length;     // log data length 
  uint8_t  data[1];         //data of log 
} TransferDeviceLoggerRequest;

typedef struct {
  uint8_t ret_code;         // 0
  uint8_t log_type;         // file index
  uint8_t file_index;      // 0 for cfg, 1 for log
  uint32_t trans_index;     //sequence of trans file
} TransferDeviceLoggerReponse;

typedef struct {
  uint8_t log_type;   // 0
  uint8_t stop_type;  // 0 noraml, 1 abnormal
  uint16_t rsvd;
} StopDeviceLoggerRequest;

typedef struct {
  uint8_t ret_code;
  uint8_t log_type;   // 0
} StopDeviceLoggerResponse;

using DataCallback = std::function<void(const uint32_t handle, LivoxDirectEthPacket *data, uint32_t data_num, void *client_data)>;
using LidarInfoCallback = std::function<void(const uint32_t, DirectLidarStateInfo*, void*)>;
using LidarCommandCallback = std::function<void(const uint32_t handle, DirectLidarCmdResInfo* response, void* client_data)>;


} //namespace direct
} // namespace livox

# endif // DEFINE_H_
