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
#ifndef LIVOX_ROS_DRIVER_COMM_H_
#define LIVOX_ROS_DRIVER_COMM_H_

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <map>

#include "ldq.h"
#include "lidar_imu_data_queue.h"

#include "livox_def.h"
#include "livox_sdk.h"
#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"
#include "livox_def_direct.h"
#include "livox_sdk_direct.h"
#include "livox_def_common.h"

namespace livox_ros {

/** Max lidar data source num */
const uint8_t kMaxSourceLidar = 32;

/** Eth packet relative info parama */
const uint32_t kMaxPointPerEthPacket = 100;
const uint32_t kMinEthPacketQueueSize = 32;   /**< must be 2^n */
const uint32_t kMaxEthPacketQueueSize = 131072; /**< must be 2^n */
const uint32_t kImuEthPacketQueueSize = 256;

const uint32_t KEthPacketHeaderLength = 18; /**< (sizeof(LivoxEthPacket) - 1) */
// const uint32_t KEthPacketMaxLength          = 1500;
const uint32_t KCartesianPointSize = 13;
const uint32_t KSphericalPointSzie = 9;

const uint64_t kRosTimeMax = 4294967296000000000; /**< 2^32 * 1000000000ns */
const int64_t kPacketTimeGap = 1000000; /**< 1ms = 1000000ns */
/**< the threshold of packet continuous */
const int64_t kMaxPacketTimeGap = 1700000;
/**< the threshold of device disconect */
const int64_t kDeviceDisconnectThreshold = 1000000000;
const int64_t kNsPerSecond = 1000000000; /**< 1s  = 1000000000ns */

const int kPathStrMinSize = 4;   /**< Must more than 4 char */
const int kPathStrMaxSize = 256; /**< Must less than 256 char */
const int kBdCodeSize = 15;

const uint32_t kPointXYZRSize = 16;
const uint32_t kPointXYZRTRSize = 18;

const double PI = 3.14159265358979323846;

constexpr uint32_t kMaxBufferSize = 0x8000;  // 32k bytes

/** Lidar connect state */
typedef enum {
  kConnectStateOff = 0,
  kConnectStateOn = 1,
  kConnectStateConfig = 2,
  kConnectStateSampling = 3,
} LidarConnectState;

/** Device data source type */
typedef enum {
  kSourceRawLidar = 0, /**< Data from raw lidar. */
  kSourceRawHub = 1,   /**< Data from lidar hub. */
  kSourceLvxFile,      /**< Data from parse lvx file. */
  kSourceUndef,
} LidarDataSourceType;

typedef enum { kCoordinateCartesian = 0, kCoordinateSpherical } CoordinateType;

typedef enum {
  kConfigFan = 1 << 0,
  kConfigReturnMode = 1 << 1,
  kConfigCoordinate = 1 << 2,
  kConfigImuRate = 1 << 3,
  kConfigGetExtrinsicParameter = 1 << 4,
  kConfigSetHighSensitivity = 1 << 5,
  kConfigUndef
} LidarConfigCodeBit;


typedef enum {
  kConfigDataType = 1 << 0,
  kConfigScanPattern = 1 << 1,
  kConfigBlindSpot = 1 << 2,
  kConfigDualEmit = 1 << 3,
  kConfigUnknown
} VehicleLidarConfigCodeBit;

typedef enum {
  kNoneExtrinsicParameter,
  kExtrinsicParameterFromLidar,
  kExtrinsicParameterFromXml
} ExtrinsicParameterType;

/** Configuration in json config file for livox lidar */
typedef struct {
  char broadcast_code[16];
  bool enable_connect;
  bool enable_fan;
  uint32_t return_mode;
  uint32_t coordinate;
  uint32_t imu_rate;
  uint32_t extrinsic_parameter_source;
  bool enable_high_sensitivity;
} UserRawConfig;

typedef struct {
  bool enable_fan;
  uint32_t return_mode;
  uint32_t coordinate;              /**< 0 for CartesianCoordinate; others for SphericalCoordinate. */
  uint32_t imu_rate;
  uint32_t extrinsic_parameter_source;
  bool enable_high_sensitivity;
  volatile uint32_t set_bits;
  volatile uint32_t get_bits;
} UserConfig;

typedef struct {
  uint8_t slot;
  uint8_t data_type;
  uint8_t scan_pattern;
  uint32_t blind_spot_set;
  bool dual_emit_enable;
  volatile uint32_t set_bits;
  volatile uint32_t get_bits;
} UserVehicleConfig;

typedef struct {
  char sn[16];
  uint8_t lidar_id;
  uint8_t lidar_ipmode;

  char lidar_ip[16];
  char lidar_submask[16];
  char lidar_gateway[16];

  char host_push_msg_ip[16];
  uint16_t host_push_msg_port;
  char host_point_data_ip[16];
  uint16_t host_point_data_port;
  char host_imu_data_ip[16];
  uint16_t host_imu_data_port;

  uint8_t sample_mode;
  uint8_t pattern_mode;
  uint8_t pcl_data_type;
  uint8_t imu_data_en;
  uint8_t work_mode;
  uint8_t work_state;
} UserDirectConfig;

typedef struct {
  uint32_t handle;
  int8_t data_type;
  int8_t pattern_mode;
  int32_t blind_spot_set;
  int8_t dual_emit_en;
  volatile uint32_t set_bits;
  volatile uint32_t get_bits;
} UserLivoxLidarConfig;

/** Lidar data source info abstract */
typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  // union {
  //   uint8_t slot : 4; //slot for LivoxLidarType::kVehicleLidarType
  //   uint8_t handle : 4;  // handle for LivoxLidarType::kIndustryLidarType
  // };
  uint8_t data_src;                  /**< From raw lidar or livox file. */
  volatile LidarConnectState connect_state;
  DeviceInfo info;

  LidarDataQueue data;
  LidarImuDataQueue imu_data;

  uint32_t firmware_ver; /**< Firmware version of lidar  */
  UserConfig config;
  UserVehicleConfig vehicle_config;
  UserDirectConfig direct_config;
  UserLivoxLidarConfig livox_config;
} LidarDevice;

struct DirectLidarParam {
 public:
  DirectLidarParam() : is_custom(false), direct_host_cfg_ptr(nullptr), direct_lidars_cfg_ptr(nullptr) {}
 public:
  bool is_custom;
  std::shared_ptr<DirectLidarHostCfg> direct_host_cfg_ptr;
  std::shared_ptr<std::vector<DirectLidarCfg>> direct_lidars_cfg_ptr;
  std::map<uint32_t, LidarExtrinsicParameters> extrinsic_params_map;
  std::map<uint8_t, std::string> devices;
};

typedef struct {
 uint8_t lidar_type;
} LidarSummaryInfo;

#pragma pack(1)

typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzrtl;

#pragma pack()

constexpr uint32_t kMaxProductType = 10;
constexpr uint32_t kDeviceTypeLidarMid70 = 6;

/**
 * Global function for general use.
 */
bool IsFilePathValid(const char *path_str);
uint32_t CalculatePacketQueueSize(const double publish_freq);
void ParseCommandlineInputBdCode(const char *cammandline_str, std::vector<std::string> &bd_code_list);
std::string IpNumToString(uint32_t ip_num);
uint32_t IpStringToNum(std::string ip_string);

}

#endif