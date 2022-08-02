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

#ifndef LIVOX_DEF_DIRECT_H_
#define LIVOX_DEF_DIRECT_H_

#include <stdint.h>

#define kMaxLidarCount 32

#pragma pack(1)

#define LIVOX_SDK_DIRECT_MAJOR_VERSION       2
#define LIVOX_SDK_DIRECT_MINOR_VERSION       3
#define LIVOX_SDK_DIRECT_PATCH_VERSION       1

#define kBroadcastCodeSize 16

/** Fuction return value defination, refer to \ref LivoxStatus. */
typedef int32_t livox_direct_status;


/** The numeric version information struct.  */
typedef struct {
  int major;      /**< major number */
  int minor;      /**< minor number */
  int patch;      /**< patch number */
} LivoxSdkDirectVersion;

typedef struct {
  char host_push_cmd_ip[16];
  char host_point_data_ip[16];
  char host_imu_data_ip[16];
  uint16_t host_cmd_port;
  uint16_t host_push_cmd_port;
  uint16_t host_point_data_port;
  uint16_t host_imu_data_port;
  uint16_t host_log_port;
} DirectLidarHostCfg;

typedef struct {
  char lidar_ipaddr[16];
  char lidar_subnet_mask[16];
  char lidar_gateway[16];
} DirectLidarIpCfg;

typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  int32_t x; //mm
  int32_t y; //mm
  int32_t z; // mm
} InstallAttitude;

typedef struct {
  char sn[16];
  uint8_t lidar_id;
  uint8_t lidar_ipmode;
  DirectLidarIpCfg lidar_ipinfo_cfg;
  DirectLidarHostCfg host_cfg;
  uint8_t sample_mode;
  uint8_t pattern_mode;
  uint8_t pcl_data_type;
  uint8_t imu_data_en;
  InstallAttitude install_attitude;
  uint8_t work_mode;
} DirectLidarCfg;

typedef struct {
  char sn[16];
  uint8_t handle;
  char product_info[128];
  //uint8_t version_app[4];
  char version_app[16];
  uint8_t version_loader[4];
  uint8_t version_hardware[4];
  //uint8_t lidar_mac[6];
  char lidar_mac[24];
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

  InstallAttitude install_attitude;

  uint8_t work_mode;
  uint8_t work_state;
  int32_t core_temp;
} DirectLidarStateInfo;

typedef struct {
  uint8_t version;              /**< Packet protocol version. */
  uint8_t lidar_id;
  uint16_t length;
  uint16_t time_interval;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t rsv;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t timestamp[8]; 
  uint8_t rsvd[16];
  uint8_t data[1];              /**< Point cloud data. */
} LivoxDirectEthPacket;

typedef enum {
  kDirectStatusSendFailed = -9,           /**< Command send failed. */
  kDirectStatusHandlerImplNotExist = -8,  /**< Handler implementation not exist. */
  kDirectStatusInvalidHandle = -7,        /**< Device handle invalid. */
  kDirectStatusChannelNotExist = -6,      /**< Command channel not exist. */
  kDirectStatusNotEnoughMemory = -5,      /**< No enough memory. */
  kDirectStatusTimeout = -4,              /**< Operation timeouts. */
  kDirectStatusNotSupported = -3,         /**< Operation is not supported on this device. */
  kDirectStatusNotConnected = -2,         /**< Requested device is not connected. */
  kDirectStatusFailure = -1,              /**< Failure. */
  kDirectStatusSuccess = 0                /**< Success. */
} LivoxDirectStatus;

/**Direct Lidar IP mode. */
typedef enum {
  kDirectLidarDynamicIpMode = 0,   /**< Dynamic IP. */
  kDirectLidarStaticIpMode = 1     /**< Static IP. */
} DirectLidarIpMode;

/**Direct Lidar scan pattern mode. */
typedef enum {
  kDirectLidarScanPatternNoneRepetive = 0x00,
  kDirectLidarScanPatternRepetive = 0x01,
} DirectLidarScanPattern;

/**Direct Lidar point data type. */
typedef enum {
  kDirectLidarImuData = 0x00,
  kDirectLidarHighResolutionPointData = 0x01,
  kDirectLidarLowResolutionPointData = 0x02,
  kDirectLidarSpherical = 0x03
} DirectLidarPointDataType;

/**Direct Lidar imu data enable. */
typedef enum {
  kDirectLidarImuDataEnable = 0,
  kDirectLidarImuDataDisable = 0x02,
} DirectLidarImuDataEnable;

/**Direct Lidar work mode. */
typedef enum {
  kDirectLidarPowerOff = 0,
  kDirectLidarIdle = 1,
  kDirectLidarReady = 2,
  kDirectLidarWorking = 3,
  kDirectLidarError = 4,
  kDirectLidarCheck = 5,
  kDirectLidarStartUp = 6,
  kDirectLidarUpgrade = 7
} DirectLidarWorkMode;

typedef struct {
  uint16_t key;                /*< Key, refer to \ref DeviceParamKeyName. */
  uint16_t length;             /*< Length of value. */
  uint8_t value[1];            /*< Value. */
} DirectKeyValueParam;

typedef struct {
  float gyro_x;            
  float gyro_y;
  float gyro_z;
  
  float acc_x;             
  float acc_y;
  float acc_z;
} LivoxDirectImuRawPoint;

typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxDirectCartesianHighRawPoint;

typedef struct {
  int16_t x;            /**< X axis, Unit:cm */
  int16_t y;            /**< Y axis, Unit:cm */
  int16_t z;            /**< Z axis, Unit:cm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxDirectCartesianLowRawPoint;

typedef struct {
  uint32_t depth;
  uint16_t theta;
  uint16_t phi; 
  uint8_t reflectivity;
  uint8_t tag;
} LivoxDirectSpherPoint;

typedef enum {
  kImuData = 0,
  kCartesianCoordinateHighData = 1,
  kCartesianCoordinateLowData = 2,
  kSphericalCoordinateData = 3
} DirectPointDataType;


typedef struct {
  uint8_t ret_code;
  uint16_t error_key;
} CommandResponse;

typedef struct {
  char sn[16];
  uint8_t lidar_id;
  CommandResponse res;
} DirectLidarCmdResInfo;

typedef struct {
  uint16_t timeout; /**< delay reboot time */
} DirectRebootRequest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} DirectRebootResponse;

typedef struct {
  uint16_t data;
} DirectLidarResetRequest;

typedef struct {
  uint8_t ret_code;
} DirectLidarResetResponse;


/**
 * Upgrade related data struct
 */
typedef struct {
  uint8_t firmware_type;    /**< firmware type. */
  uint8_t encrypt_type;     /**< encrypt type. */
  uint32_t firmware_length; /**< the length of firmware. */
  uint8_t dev_type;         /**< the device type of the firmware. */
} DirectStartUpgradeRequest;

typedef struct {
  uint8_t firmware_type;     /**< firmware type. */
  uint8_t encrypt_type;      /**< encrypt type. */
  uint32_t firmware_length;  /**< the length of firmware. */
  uint8_t dev_type;          /**< the device type of the firmware. */
  uint32_t firmware_version; /**< the version of this firmware. */
  uint64_t firmware_buildtime; /**< the buildtime of this firmware. */
  uint8_t hw_whitelist[32]; /**< the hardware version list that this firmware can be used for. */
} DirectStartUpgradeRequestV3;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} DirectStartUpgradeResponse;

typedef struct {
  uint32_t offset; /**< Return code. */
  uint32_t length; /**< Working state. */
  uint8_t encrypt_type;
  uint8_t rsvd[3];
  uint8_t data[1]; /**< LiDAR feature. */
} DirectXferFirmwareResquest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint32_t offset;  /**< Return code. */
  uint32_t length;  /**< Working state. */
} DirectXferFirmwareResponse;

typedef struct {
  uint8_t checksum_type;   /**< Return code. */
  uint8_t checksum_length; /**< Working state. */
  uint8_t checksum[1];     /**< LiDAR feature. */
} DirectCompleteXferFirmwareResquest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} DirectCompleteXferFirmwareResponse;

/*typedef struct {*/
/* no command data */
/*} GetUpgradeProgressResquest;*/

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint16_t length;  /**< The length of firmware info string, include '\0'. */
  uint8_t info[1];  /**< Firmware info string, include '\0'. */
} DirectRequestFirmwareInfoResponse;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint8_t progress; /**< progress of upgrade. */
} DirectGetUpgradeProgressResponse;

#pragma pack()

#endif  // LIVOX_DEF_H_
