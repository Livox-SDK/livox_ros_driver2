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

#ifndef LIVOX_LIDAR_DEF_H_
#define LIVOX_LIDAR_DEF_H_

#include <stdint.h>

#define kMaxLidarCount 32

#pragma pack(1)

#define LIVOX_LIDAR_SDK_MAJOR_VERSION       3
#define LIVOX_LIDAR_SDK_MINOR_VERSION       1
#define LIVOX_LIDAR_SDK_PATCH_VERSION       1

#define kBroadcastCodeSize 16

/** Fuction return value defination, refer to \ref LivoxStatus. */
typedef int32_t livox_status;


/** The numeric version information struct.  */
typedef struct {
  int major;      /**< major number */
  int minor;      /**< minor number */
  int patch;      /**< patch number */
} LivoxLidarSdkVer;

typedef struct {
  uint8_t dev_type;
  char sn[16];
  char lidar_ip[16];
} LivoxLidarInfo;

/** Device type. */
typedef enum {
  kLivoxLidarTypeHub = 0,
  kLivoxLidarTypeMid40 = 1,
  kLivoxLidarTypeTele = 2,
  kLivoxLidarTypeHorizon = 3,
  kLivoxLidarTypeMid70 = 6,
  kLivoxLidarTypeAvia = 7,
  kLivoxLidarTypeMid360 = 9,
  kLivoxLidarTypeIndustrialHAP = 10,
  kLivoxLidarTypeHAP = 15,
  kLivoxLidarTypePA = 16,
} LivoxLidarDeviceType;

typedef enum {
  kKeyPclDataType = 0x0000,
  kKeyPatternMode = 0x0001,
  kKeyDualEmitEnable = 0x0002,
  kKeyPointSendEnable = 0x0003,
  kKeyLidarIPCfg = 0x0004,
  kKeyStateInfoHostIPCfg = 0x0005,


  kKeyLidarPointDataHostIPCfg = 0x0006,
  kKeyLidarImuHostIPCfg = 0x0007,

  kKeyInstallAttitude = 0x0012,
  kKeyBlindSpotSet = 0x0013,

  kKeyRoiCfg0 = 0x0015,
  kKeyRoiCfg1 = 0x0016,

  kKeyRoiEn = 0x0017,
  kKeyDetectMode = 0x0018,
  kKeyFuncIOCfg = 0x0019,
  
  kKeyWorkMode = 0x001A,
  kKeyGlassHeat = 0x001B,
  kKeyImuDataEn = 0x001C,
  kKeyFusaEn = 0x001D,

  kKeyLogParamSet = 0x7FFF,

  kKeySn = 0x8000,
  kKeyProductInfo = 0x8001,
  kKeyVersionApp = 0x8002,
  kKeyVersionLoader = 0x8003,
  kKeyVersionHardware = 0x8004,
  kKeyMac = 0x8005,
  kKeyCurWorkState = 0x8006,
  kKeyCoreTemp = 0x8007,
  kKeyPowerUpCnt = 0x8008,
  kKeyLocalTimeNow = 0x8009,
  kKeyLastSyncTime = 0x800A,
  kKeyTimeOffset = 0x800B,
  kKeyTimeSyncType = 0x800C,
  
  kKeyStatusCode = 0x800D,
  kKeyLidarDiagStatus = 0x800E,
  kKeyLidarFlashStatus = 0x800F,
  kKeyHmsCode = 0x8011,
  kKeyFwType = 0x8010,
  
  kKeyLidarDiagInfoQuery = 0xFFFF
} ParamKeyName;

typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;      /**< unit: 0.1 us */
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  uint8_t data[1];             /**< Point cloud data. */
} LivoxLidarEthernetPacket;

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
} LivoxLidarImuRawPoint;

typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarCartesianHighRawPoint;

typedef struct {
  int16_t x;            /**< X axis, Unit:cm */
  int16_t y;            /**< Y axis, Unit:cm */
  int16_t z;            /**< Z axis, Unit:cm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarCartesianLowRawPoint;

typedef struct {
  uint32_t depth;
  uint16_t theta;
  uint16_t phi;
  uint8_t reflectivity;
  uint8_t tag;
} LivoxLidarSpherPoint;

typedef enum {
  kLivoxLidarImuData = 0,
  kLivoxLidarCartesianCoordinateHighData = 0x01,
  kLivoxLidarCartesianCoordinateLowData = 0x02,
  kLivoxLidarSphericalCoordinateData = 0x03
} LivoxLidarPointDataType;

typedef enum {
  kLivoxLidarStatusSendFailed = -9,           /**< Command send failed. */
  kLivoxLidarStatusHandlerImplNotExist = -8,  /**< Handler implementation not exist. */
  kLivoxLidarStatusInvalidHandle = -7,        /**< Device handle invalid. */
  kLivoxLidarStatusChannelNotExist = -6,      /**< Command channel not exist. */
  kLivoxLidarStatusNotEnoughMemory = -5,      /**< No enough memory. */
  kLivoxLidarStatusTimeout = -4,              /**< Operation timeouts. */
  kLivoxLidarStatusNotSupported = -3,         /**< Operation is not supported on this device. */
  kLivoxLidarStatusNotConnected = -2,         /**< Requested device is not connected. */
  kLivoxLidarStatusFailure = -1,              /**< Failure. */
  kLivoxLidarStatusSuccess = 0                /**< Success. */
} LivoxLidarStatus;

typedef struct {
  uint16_t key;                /*< Key, refer to \ref DeviceParamKeyName. */
  uint16_t length;             /*< Length of value. */
  uint8_t value[1];            /*< Value. */
} LivoxLidarKeyValueParam;

typedef struct {
  uint8_t ret_code;
  uint16_t error_key;
} LivoxLidarAsyncControlResponse;

typedef struct {
  uint8_t ret_code;
  const char* lidar_info;
} LivoxLidarInfoResponse;

typedef struct {
  uint8_t ret_code;
  uint16_t param_num;
  uint8_t data[1];
} LivoxLidarDiagInternalInfoResponse;

typedef enum {
  kLivoxLidarScanPatternNoneRepetive = 0x00,
  kLivoxLidarScanPatternRepetive = 0x01,
  kLivoxLidarScanPatternRepetiveLowFrameRate = 0x02
} LivoxLidarScanPattern;

typedef enum {
  kLivoxLidarFrameRate10Hz = 0x00,
  kLivoxLidarFrameRate15Hz = 0x01,
  kLivoxLidarFrameRate20Hz = 0x02,
  kLivoxLidarFrameRate25Hz = 0x03,
} LivoxLidarPointFrameRate;

typedef enum {
  kLivoxLidarNormal = 0x01,
  kLivoxLidarWakeUp = 0x02,
  kLivoxLidarSleep = 0x03,
  kLivoxLidarError = 0x04,
  kLivoxLidarPowerOnSelfTest = 0x05,
  kLivoxLidarMotorStarting = 0x06,
  kLivoxLidarMotorStoping = 0x07,
  kLivoxLidarUpgrade = 0x08
} LivoxLidarWorkMode;

typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  int32_t x; //mm
  int32_t y; //mm
  int32_t z; // mm
} LivoxLidarInstallAttitude;

typedef struct {
  int32_t yaw_start;
  int32_t yaw_stop;
  int32_t pitch_start;
  int32_t pitch_stop;
  uint32_t rsvd;
} RoiCfg;

typedef enum {
  kLivoxLidarDetectNormal = 0x00,
  kLivoxLidarDetectSensitive = 0x01
} LivoxLidarDetectMode;

typedef struct {
  uint8_t in0;
  uint8_t int1;
  uint8_t out0;
  uint8_t out1;
} FuncIOCfg;

typedef struct {
  char ip_addr[16];  /**< IP address. */
  char net_mask[16]; /**< Subnet mask. */
  char gw_addr[16];  /**< Gateway address. */
} LivoxLidarIpInfo;

typedef struct {
  char host_ip_addr[16];  /**< IP address. */
  uint16_t host_state_info_port;
  uint16_t lidar_state_info_port;
} HostStateInfoIpInfo;

typedef struct {
  char host_ip_addr[16];  /**< IP address. */
  uint16_t host_point_data_port;
  uint16_t lidar_point_data_port;
} HostPointIPInfo;

typedef struct {
  char host_ip_addr[16];  /**< IP address. */
  uint16_t host_imu_data_port; // resv
  uint16_t lidar_imu_data_port; // resv
} HostImuDataIPInfo;

typedef struct {
  uint8_t pcl_data_type;
  uint8_t pattern_mode;
  uint8_t dual_emit_en;
  uint8_t point_send_en;
  LivoxLidarIpInfo lidar_ip_info;
  HostPointIPInfo host_point_ip_info;
  HostImuDataIPInfo host_imu_ip_info;
  LivoxLidarInstallAttitude install_attitude;
  uint32_t blind_spot_set;
  uint8_t work_mode;
  uint8_t glass_heat;
  uint8_t imu_data_en;
  uint8_t fusa_en;
  char sn[16];
  char product_info[64];
  uint8_t version_app[4];
  uint8_t version_load[4];
  uint8_t version_hardware[4];
  uint8_t mac[6];
  uint8_t cur_work_state;
  uint64_t status_code;
} LivoxLidarStateInfo;

typedef struct {
  uint8_t pcl_data_type;
  uint8_t pattern_mode;
  
  LivoxLidarIpInfo livox_lidar_ip_info;
  HostStateInfoIpInfo host_state_info;
  HostPointIPInfo host_point_ip_info;
  HostImuDataIPInfo host_imu_data_ip_info;
  LivoxLidarInstallAttitude install_attitude;
  
  RoiCfg roi_cfg0;
  RoiCfg roi_cfg1;

  uint8_t roi_en;
  uint8_t work_mode;
  uint8_t imu_data_en;

  char sn[16];
  char product_info[64];
  uint8_t version_app[4];
  uint8_t version_load[4];
  uint8_t version_hardware[4];
  uint8_t mac[6];

  uint8_t cur_work_state;
  int32_t core_temp;
  uint32_t powerup_cnt;

  uint64_t local_time_now;
  uint64_t last_sync_time;
  int64_t time_offset;

  uint8_t time_sync_type;

  uint16_t diag_status;
  uint8_t fw_type;
  uint32_t hms_code[8];
} DirectLidarStateInfo;

typedef struct {
  uint8_t ret_code;
  LivoxLidarStateInfo livox_lidar_state_info;
} LivoxLidarQueryInternalInfoResponse;

typedef enum {
  kLivoxLidarStopPowerOnHeatingOrDiagnosticHeating = 0x00,
  kLivoxLidarTurnOnHeating = 0x01,
  kLivoxLidarDiagnosticHeating = 0x02,
  kLivoxLidarStopSelfHeating = 0x03
} LivoxLidarGlassHeat;

typedef struct {
  uint8_t log_send_method;
  uint16_t log_id;
  uint16_t log_frequency;
  uint8_t is_save_setting;
  uint8_t check_code;
} LivoxLidarLogParam;

/**
 * Upgrade related data struct
 */
typedef struct {
  uint8_t firmware_type;    /**< firmware type. */
  uint8_t encrypt_type;     /**< encrypt type. */
  uint32_t firmware_length; /**< the length of firmware. */
  uint8_t dev_type;         /**< the device type of the firmware. */
} LivoxLidarStartUpgradeRequest;

typedef struct {
  uint8_t firmware_type;     /**< firmware type. */
  uint8_t encrypt_type;      /**< encrypt type. */
  uint32_t firmware_length;  /**< the length of firmware. */
  uint8_t dev_type;          /**< the device type of the firmware. */
  uint32_t firmware_version; /**< the version of this firmware. */
  uint64_t firmware_buildtime; /**< the buildtime of this firmware. */
  uint8_t hw_whitelist[32]; /**< the hardware version list that this firmware can be used for. */
} LivoxLidarStartUpgradeRequestV3;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} LivoxLidarStartUpgradeResponse;

typedef struct {
  uint32_t offset; /**< Return code. */
  uint32_t length; /**< Working state. */
  uint8_t encrypt_type;
  uint8_t rsvd[3];
  uint8_t data[1]; /**< LiDAR feature. */
} LivoxLidarXferFirmwareResquest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint32_t offset;  /**< Return code. */
  uint32_t length;  /**< Working state. */
} LivoxLidarXferFirmwareResponse;

typedef struct {
  uint8_t checksum_type;   /**< Return code. */
  uint8_t checksum_length; /**< Working state. */
  uint8_t checksum[1];     /**< LiDAR feature. */
} LivoxLidarCompleteXferFirmwareResquest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} LivoxLidarCompleteXferFirmwareResponse;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint8_t progress; /**< progress of upgrade. */
} LivoxLidarGetUpgradeProgressResponse;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint16_t length;  /**< The length of firmware info string, include '\0'. */
  uint8_t info[1];  /**< Firmware info string, include '\0'. */
} LivoxLidarRequestFirmwareInfoResponse;

typedef struct {
  uint16_t timeout; /**< delay reboot time */
} LivoxLidarRebootRequest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} LivoxLidarRebootResponse;

typedef struct {
  uint8_t data[16];
} LivoxLidarResetRequest;

typedef struct {
  uint8_t ret_code;
} LivoxLidarResetResponse;

#pragma pack()

#endif  // LIVOX_LIDAR_DEF_H_
