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

#ifndef LIVOX_LIDAR_DEF_H_
#define LIVOX_LIDAR_DEF_H_

#include <stdint.h>

#define kMaxLidarCount 32

#pragma pack(1)

#define LIVOX_LIDAR_SDK_MAJOR_VERSION       1
#define LIVOX_LIDAR_SDK_MINOR_VERSION       5
#define LIVOX_LIDAR_SDK_PATCH_VERSION       2

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
  kLivoxLidarTypeMid360s = 35,
  kLivoxLidarTypeAvia2 = 40,
  kLivoxLidarTypeMid360l = 41,
} LivoxLidarDeviceType;

typedef enum {
  kKeyPclDataType             = 0x0000,
  kKeyPatternMode             = 0x0001,
  kKeyDualEmitEn              = 0x0002,
  kKeyPointSendEn             = 0x0003,
  kKeyLidarIpCfg              = 0x0004,
  kKeyStateInfoHostIpCfg      = 0x0005,
  kKeyLidarPointDataHostIpCfg = 0x0006,
  kKeyLidarImuHostIpCfg       = 0x0007,
  kKeyCtlHostIpCfg            = 0x0008,
  kKeyLogHostIpCfg            = 0x0009,
  
  kKeyVehicleSpeed            = 0x0010,
  kKeyEnvironmentTemp         = 0x0011,
  kKeyInstallAttitude         = 0x0012,
  kKeyBlindSpotSet            = 0x0013,
  kKeyFrameRate               = 0x0014,
  kKeyFovCfg0                 = 0x0015,
  kKeyFovCfg1                 = 0x0016,
  kKeyFovCfgEn                = 0x0017,
  kKeyDetectMode              = 0x0018,
  kKeyFuncIoCfg               = 0x0019,
  kKeyWorkModeAfterBoot       = 0x0020,
  kKeyWorkMode                = 0x001A,
  kKeyGlassHeat               = 0x001B,
  kKeyImuDataEn               = 0x001C,
  kKeyFusaEn                  = 0x001D,
  kKeyForceHeatEn             = 0x001E,
  kKeySetEscMode              = 0x0021,
  kKeySetFovMode              = 0x0022,
  kKeySetEchoMode             = 0x0024,
  kKeySetNTPServerIp          = 0x0025,
  kKeySetTimeFilterMode       = 0x0026,
  kKeySetITOCtrl              = 0x0027,
  kKeySetFogNoiseFilter       = 0x0028,
  kKeySetPclFreqMod           = 0x0029,
  kKeySetImuRange             = 0x002B,

  kKeyLogParamSet             = 0x7FFF,

  kKeySn                      = 0x8000,
  kKeyProductInfo             = 0x8001,
  kKeyVersionApp              = 0x8002,
  kKeyVersionLoader           = 0x8003,
  kKeyVersionHardware         = 0x8004,
  kKeyMac                     = 0x8005,
  kKeyCurWorkState            = 0x8006,
  kKeyCoreTemp                = 0x8007,
  kKeyPowerUpCnt              = 0x8008,
  kKeyLocalTimeNow            = 0x8009,
  kKeyLastSyncTime            = 0x800A,
  kKeyTimeOffset              = 0x800B,
  kKeyTimeSyncType            = 0x800C,  
  kKeyStatusCode              = 0x800D,
  kKeyLidarDiagStatus         = 0x800E,
  kKeyLidarFlashStatus        = 0x800F,
  kKeyFwType                  = 0x8010,
  kKeyHmsCode                 = 0x8011,
  kKeyCurGlassHeatState       = 0x8012,
  
  kKeyRoiMode                 = 0xFFFE,
  kKeyLidarDiagInfoQuery      = 0xFFFF
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
  uint8_t  sof;
  uint8_t  version;
  uint16_t length;
  uint32_t seq_num;
  uint16_t cmd_id;
  uint8_t  cmd_type;
  uint8_t  sender_type;
  char     rsvd[6];
  uint16_t crc16_h;
  uint32_t crc32_d;
  uint8_t  data[1];
} LivoxLidarCmdPacket;

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
} LivoxLidarImuRawPoint;

typedef struct {
  int32_t x1;            /**< X axis, Unit:mm */
  int32_t y1;            /**< Y axis, Unit:mm */
  int32_t z1;            /**< Z axis, Unit:mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  int32_t x2;            /**< X axis, Unit:mm */
  int32_t y2;            /**< Y axis, Unit:mm */
  int32_t z2;            /**< Z axis, Unit:mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
} LivoxLidarDoubleEchoRawPoint;

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
  kLivoxLidarSphericalCoordinateData = 0x03,
  kLivoxLidarDoubleEchoData          = 0x11
} LivoxLidarPointDataType;

typedef enum {
  kLivoxLidarRealTimeLog = 0,
  kLivoxLidarExceptionLog = 0x01
} LivoxLidarLogType;

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

typedef enum {
  kLivoxFogNoiseFilterDisable = 0x00,
  kLivoxRainFilterMode= 0x01,
  kLivoxFogFilterMode= 0x02
} LivoxFogNoiseFilterMode;

typedef enum {
  kLivoxItoCtrlDisable = 0x00,
  kLivoxItoCtrlEnable = 0x01,
  kLivoxItoCtrlAuto   = 0x02
} LivoxLidarItoCtrlMode;


typedef enum {
  kLivoxLidarPclFreq80k  = 0x00,
  kLivoxLidarPclFreq50k = 0x01,
  kLivoxLidarPclFreq100k = 0x02
} LivoxLidarPclFreqMod;

typedef enum {
  kLivoxLidarTimeFilterNormal = 0x00,
  kLivoxLidarTimeFilterSpec   = 0x01
} LivoxLidarTimeFilterMode;

typedef enum {
  kLivoxLidarImuOutRate200Hz = 0x00,
  kLivoxLidarImuOutRate500Hz = 0x01,
  kLivoxLidarImuOutRate100Hz = 0x02,
  kLivoxLidarImuOutRate50Hz  = 0x03
} LivoxLidarImuOutRate;

typedef enum {
  kLivoxLidarAccelRange4G  = 0x00,
  kLivoxLidarAccelRange8G  = 0x01,
  kLivoxLidarAccelRange16G = 0x02,
  kLivoxLidarAccelRange32G = 0x03
} LivoxLidarAccelRange;

typedef enum {
  kLivoxLidarGyroRange2000Dps    = 0x00,
  kLivoxLidarGyroRange1000Dps    = 0x01,
  kLivoxLidarGyroRange500Dps     = 0x02,
  kLivoxLidarGyroRange250Dps     = 0x03,
  kLivoxLidarGyroRange125Dps     = 0x04,
  kLivoxLidarGyroRange62_5Dps    = 0x05,
  kLivoxLidarGyroRange31_25Dps   = 0x06,
  kLivoxLidarGyroRange15_625Dps  = 0x07
} LivoxLidarGyroRange;

typedef struct {
  uint8_t imu_out_rate;   /**< IMU output rate, refer to \ref LivoxLidarImuOutRate. */
  uint8_t accel_range;    /**< Accelerometer range, refer to \ref LivoxLidarAccelRange. */
  uint8_t gyro_range;     /**< Gyroscope range, refer to \ref LivoxLidarGyroRange. */
} LivoxLidarImuRange;

typedef enum {
  kLivoxEscSpeedNormal = 0x00, // mid360s / mid360l 通用
  kLivoxEscSpeedSlow = 0x01, // mid360s / mid360l 通用
  kLivoxEscSpeedHigh = 0x02 // mid360l only
} LivoxLidarEscMode;

typedef struct {
  char host_ip[16];
} NTPServerIpInfo;

typedef enum {
  kLivoxSmallFovMode = 0x00,
  kLivoxBigFovMode   = 0x01,
} LivoxLidarFovMode;

typedef enum {
  kLivoxStrongEchoMode = 0x00,
  kLivoxFirstEchoMode  = 0x01,
} LivoxLidarEchoMode;

typedef enum {
  kLivoxLidarWorkModeAfterBootDefault = 0x00,
  kLivoxLidarWorkModeAfterBootNormal = 0x01,
  kLivoxLidarWorkModeAfterBootWakeUp = 0x02
} LivoxLidarWorkModeAfterBoot;

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
} FovCfg;

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
  bool lidar_log_enable;
  uint32_t lidar_log_cache_size;
  char lidar_log_path[1024];
} LivoxLidarLoggerCfgInfo;

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
  char ip_addr[16];  /**< IP address. */
  uint16_t dst_port;
  uint16_t src_port;
} LivoxIpCfg;

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
  uint8_t             pcl_data_type;            // 0x0000
  uint8_t             pattern_mode;             // 0x0001
  uint8_t             dual_emit_en;             // 0x0002
  uint8_t             point_send_en;            // 0x0003  
  LivoxLidarIpInfo    lidar_ipcfg;              // 0x0004
  HostStateInfoIpInfo host_state_info;          // 0x0005
  HostPointIPInfo     pointcloud_host_ipcfg;    // 0x0006
  HostImuDataIPInfo   imu_host_ipcfg;           // 0x0007
  LivoxIpCfg          ctl_host_ipcfg;           // 0x0008
  LivoxIpCfg          log_host_ipcfg;           // 0x0009
  
  int32_t             vehicle_speed;            // 0x0010
  int32_t             environment_temp;         // 0x0011  
  LivoxLidarInstallAttitude install_attitude;   // 0x0012
  uint32_t            blind_spot_set;           // 0x0013
  uint8_t             frame_rate;               // 0x0014
  FovCfg              fov_cfg0;                 // 0x0015
  FovCfg              fov_cfg1;                 // 0x0016
  uint8_t             fov_cfg_en;               // 0x0017 
  uint8_t             detect_mode;              // 0x0018
  uint8_t             func_io_cfg[4];           // 0x0019
  uint8_t             work_tgt_mode;            // 0x001A
  uint8_t             glass_heat;               // 0x001B
  uint8_t             imu_data_en;              // 0x001C
  uint8_t             fusa_en;                  // 0x001D
  uint8_t             esc_mode;                 // 0x0021
  uint8_t             fov_mode;                 // 0x0022
  uint8_t             echo_mode;                // 0x0024
  NTPServerIpInfo     ntp_server_ip;            // 0x0025
  uint8_t             time_filter_mode;         // 0x0026
  uint8_t             ito_mode;                 // 0x0027
  uint8_t             fog_noise_filter;         // 0x0028
  uint8_t             pcl_freq_mode;            // 0x0029
  LivoxLidarImuRange  imu_range;                // 0x002B

  char                sn[16];                   // 0x8000
  char                product_info[64];         // 0x8001
  uint8_t             version_app[4];           // 0x8002
  uint8_t             version_loader[4];        // 0x8003
  uint8_t             version_hardware[4];      // 0x8004
  uint8_t             mac[6];                   // 0x8005
  uint8_t             cur_work_state;           // 0x8006
  int32_t             core_temp;                // 0x8007
  uint32_t            powerup_cnt;              // 0x8008
  uint64_t            local_time_now;           // 0x8009
  uint64_t            last_sync_time;           // 0x800A
  int64_t             time_offset;              // 0x800B
  uint8_t             time_sync_type;           // 0x800C
  uint8_t             status_code[32];          // 0x800D
  uint16_t            lidar_diag_status;        // 0x800E
  uint8_t             lidar_flash_status;       // 0x800F
  uint8_t             fw_type;                  // 0x8010
  uint32_t            hms_code[8];              // 0x8011
  uint8_t             ROI_Mode;                 // 0xFFFE
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

typedef struct {
  uint8_t ret_code;
} LivoxLidarLoggerResponse;

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

/**
 * Upgrade related data struct
 */
typedef enum {
  kLivoxLidarUpgradeIdle = 0,
  kLivoxLidarUpgradeRequest = 1,
  kLivoxLidarUpgradeXferFirmware = 2,
  kLivoxLidarUpgradeCompleteXferFirmware = 3,
  kLivoxLidarUpgradeGetUpgradeProgress = 4,
  kLivoxLidarUpgradeComplete = 5,
  kLivoxLidarUpgradeTimeout = 6,
  kLivoxLidarUpgradeErr = 7,
  kLivoxLidarUpgradeUndef = 8
} LivoxLidarFsmState;

typedef enum {
  kLivoxLidarEventRequestUpgrade = 0,
  kLivoxLidarEventXferFirmware = 1,
  kLivoxLidarEventCompleteXferFirmware = 2,
  kLivoxLidarEventGetUpgradeProgress = 3,
  kLivoxLidarEventComplete = 4,
  kLivoxLidarEventReinit = 5,
  kLivoxLidarEventTimeout = 6,
  kLivoxLidarEventErr = 7,
  kLivoxLidarEventUndef = 8
} LivoxLidarFsmEvent;

typedef struct {
  LivoxLidarFsmEvent state;
  uint8_t progress;
} LivoxLidarUpgradeState;

typedef struct {
  uint8_t ret; // succ: 0, fail: 1
} LivoxLidarRmcSyncTimeResponse;

#pragma pack()

/**
 * Callback function for receiving point cloud data.
 * @param handle                 device handle.
 * @param data                   device's data.
 * @param data_num               number of points in data.
 * @param client_data            user data associated with the command.
 */
typedef void (*LivoxLidarPointCloudCallBack)(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);

/**
 * Callback function for receiving point cloud data.
 * @param handle                 device handle.
 * @param data                   device's command data.
 * @param client_data            user data associated with the command.
 */
typedef void (*LivoxLidarCmdObserverCallBack)(const uint32_t handle, const LivoxLidarCmdPacket* data, void* client_data);

/**
 * Callback function for point cloud observer.
 * @param handle                 device handle.
 * @param data                   device's data.
 * @param data_num               number of points in data.
 * @param client_data            user data associated with the command.
 */
typedef void (*LivoxLidarPointCloudObserver) (uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data);

/**
 * Callback function for receiving IMU data.
 * @param data                   device's data.
 * @param data_num               number of IMU data.
 * @param client_data            user data associated with the command.
 */
typedef void (*LivoxLidarImuDataCallback)(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);

/**
 * Callback function for receiving Status Info.
 * @param status_info            status info.
 * @param client_data            user data associated with the command.
 */
typedef void(*LivoxLidarInfoCallback)(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);

/**
 * Callback function for receiving Status Info.
 * @param status                 status info.
 * @param client_data            user data associated with the command.
 */
typedef void(*LivoxLidarInfoChangeCallback)(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);

typedef void (*QueryLivoxLidarInternalInfoCallback)(livox_status status, uint32_t handle,
        LivoxLidarDiagInternalInfoResponse* response, void* client_data);

/**
 * Callback function for receiving response from the lidar on the configuration of parameter.
 * @param  status                 status info.
 * @param  response               lidar return code and error key.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
typedef void (*LivoxLidarAsyncControlCallback)(livox_status status, uint32_t handle, 
                                               LivoxLidarAsyncControlResponse *response, void *client_data);
/**
 * Callback function for receiving the reset setting response from lidar.
 * @param  status                 status info.
 * @param  response               lidar return code.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
typedef void (*LivoxLidarResetCallback)(livox_status status, uint32_t handle, LivoxLidarResetResponse* response, void* client_data);


/**
 * Callback function for receiving the log setting response from lidar.
 * @param  status                 status info.
 * @param  response               lidar return code.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
typedef void (*LivoxLidarLoggerCallback)(livox_status status, uint32_t handle,
                                         LivoxLidarLoggerResponse* response, void* client_data);

/**
 * Callback function for receiving the Reboot setting response from lidar.
 * @param  status                 status info.
 * @param  response               lidar return code.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
typedef void (*LivoxLidarRebootCallback)(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data);

typedef void (*OnLivoxLidarUpgradeProgressCallback)(uint32_t handle, LivoxLidarUpgradeState state, void *client_data);

/**
 * Callback function for receiving point cloud data.
 * @param status                 status info.
 * @param handle                 device handle.
 * @param data                   device's command data.
 * @param client_data            user data associated with the command.
 */
typedef void (*LivoxLidarRmcSyncTimeCallBack)(livox_status status, uint32_t handle, LivoxLidarRmcSyncTimeResponse* data, void* client_data);

#endif  // LIVOX_LIDAR_DEF_H_
