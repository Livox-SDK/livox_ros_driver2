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

#ifndef LIVOX_DEF_VEHICLE_H_
#define LIVOX_DEF_VEHICLE_H_

#include <stdint.h>

/** Function return value definition. */
typedef enum {
  kVehicleStatusSendFailed = -9,           /**< Command send failed. */
  kVehicleStatusHandlerImplNotExist = -8,  /**< Handler implementation not exist. */
  kVehicleStatusInvalidHandle = -7,        /**< Device handle invalid. */
  kVehicleStatusChannelNotExist = -6,      /**< Command channel not exist. */
  kVehicleStatusNotEnoughMemory = -5,      /**< No enough memory. */
  kVehicleStatusTimeout = -4,              /**< Operation timeouts. */
  kVehicleStatusNotSupported = -3,         /**< Operation is not supported on this device. */
  kVehicleStatusNotConnected = -2,         /**< Requested device is not connected. */
  kVehicleStatusFailure = -1,              /**< Failure. */
  kVehicleStatusSuccess = 0                /**< Success. */
} LivoxVehicleStatus;

typedef enum {
  kVehicleFrameRate10Hz = 0x00,
  kVehicleFrameRate15Hz = 0x01,
  kVehicleFrameRate20Hz = 0x02,
  kVehicleFrameRate25Hz = 0x03,
} LivoxVehiclePointFrameRate;

/** Fuction return value defination, refer to \ref LivoxVehicleStatus. */
typedef int32_t livox_vehicle_status;


#pragma pack(1)

#define LIVOX_SDK_VEHICLE_MAJOR_VERSION       0
#define LIVOX_SDK_VEHICLE_MINOR_VERSION       0
#define LIVOX_SDK_VEHICLE_PATCH_VERSION       3

#define kBroadcastCodeSize 16

/** The numeric version information struct.  */
typedef struct {
  int major;      /**< major number */
  int minor;      /**< minor number */
  int patch;      /**< patch number */
} LivoxVehicleSdkVersion;

typedef struct {
  uint32_t ip_addr;  /**< IP address. */
  uint32_t net_mask; /**< Subnet mask. */
  uint32_t gw_addr;  /**< Gateway address. */
} LidarIpConfig;

typedef struct {
  uint32_t ip_addr;  /**< IP address. */
  uint32_t net_mask; /**< Subnet mask. */
  uint32_t gw_addr;  /**< Gateway address. */
  uint8_t slot_id;
} LidarPreConfigParam;

/** Point cloud packet. */
typedef struct {
  uint8_t version;
  uint8_t slot;
  uint16_t length;
  uint16_t time_interval;
  uint16_t frame_counter;
  uint8_t data_type;
  uint8_t timestamp_type;
  uint32_t crc32;
  uint8_t safety_info;
  uint8_t rsvd;
  uint8_t timestamp[8];         /**< Nanosecond or UTC format timestamp. */
  uint8_t data[1];              /**< Point cloud data. */
} LivoxVehicleEthPacket;

typedef struct {
  uint32_t system_status : 2;
  uint32_t rx_temp_err : 2;
  uint32_t rx_vol_err : 2;
  uint32_t rx_hv_err : 2;
  uint32_t rx_cur_err : 2;
  uint32_t tx_temp_err : 2;
  uint32_t tx_vol_err : 2;
  uint32_t tx_cur_err : 2; //16bits

  uint32_t sample_vol_err : 2;
  uint32_t tx_rx_link_err : 2;
  uint32_t motor_fatal_err : 2;
  uint32_t motor_speed_err : 2;
  uint32_t motor_vol_err : 2;
  uint32_t motor_cur_err : 2;
  uint32_t motor_temp_err : 2;
  uint32_t motor_comm_err : 2; //32bits

  uint32_t glass_heating_err : 2;
  uint32_t ext_vol_mon_err : 2;
  uint32_t motor_self_check_err : 2;
  uint32_t eth_link_err : 2;
  uint32_t eth_drv_err : 2;
  uint32_t pro_mon_err : 2;
  uint32_t cpu_temp_err : 2;
  uint32_t cpu_vol_err : 2; //48bits

  uint32_t ext_vol_err : 2;
  uint32_t ext_cur_err : 2;
  uint32_t glass_dirty_err : 2;
  uint32_t glass_block_err : 2;
  uint32_t internal_err : 2;
  uint32_t eth_comm_err : 2;
  uint32_t time_sync_err : 2;
  uint32_t eth_data_err : 2; //64bits
  
  uint32_t his_comm_err : 2;
  uint64_t rsvd : 62;
} LidarStatusCode;

typedef struct {
  uint64_t sys_time;
  uint8_t rx_broad_heating_status;
  uint16_t rx_broad_temp; //0.01
  uint16_t rx_broad_vol; //mV
  uint16_t apd_hv; //mV
  uint16_t apd_quiet_current;
  uint16_t rx_broad_current;
  uint8_t tx_broad_heating_status;
  uint16_t tx_broad_temp; //0.01
  uint16_t tx_broad_vol; //mV
  uint16_t tx_broad_current; //mA

  uint8_t motor1_esc_state;
  uint8_t motor2_esc_state;
  uint8_t motor3_esc_state;
  uint16_t motor1_self_check_code;
  uint16_t motor2_self_check_code;
  uint16_t motor3_self_check_code;
  uint16_t motor1_run_error_code;
  uint16_t motor2_run_error_code;
  uint16_t motor3_run_error_code;
  int16_t motor1_target_speed; //all speed : rpm
  int16_t motor2_target_speed;
  int16_t motor3_target_speed;
  int16_t motor1_esc_real_speed;
  int16_t motor2_esc_real_speed;
  int16_t motor3_esc_real_speed;
  int16_t motor1_esc_PL_speed;
  int16_t motor2_esc_PL_speed;
  int16_t motor3_esc_PL_speed;
  int16_t motor1_current; //mA
  int16_t motor2_current;
  int16_t motor3_current;
  int16_t motor1_power; //mW
  int16_t motor2_power;
  int16_t motor3_power;
  uint16_t motor_bus_vol; //mV
  uint16_t motor_bus_current; //mA
  int16_t esc_mcu_temp; //0.01
  uint16_t esc_comm_seq;
  uint8_t esc1_comm_loss_rate; //%
  uint8_t esc2_comm_loss_rate; //%
  uint8_t esc3_comm_loss_rate; //%
  uint8_t esc_gear_number[3];
  uint16_t motor1_sync_offset;
  uint16_t motor2_sync_offset;
  uint16_t motor3_sync_offset;
  int16_t motor1_sync_accuracy;
  int16_t motor2_sync_accuracy;
  int16_t motor3_sync_accuracy;
  int16_t esc1_Vout;
  int16_t esc2_Vout;
  int16_t esc3_Vout;

  uint8_t lidar_work_state;
  uint8_t cable_status;
  uint16_t eth_phy_status;
  uint16_t eth_drv_status;
  uint8_t watch_dog_reset_cnt;
  uint32_t err_ram_addr;
  uint32_t flash_err_status;
  int16_t MCU_LPD_temp; //0.01
  int16_t MCU_FPD_temp;
  int16_t MCU_PLPD_temp;
  int16_t MCU_LPD_temp_up;
  uint16_t vcc_psintlp; //mV
  uint16_t vcc_psaux;
  uint16_t vcc_psio1;
  uint16_t vcc_pspll;
  uint16_t vcc_psintfp;
  uint16_t vcc_psaddr_pll;
  uint16_t vcc_int;
  uint16_t PL_vcc_aux;
  uint16_t PL_2V5_io_vol;
  uint16_t PL_3V3_io_vol;
  uint8_t cur_reset_status;
  uint8_t total_reset_cnt;
  uint8_t unlock_reset_cnt;
  uint8_t pl_watch_dog_reset_cnt;
  uint8_t R5_bus_reset_cnt;
  uint8_t R5_reg_reset_cnt;
  uint8_t fpga_sample_status1;
  uint8_t fpga_sample_status2;
  uint8_t fpga_sample_out_status;
  uint32_t sample_rate;

  uint8_t heating_module_status;
  uint16_t heating_module_vol; //mV
  int16_t glass_temp; //0.01
  uint8_t heating_module_self_check;
  uint8_t heating_module_run_error;

  uint16_t main_power_voltage; //mV
  uint16_t main_power_current;
  uint16_t main_power_run_error;

  uint16_t tdc_timestamp;
  uint8_t glass_status;
  uint32_t tia_dc; //mV
  uint8_t T0_threshold_num;

  uint16_t point_data_rate;
  uint16_t point_err_seq_id;

  uint8_t imu_status;
  uint8_t imu_comm_loss_rate;

  uint64_t rsvd[12];
} LidarDiagInfo;

typedef struct {
  LidarStatusCode status_code;
  LidarDiagInfo diag_info;
} LivoxDetailExceptionInfo;

typedef struct {
  uint8_t slot;
  char ip_addr[16];
} LidarRegisterInfo;

typedef struct {
  uint8_t ret_code;
  uint8_t lidar_state; //work mode
  LidarStatusCode status_code;
  uint16_t error_key;
} LidarSyncControlResponse;

typedef struct {
  uint8_t work_mode;
  int32_t vehicle_speed;
  int32_t ev_temp;
  uint8_t slot;   //0xff no need to config
} SyncControlInfo;

/**
 * Key and value of device's parameters.
 */
typedef struct {
  uint16_t key;                /*< Key, refer to \ref DeviceParamKeyName. */
  uint16_t length;             /*< Length of value. */
  uint8_t value[1];            /*< Value. */
} VehicleKeyValueParam;

typedef struct {
  uint16_t key;                /*< Key, refer to \ref InnerInfoParamKeyName. */
} VehicleKeyListParam;

typedef enum {
  kVehicleKeyWorkMode = 0x00,
  kVehicleKeySpeed = 0x01,
  kVehicleKeyEnvironmentTemp =  0x02,
  kVehicleKeySlotNum = 0x03,
  kVehicleKeyAddr = 0x04,
  kVehicleKeyDeviceVer = 0x05,
  kVehicleKeyProductInfo = 0x06,
  kVehicleKeyPointType = 0x07,
  kVehicleKeyGlassHeat = 0x08,
  kVehicleKeyScanPattern = 0x09,
  kVehicleKeyBlindSpotSet = 0x0A,
  kVehicleKeyGlassTemp = 0x0B,
  kVehicleKeySafetyInfo = 0x0C,
  kVehicleKeyDualEmitEnable = 0x0D,
  kVehicleKeyPointSendEnable = 0x0E,
  kVehicleKeyFrameRate = 0x0F,
  kVehicleKeyFusaFunciont = 0x13,
  kVehicleKeyPointCloudMulticastIp = 0x14
} VehicleDeviceParamKeyName;

typedef enum {
	kVehicleLidarImuData = 0x00,
  kHighResolutionPointData = 0x01,
  kLowResolutionPointData = 0x02,
} VehiclePointDataType;

typedef enum {
  kStopPowerOnHeatingOrDiagnosticHeating = 0x00,
  kTurnOnHeating = 0x01,
  kDiagnosticHeating = 0x02,
  kStopSelfHeating = 0x03
} VehicleGlassHeat;

typedef enum {
  kScanPatternNoneRepetive = 0x00,
  kScanPatternRepetive = 0x01,
} VehicleScanPattern;

typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxVehicleExtendRawPoint;

typedef struct {
  int16_t x;            /**< X axis, Unit:cm */
  int16_t y;            /**< Y axis, Unit:cm */
  int16_t z;            /**< Z axis, Unit:cm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxVehicleExtendHalfRawPoint;

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;

  float acc_x;
  float acc_y;
  float acc_z;
} LivoxVehicleImuRawPoint;

typedef struct {
  uint8_t slot;
  uint8_t broadcast_code[16];
  uint32_t ip_addr;
} VehicleDeviceInfo;

typedef struct {
  uint8_t ret_code;
  uint8_t param_num;
  uint8_t data[1];
} LidarDiagInternalInfoResponse;

typedef struct {
  uint8_t log_type;
  uint8_t file_index;
  uint8_t rsvd;
  uint32_t trans_index;
  uint16_t data_length;
  uint8_t data[1];
} LidarLogInfo;

typedef struct {
  uint8_t ret_code;
  uint8_t log_type;
  uint8_t file_index;
  uint32_t trans_index;
} LidarLogInfoAck;

typedef struct {
  uint32_t safety_info;
} LidarSafetyInfo;

typedef struct {
  uint8_t error_enable;
  LidarStatusCode error_set;
} LidarFaultInfo;

typedef struct {
  uint16_t timeout; /**< delay reboot time */
} VehicleRebootRequest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} VehicleRebootResponse;

/**
 * Upgrade related data struct
 */
typedef struct {
  uint8_t firmware_type;    /**< firmware type. */
  uint8_t encrypt_type;     /**< encrypt type. */
  uint32_t firmware_length; /**< the length of firmware. */
  uint8_t dev_type;         /**< the device type of the firmware. */
} VehicleStartUpgradeRequest;

typedef struct {
  uint8_t firmware_type;     /**< firmware type. */
  uint8_t encrypt_type;      /**< encrypt type. */
  uint32_t firmware_length;  /**< the length of firmware. */
  uint8_t dev_type;          /**< the device type of the firmware. */
  uint32_t firmware_version; /**< the version of this firmware. */
  uint64_t firmware_buildtime; /**< the buildtime of this firmware. */
  uint8_t hw_whitelist[32]; /**< the hardware version list that this firmware can be used for. */
} VehicleStartUpgradeRequestV3;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} VehicleStartUpgradeResponse;

typedef struct {
  uint32_t offset; /**< Return code. */
  uint32_t length; /**< Working state. */
  uint8_t encrypt_type;
  uint8_t rsvd[3];
  uint8_t data[1]; /**< LiDAR feature. */
} VehicleXferFirmwareResquest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint32_t offset;  /**< Return code. */
  uint32_t length;  /**< Working state. */
} VehicleXferFirmwareResponse;

typedef struct {
  uint8_t checksum_type;   /**< Return code. */
  uint8_t checksum_length; /**< Working state. */
  uint8_t checksum[1];     /**< LiDAR feature. */
} VehicleCompleteXferFirmwareResquest;

typedef struct {
  uint8_t ret_code; /**< Return code. */
} VehicleCompleteXferFirmwareResponse;

/*typedef struct {*/
/* no command data */
/*} GetUpgradeProgressResquest;*/

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint16_t length;  /**< The length of firmware info string, include '\0'. */
  uint8_t info[1];  /**< Firmware info string, include '\0'. */
} VehicleRequestFirmwareInfoResponse;

typedef struct {
  uint8_t ret_code; /**< Return code. */
  uint8_t progress; /**< progress of upgrade. */
} VehicleGetUpgradeProgressResponse;

/**
 * The request body of the command for setting device's point cloud multicast ip.
 */
typedef struct {
  /** byte[0]: multicast enable flag
   *          1: enable point cloud multicast; 0: disable point cloud multicast
   *  If the multicast ip address 224.1.2.3, the byte[1]=3, byte[2] = 2, byte[3] = 1, byte[4] = 224
   *  After setting the lidar restart it
   *  Default disable point cloud multicast*/
  uint8_t byte[5];
} SetPointCloudMulticastIpRequest;


#pragma pack()

#endif  // LIVOX_DEF_VEHICLE_H_
