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

#ifndef LIVOX_DEFINE_H_
#define LIVOX_DEFINE_H_

#include <stdio.h>
#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <cstdint>
#include <atomic>

#include "livox_lidar_def.h"

namespace livox {
namespace lidar {

#pragma pack(1)

const uint16_t KDefaultTimeOut = 1000;
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
  std::string host_ip;
  std::string multicast_ip;

  uint16_t cmd_data_port;
  uint16_t push_msg_port;
  uint16_t point_data_port;
  uint16_t imu_data_port;
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

typedef struct {
  bool lidar_log_enable;
  uint64_t lidar_log_cache_size;
  std::string lidar_log_path;
} LivoxLidarLoggerCfg;

typedef struct {
  bool master_sdk;
} LivoxLidarSdkFrameworkCfg;

typedef enum {
  /**
   * Lidar command set, set the working mode and sub working mode of a LiDAR.
   */
  kCommandIDLidarSearch = 0x0000,
  // kCommandIDLidarPreconfig = 0x01,
  
  kCommandIDLidarWorkModeControl = 0x0100,
  kCommandIDLidarGetInternalInfo = 0x0101,
  kCommandIDLidarPushMsg         = 0x0102,

  kCommandIDLidarRebootDevice    = 0x0200,
  kCommandIDLidarResetDevice     = 0x0201,
  kCommandIDLidarSetPPSSync      = 0x0202,

  kCommandIDLidarPushLog                = 0x0300,
  kCommandIDLidarCollectionLog          = 0x0301,
  kCommandIDLidarLogSysTimeSync         = 0x0302,
  kCommandIDLidarDebugPointCloudControl = 0x0303,


  kCommandIDGeneralRequestUpgrade         = 0x0400,
  kCommandIDGeneralXferFirmware           = 0x0401,
  kCommandIDGeneralCompleteXferFirmware   = 0x0402,
  kCommandIDGeneralRequestUpgradeProgress = 0x0403,
  kCommandIDGeneralRequestFirmwareInfo    = 0xFF,
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
  uint8_t server_ipaddr[4];
} NTPServerIpInfoValue;

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

static const uint16_t kDetectionPort           = 56000;
static const uint16_t kDetectionListenPort     = 56001;
static const uint16_t kHostDebugPointCloudPort = 44332;

static const uint16_t kHAPCmdPort       = 56000;
static const uint16_t kHAPPushMsgPort   = 56000;
static const uint16_t kHAPPointDataPort = 57000;
static const uint16_t kHAPIMUPort       = 58000;
static const uint16_t kHAPLogPort       = 59000;
static const uint16_t kHAPDebugPointCloudPort = 60000;

/** kLogPort, which is the log port to be banned. */
static const uint16_t kLogPort = 0;

static const uint16_t kHAPLidarCmdPort = 56000;

static const uint16_t kMid360LidarCmdPort             = 56100;
static const uint16_t kMid360LidarPushMsgPort         = 56200;
static const uint16_t kMid360LidarPointCloudPort      = 56300;
static const uint16_t kMid360LidarImuDataPort         = 56400;
static const uint16_t kMid360LidarLogPort             = 56500;
static const uint16_t kMid360LidarDebugPointCloudPort = 60301;

static const uint16_t kMid360HostCmdPort        = 56101;
static const uint16_t kMid360HostPushMsgPort    = 56201;
static const uint16_t kMid360HostPointCloudPort = 56301;
static const uint16_t kMid360HostImuDataPort    = 56401;
static const uint16_t kMid360HostLogPort        = 56501;

static const uint16_t kPaLidarCmdPort        = 9347;
static const uint16_t kPaLidarPointCloudPort = 10000;
static const uint16_t kPaLidarFaultPort      = 10001;
static const uint16_t kPaLidarLogPort        = 1002;

static const uint16_t kPaHostFaultPort = 42867;

static const uint16_t kMid360sLidarCmdPort             = 56100;
static const uint16_t kMid360sLidarPushMsgPort         = 56200;
static const uint16_t kMid360sLidarPointCloudPort      = 56300;
static const uint16_t kMid360sLidarImuDataPort         = 56400;
static const uint16_t kMid360sLidarLogPort             = 56500;
static const uint16_t kMid360sLidarDebugPointCloudPort = 60301;

static const uint16_t kMid360sHostCmdPort        = 56101;
static const uint16_t kMid360sHostPushMsgPort    = 56201;
static const uint16_t kMid360sHostPointCloudPort = 56301;
static const uint16_t kMid360sHostImuDataPort    = 56401;
static const uint16_t kMid360sHostLogPort        = 56501;

static const uint16_t kAvia2LidarCmdPort             = 56100;
static const uint16_t kAvia2LidarPushMsgPort         = 56200;
static const uint16_t kAvia2LidarPointCloudPort      = 56300;
static const uint16_t kAvia2LidarImuDataPort         = 56400;
static const uint16_t kAvia2LidarLogPort             = 56500;
static const uint16_t kAvia2LidarDebugPointCloudPort = 60301;

static const uint16_t kAvia2HostCmdPort        = 56101;
static const uint16_t kAvia2HostPushMsgPort    = 56201;
static const uint16_t kAvia2HostPointCloudPort = 56301;
static const uint16_t kAvia2HostImuDataPort    = 56401;
static const uint16_t kAvia2HostLogPort        = 56501;

static const uint16_t kMid360lLidarCmdPort             = 56100;
static const uint16_t kMid360lLidarPushMsgPort         = 56200;
static const uint16_t kMid360lLidarPointCloudPort      = 56300;
static const uint16_t kMid360lLidarImuDataPort         = 56400;
static const uint16_t kMid360lLidarLogPort             = 56500;
static const uint16_t kMid360lLidarDebugPointCloudPort = 60301;

static const uint16_t kMid360lHostCmdPort        = 56101;
static const uint16_t kMid360lHostPushMsgPort    = 56201;
static const uint16_t kMid360lHostPointCloudPort = 56301;
static const uint16_t kMid360lHostImuDataPort    = 56401;
static const uint16_t kMid360lHostLogPort        = 56501;

typedef enum {
  kCmd = 0,
  kPush = 1,
  kPointCloud = 2,
  kImuData = 3,
  kLog = 4,
  kFault = 5
} HostSocketType;

typedef enum {
  kLidarCmdPort = 56100,
  kLidarPushCmdPort = 56200,
  kLidarPointDataPort = 56300,
  kLidarImuDataPort = 56400,
  kLidarLogPort = 56500
} LidarPort;

typedef struct {
  uint8_t log_type;
  uint8_t enable;
} EnableDeviceLoggerRequest;

typedef struct {
  std::string   sn;
  std::uint8_t  dev_type;
  std::string   lidar_ip;
  std::uint16_t cmd_port;
} LidarDeviceInfo;

typedef struct {
  uint8_t log_type;         // 0
  uint8_t file_index;       // file index
  uint8_t file_num;
  uint8_t flag;
  uint32_t timestamp;
  uint16_t rsvd;
  uint32_t trans_index;
  uint16_t data_length;     // log data length 
  uint8_t  data[1];         //data of log  
} DeviceLoggerFilePushRequest;

typedef struct {
  uint8_t ret_code;         // 0
  uint8_t log_type;         // file index
  uint8_t file_index;      // 0 for cfg, 1 for log
  uint32_t trans_index;     //sequence of trans file
} DeviceLoggerFilePushReponse;

enum class Flag : uint8_t {
  kNull,
  kCreateFile,
  kEndFile,
  kTransferData
};

using DataCallback = std::function<void(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)>;
using LidarInfoCallback = std::function<void(const uint32_t, const uint8_t, const char*, void*)>;

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
  uint8_t file_ver;   /**< file format version. */
  uint8_t dev_type;   /**< the device type of the firmware. */
  uint8_t data_type;  /**< type of data. */
  uint8_t sn[16];
  uint8_t rsvd[107];
  uint16_t crc16;
} LivoxLidarDebugPointCloudFileHeader;

typedef struct {
  uint8_t   enable;
  uint8_t   host_ip_addr[4];
  uint16_t  host_port;
  uint16_t  bandwidth;
} LivoxLidarDebugPointCloudRequest;

typedef struct {
  enum class SyncTimeType : uint8_t {
    kRmcSyncTime = 2,
  } type;
  uint64_t ns;
} LivoxLidarRmcSyncTimeRequest;

typedef void(*LivoxLidarStartUpgradeCallback)(livox_status status, uint32_t handle,
    LivoxLidarStartUpgradeResponse* response, void* client_data);

typedef void(*LivoxLidarXferFirmwareCallback)(livox_status status, uint32_t handle,
    LivoxLidarXferFirmwareResponse* response, void* client_data);

typedef void(*LivoxLidarCompleteXferFirmwareCallback)(livox_status status, uint32_t handle,
    LivoxLidarCompleteXferFirmwareResponse* response, void* client_data);

typedef void(*LivoxLidarGetUpgradeProgressCallback)(livox_status status,
    uint32_t handle, LivoxLidarGetUpgradeProgressResponse* response, void* client_data);

typedef void(*LivoxLidarRequestFirmwareInfoCallback)(livox_status status,
    uint32_t handle, LivoxLidarRequestFirmwareInfoResponse* response, void* client_data);


#pragma pack()

} // namespace lidar
} // namespace livox

# endif // DEFINE_H_
