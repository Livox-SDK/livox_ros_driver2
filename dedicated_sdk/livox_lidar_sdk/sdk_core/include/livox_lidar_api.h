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

#ifndef LIVOX_LIDAR_SDK_H_
#define LIVOX_LIDAR_SDK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "livox_lidar_def.h"

/**
* Return SDK's version information in a numeric form.
* @param version Pointer to a version structure for returning the version information.
*/
void GetLivoxLidarSdkVer(LivoxLidarSdkVer *version);

/**
 * Initialize the SDK.
 * @return true if successfully initialized, otherwise false.
 */
bool LivoxLidarSdkInit(const char* path, const char* host_ip = "");

/**
 * Start the device scanning routine which runs on a separate thread.
 * @return true if successfully started, otherwise false.
 */
bool LivoxLidarSdkStart();

/**
 * Uninitialize the SDK.
 */
void LivoxLidarSdkUninit();

/**
 * Callback function for receiving point cloud data.
 * 
 * @param handle        device handle.
 * @param data        device's data.
 * @param data_num    number of points in data.
 * @param client_data user data associated with the command.
 */
typedef void (*LivoxLidarPointCloudCallBack)(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);

/**
 * Set the callback to receive point cloud data.
 * @param handle        device handle.
 * @param client_data user data associated with the command.
 */
void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void* client_data);

/**
 * Callback function for point cloud observer.
 * @param handle        device handle.
 * @param data        device's data.
 * @param data_num    number of points in data.
 * @param client_data user data associated with the command.
 */
typedef void (*LivoxLidarPointCloudObserver) (uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data);

/**
 * Set the point cloud observer.
 * @param cb          callback to receive Status Info.
 * @param client_data user data associated with the command.
 */
uint16_t LivoxLidarAddPointCloudObserver(LivoxLidarPointCloudObserver cb, void *client_data);

/**
 * remove point cloud observer.
 * @param id         the observer id.
 */
void LivoxLidarRemovePointCloudObserver(uint16_t id);

/**
 * Callback function for receiving IMU data.
 * @param data        device's data.
 * @param data_num    number of IMU data.
 * @param client_data user data associated with the command.
 */
typedef void (*LivoxLidarImuDataCallback)(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);

/**
 * Set the callback to receive IMU data.
 * @param cb          callback to receive IMU data.
 * @param client_data user data associated with the command.
 */
void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data);

/**
 * Callback function for receiving Status Info.
 * @param status_info   status info.
 * @param client_data user data associated with the command.
 */
typedef void(*LivoxLidarInfoCallback)(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);

/**
 * Set the callback to receive Status Info.
 * @param cb          callback to receive Status Info.
 * @param client_data user data associated with the command.
 */
void SetLivoxLidarInfoCallback(LivoxLidarInfoCallback cb, void* client_data);

/**
 * DisableLivoxLidar console log output.
 */
void DisableLivoxSdkConsoleLogger();

/**
* Save the log file.
*/
void SaveLivoxLidarSdkLoggerFile();

/**
 * Callback function for receiving Status Info.
 * @param status_info   status info.
 * @param client_data user data associated with the command.
 */
typedef void(*LivoxLidarInfoChangeCallback)(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);

/**
 * Set the callback to receive Status Info.
 * @param cb          callback to receive Status Info.
 * @param client_data user data associated with the command.
 */
void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data);

typedef void (*QueryLivoxLidarInternalInfoCallback)(livox_status status, uint32_t handle,
        LivoxLidarDiagInternalInfoResponse* response, void* client_data);
livox_status QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);

typedef void (*LivoxLidarAsyncControlCallback)(livox_status status, uint32_t handle, 
    LivoxLidarAsyncControlResponse *response, void *client_data);


/**
 * Set LiDAR pcl data type.
 * @param  handle        device handle.
 * @param  data_type     the type to change, the val:kLivoxLidarCartesianCoordinateHighData, 
 *                                                   kLivoxLidarCartesianCoordinateLowData, 
 *                                                   kLivoxLidarSphericalCoordinateData
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR sacn pattern.
 * @param  handle        device handle.
 * @param  scan_type     the type to change, the val:kLivoxLidarScanPatternRepetive, 
 *                                                   kLivoxLidarScanPatternNoneRepetive
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR dual emit.
 * @param  handle        device handle.
 * @param  enable        if set dual emit the enable is true, else the enable is false
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Embale LiDAR point send.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Disable LiDAR point send.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR Ip info.
 * @param  handle        device handle.
 * @param  ipconfig      lidar ip info.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarIp(uint32_t handle, LivoxLidarIpInfo* ip_config,
    LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR state Ip info.
 * @param  handle                 device handle.
 * @param  host_state_info_ipcfg  lidar ip info.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarStateInfoHostIPCfg(uint32_t handle, HostStateInfoIpInfo* host_state_info_ipcfg,
  LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR point cloud host ip info.
 * @param  handle                 device handle.
 * @param  host_point_ipcfg       lidar ip info.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarPointDataHostIPCfg(uint32_t handle, HostPointIPInfo* host_point_ipcfg,
  LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR imu data host ip info.
 * @param  handle                 device handle.
 * @param  host_imu_ipcfg         lidar ip info.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarImuDataHostIPCfg(uint32_t handle, HostImuDataIPInfo* host_imu_ipcfg,
  LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Get LiDAR extrinsic parameters.
 * @param  handle           device handle.
 * @param  install_attitude extrinsic parameters.
 * @param  cb               callback for the command.
 * @param  client_data      user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarInstallAttitude(uint32_t handle, LivoxLidarInstallAttitude* install_attitude,
    LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR roi cfg0.
 * @param  handle          device handle.
 * @param  roi_cfg0        roi cfg.
 * @param  cb              callback for the command.
 * @param  client_data     user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarRoiCfg0(uint32_t handle, RoiCfg* roi_cfg0, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR roi.
 * @param  handle          device handle.
 * @param  roi_cfg1        roi cfg.
 * @param  cb              callback for the command.
 * @param  client_data     user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarRoiCfg1(uint32_t handle, RoiCfg* roi_cfg1, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Enable LiDAR roi cfg1.
 * @param  handle          device handle.
 * @param  cb              callback for the command.
 * @param  client_data     user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarRoi(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Disable LiDAR roi.
 * @param  handle          device handle.
 * @param  cb              callback for the command.
 * @param  client_data     user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarRoi(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR detect mode.
 * @param  handle          device handle.
 * @param  mode            detect mode
 * @param  cb              callback for the command.
 * @param  client_data     user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarDetectMode(uint32_t handle, LivoxLidarDetectMode mode, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR func io cfg.
 * @param  handle          device handle.
 * @param  func_io_cfg     func io cfg; 0:IN0,1:IN1, 2:OUT0, 3:OUT1;Mid360 lidar 8, 10, 12, 11
 * @param  cb              callback for the command.
 * @param  client_data     user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarFuncIOCfg(uint32_t handle, FuncIOCfg* func_io_cfg, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR blind spot.
 * @param  handle                 device handle.
 * @param  blind_spot             blind spot.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR work mode.
 * @param  handle                 device handle.
 * @param  work_mode              lidar work mode.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Enable LiDAR glass heat.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);


/**
 * Disable LiDAR glass heat.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR glass heat.
 * @param  handle         device handle.
 * @param  glass_heat     lidar glass heat.
 * @param  cb             callback for the command.
 * @param  client_data    user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Enable LiDAR imu data.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Disable LiDAR imu data.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

// mid360 lidar does not support
/**
 * Enable LiDAR fusa function.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

// mid360 lidar does not support
/**
 * Enable LiDAR fusa function.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

typedef void (*LivoxLidarResetCallback)(livox_status status, uint32_t handle, LivoxLidarResetResponse* response, void* client_data);
/**
 * Reset LiDAR.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data);

/*******Upgrade Module***********/
typedef void (*LivoxLidarRebootCallback)(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data);
livox_status LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb, void* client_data);

  /**
 * Upgrade related command
 */
typedef void (*LivoxLidarStartUpgradeCallback)(livox_status status, uint32_t handle,
    LivoxLidarStartUpgradeResponse* response, void* client_data);
/**
 * LiDAR start upgrade
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarStartUpgrade(uint32_t handle,  uint8_t *data, uint16_t length,
    LivoxLidarStartUpgradeCallback cb, void* client_data);

typedef void (*LivoxLidarXferFirmwareCallback)(livox_status status, uint32_t handle,
    LivoxLidarXferFirmwareResponse* response, void* client_data);
/**
 * Transfer LiDAR firmware.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarXferFirmwareCallback cb, void* client_data);

typedef void (*LivoxLidarCompleteXferFirmwareCallback)(livox_status status, uint32_t handle,
    LivoxLidarCompleteXferFirmwareResponse* response, void* client_data);
/**
 * Verify the integrity of the LiDAR transmission firmware.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarCompleteXferFirmware(uint32_t handle, uint8_t *data,
    uint16_t length, LivoxLidarCompleteXferFirmwareCallback cb, void* client_data);

typedef void (*LivoxLidarGetUpgradeProgressCallback)(livox_status status,
    uint32_t handle, LivoxLidarGetUpgradeProgressResponse* response, void* client_data);
/**
 * Get LiDAR upgrade progress.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
    uint16_t length, LivoxLidarGetUpgradeProgressCallback cb, void* client_data);

typedef void (*LivoxLidarRequestFirmwareInfoCallback)(livox_status status,
    uint32_t handle, LivoxLidarRequestFirmwareInfoResponse* response, void* client_data);
/**
 * Query LiDAR firmware info.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarRequestFirmwareInfo(uint32_t handle,
    LivoxLidarRequestFirmwareInfoCallback cb, void* client_data);

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_LIDAR_SDK_H_
