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

#ifndef LIVOX_SDK_DIRECT_H_
#define LIVOX_SDK_DIRECT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "livox_def_direct.h"

/**
* Return SDK's version information in a numeric form.
* @param version Pointer to a version structure for returning the version information.
*/
void GetLivoxSdkDirectVersion(LivoxSdkDirectVersion *version);

/**
 * Disable console log output.
 */
void DisableDirectLidarConsoleLogger();

/**
 * Initialize the SDK.
 * @return true if successfully initialized, otherwise false.
 */
bool LivoxDirectInit(DirectLidarHostCfg* direct_host_cfg_ptr, DirectLidarCfg* direct_lidar_cfg_ptr, const uint8_t lidars_num = 1);

/**
 * Start the device scanning routine which runs on a separate thread.
 * @return true if successfully started, otherwise false.
 */
bool LivoxDirectStart();

/**
 * Uninitialize the SDK.
 */
void LivoxDirectUninit();

/**
* Save the log file.
*/
void SaveDirectLidarLoggerFile();

/**
 * Callback function for receiving point cloud data.
 * @param slot        device slot.
 * @param data        device's data.
 * @param data_num    number of points in data.
 * @param client_data user data associated with the command.
 */
typedef void (*DirectPointDataCallback)(const uint32_t handle, LivoxDirectEthPacket* data, uint32_t data_num, void* client_data);

/**
 * Set the callback to receive point cloud data.
 * @param slot        device slot.
 * @param client_data user data associated with the command.
 */
void SetDirectLidarPointDataCallback(DirectPointDataCallback cb, void* client_data);

/**
 * Callback function for receiving IMU data.
 * @param data        device's data.
 * @param data_num    number of IMU data.
 * @param client_data user data associated with the command.
 */
typedef void (*DirectImuDataCallback)(const uint32_t handle, LivoxDirectEthPacket* data, uint32_t data_num, void* client_data);

/**
 * Set the callback to receive IMU data.
 * @param cb          callback to receive IMU data.
 * @param client_data user data associated with the command.
 */
void SetDirectLidarImuDataCallback(DirectImuDataCallback cb, void* client_data);

/**
 * Callback function for receiving Status Info.
 * @param status_info   status info.
 * @param client_data user data associated with the command.
 */
typedef void(*DirectLidarInfoCallback)(const uint32_t handle, DirectLidarStateInfo* info, void* client_data);

/**
 * Set the callback to receive Status Info.
 * @param cb          callback to receive Status Info.
 * @param client_data user data associated with the command.
 */
void SetDirectLidarInfoCallback(DirectLidarInfoCallback cb, void* client_data);

/**
 * Callback function for point cloud observer.
 * @param slot        device slot.
 * @param data        device's data.
 * @param data_num    number of points in data.
 * @param client_data user data associated with the command.
 */
typedef void (*DirectPointCloudObserver) (uint32_t handle, LivoxDirectEthPacket *data, uint32_t data_num, void *client_data);

/**
 * Set the point cloud observer.
 * @param cb          callback to receive Status Info.
 * @param client_data user data associated with the command.
 */
uint16_t DirectLidarAddPointCloudObserver(DirectPointCloudObserver cb, void *client_data);

/**
 * remove point cloud observer.
 * @param id         the observer id.
 */
void DirectLidarRemovePointCloudObserver(uint16_t id);

/**
 * Callback function for send command to lidar.
 * @param response        lidar response.
 * @param client_data user data associated with the command.
 */
typedef void (*DirectLidarCfgUpdateCallabck)(const uint32_t handle, DirectLidarCmdResInfo* response, void* client_data);

/**
 * Set the callback to set lidar.
 * @param cb          callback to receive Status Info.
 * @param client_data user data associated with the command.
 */
void SetDirectLidarCfgUpdateCallback(DirectLidarCfgUpdateCallabck cb, void* client_data);

/**
 * Update direct lidar config.
 * @param  cfg   direct lidar config.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
void UpdateDirectLidarCfg(const uint32_t handle, DirectLidarCfg* cfg);


/*******Upgrade Module***********/
typedef void (*DirectRebootCallback)(livox_direct_status status, uint32_t handle,
    DirectRebootResponse* response, void* client_data);
    
livox_direct_status DirectLidarRequestReboot(uint32_t handle, DirectRebootCallback cb,
    void* client_data);
/**
 * Upgrade related command
 */
typedef void (*DirectStartUpgradeCallback)(livox_direct_status status, uint32_t handle,
    DirectStartUpgradeResponse* response, void* client_data);
livox_direct_status DirectLidarStartUpgrade(uint32_t handle,  uint8_t *data, uint16_t length,
    DirectStartUpgradeCallback cb, void* client_data);

typedef void (*DirectXferFirmwareCallback)(livox_direct_status status, uint32_t handle,
    DirectXferFirmwareResponse* response, void* client_data);
livox_direct_status DirectLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    DirectXferFirmwareCallback cb, void* client_data);

typedef void (*DirectCompleteXferFirmwareCallback)(livox_direct_status status, uint32_t handle,
    DirectCompleteXferFirmwareResponse* response, void* client_data);
livox_direct_status DirectLidarCompleteXferFirmware(uint32_t handle, uint8_t *data,
    uint16_t length, DirectCompleteXferFirmwareCallback cb, void* client_data);

typedef void (*DirectGetUpgradeProgressCallback)(livox_direct_status status,
    uint32_t handle, DirectGetUpgradeProgressResponse* response, void* client_data);
livox_direct_status DirectLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
    uint16_t length, DirectGetUpgradeProgressCallback cb, void* client_data);

// typedef void (*DirectRequestFirmwareInfoCallback)(livox_direct_status status,
//     uint32_t handle, DirectRequestFirmwareInfoResponse* response, void* client_data);
// livox_direct_status DirectLidarRequestFirmwareInfo(uint32_t handle,
//     DirectRequestFirmwareInfoCallback cb, void* client_data);

// reset lidar
typedef void (*DirectLidarResetCallback)(livox_direct_status status,
    uint32_t handle, DirectLidarResetResponse* response, void* client_data);
livox_direct_status DirectLidarRequestReset(uint32_t handle, DirectLidarResetCallback cb, void* client_data);

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_SDK_H_
