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
bool LivoxLidarSdkInit(const char* path, const char* host_ip = "", const LivoxLidarLoggerCfgInfo* log_cfg_info = nullptr);

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
 * Set the callback to receive point cloud data.
 * @param handle                 device handle.
 * @param client_data            user data associated with the command.
 */
void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void* client_data);

/**
 * Add the lidar command data observer.
 * @param handle                 device handle.
 * @param client_data            user data associated with the command.
 */
void LivoxLidarAddCmdObserver(LivoxLidarCmdObserverCallBack cb, void* client_data);

/**
 * Remove the lidar command data observer.
 */
void LivoxLidarRemoveCmdObserver();

/**
 * Set the point cloud observer.
 * @param cb                     callback to receive Status Info.
 * @param client_data            user data associated with the command.
 */
uint16_t LivoxLidarAddPointCloudObserver(LivoxLidarPointCloudObserver cb, void *client_data);

/**
 * remove point cloud observer.
 * @param id                     the observer id.
 */
void LivoxLidarRemovePointCloudObserver(uint16_t id);

/**
 * Set the callback to receive IMU data.
 * @param cb                     callback to receive Status Info.
 * @param client_data            user data associated with the command.
 */
void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data);

/**
 * Set the callback to receive Status Info.
 * @param cb                     callback to receive Status Info.
 * @param client_data            user data associated with the command.
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
 * Set the callback to receive Status Info.
 * @param cb                     callback to receive Status Info.
 * @param client_data            user data associated with the command.
 */
void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data);

livox_status QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);

livox_status QueryLivoxLidarFwType(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);

livox_status QueryLivoxLidarFirmwareVer(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);

/**
 * Set LiDAR pcl data type.
 * @param  handle                 device handle.
 * @param  data_type              the type to change, the val:kLivoxLidarCartesianCoordinateHighData, 
 *                                                            kLivoxLidarCartesianCoordinateLowData, 
 *                                                            kLivoxLidarSphericalCoordinateData
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR sacn pattern.
 * @param  handle                 device handle.
 * @param  scan_type              the type to change, the val:kLivoxLidarScanPatternRepetive, 
 *                                                            kLivoxLidarScanPatternNoneRepetive
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR dual emit.
 * @param  handle                 device handle.
 * @param  enable                 if set dual emit the enable is true, else the enable is false
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Enbale LiDAR point send.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Disable LiDAR point send.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR Ip info.
 * @param  handle                 device handle.
 * @param  ipconfig               lidar ip info.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
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
 * @param  handle                 device handle.
 * @param  install_attitude       extrinsic parameters.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarInstallAttitude(uint32_t handle, LivoxLidarInstallAttitude* install_attitude,
                                          LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR fov cfg0.
 * @param  handle                 device handle.
 * @param  fov_cfg0               fov cfg.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarFovCfg0(uint32_t handle, FovCfg* fov_cfg0, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR fov.
 * @param  handle                 device handle.
 * @param  fov_cfg1               fov cfg.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarFovCfg1(uint32_t handle, FovCfg* fov_cfg1, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Enable LiDAR fov cfg1.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarFov(uint32_t handle, uint8_t fov_en, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Disable LiDAR fov.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarFov(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR detect mode.
 * @param  handle                 device handle.
 * @param  mode                   detect mode
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarDetectMode(uint32_t handle, LivoxLidarDetectMode mode, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR func io cfg.
 * @param  handle                 device handle.
 * @param  func_io_cfg            func io cfg; 0:IN0,1:IN1, 2:OUT0, 3:OUT1;Mid360 lidar 8, 10, 12, 11
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
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
 * Enable glass heating functionality.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Disable glass heating functionality.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

// mid360 lidar does not support this function.
/**
 * Enable LiDAR force heat function.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status StartForcedHeating(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

// mid360 lidar does not support this function.
/**
 * Disable LiDAR force heat function.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status StopForcedHeating(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

// only avia2 support this function.
/**
 * Set LiDAR fog noise filter mode.
 * @param  handle                 device handle.
 * @param  fog_filter_mode        lidar fog filter mode
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLidarFogNoiseFilterMode(uint32_t handle, LivoxFogNoiseFilterMode fog_filter_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

// only avia2 support this function.
/**
 * Set LiDAR ITO control mode.
 * @param  handle                 device handle.
 * @param  ito_mode               lidar ito mode
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarITOCtrlMode(uint32_t handle, LivoxLidarItoCtrlMode ito_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

// only mid360l support this function.
/**
 * Set LiDAR point cloud frequency mode.
 * @param  handle                 device handle.
 * @param  pcl_freq_mod           point cloud frequency mode: 0=80k, 1=50k, 2=100k.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarPclFreqMod(uint32_t handle, LivoxLidarPclFreqMod pcl_freq_mod, LivoxLidarAsyncControlCallback cb, void* client_data);

// only mid360l / mid360s support this function.
/**
 * Set LiDAR time filter mode. Controls whether point cloud frames whose
 * timestamp goes backwards are still written out.
 * @param  handle                 device handle.
 * @param  time_filter_mode       0 = normal mode (drop backward-timestamp frames),
 *                                1 = special mode (write point cloud even if the
 *                                timestamp goes backwards).
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarTimeFilterMode(uint32_t handle, LivoxLidarTimeFilterMode time_filter_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

// only mid360l / mid360s support this function.
/**
 * Set LiDAR IMU range (output rate, accelerometer full-scale, gyroscope full-scale).
 * @param  handle                 device handle.
 * @param  imu_out_rate           IMU output rate: 0=200Hz, 1=500Hz, 2=100Hz, 3=50Hz.
 * @param  accel_range            Accelerometer range: 0=+/-4g, 1=+/-8g, 2=+/-16g, 3=+/-32g.
 * @param  gyro_range             Gyroscope range: 0=+/-2000dps, 1=+/-1000dps, 2=+/-500dps,
 *                                3=+/-250dps, 4=+/-125dps, 5=+/-62.5dps, 6=+/-31.25dps,
 *                                7=+/-15.625dps.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarImuRange(uint32_t handle,
                                   LivoxLidarImuOutRate imu_out_rate,
                                   LivoxLidarAccelRange accel_range,
                                   LivoxLidarGyroRange  gyro_range,
                                   LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set NTP server Ip info, only avia2 support this function.
 * @param  handle                 device handle.
 * @param  ipconfig               NTP server ip info.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetNTPServerIp(uint32_t handle, NTPServerIpInfo* server_ip, LivoxLidarAsyncControlCallback cb, void* client_data);

// mid360 and hap lidar does not support this function.
/**
 * Set LiDAR esc mode.
 * @param  handle                 device handle.
 * @param  esc_mode               lidar esc mode 
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarEscMode(uint32_t handle, LivoxLidarEscMode esc_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

// avia2 support this function, other not support.
/**
 * Set LiDAR fov mode.
 * @param  handle                 device handle.
 * @param  fov_mode               lidar fov mode 
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarFovMode(uint32_t handle, LivoxLidarFovMode fov_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

// avia2 support this function, other not support.
/**
 * Set LiDAR Echo mode.
 * @param  handle                 device handle.
 * @param  echo_mode              lidar echo mode 
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarEchoMode(uint32_t handle, LivoxLidarEchoMode echo_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Set LiDAR glass heat.
 * @param  handle                 device handle.
 * @param  glass_heat             lidar glass heat.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
[[deprecated("Please USE EnableLivoxLidarGlassHeat() / StartForcedHeating() / ... instead")]]
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
 * Disable LiDAR fusa function.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

/**
 * Reset LiDAR.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data);

/**
 * Start logger. Currently, only setting real-time log is supported.
 * @param  handle                 device handle.
 * @param  log_type               The type of log file that has been configured.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarStartLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerCallback cb, void* client_data);

/**
 * Start logger. Currently, only setting real-time log is supported.
 * @param  handle                 device handle.
 * @param  log_type               The type of log file that has been configured.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarStopLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerCallback cb, void* client_data);

/**
 * Set LiDAR debug point cloud switch.
 * @param  handle                           device handle.
 * @param  enable                           true for enabling debug point cloud
 * @param  cb                               callback for the command.
 * @param  client_data                      user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error codes.
 */
livox_status SetLivoxLidarDebugPointCloud(uint32_t handle, bool enable, LivoxLidarLoggerCallback cb, void* client_data);

/**
 * Set LiDAR GPS "GPRMC" string to synchronize the time.
 * @param  handle                           device handle.
 * @param  rmc                              GPS "GPRMC" string.
 * @param  rmc_length                       GPS "GPRMC" string length.
 * @param  cb                               callback for the command.
 * @param  client_data                      user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error codes.
 */
livox_status SetLivoxLidarRmcSyncTime(uint32_t handle, const char* rmc, uint16_t rmc_length, LivoxLidarRmcSyncTimeCallBack cb, void* client_data);

/**
 * Set LiDAR work mode after boot.
 * @param  handle                 device handle.
 * @param  work_mode              lidar work mode after boot.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetLivoxLidarWorkModeAfterBoot(const uint32_t handle,const LivoxLidarWorkModeAfterBoot work_mode, LivoxLidarAsyncControlCallback cb, void* client_data);
/*******Upgrade Module***********/

/**
 * Reboot lidar.
 * @param  handle                 device handle.
 * @param  cb                     callback for the command.
 * @param  client_data            user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb, void* client_data);

/**
 * Start the device scanning routine which runs on a separate thread.
 * @return true if successfully Open firmware file path, otherwise false.
 */
bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path);

void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data);

void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num);

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_LIDAR_SDK_H_
