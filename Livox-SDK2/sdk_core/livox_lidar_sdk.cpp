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

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include "base/command_callback.h"
#include "base/logging.h"
#include "comm/define.h"

#include "command_handler/command_impl.h"
#include "command_handler/general_command_handler.h"
#include "data_handler/data_handler.h"
#include "logger_handler/logger_manager.h"
#include "upgrade_manager.h"


#include "parse_cfg_file.h"
#include "params_check.h"
#include "device_manager.h"

#ifdef WIN32
#include<winsock2.h>
#endif // WIN32

#include <memory>
#include <vector>

using namespace livox::lidar;

static bool is_initialized = false;

void GetLivoxLidarSdkVer(LivoxLidarSdkVer *version) {
  if (version != NULL) {
    version->major = LIVOX_LIDAR_SDK_MAJOR_VERSION;
    version->minor = LIVOX_LIDAR_SDK_MINOR_VERSION;
    version->patch = LIVOX_LIDAR_SDK_PATCH_VERSION;
  }
}

bool LivoxLidarSdkInit(const char* path, const char* host_ip, const LivoxLidarLoggerCfgInfo* log_cfg_info) {
  if (is_initialized) {
    return false;
  }

#ifdef WIN32
  WORD sockVersion = MAKEWORD(2, 0);
  WSADATA wsdata;
  if (WSAStartup(sockVersion, &wsdata) != 0) {
    return false;
  }
#endif // WIN32

  InitLogger();

  if (path == NULL && host_ip == NULL) {
    return false;
  }

  if (path) {
    std::shared_ptr<std::vector<LivoxLidarCfg>> lidars_cfg_ptr = nullptr;
    std::shared_ptr<std::vector<LivoxLidarCfg>> custom_lidars_cfg_ptr = nullptr;
    std::shared_ptr<LivoxLidarLoggerCfg> lidar_logger_cfg_ptr = nullptr;
    std::shared_ptr<LivoxLidarSdkFrameworkCfg> sdk_framework_cfg_ptr = nullptr;

    if (!ParseCfgFile(path).Parse(lidars_cfg_ptr, custom_lidars_cfg_ptr, lidar_logger_cfg_ptr, sdk_framework_cfg_ptr)) {
      return false;
    }

    if (!ParamsCheck(lidars_cfg_ptr, custom_lidars_cfg_ptr).Check()) {
      return false;
    }

    if (!DeviceManager::GetInstance().Init(lidars_cfg_ptr, custom_lidars_cfg_ptr, lidar_logger_cfg_ptr, sdk_framework_cfg_ptr)) {
      return false;
    }
  } else {
    if (!DeviceManager::GetInstance().Init(host_ip, log_cfg_info)) {
      return false;
    }
  }

  is_initialized = true;
  return true;
}

void LivoxLidarSdkUninit() {
  if (!is_initialized) {
    return;
  }

  LoggerManager::GetInstance().Destory();
  // The reason for using WSACleanup() after previous statement is that Destory() still needs to send socket messages.
#ifdef WIN32
    WSACleanup();
#endif // WIN32
  DeviceManager::GetInstance().Destory();
  DataHandler::GetInstance().Destory();
  GeneralCommandHandler::GetInstance().Destory();

  UninitLogger();
  is_initialized = false;
}

bool LivoxLidarSdkStart() {
  return true;
}

void SaveLivoxLidarSdkLoggerFile() {
  is_save_log_file = true;
}

void DisableLivoxSdkConsoleLogger() {
  is_console_log_enable = false;
}

uint16_t LivoxLidarAddPointCloudObserver(LivoxLidarPointCloudObserver cb, void *client_data) {
  return DataHandler::GetInstance().AddPointCloudObserver(cb, client_data);
}

void LivoxLidarRemovePointCloudObserver(uint16_t id) {
  DataHandler::GetInstance().RemovePointCloudObserver(id);
}

void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void *client_data) {
  DataHandler::GetInstance().SetPointDataCallback(cb, client_data);
}

void LivoxLidarAddCmdObserver(LivoxLidarCmdObserverCallBack cb, void *client_data) {
  GeneralCommandHandler::GetInstance().LivoxLidarAddCmdObserver(cb, client_data);
}

void LivoxLidarRemoveCmdObserver() {
  GeneralCommandHandler::GetInstance().LivoxLidarRemoveCmdObserver();
}

void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data) {
  DataHandler::GetInstance().SetImuDataCallback(cb, client_data);
}

void SetLivoxLidarInfoCallback(LivoxLidarInfoCallback cb, void* client_data) {
  GeneralCommandHandler::GetInstance().SetLivoxLidarInfoCallback(cb, client_data);
}

void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data) {
  GeneralCommandHandler::GetInstance().SetLivoxLidarInfoChangeCallback(cb, client_data);
}

livox_status QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  return CommandImpl::QueryLivoxLidarInternalInfo(handle, cb, client_data);
}

livox_status QueryLivoxLidarFwType(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  return CommandImpl::QueryLivoxLidarFwType(handle, cb, client_data);
}

livox_status QueryLivoxLidarFirmwareVer(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  return CommandImpl::QueryLivoxLidarFirmwareVer(handle, cb, client_data);
}

livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data) {
  if (data_type == kLivoxLidarImuData) {
    return EnableLivoxLidarImuData(handle, cb, client_data);
  }
  return CommandImpl::SetLivoxLidarPclDataType(handle, data_type, cb, client_data);
}

livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarScanPattern(handle, scan_pattern, cb, client_data);
}

livox_status SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarDualEmit(handle, enable, cb, client_data);
}

livox_status EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::EnableLivoxLidarPointSend(handle, cb, client_data);
}
livox_status DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::DisableLivoxLidarPointSend(handle, cb, client_data);
}

livox_status SetLivoxLidarIp(uint32_t handle, LivoxLidarIpInfo* ip_config,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarIp(handle, ip_config, cb, client_data);
}

livox_status SetLivoxLidarStateInfoHostIPCfg(uint32_t handle, HostStateInfoIpInfo* host_state_info_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarStateInfoHostIPCfg(handle, *host_state_info_ipcfg, cb, client_data);
}

livox_status SetLivoxLidarPointDataHostIPCfg(uint32_t handle, HostPointIPInfo* host_point_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarPointDataHostIPCfg(handle, *host_point_ipcfg, cb, client_data);
}

livox_status SetLivoxLidarImuDataHostIPCfg(uint32_t handle, HostImuDataIPInfo* host_imu_ipcfg,
  LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarImuDataHostIPCfg(handle, *host_imu_ipcfg, cb, client_data);
}

livox_status SetLivoxLidarInstallAttitude(uint32_t handle, LivoxLidarInstallAttitude* install_attitude,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarInstallAttitude(handle, *install_attitude, cb, client_data);
}

livox_status SetLivoxLidarFovCfg0(uint32_t handle, FovCfg* fov_cfg0, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarFovCfg0(handle, *fov_cfg0, cb, client_data);
}

livox_status SetLivoxLidarFovCfg1(uint32_t handle, FovCfg* fov_cfg1, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarFovCfg1(handle, *fov_cfg1, cb, client_data);
}

livox_status EnableLivoxLidarFov(uint32_t handle, uint8_t fov_en, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::EnableLivoxLidarFov(handle, fov_en, cb, client_data);
}

livox_status DisableLivoxLidarFov(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::DisableLivoxLidarFov(handle, cb, client_data);
}

livox_status SetLivoxLidarDetectMode(uint32_t handle, LivoxLidarDetectMode mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarDetectMode(handle, mode, cb, client_data);
}

livox_status SetLivoxLidarFuncIOCfg(uint32_t handle, FuncIOCfg* func_io_cfg, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarFuncIOCfg(handle, *func_io_cfg, cb, client_data);
}

livox_status SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarBlindSpot(handle, blind_spot, cb, client_data);
}

livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarWorkMode(handle, work_mode, cb, client_data);
}

livox_status EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::EnableLivoxLidarGlassHeat(handle, cb, client_data);
}
livox_status DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::DisableLivoxLidarGlassHeat(handle, cb, client_data);
}
livox_status SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarGlassHeat(handle, glass_heat, cb, client_data);
}

livox_status StartForcedHeating(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::StartForcedHeating(handle, cb, client_data);
}

livox_status StopForcedHeating(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::StopForcedHeating(handle, cb, client_data);
}

livox_status SetLidarFogNoiseFilterMode(uint32_t handle, LivoxFogNoiseFilterMode fog_filter_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLidarFogNoiseFilterMode(handle, fog_filter_mode, cb, client_data);
}

livox_status SetLivoxLidarITOCtrlMode(uint32_t handle, LivoxLidarItoCtrlMode ito_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarITOCtrlMode(handle, ito_mode, cb, client_data);
}

livox_status SetLivoxLidarPclFreqMod(uint32_t handle, LivoxLidarPclFreqMod pcl_freq_mod, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarPclFreqMod(handle, pcl_freq_mod, cb, client_data);
}

livox_status SetLivoxLidarTimeFilterMode(uint32_t handle, LivoxLidarTimeFilterMode time_filter_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarTimeFilterMode(handle, time_filter_mode, cb, client_data);
}

livox_status SetLivoxLidarImuRange(uint32_t handle, LivoxLidarImuOutRate imu_out_rate,
    LivoxLidarAccelRange accel_range, LivoxLidarGyroRange gyro_range,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarImuRange(handle, imu_out_rate, accel_range, gyro_range, cb, client_data);
}

livox_status SetNTPServerIp(uint32_t handle, NTPServerIpInfo* server_ip, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetNTPServerIp(handle, server_ip, cb, client_data);
}

livox_status SetLivoxLidarEscMode(uint32_t handle, LivoxLidarEscMode esc_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarEscMode(handle, esc_mode, cb, client_data);
}

livox_status SetLivoxLidarFovMode(uint32_t handle, LivoxLidarFovMode fov_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarFovMode(handle, fov_mode, cb, client_data);
}

livox_status SetLivoxLidarEchoMode(uint32_t handle, LivoxLidarEchoMode echo_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarEchoMode(handle, echo_mode, cb, client_data);
}

livox_status EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::EnableLivoxLidarImuData(handle, cb, client_data);
}
livox_status DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::DisableLivoxLidarImuData(handle, cb, client_data);
}

livox_status EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::EnableLivoxLidarFusaFunciont(handle, cb, client_data);
}
livox_status DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return CommandImpl::DisableLivoxLidarFusaFunciont(handle, cb, client_data);
}

livox_status SetLivoxLidarDebugPointCloud(uint32_t handle, bool enable, LivoxLidarLoggerCallback cb, void* client_data) {
  return CommandImpl::SetLivoxLidarDebugPointCloud(handle, enable, cb, client_data);
}

livox_status SetLivoxLidarRmcSyncTime(uint32_t handle, const char* rmc, uint16_t rmc_length, LivoxLidarRmcSyncTimeCallBack cb, void* client_data) {
  return CommandImpl::SetLivoxLidarRmcSyncTime(handle, rmc, rmc_length, cb, client_data);
}

livox_status SetLivoxLidarWorkModeAfterBoot(const uint32_t handle,const LivoxLidarWorkModeAfterBoot work_mode, LivoxLidarAsyncControlCallback cb, void* client_data){
  return CommandImpl::SetLivoxLidarWorkModeAfterBoot(handle, work_mode, cb, client_data);
}

// reset lidar
livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data) {
  return CommandImpl::LivoxLidarRequestReset(handle, cb, client_data);
}

livox_status LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb, void* client_data) {
  return CommandImpl::LivoxLidarRequestReboot(handle, cb, client_data);
}

// upgrade
bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path) {
  return UpgradeManager::GetInstance().SetLivoxLidarUpgradeFirmwarePath(firmware_path);
}

void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data) {
  UpgradeManager::GetInstance().SetLivoxLidarUpgradeProgressCallback(cb, client_data);
}

void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num) {
  UpgradeManager::GetInstance().UpgradeLivoxLidars(handle, lidar_num);
}

livox_status LivoxLidarStartLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerCallback cb, void* client_data) {
  return LoggerManager::GetInstance().StartLogger(handle, log_type, cb, client_data);
}

livox_status LivoxLidarStopLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerCallback cb, void* client_data) {
  return LoggerManager::GetInstance().StopLogger(handle, log_type, cb, client_data);
}
