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

#include "livox_lidar_sdk.h"
#include "livox_lidar_def.h"

#include "base/command_callback.h"
#include "base/logging.h"

#include "comm/define.h"

// #include "command_handler/command_handler.h"
// #include "data_handler/data_handler.h"


#include "parse_cfg_file.h"
#include "params_check.h"

#include "device_manager.h"

#include "command_handler/general_command_handler.h"
#include "data_handler/data_handler.h"

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

bool LivoxLidarSdkInit(const char* path, const char* host_ip) {
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

  do {
    InitLogger();

    if (path == NULL && host_ip == NULL) {
      return false;
    }

    if (path) {
      std::shared_ptr<std::vector<LivoxLidarCfg>> lidars_cfg_ptr = nullptr;
      std::shared_ptr<std::vector<LivoxLidarCfg>> custom_lidars_cfg_ptr = nullptr;

      if (!ParseCfgFile(path).Parse(lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
        return false;
      }

      if (!ParamsCheck(lidars_cfg_ptr, custom_lidars_cfg_ptr).Check()) {
        return false;
      }

      if (!DeviceManager::GetInstance().Init(lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
        return false;
      }
    } else {
      if (!DeviceManager::GetInstance().Init(host_ip)) {
        return false;
      }
    }
  } while (0);

  is_initialized = true;
  return true;
}

void LivoxLidarSdkUninit() {
  if (!is_initialized) {
    return;
  }
#ifdef WIN32
    WSACleanup();
#endif // WIN32

  // command_handler().Destory();
  // data_handler().Destory();

  DeviceManager::GetInstance().Destory();
  GeneralCommandHandler::GetInstance().Destory();
  DataHandler::GetInstance().Destory();

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

void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data) {
  DataHandler::GetInstance().SetImuDataCallback(cb, client_data);
}


void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data) {
  GeneralCommandHandler::GetInstance().SetLivoxLidarInfoChangeCallback(cb, client_data);
}


livox_status QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().QueryLivoxLidarInternalInfo(handle, cb, client_data);
}

livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data) {
  if (data_type == kLivoxLidarImuData) {
    return EnableLivoxLidarImuData(handle, cb, client_data);
  }
  return GeneralCommandHandler::GetInstance().SetLivoxLidarPclDataType(handle, data_type, cb, client_data);
}

livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarScanPattern(handle, scan_pattern, cb, client_data);
}

livox_status SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarDualEmit(handle, enable, cb, client_data);
}

livox_status EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().EnableLivoxLidarPointSend(handle, cb, client_data);
}
livox_status DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().DisableLivoxLidarPointSend(handle, cb, client_data);
}

livox_status SetLivoxLidarIp(uint32_t handle, LivoxLidarIpInfo* ip_config,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarIp(handle, ip_config, cb, client_data);
}

livox_status SetLivoxLidarPointDataHostIPCfg(uint32_t handle, HostPointIPInfo* host_point_ipcfg,
  LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarPointDataHostIPCfg(handle, *host_point_ipcfg, cb, client_data);
}

livox_status SetLivoxLidarImuDataHostIPCfg(uint32_t handle, HostImuDataIPInfo* host_imu_ipcfg,
  LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarImuDataHostIPCfg(handle, *host_imu_ipcfg, cb, client_data);
}

livox_status SetLivoxLidarInstallAttitude(uint32_t handle, LivoxLidarInstallAttitude* install_attitude,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarInstallAttitude(handle, *install_attitude, cb, client_data);
}

livox_status SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarBlindSpot(handle, blind_spot, cb, client_data);
}

// livox_status SetLivoxLidarFrameRate(uint32_t handle, LivoxLidarPointFrameRate rate, LivoxLidarAsyncControlCallback cb, void* client_data) {
//   return GeneralCommandHandler::GetInstance().SetLivoxLidarFrameRate(handle, rate, cb, client_data);
// }

livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarWorkMode(handle, work_mode, cb, client_data);
}

livox_status EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().EnableLivoxLidarGlassHeat(handle, cb, client_data);
}
livox_status DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().DisableLivoxLidarGlassHeat(handle, cb, client_data);
}
livox_status SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SetLivoxLidarGlassHeat(handle, glass_heat, cb, client_data);
}

livox_status EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().EnableLivoxLidarImuData(handle, cb, client_data);
}
livox_status DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().DisableLivoxLidarImuData(handle, cb, client_data);
}

livox_status EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().EnableLivoxLidarFusaFunciont(handle, cb, client_data);
}
livox_status DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().DisableLivoxLidarFusaFunciont(handle, cb, client_data);
}





// livox_status SetLivoxLidarLogParam(uint32_t handle, LivoxLidarLogParam* log_param, LivoxLidarAsyncControlCallback cb, void* client_data) {
//   return GeneralCommandHandler::GetInstance().SetLivoxLidarLogParam(handle, *log_param, cb, client_data);
// }



// reset lidar
livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarRequestReset(handle, cb, client_data);
}

// upgrade    
livox_status LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarRequestReboot(handle, cb, client_data);
}

livox_status LivoxLidarStartUpgrade(uint32_t handle,  uint8_t *data, uint16_t length,
    LivoxLidarStartUpgradeCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarStartUpgrade(handle, data, length, cb, client_data);
}

livox_status LivoxLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarXferFirmwareCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarXferFirmware(handle, data, length, cb, client_data);
}

livox_status LivoxLidarCompleteXferFirmware(uint32_t handle, uint8_t *data,
    uint16_t length, LivoxLidarCompleteXferFirmwareCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarCompleteXferFirmware(handle, data, length, cb, client_data);
}

livox_status LivoxLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
    uint16_t length, LivoxLidarGetUpgradeProgressCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarGetUpgradeProgress(handle, data, length, cb, client_data);
}

livox_status LivoxLidarRequestFirmwareInfo(uint32_t handle,
    LivoxLidarRequestFirmwareInfoCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarRequestFirmwareInfo(handle, cb, client_data);
}






