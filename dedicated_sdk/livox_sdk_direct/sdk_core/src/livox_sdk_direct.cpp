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

#include "livox_sdk_direct.h"
#include "livox_def_direct.h"

#include "base/command_callback.h"
#include "base/logging.h"

#include "comm/define.h"

#include "command_handler/command_handler.h"
#include "data_handler/data_handler.h"


#include "parse_input_params.h"

#ifdef WIN32
#include<winsock2.h>
#endif // WIN32

#include <memory>

using namespace livox::direct;

static bool is_initialized = false;

void GetLivoxSdkDirectVersion(LivoxSdkDirectVersion *version) {
  if (version != NULL) {
    version->major = LIVOX_SDK_DIRECT_MAJOR_VERSION;
    version->minor = LIVOX_SDK_DIRECT_MINOR_VERSION;
    version->patch = LIVOX_SDK_DIRECT_PATCH_VERSION;
  }
}

bool LivoxDirectInit(DirectLidarHostCfg* direct_host_cfg_ptr, DirectLidarCfg* direct_lidar_cfg_ptr, const uint8_t lidars_num) {
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

  //bool result = false;
  do {
    InitLogger();

    std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr = nullptr;
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr = nullptr;
    ParseInputParams parse_input_params(direct_host_cfg_ptr, direct_lidar_cfg_ptr, lidars_num);

    if (!(parse_input_params.Parse(direct_host_ipinfo_ptr, direct_lidars_info_ptr))) {
      LOG_ERROR("Parse input params failed.");
      return false;
    }

    if (direct_host_ipinfo_ptr == nullptr && direct_lidars_info_ptr == nullptr) {
      LOG_ERROR("Device manager init failed, pointers to input parameters are null");
      return false;
    }

    if (lidars_num == 0 || lidars_num >= kMaxLidarCount) {
      LOG_ERROR("Device manager init failed, lidar_num is zero or exceeds the maximum 32");
      return false;
    }

    if (!command_handler().Init(direct_host_ipinfo_ptr, direct_lidars_info_ptr, lidars_num)) {
      return false;
    }

    if (!data_handler().Init(direct_host_ipinfo_ptr, direct_lidars_info_ptr)) {
      return false;
    }
  } while (0);

  is_initialized = true;
  return true;
}

void LivoxDirectUninit() {
  if (!is_initialized) {
    return;
  }
#ifdef WIN32
    WSACleanup();
#endif // WIN32

  command_handler().Destory();
  data_handler().Destory();

  UninitLogger();
  is_initialized = false;
}

bool LivoxDirectStart() {
  if (!command_handler().Start()) {
    LOG_ERROR("Command Handler Start Failed.");
    return false;
  }

  if (!data_handler().Start()) {
    LOG_ERROR("Data Handler Start Failed.");
    return false;
  }
  return true;
}

uint16_t DirectLidarAddPointCloudObserver(DirectPointCloudObserver cb, void *client_data) {
  return data_handler().AddPointCloudObserver(cb, client_data);
}

void DirectLidarRemovePointCloudObserver(uint16_t id) {
  data_handler().RemovePointCloudObserver(id);
}

void SetDirectLidarPointDataCallback(DirectPointDataCallback cb, void *client_data) {
  data_handler().SetPointDataCallback(cb, client_data);
}

void SetDirectLidarImuDataCallback(DirectImuDataCallback cb, void* client_data) {
  data_handler().SetImuDataCallback(cb, client_data);
}

void SetDirectLidarInfoCallback(DirectLidarInfoCallback cb, void* client_data) {
  command_handler().SetLidarInfoCallback(cb, client_data);
}

void SetDirectLidarCfgUpdateCallback(DirectLidarCfgUpdateCallabck cb, void* client_data) {
  command_handler().SetLidarCfgUpdateCallback(cb, client_data);
}

void SaveDirectLidarLoggerFile() {
  is_save_log_file = true;
}

void DisableDirectLidarConsoleLogger() {
  is_console_log_enable = false;
}

void UpdateDirectLidarCfg(const uint32_t handle, DirectLidarCfg* cfg) {
  std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr = nullptr;
  std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr = nullptr;
  ParseInputParams parse_input_params(nullptr, cfg, 1);
  if (!(parse_input_params.Parse(direct_host_ipinfo_ptr, direct_lidars_info_ptr))) {
    LOG_ERROR("Parse input params failed.");
    return;
  }
  command_handler().UpdateDirectLidarCfg(direct_lidars_info_ptr);
}

// Upgrade
livox_direct_status DirectLidarStartUpgrade(uint32_t handle, uint8_t *data, uint16_t length,
    DirectStartUpgradeCallback cb, void* client_data) {
  return command_handler().SendCommand(handle, 
                                      static_cast<uint16_t>(kKeyRequestUpgrade),
                                      data, 
                                      length,
                                      MakeCommandCallback<DirectStartUpgradeResponse>(cb, client_data));
}

livox_direct_status DirectLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    DirectXferFirmwareCallback cb, void* client_data) {
  return command_handler().SendCommand(handle, 
                                      kKeyXferFirmware,
                                      data, 
                                      length,
                                      MakeCommandCallback<DirectXferFirmwareResponse>(cb, client_data));
}

livox_direct_status DirectLidarCompleteXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    DirectCompleteXferFirmwareCallback cb, void* client_data) {
  return command_handler().SendCommand(handle,
                                      kKeyCompleteXferFirmware,
                                      data, 
                                      length,
                                      MakeCommandCallback<DirectCompleteXferFirmwareResponse>(cb, client_data));
}

livox_direct_status DirectLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
    uint16_t length, DirectGetUpgradeProgressCallback cb, void* client_data) {
  return command_handler().SendCommand(handle,
                                      kKeyRequestUpgradeProgress,
                                      data, 
                                      length,
                                      MakeCommandCallback<DirectGetUpgradeProgressResponse>(cb, client_data));
}

// livox_direct_status DirectLidarRequestFirmwareInfo(uint32_t handle,
//     DirectRequestFirmwareInfoCallback cb, void* client_data) {
//   return command_handler().SendCommand(handle,
//                                       kKeyRequestFirmwareInfo, 
//                                       nullptr, 
//                                       0,
//                                       MakeCommandCallback<DirectRequestFirmwareInfoResponse>(cb, client_data));
// }

livox_direct_status DirectLidarRequestReboot(uint32_t handle, DirectRebootCallback cb,
    void* client_data) {
  DirectRebootRequest reboot_request;
  reboot_request.timeout = 100;
  return command_handler().SendCommand(handle,
      kKeyRebootDevice, (uint8_t *)&reboot_request,
      sizeof(reboot_request), MakeCommandCallback<DirectRebootResponse>(cb,
      client_data));
}

// reset lidar
livox_direct_status DirectLidarRequestReset(uint32_t handle, DirectLidarResetCallback cb, void* client_data) {
  DirectLidarResetRequest reset_request;
  reset_request.data = 0;
  return command_handler().SendCommand(handle,
      kKeyReSetDevice, (uint8_t *)&reset_request,
      sizeof(reset_request), MakeCommandCallback<DirectLidarResetResponse>(cb,
      client_data));
}


// // Log 
// bool DeiviceLoggerInit(const char* log_save_path, uint32_t cache_size, const char* netif) {
//   InitLogger();
//   return logger_manager().Init(std::string(log_save_path), cache_size, std::string(netif));
// }

// void DeviceLoggerUninit() {
//   UninitLogger();
//   logger_manager().Uninit();
//   return;
// }

// void SetTransStateCallback(TransStateCallback cb) {
//   logger_manager().SetTransStateCallback(cb);
//   return;
// }

// void SetLoggerDeviceStateCallback(LoggerDeviceStateCallback cb) {
//   logger_manager().SetLoggerDeviceStateCallback(cb);
//   return;
// }

// livox_direct_status StartLogger(const char* broadcast_code, uint8_t log_type, uint32_t time_duaration, LoggerStartCallback cb) {
//   return logger_manager().StartLogger(broadcast_code,
//                                       log_type,
//                                       time_duaration,
//                                       cb);
// }


