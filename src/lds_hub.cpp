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

#include "lds_hub.h"
#include "parse_cfg_file/parse_industrial_lidar_cfg.h"

#include "call_back/hub_callback.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <thread>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/

/** For callback use only */
LdsHub *g_lds_hub = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds hub function ---------------------------------------------------------*/
LdsHub::LdsHub(double publish_freq) : Lds(publish_freq, kSourceRawHub) {
  auto_connect_mode_ = true;
  whitelist_count_ = 0;
  is_initialized_ = false;

  whitelist_count_ = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  ResetLdsHub();
}

LdsHub::~LdsHub() {}

void LdsHub::ResetLdsHub(void) {
  ResetLds(kSourceRawHub);
  ResetLidar(&hub_, kSourceRawHub);
}

int LdsHub::InitLdsHub(std::vector<std::string> &broadcast_code_strs, const char *user_config_path) {
  if (is_initialized_) {
    printf("LiDAR data source is already inited!\n");
    return -1;
  }

#ifdef BUILDING_ROS2
  DisableConsoleLogger();
#endif

  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(HubCallback::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(HubCallback::OnDeviceChange);

  for (auto input_str : broadcast_code_strs) {
    AddBroadcastCodeToWhitelist(input_str.c_str());
    printf("Cmdline input broadcast code : %s\n", input_str.c_str());
  }

  std::vector<UserRawConfig> lidar_raw_config;
  ParseIndustrialLidarCfgFile parse(user_config_path);
  parse.ParseHubConfig(hub_raw_config_, lidar_raw_config);
  ProcessHubConfig(lidar_raw_config);
  
  if (whitelist_count_) {
    DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i = 0; i < whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    EnableAutoConnectMode();
    printf(
        "No broadcast code was added to whitelist, swith to automatic "
        "connection mode!\n");
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lds_hub == nullptr) {
    g_lds_hub = this;
  }
  is_initialized_ = true;
  printf("Livox-SDK init success!\n");

  return 0;
}

void LdsHub::ProcessHubConfig(std::vector<UserRawConfig>& lidar_raw_config) {
  if (hub_raw_config_.enable_connect) {
    if (!AddBroadcastCodeToWhitelist(hub_raw_config_.broadcast_code)) {
      printf("Add hub[%s] [%d] to whitelist\n", hub_raw_config_.broadcast_code, hub_raw_config_.coordinate);
    } else {
      printf("Add hub[%s] to whitelist fail\n", hub_raw_config_.broadcast_code);
      memset(&hub_raw_config_, 0, sizeof(hub_raw_config_)); 
    }
  }

  for (size_t i = 0; i < lidar_raw_config.size(); ++i) {
    if (AddRawUserConfig(lidar_raw_config[i])) {
      printf("Lidar Raw config is already exist : %s \n", lidar_raw_config[i].broadcast_code);
    }
  }
}

int LdsHub::AddRawUserConfig(UserRawConfig &config) {
  if (IsExistInRawConfig(config.broadcast_code)) {
    return -1;
  }

  printf("Add lidar raw user config : %s \n", config.broadcast_code);
  lidar_raw_config_.push_back(std::move(config));
  return 0;
}

bool LdsHub::IsExistInRawConfig(const char *broadcast_code) {
  if (broadcast_code == nullptr) {
    return false;
  }

  for (auto ite_config : lidar_raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

/** Add broadcast code to whitelist */
int LdsHub::AddBroadcastCodeToWhitelist(const char *broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize) ||
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (IsBroadcastCodeExistInWhitelist(broadcast_code)) {
    printf("%s is alrealy exist!\n", broadcast_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code);
  ++whitelist_count_;

  return 0;
}

bool LdsHub::IsBroadcastCodeExistInWhitelist(const char *broadcast_code) {
  if (!broadcast_code) {
    return false;
  }

  for (uint32_t i = 0; i < whitelist_count_; i++) {
    if (strncmp(broadcast_code, broadcast_code_whitelist_[i],
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsHub::GetRawConfig(const char *broadcast_code, UserRawConfig &config) {
  if (broadcast_code == nullptr) {
    return -1;
  }

  for (auto ite_config : lidar_raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      config.enable_fan = ite_config.enable_fan;
      config.return_mode = ite_config.return_mode;
      config.coordinate = ite_config.coordinate;
      config.imu_rate = ite_config.imu_rate;
      return 0;
    }
  }

  return -1;
}

void LdsHub::PrepareExit(void) { DeInitLdsHub(); }

int LdsHub::DeInitLdsHub(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  return 0;
}

}  // namespace livox_ros
