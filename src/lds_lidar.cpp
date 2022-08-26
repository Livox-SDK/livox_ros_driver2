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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>


#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

#include "comm/comm.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_industrial_lidar_cfg.h"
#include "parse_cfg_file/parse_vehicle_lidar_cfg.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/industrial_lidar_callback.h"
#include "call_back/vehicle_lidar_callback.h"
#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq) : Lds(publish_freq, kSourceRawLidar) {
  has_vehicle_lidar_ = false;
  auto_connect_mode_ = true;
  is_initialized_ = false;
  enable_timesync_ = false;

  whitelist_count_ = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }



bool LdsLidar::InitLdsLidar(const std::string& path_name, const std::vector<std::string>& broadcast_code_strs) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
  if (!InitLidars(broadcast_code_strs)) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars(const std::vector<std::string>& broadcast_code_strs) {
  if (!ParseSummaryConfig()) {
    return false;
  }
  printf("parse info, lidar type: %d \n", lidar_summary_info_.lidar_type);

  if (lidar_summary_info_.lidar_type & kIndustryLidarType) {
    if (!InitIndustrialLidar(broadcast_code_strs)) {
      return false;
    }
  } else if (lidar_summary_info_.lidar_type & kVehicleLidarType) {
    if (!InitVehicleLidar()) {
      return false;
    }
  } else if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kIndustryLidarType) {
    if (!IndustrialLidarStart()) {
      return false;
    }
  } else if (lidar_summary_info_.lidar_type & kVehicleLidarType) {
    if (!VehicleLidarStart()) {
      return false;
    }
  } else if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitIndustrialLidar(const std::vector<std::string> &broadcast_code_strs) {
#ifdef BUILDING_ROS2
  DisableConsoleLogger();
#endif

  if (!Init()) {
    Uninit();
    printf("Industrial Livox-SDK init fail!\n");
    return false;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(IndustrialLidarCallback::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(IndustrialLidarCallback::OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  /** progress industrial lidar config */
  std::vector<UserRawConfig> raw_industrial_config;
  ParseIndustrialLidarCfgFile parse(path_);
  if (!parse.ParseIndustrialLidarConfig(raw_industrial_config, enable_timesync_, timesync_config_)) {
    return false;
  }

  ProcessRawIndustrialConfig(raw_industrial_config);

  if (whitelist_count_) {
    DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i = 0; i < whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    EnableAutoConnectMode();
    printf("No broadcast code was added to whitelist, swith to automatic connection mode!\n");
  }

  if (enable_timesync_) {
    timesync_ = TimeSync::GetInstance();
    if (timesync_->InitTimeSync(timesync_config_)) {
      printf("Timesync init fail\n");
      return false;
    }

    if (timesync_->SetReceiveSyncTimeCb(IndustrialLidarCallback::ReceiveSyncTimeCallback, this)) {
      printf("Set Timesync callback fail\n");
      return false;
    }

    timesync_->StartTimesync();
  }

  printf("Init Industrial lidar succ.\n");
  return true;
}

bool LdsLidar::InitVehicleLidar() {
#ifdef BUILDING_ROS2
  DisableConsoleLogger();
#endif

  //std::string local_addr;
  VehicleLidarOption vehicle_lidar_option;
  std::vector<UserVehicleConfig> vehicle_config;
  std::vector<LidarRegisterInfo> vehicle_lidars_register_info;

  std::vector<LidarExtrinsicParameters> vehicle_lidars_extrins_params;

  ParseVehicleLidarCfgFile parse(path_);
  if (!parse.Parse(vehicle_lidar_option, vehicle_config,
      vehicle_lidars_register_info, vehicle_lidars_extrins_params)) {
    printf("Parse Vehicle lidar config file failed.\n");
    return false;
  }
  if (vehicle_config.empty()) {
    printf("No vehicle lidars needs to be initialized.\n");
    return false;
  }
  has_vehicle_lidar_ = true;

  ProcessVehicleConfig(vehicle_config);

	/** Init with host's network card's ip address. */
  if (!LivoxVehicleInit(/*vehicle_lidar_option.product_type,*/ vehicle_lidar_option.local_addr.c_str())) {
    printf("Livox Vehicle Init Failed.\n");
    LivoxVehicleUninit();
    return false;
  }

  LivoxVehicleSdkVersion _vehicle_sdk_version;
  GetLivoxVehicleSdkVersion(&_vehicle_sdk_version);
  printf("Livox SDK version %d.%d.%d\n", _vehicle_sdk_version.major, _vehicle_sdk_version.minor, _vehicle_sdk_version.patch);

  /** Set Lidar Inforamtion Change Callback. */
  SetVehicleLidarInfoChangeCallback(VehicleLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);

  /** Register lidars slot and ip address to listen. */
  RegisterLidarInfo(vehicle_lidars_register_info.data(), vehicle_lidars_register_info.size());

  /** Set vehicle lidars extrinsic params */
	for (size_t i = 0; i < vehicle_lidars_extrins_params.size(); ++i) {
		AddLidarsExtrinsicParams(vehicle_lidars_extrins_params[i]);
	}

  printf("Vehicle lidar livox-sdk init succ.\n");
  return true;
}

bool LdsLidar::InitLivoxLidar() {
#ifdef BUILDING_ROS2
  DisableConsoleLogger();
#endif

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;

    LidarExtrinsicParameters param;
    param.handle = config.handle;
    param.lidar_type = kLivoxLidarType;
    param.roll  = config.extrinsic_param.roll;
    param.pitch = config.extrinsic_param.pitch;
    param.yaw   = config.extrinsic_param.yaw;
    param.x     = config.extrinsic_param.x;
    param.y     = config.extrinsic_param.y;
    param.z     = config.extrinsic_param.z;
    AddLidarsExtrinsicParams(param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  SetPointCloudCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  SetLidarImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  PointCloudConfig point_cloud_config;
  point_cloud_config.publish_freq = Lds::GetLdsFrequency();
  SetPointCloudConfig(point_cloud_config);
}

bool LdsLidar::IndustrialLidarStart() {
  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Industrial Livox-SDK start fail!\n");
    return false;
  }
  printf("Start Industrial lidar succ.\n");
  return true;
}

bool LdsLidar::VehicleLidarStart() {
  /** Sync work mode to lidars by 1 Hz. */
  return vehicle_lidar_thread_.Start();
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

void LdsLidar::ProcessRawIndustrialConfig(const std::vector<UserRawConfig>& raw_industrial_config) {
  for (size_t i = 0; i < raw_industrial_config.size(); ++i) {
    const UserRawConfig& config = raw_industrial_config[i];
    if (config.enable_connect) {
      if (!AddBroadcastCodeToWhitelist(config.broadcast_code)) {
        if (AddRawIndustrialUserConfig(config)) {
          printf("Industrial Raw config is already exist : %s \n", config.broadcast_code);
        }
      }
    }
  }
}

void LdsLidar::ProcessVehicleConfig(const std::vector<UserVehicleConfig>& vehicle_config) {
  for (size_t i = 0; i < vehicle_config.size(); ++i) {
    uint8_t index = 0;
    printf("Process Vehicle Config size:%lu, slot:%u.\n", vehicle_config.size(), vehicle_config[i].slot);

    int8_t ret = cache_index_.GetFreeIndex(kVehicleLidarType, vehicle_config[i].slot, index);
    if (ret != 0) {
      printf("Process raw vechicle config failed, can not get index by slot, the slot:%u\n", vehicle_config[i].slot);
      continue;
    }

    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);

    p_lidar->lidar_type = kVehicleLidarType;

    UserVehicleConfig &user_vehicle_config = p_lidar->vehicle_config;
    user_vehicle_config.slot = vehicle_config[i].slot;
    user_vehicle_config.data_type = vehicle_config[i].data_type;
    user_vehicle_config.scan_pattern = vehicle_config[i].scan_pattern;
    user_vehicle_config.dual_emit_enable = vehicle_config[i].dual_emit_enable;
    user_vehicle_config.blind_spot_set = vehicle_config[i].blind_spot_set;
  }
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char *broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize) || (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::IsBroadcastCodeExistInWhitelist(broadcast_code)) {
    printf("%s is alrealy exist!\n", broadcast_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code);
  ++whitelist_count_;

  return 0;
}

bool LdsLidar::IsBroadcastCodeExistInWhitelist(const char *broadcast_code) {
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

int LdsLidar::AddRawIndustrialUserConfig(const UserRawConfig &config) {
  if (IsExistInRawIndustrialConfig(config.broadcast_code)) {
    return -1;
  }

  raw_industrial_config_.push_back(config);
  printf("Add Raw Industrial user config : %s \n", config.broadcast_code);

  return 0;
}

bool LdsLidar::IsExistInRawIndustrialConfig(const char *broadcast_code) {
  if (broadcast_code == nullptr) {
    return false;
  }

  for (auto ite_config : raw_industrial_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code, kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::GetRawIndustrialConfig(const char *broadcast_code, UserRawConfig &config) {
  if (broadcast_code == nullptr) {
    return -1;
  }

  for (auto ite_config : raw_industrial_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code, kBroadcastCodeSize) == 0) {
      config.enable_fan = ite_config.enable_fan;
      config.return_mode = ite_config.return_mode;
      config.coordinate = ite_config.coordinate;
      config.imu_rate = ite_config.imu_rate;
      config.extrinsic_parameter_source = ite_config.extrinsic_parameter_source;
      config.enable_high_sensitivity = ite_config.enable_high_sensitivity;
      return 0;
    }
  }

  return -1;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Industrial Livox SDK Deinit completely!\n");

  vehicle_lidar_thread_.Destory();
  LivoxVehicleUninit();
  printf("Vehicle Livox SDK Deinit completely!\n");

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  if (timesync_) {
    timesync_->DeInitTimeSync();
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}  // namespace livox_ros
