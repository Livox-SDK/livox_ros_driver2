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
#include "parse_cfg_file/parse_direct_lidar_cfg.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/industrial_lidar_callback.h"
#include "call_back/vehicle_lidar_callback.h"
#include "call_back/direct_lidar_callback.h"
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
  } else if (lidar_summary_info_.lidar_type & kDirectLidarType) {
    if (!InitDirectLidar()) {
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
  } else if (lidar_summary_info_.lidar_type & kDirectLidarType) {
    if (!DirectLidarStart()) {
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
    LivoxLidarInstallAttitude attitude;
    attitude.roll_deg = vehicle_lidars_extrins_params[i].roll;
    attitude.pitch_deg = vehicle_lidars_extrins_params[i].pitch;
    attitude.yaw_deg = vehicle_lidars_extrins_params[i].yaw;
    attitude.x = vehicle_lidars_extrins_params[i].x;
    attitude.y = vehicle_lidars_extrins_params[i].y;
    attitude.z = vehicle_lidars_extrins_params[i].z;
    SetLivoxLidarInstallAttitude(vehicle_lidars_extrins_params[i].handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, nullptr);
	}

  printf("Vehicle lidar livox-sdk init succ.\n");
  return true;
}

bool LdsLidar::InitDirectLidar() {
#ifdef BUILDING_ROS2
  DisableDirectLidarConsoleLogger();
#endif

  if (!ParseDirectLidarCfgFile(path_).Parse(direct_lidar_param_)) {
    return false;
  }
  if (direct_lidar_param_.direct_host_cfg_ptr == nullptr && direct_lidar_param_.direct_lidars_cfg_ptr == nullptr) {
    return false;
  }

  if (direct_lidar_param_.direct_host_cfg_ptr) {
    if (!LivoxDirectInit(direct_lidar_param_.direct_host_cfg_ptr.get(), nullptr)) {
      return false;
    }
  } else {
    direct_lidar_param_.is_custom = true;
    if (!LivoxDirectInit(nullptr, direct_lidar_param_.direct_lidars_cfg_ptr->data(),
        direct_lidar_param_.direct_lidars_cfg_ptr->size())) {
      return false;
    }
    SetDirectLidarExtrinsicParams();
    ProcessDirectConfig();
  }

  SetDirectLidarInfoCallback(DirectLidarCallback::DirectLidarInfoCb, this);
  SetDirectLidarCfgUpdateCallback(DirectLidarCallback::DirectLidarCfgUpdateCb, this);
  printf("Init direct lidar succ.\n");
  return true;
}

bool LdsLidar::InitLivoxLidar() {
  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<LidarExtrinsicParameters> extrinsic_params;
  std::vector<UserLivoxLidarConfig> basic_configs;
  if (!parser.Parse(basic_configs, extrinsic_params)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // fill in lidar devices
  for (auto& config : basic_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
  }

  // SDK initializtion
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // register extrinsic parameters to sdk
  for (auto& param : extrinsic_params) {
    AddLidarsExtrinsicParams(param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetDirectLidarExtrinsicParams() {
  for (size_t i = 0; i < direct_lidar_param_.direct_lidars_cfg_ptr->size(); ++i) {
    //uint8_t slot = direct_lidar_param_.direct_lidars_cfg_ptr->at(i).lidar_id;
    const InstallAttitude& install_attitude = direct_lidar_param_.direct_lidars_cfg_ptr->at(i).install_attitude;
    LidarExtrinsicParameters extrinsic_params;
    //extrinsic_params.slot = slot;
    extrinsic_params.handle = inet_addr(direct_lidar_param_.direct_lidars_cfg_ptr->at(i).lidar_ipinfo_cfg.lidar_ipaddr);
    extrinsic_params.lidar_type = static_cast<uint8_t>(kDirectLidarType);
    extrinsic_params.roll = install_attitude.roll_deg;
    extrinsic_params.pitch = install_attitude.pitch_deg;
    extrinsic_params.yaw = install_attitude.yaw_deg;
    extrinsic_params.x = install_attitude.x;
    extrinsic_params.y = install_attitude.y;
    extrinsic_params.z = install_attitude.z;
    AddLidarsExtrinsicParams(extrinsic_params);
  }
}

void LdsLidar::SetDirectLidarExtrinsicParamsByDeviceInfo(const uint32_t handle, const InstallAttitude& install_attitude) {
  if (direct_lidar_param_.is_custom) {
    return;
  }

  std::map<uint32_t, LidarExtrinsicParameters>& extrinsic_params_map = direct_lidar_param_.extrinsic_params_map;
  if (extrinsic_params_map.find(handle) == extrinsic_params_map.end()) {
    LidarExtrinsicParameters extrinsic_params;
    //extrinsic_params.slot = slot;
    extrinsic_params.handle = handle;
    extrinsic_params.lidar_type = static_cast<uint8_t>(kDirectLidarType);
    extrinsic_params.roll = install_attitude.roll_deg;
    extrinsic_params.pitch = install_attitude.pitch_deg;
    extrinsic_params.yaw = install_attitude.yaw_deg;
    extrinsic_params.x = install_attitude.x;
    extrinsic_params.y = install_attitude.y;
    extrinsic_params.z = install_attitude.z;
    AddLidarsExtrinsicParams(extrinsic_params);
    extrinsic_params_map[handle] = extrinsic_params;
  }
}

void LdsLidar::ProcessDirectConfig() {
  std::vector<DirectLidarCfg>& direct_lidar_cfg_vec = *direct_lidar_param_.direct_lidars_cfg_ptr;
  for (size_t i = 0; i < direct_lidar_cfg_vec.size(); ++i) {
    const DirectLidarCfg& direct_lidar_cfg = direct_lidar_cfg_vec[i];
    uint32_t handle = inet_addr(direct_lidar_cfg.lidar_ipinfo_cfg.lidar_ipaddr);
    uint8_t index = 0;
    int8_t ret = cache_index_.GetFreeIndex(kDirectLidarType, handle, index);
    if (ret != 0) {
      printf("Process raw vechicle config failed, can not get index by slot, the slot:%u\n", direct_lidar_cfg.lidar_id);
      continue;
    }

    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    UserDirectConfig &user_direct_config = p_lidar->direct_config;

    strcpy(user_direct_config.sn, direct_lidar_cfg.sn);
    user_direct_config.lidar_id = direct_lidar_cfg.lidar_id;
    user_direct_config.lidar_ipmode = direct_lidar_cfg.lidar_ipmode;

    strcpy(user_direct_config.lidar_ip, direct_lidar_cfg.lidar_ipinfo_cfg.lidar_ipaddr);
    strcpy(user_direct_config.lidar_submask, direct_lidar_cfg.lidar_ipinfo_cfg.lidar_subnet_mask);
    strcpy(user_direct_config.lidar_gateway, direct_lidar_cfg.lidar_ipinfo_cfg.lidar_gateway);

    strcpy(user_direct_config.host_push_msg_ip, direct_lidar_cfg.host_cfg.host_push_cmd_ip);
    user_direct_config.host_push_msg_port = direct_lidar_cfg.host_cfg.host_push_cmd_port;

    strcpy(user_direct_config.host_point_data_ip, direct_lidar_cfg.host_cfg.host_point_data_ip);
    user_direct_config.host_point_data_port = direct_lidar_cfg.host_cfg.host_point_data_port;

    strcpy(user_direct_config.host_imu_data_ip, direct_lidar_cfg.host_cfg.host_imu_data_ip);
    user_direct_config.host_imu_data_port = direct_lidar_cfg.host_cfg.host_imu_data_port;

    user_direct_config.sample_mode = direct_lidar_cfg.sample_mode;
    user_direct_config.pattern_mode = direct_lidar_cfg.pattern_mode;
    user_direct_config.pcl_data_type = direct_lidar_cfg.pcl_data_type;
    user_direct_config.imu_data_en = direct_lidar_cfg.imu_data_en;
    user_direct_config.work_mode = direct_lidar_cfg.work_mode;
  }
}

void LdsLidar::ProcessDirectConfigByDeviceInfo(const uint32_t handle, const DirectLidarStateInfo* info) {
  if (direct_lidar_param_.is_custom) {
    return;
  }
  //printf("ProcessDirectConfigByDeviceInfo handle:%u, sn:%s.\n", handle, info->sn);
  if (direct_lidar_param_.devices.find(handle) != direct_lidar_param_.devices.end()) {
    return;
  }

  uint8_t index = 0;
  //printf("ProcessDirectConfigByDeviceInfo lidar_type:%u, handle:%u.\n", kDirectLidarType, handle);
  int8_t ret = cache_index_.GetFreeIndex(kDirectLidarType, handle, index);
  if (ret != 0) {
    printf("Process raw vechicle config failed, can not get index by handle, the handle:%u\n", handle);
    return;
  }


  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
  UserDirectConfig &user_direct_config = p_lidar->direct_config;

  strcpy(user_direct_config.sn, info->sn);
  user_direct_config.lidar_id = info->lidar_id;
  user_direct_config.lidar_ipmode = info->lidar_ipmode;

  strcpy(user_direct_config.lidar_ip, info->lidar_ip);
  strcpy(user_direct_config.lidar_submask, info->lidar_submask);
  strcpy(user_direct_config.lidar_gateway, info->lidar_gateway);

  strcpy(user_direct_config.host_push_msg_ip, info->host_push_msg_ip);
  user_direct_config.host_push_msg_port = info->host_push_msg_port;

  strcpy(user_direct_config.host_point_data_ip, info->host_point_data_ip);
  user_direct_config.host_point_data_port = info->host_point_data_port;

  strcpy(user_direct_config.host_imu_data_ip, info->host_imu_data_ip);
  user_direct_config.host_imu_data_port = info->host_imu_data_port;

  user_direct_config.sample_mode = info->sample_mode;
  user_direct_config.pattern_mode = info->pattern_mode;
  user_direct_config.pcl_data_type = info->pcl_data_type;
  user_direct_config.imu_data_en = info->imu_data_en;
  user_direct_config.work_mode = info->work_mode;

  direct_lidar_param_.devices[user_direct_config.lidar_id] = user_direct_config.sn;
}

void LdsLidar::DirectLidarStartSamp(const uint32_t handle) {
  uint8_t index = 0;
  int8_t ret = cache_index_.GetFreeIndex(kDirectLidarType, handle, index);
  if (ret != 0) {
    printf("Process raw vechicle config failed, can not get index by slot, the slot:%u\n", handle);
    return;
  }

  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
  p_lidar->connect_state = kConnectStateSampling;
}

void LdsLidar::SetLidarPubHandle() {
  SetPointCloudCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  SetLidarImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  PointCloudConfig point_cloud_config;
  point_cloud_config.publish_freq = 10; // 10Hz;
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

bool LdsLidar::DirectLidarStart() {
  if (!LivoxDirectStart()) {
    printf("Direct lidar start failed.\n");
    return false;
  }
  printf("Direct lidar start succ.\n");
  return true;
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
