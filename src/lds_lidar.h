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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <memory>
#include <mutex>
#include <vector>

#include "lds.h"
#include "vehicle_lidar_thread.h"
#include "comm/comm.h"

#include "livox_def.h"
#include "livox_sdk.h"

#include "livox_def_common.h"
#include "livox_sdk_common.h"

#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include "rapidjson/document.h"
#include "timesync.h"

namespace livox_ros {

class LdsLidar : public Lds {
 public:
  static LdsLidar *GetInstance(double publish_freq) {
    printf("LdsLidar *GetInstance\n");
    static LdsLidar lds_lidar(publish_freq);
    return &lds_lidar;
  }

  bool InitLdsLidar(const std::string& path_name, const std::vector<std::string>& broadcast_code_strs);
  bool Start();


  int DeInitLdsLidar(void);
 private:
  LdsLidar(double publish_freq);
  LdsLidar(const LdsLidar &) = delete;
  ~LdsLidar();
  LdsLidar &operator=(const LdsLidar &) = delete;

  bool ParseSummaryConfig();

  bool InitLidars(const std::vector<std::string>& broadcast_code_strs);
  bool InitIndustrialLidar(const std::vector<std::string> &broadcast_code_strs);
  bool InitVehicleLidar();
  bool InitLivoxLidar();    // for new SDK

  bool IndustrialLidarStart();
  bool VehicleLidarStart();
  bool LivoxLidarStart();

  void ResetLdsLidar(void);

  void SetLidarPubHandle();

  // Process config
  void ProcessRawIndustrialConfig(const std::vector<UserRawConfig>& raw_industrial_config);
  void ProcessVehicleConfig(const std::vector<UserVehicleConfig>& raw_vehicle_config);

	// white list
  int AddBroadcastCodeToWhitelist(const char *broadcast_code);
  bool IsBroadcastCodeExistInWhitelist(const char *broadcast_code);

	// auto connect mode
	void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

	// Raw industrial user config
 	int AddRawIndustrialUserConfig(const UserRawConfig &config);
  bool IsExistInRawIndustrialConfig(const char *broadcast_code);
	int GetRawIndustrialConfig(const char *broadcast_code, UserRawConfig &config);

  // Raw vehicle user config
	int AddRawVehicleUserConfig(const UserVehicleConfig &config);
	bool IsExistInRawVehicleConfig(const char *broadcast_code);
	int GetRawVehicleConfig(const char *broadcast_code, UserRawConfig &config);

  virtual void PrepareExit(void);

 public:
  std::mutex config_mutex_;

 private:
  friend class IndustrialLidarCallback;
  friend class VehicleLidarCallback;
  std::string path_;
  LidarSummaryInfo lidar_summary_info_;

  bool has_vehicle_lidar_;
  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];

  std::vector<UserRawConfig> raw_industrial_config_;
  bool enable_timesync_;
  TimeSync *timesync_;
  TimeSyncConfig timesync_config_;

  VehicleLidarThread vehicle_lidar_thread_;
};

}  // namespace livox_ros
#endif
