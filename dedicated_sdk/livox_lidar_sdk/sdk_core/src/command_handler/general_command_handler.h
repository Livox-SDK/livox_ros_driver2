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

#ifndef GENERAL_COMMAND_HANDLER_H_
#define GENERAL_COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/define.h"

#include "livox_lidar_sdk.h"

#include "livox_lidar_def.h"
#include "device_manager.h"
#include "command_handler.h"

namespace livox {
namespace lidar {

typedef struct {
  std::string sn;
  std::string lidar_ip;
  uint8_t dev_type;
  std::atomic<bool> is_update_cfg;
} DeviceInfo;

class HapCommandHandle;

class GeneralCommandHandler : public noncopyable {
 private:
  GeneralCommandHandler() {}
  GeneralCommandHandler(const GeneralCommandHandler& other) = delete;
  GeneralCommandHandler& operator=(const GeneralCommandHandler& other) = delete;
 public:
  ~GeneralCommandHandler();
  void Destory();
  static GeneralCommandHandler& GetInstance();

  bool Init(const std::string& host_ip, const bool is_view, DeviceManager* device_manager);

  bool Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr, DeviceManager* device_manager);
  
  // void SetDeviceManager(DeviceManager* device_manager);

  void Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size);

  void Handler(const uint8_t dev_type, const uint32_t handle, const uint16_t lidar_port,
      uint8_t *buf, uint32_t buf_size);

  void CommandsHandle(TimePoint now);
  void AddCommand(const Command& command);

  void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data) {
    livox_lidar_info_change_cb_ = cb;
    livox_lidar_info_change_client_data_ = client_data;
  }
  void UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info);
  void UpdateLidarCfg(const uint8_t dev_type, const uint32_t handle, const uint16_t lidar_cmd_port);
  void LivoxLidarInfoChange(const uint32_t handle);
 private:
  bool VerifyNetSegment(const DetectionData* detection_data);
  void CreateCommandHandle(const uint8_t dev_type);
 public:
  // livox_status GetFrameRate(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);
  // livox_status QueryLidarStateInfo(uint32_t handle, OnQueryLidarStateInfoCallback cb, void* client_data);
  //livox_status QueryLivoxLidarIpInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);
  livox_status QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);

  livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data);
  // livox_status EnableLivoxLidarHighResolutionPointType(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  // livox_status DisableLivoxLidarHighResolutionPointType(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  livox_status DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarIp(uint32_t handle, const LivoxLidarIpInfo* ip_config, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarPointDataHostIPCfg(uint32_t handle, const HostPointIPInfo& host_point_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarImuDataHostIPCfg(uint32_t handle, const HostImuDataIPInfo& host_imu_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarInstallAttitude(uint32_t handle, const LivoxLidarInstallAttitude& install_attitude,
      LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data);

  // livox_status SetLivoxLidarFrameRate(uint32_t handle, LivoxLidarPointFrameRate rate, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  livox_status DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  livox_status SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  livox_status DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  livox_status DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status SetLivoxLidarLogParam(uint32_t handle, const LivoxLidarLogParam& log_param, LivoxLidarAsyncControlCallback cb, void* client_data);

  livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data);

  /*******Upgrade Module***********/    
  livox_status LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb, void* client_data);

    /**
   * Upgrade related command
   */
  livox_status LivoxLidarStartUpgrade(uint32_t handle,  uint8_t *data, uint16_t length,
      LivoxLidarStartUpgradeCallback cb, void* client_data);

  livox_status LivoxLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
      LivoxLidarXferFirmwareCallback cb, void* client_data);

  livox_status LivoxLidarCompleteXferFirmware(uint32_t handle, uint8_t *data,
      uint16_t length, LivoxLidarCompleteXferFirmwareCallback cb, void* client_data);

  livox_status LivoxLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
      uint16_t length, LivoxLidarGetUpgradeProgressCallback cb, void* client_data);


  livox_status LivoxLidarRequestFirmwareInfo(uint32_t handle,
      LivoxLidarRequestFirmwareInfoCallback cb, void* client_data);

 private:
  std::shared_ptr<CommandHandler> GetLidarCommandHandler(const uint8_t dev_type);
  void HandleDetectionData(uint32_t handle, uint16_t lidar_port, const CommPacket& packet);
  livox_status SendCommand(uint32_t handle, uint16_t command_id, uint8_t *data,
      uint16_t length, const std::shared_ptr<CommandCallback> &cb);
 private:
  DeviceManager* device_manager_;
  std::unique_ptr<CommPort> comm_port_;

  std::mutex dev_type_mutex_;
  std::map<uint32_t, uint8_t> device_dev_type_;

  std::mutex devices_mutex_;
  std::map<uint32_t, DeviceInfo> devices_;

  std::map<uint8_t, std::shared_ptr<CommandHandler>> lidars_command_handler_;

  std::mutex commands_mutex_;
  std::map<uint32_t, std::pair<Command, TimePoint> > commands_;

  LivoxLidarInfoChangeCallback livox_lidar_info_change_cb_;
  void* livox_lidar_info_change_client_data_;

  std::string host_ip_;
  bool is_view_;

};

}  // namespace livox
} // namespace lidar

#endif  // GENERAL_COMMAND_HANDLER_H_
