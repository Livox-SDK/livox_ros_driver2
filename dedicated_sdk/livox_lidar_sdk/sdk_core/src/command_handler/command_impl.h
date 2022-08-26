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

#ifndef COMMAND_IMPL_H_
#define COMMAND_IMPL_H_

#include <memory>
#include <map>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/define.h"

#include "livox_lidar_api.h"

#include "livox_lidar_def.h"
#include "device_manager.h"
#include "command_handler.h"

namespace livox {
namespace lidar {

class CommandImpl {
 public:
  static livox_status QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data);

  static livox_status SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarIp(uint32_t handle, const LivoxLidarIpInfo* ip_config, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status SetLivoxLidarStateInfoHostIPCfg(uint32_t handle, const HostStateInfoIpInfo& host_state_info_ipcfg,
      LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status SetLivoxLidarPointDataHostIPCfg(uint32_t handle, const HostPointIPInfo& host_point_ipcfg,
      LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarImuDataHostIPCfg(uint32_t handle, const HostImuDataIPInfo& host_imu_ipcfg,
      LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarInstallAttitude(uint32_t handle, const LivoxLidarInstallAttitude& install_attitude,
      LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarRoiCfg0(uint32_t handle, const RoiCfg& roi_cfg0, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status SetLivoxLidarRoiCfg1(uint32_t handle, const RoiCfg& roi_cfg1, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status EnableLivoxLidarRoi(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status DisableLivoxLidarRoi(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarDetectMode(uint32_t handle, LivoxLidarDetectMode mode,
    LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarFuncIOCfg(uint32_t handle, const FuncIOCfg& func_io_cfg,
    LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);
  static livox_status DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status SetLivoxLidarLogParam(uint32_t handle, const LivoxLidarLogParam& log_param, LivoxLidarAsyncControlCallback cb, void* client_data);

  static livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data);

  /*******Upgrade Module***********/    
  static livox_status LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb, void* client_data);

    /**
   * Upgrade related command
   */
  static livox_status LivoxLidarStartUpgrade(uint32_t handle,  uint8_t *data, uint16_t length,
      LivoxLidarStartUpgradeCallback cb, void* client_data);

  static livox_status LivoxLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
      LivoxLidarXferFirmwareCallback cb, void* client_data);

  static livox_status LivoxLidarCompleteXferFirmware(uint32_t handle, uint8_t *data,
      uint16_t length, LivoxLidarCompleteXferFirmwareCallback cb, void* client_data);

  static livox_status LivoxLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
      uint16_t length, LivoxLidarGetUpgradeProgressCallback cb, void* client_data);


  static livox_status LivoxLidarRequestFirmwareInfo(uint32_t handle,
      LivoxLidarRequestFirmwareInfoCallback cb, void* client_data);
};

}  // namespace livox
} // namespace lidar

#endif  // COMMAND_IMPL_H_
