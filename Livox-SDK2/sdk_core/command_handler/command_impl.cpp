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

#include "command_impl.h"

#include "livox_lidar_def.h"
#include "general_command_handler.h"
#include "debug_point_cloud_handler/debug_point_cloud_manager.h"
#include "spdlog/fmt/fmt.h"

#include "base/logging.h"
#include "comm/protocol.h"
#include "comm/generate_seq.h"

#include "build_request.h"

#include <sstream>
#include <inttypes.h>
#include <string>
#include <iomanip>
#include <chrono>
#include <vector>

namespace livox {
namespace lidar {

std::int64_t StringToTimestamp(std::string const& fmt, std::string const& date) {
    std::tm timestamp = {};
    std::stringstream date_ss(date);
    date_ss >> std::get_time(&timestamp, fmt.c_str());
    auto time_point = std::chrono::system_clock::from_time_t(std::mktime(&timestamp));

    return std::chrono::duration_cast<std::chrono::seconds>(time_point.time_since_epoch()).count();
}

std::vector<std::string> Split(std::string const& str, char const pattern) {
    std::vector<std::string> res;
    std::stringstream input(str);
    std::string part;
    while (getline(input, part, pattern)) {
        res.push_back(part);
    }
    return res;
}


std::uint64_t ParseGPRMC(std::string const& gprmc) {
    std::vector<std::string> gprmc_vec = Split(gprmc, ',');
    if (gprmc_vec.size() < 9 || gprmc_vec[1].length() < 6 || gprmc_vec[9].length() < 6) {
      LOG_ERROR("gprmc check failed. gprmc is : {}", gprmc);
      return 0;
    }
    auto year        = gprmc_vec[9].substr(4);
    auto month       = gprmc_vec[9].substr(2, 2);
    auto day         = gprmc_vec[9].substr(0, 2);
    auto hour        = gprmc_vec[1].substr(0, 2);
    auto minute      = gprmc_vec[1].substr(2, 2);
    auto second      = gprmc_vec[1].substr(4, 2);

    std::string time = fmt::format("{}-{}-{} {}:{}:{}", "20" + year, month, day, hour, minute, second);
    std::uint64_t time_ms = StringToTimestamp("%Y-%m-%d %H:%M:%S", time) * 1000;
    return time_ms * 1000 * 1000;
}

livox_status CommandImpl::QueryLivoxLidarInternalInfo(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  std::set<ParamKeyName> key_sets;

  if (!GeneralCommandHandler::GetInstance().GetQueryLidarInternalInfoKeys(handle, key_sets)) {
    LOG_ERROR("Query livox lidar internal info failed.");
    return kLivoxLidarStatusInvalidHandle;
  }

  uint16_t key_num = key_sets.size();
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  for (const auto &key : key_sets) {
    LivoxLidarKeyValueParam* kList = (LivoxLidarKeyValueParam*)&req_buff[req_len];
    kList->key = static_cast<uint16_t>(key);
    req_len += sizeof(uint16_t);
  }

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarGetInternalInfo,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarDiagInternalInfoResponse>(cb, client_data));
}

livox_status CommandImpl::QueryLivoxLidarFwType(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);


  LivoxLidarKeyValueParam* kList = (LivoxLidarKeyValueParam*)&req_buff[req_len];
  kList->key = static_cast<uint16_t>(kKeyFwType);
  req_len += sizeof(uint16_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarGetInternalInfo,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarDiagInternalInfoResponse>(cb, client_data));
}

livox_status CommandImpl::QueryLivoxLidarFirmwareVer(uint32_t handle, QueryLivoxLidarInternalInfoCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);


  LivoxLidarKeyValueParam* kList = (LivoxLidarKeyValueParam*)&req_buff[req_len];
  kList->key = static_cast<uint16_t>(kKeyVersionApp);
  req_len += sizeof(uint16_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarGetInternalInfo,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarDiagInternalInfoResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarPclDataType(uint32_t handle, LivoxLidarPointDataType data_type, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPclDataType);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(data_type);
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarScanPattern(uint32_t handle, LivoxLidarScanPattern scan_pattern, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPatternMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(scan_pattern);
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarDualEmit(uint32_t handle, bool enable, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyDualEmitEn);
  kv->length = sizeof(uint8_t);
  if (enable) {
    kv->value[0] = 0x01;
  } else {
    kv->value[0] = 0x00;
  }
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::EnableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPointSendEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::DisableLivoxLidarPointSend(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPointSendEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarIp(uint32_t handle, const LivoxLidarIpInfo* ip_config,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetLidarIPInfoRequest(*ip_config, req_buff, req_len)) {
    return -1;
  }

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarStateInfoHostIPCfg(uint32_t handle, const HostStateInfoIpInfo& host_state_info_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetHostStateInfoIPCfgRequest(host_state_info_ipcfg, req_buff, req_len)) {
    return -1;
  }

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarPointDataHostIPCfg(uint32_t handle, const HostPointIPInfo& host_point_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetHostPointDataIPInfoRequest(host_point_ipcfg, req_buff, req_len)) {
    return -1;
  }

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarImuDataHostIPCfg(uint32_t handle, const HostImuDataIPInfo& host_imu_ipcfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetHostImuDataIPInfoRequest(host_imu_ipcfg, req_buff, req_len)) {
    return -1;
  }

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status CommandImpl::SetLivoxLidarInstallAttitude(uint32_t handle, const LivoxLidarInstallAttitude& install_attitude,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyInstallAttitude);
  kv->length = sizeof(LivoxLidarInstallAttitude);

  LivoxLidarInstallAttitude* install_attitude_val = (LivoxLidarInstallAttitude*)&kv->value;
  memcpy(install_attitude_val, &install_attitude, sizeof(LivoxLidarInstallAttitude));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(LivoxLidarInstallAttitude);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarFovCfg0(uint32_t handle, const FovCfg& fov_cfg0, LivoxLidarAsyncControlCallback cb,
    void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFovCfg0);
  kv->length = sizeof(FovCfg);

  FovCfg* fov_cfg = (FovCfg*)&kv->value;
  memcpy(fov_cfg, &fov_cfg0, sizeof(FovCfg));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(FovCfg);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
  
}

livox_status CommandImpl::SetLivoxLidarFovCfg1(uint32_t handle, const FovCfg& fov_cfg1, LivoxLidarAsyncControlCallback cb,
    void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFovCfg1);
  kv->length = sizeof(FovCfg);

  FovCfg* fov_cfg = (FovCfg*)&kv->value;
  memcpy(fov_cfg, &fov_cfg1, sizeof(FovCfg));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(FovCfg);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::EnableLivoxLidarFov(uint32_t handle, uint8_t fov_en, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFovCfgEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = fov_en;

  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::DisableLivoxLidarFov(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFovCfgEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00;
  
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));

}

livox_status CommandImpl::SetLivoxLidarDetectMode(uint32_t handle, LivoxLidarDetectMode mode,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyDetectMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(mode);
  
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarFuncIOCfg(uint32_t handle, const FuncIOCfg& func_io_cfg,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFuncIoCfg);
  kv->length = sizeof(FuncIOCfg);

  FuncIOCfg* func_io_cfg_val = (FuncIOCfg*)&kv->value;
  memcpy(func_io_cfg_val, &func_io_cfg, sizeof(FuncIOCfg));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(FuncIOCfg);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status CommandImpl::SetLivoxLidarBlindSpot(uint32_t handle, uint32_t blind_spot, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //blind spot
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyBlindSpotSet);
  kv->length = sizeof(uint32_t);
  uint32_t* blind_spot_set = reinterpret_cast<uint32_t*>(&kv->value[0]);
  *blind_spot_set = blind_spot;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint32_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  
  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyWorkMode);
  kv->length = sizeof(uint8_t);
  uint8_t* val_work_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_work_mode = work_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}


livox_status CommandImpl::EnableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t); 

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01; //enable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::DisableLivoxLidarGlassHeat(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00; //disable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarGlassHeat(uint32_t handle, LivoxLidarGlassHeat glass_heat, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(glass_heat);
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::EnableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyImuDataEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::DisableLivoxLidarImuData(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyImuDataEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::EnableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFusaEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01; //enable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::DisableLivoxLidarFusaFunciont(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  //glass heat
  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyFusaEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00; //disable glass heat
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::StartForcedHeating(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return SendSingleControlCommand(handle, cb, client_data, kKeyForceHeatEn, 0x01/*enable forced heating*/);
}

livox_status CommandImpl::StopForcedHeating(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  return SendSingleControlCommand(handle, cb, client_data, kKeyForceHeatEn, 0x00/*disable forced heating*/);
}

livox_status CommandImpl::SetLidarFogNoiseFilterMode(uint32_t handle, LivoxFogNoiseFilterMode fog_filter_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetFogNoiseFilter);
  kv->length = sizeof(uint8_t);
  uint8_t* val_fog_filter_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_fog_filter_mode = fog_filter_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarITOCtrlMode(uint32_t handle, LivoxLidarItoCtrlMode ito_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];

  kv->key = static_cast<uint16_t>(kKeySetITOCtrl);
  kv->length = sizeof(uint8_t);
  uint8_t* val_ito_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_ito_mode = ito_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarPclFreqMod(uint32_t handle, LivoxLidarPclFreqMod pcl_freq_mod, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetPclFreqMod);
  kv->length = sizeof(uint8_t);
  uint8_t* val_pcl_freq_mod = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_pcl_freq_mod = pcl_freq_mod;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarTimeFilterMode(uint32_t handle, LivoxLidarTimeFilterMode time_filter_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetTimeFilterMode);
  kv->length = sizeof(uint8_t);
  uint8_t* val_time_filter_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_time_filter_mode = time_filter_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarImuRange(uint32_t handle, LivoxLidarImuOutRate imu_out_rate,
    LivoxLidarAccelRange accel_range, LivoxLidarGyroRange gyro_range,
    LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetImuRange);
  kv->length = sizeof(LivoxLidarImuRange);
  LivoxLidarImuRange* val_imu_range = reinterpret_cast<LivoxLidarImuRange*>(&kv->value[0]);
  val_imu_range->imu_out_rate = static_cast<uint8_t>(imu_out_rate);
  val_imu_range->accel_range  = static_cast<uint8_t>(accel_range);
  val_imu_range->gyro_range   = static_cast<uint8_t>(gyro_range);
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(LivoxLidarImuRange);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetNTPServerIp(uint32_t handle, const NTPServerIpInfo* server_ip, LivoxLidarAsyncControlCallback cb, void* client_data) {
  
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  if (!BuildRequest::BuildSetNTPserverIPInfoRequest(*server_ip, req_buff, req_len)) {
    return -1;
  }

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarEscMode(uint32_t handle, LivoxLidarEscMode esc_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  
  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetEscMode);
  kv->length = sizeof(uint8_t);
  uint8_t* val_esc_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_esc_mode = esc_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarFovMode(uint32_t handle, LivoxLidarFovMode fov_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  
  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetFovMode);
  kv->length = sizeof(uint8_t);
  uint8_t* val_fov_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_fov_mode = fov_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarEchoMode(uint32_t handle, LivoxLidarEchoMode echo_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  
  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySetEchoMode);
  kv->length = sizeof(uint8_t);
  uint8_t* val_echo_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_echo_mode = echo_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarLogParam(uint32_t handle, const LivoxLidarLogParam& log_param, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyLogParamSet);
  kv->length = sizeof(LivoxLidarLogParam);
  LivoxLidarLogParam* log_param_val = (LivoxLidarLogParam*)&kv->value;
  memcpy(log_param_val, &log_param, sizeof(LivoxLidarLogParam));

  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(LivoxLidarLogParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().LivoxLidarRequestReset(handle, cb, client_data);
}

livox_status CommandImpl::SetLivoxLidarDebugPointCloud(uint32_t handle, bool enable,
                                                       LivoxLidarLoggerCallback cb, void* client_data) {
  DebugPointCloudManager::GetInstance().Enable(enable);

  LivoxLidarDebugPointCloudRequest req_buff {};
  req_buff.enable    = enable ? 1 : 0;
  req_buff.host_port = kHostDebugPointCloudPort; // 44332
  req_buff.bandwidth = 0;  // units Mbps
  sscanf(GeneralCommandHandler::GetInstance().GetLidarCfg(handle).host_net_info.host_ip.c_str(),
                             "%" SCNu8 ".%" SCNu8 ".%" SCNu8 ".%" SCNu8, &req_buff.host_ip_addr[0]
                                                                       , &req_buff.host_ip_addr[1]
                                                                       , &req_buff.host_ip_addr[2]
                                                                       , &req_buff.host_ip_addr[3]);
  return GeneralCommandHandler::GetInstance().SendLoggerCommand(handle,
                    kCommandIDLidarDebugPointCloudControl,
                    reinterpret_cast<uint8_t*>(&req_buff),
                    uint16_t(sizeof(LivoxLidarDebugPointCloudRequest)),
                    MakeCommandCallback<LivoxLidarLoggerResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarRmcSyncTime(uint32_t handle, const char* rmc, uint16_t rmc_length,
                                                       LivoxLidarRmcSyncTimeCallBack cb, void* client_data) {
  LivoxLidarRmcSyncTimeRequest req_buff {};
  req_buff.type = LivoxLidarRmcSyncTimeRequest::SyncTimeType::kRmcSyncTime;
  req_buff.ns   = ParseGPRMC(std::string(rmc, rmc_length));

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarSetPPSSync,
                    reinterpret_cast<uint8_t*>(&req_buff),
                    uint16_t(sizeof(LivoxLidarRmcSyncTimeRequest)),
                    MakeCommandCallback<LivoxLidarRmcSyncTimeResponse>(cb, client_data));
}

livox_status CommandImpl::SetLivoxLidarWorkModeAfterBoot(uint32_t handle, LivoxLidarWorkModeAfterBoot work_mode, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  
  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyWorkModeAfterBoot);
  kv->length = sizeof(uint8_t);
  uint8_t* val_work_mode = reinterpret_cast<uint8_t*>(&kv->value[0]);
  *val_work_mode = work_mode;
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(uint8_t);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

// Upgrade
livox_status CommandImpl::LivoxLidarStartUpgrade(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarStartUpgradeCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SendCommand(handle, 
                    kCommandIDGeneralRequestUpgrade,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarStartUpgradeResponse>(cb, client_data));
}

livox_status CommandImpl::LivoxLidarXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarXferFirmwareCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SendCommand(handle, 
                    kCommandIDGeneralXferFirmware,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarXferFirmwareResponse>(cb, client_data));
}

livox_status CommandImpl::LivoxLidarCompleteXferFirmware(uint32_t handle, uint8_t *data, uint16_t length,
    LivoxLidarCompleteXferFirmwareCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDGeneralCompleteXferFirmware,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarCompleteXferFirmwareResponse>(cb, client_data));
}

livox_status CommandImpl::LivoxLidarGetUpgradeProgress(uint32_t handle, uint8_t *data,
    uint16_t length, LivoxLidarGetUpgradeProgressCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDGeneralRequestUpgradeProgress,
                    data, 
                    length,
                    MakeCommandCallback<LivoxLidarGetUpgradeProgressResponse>(cb, client_data));
}

livox_status CommandImpl::LivoxLidarRequestFirmwareInfo(uint32_t handle,
    LivoxLidarRequestFirmwareInfoCallback cb, void* client_data) {
  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDGeneralRequestFirmwareInfo, 
                    nullptr, 
                    0,
                    MakeCommandCallback<LivoxLidarRequestFirmwareInfoResponse>(cb, client_data));
}

livox_status CommandImpl::LivoxLidarRequestReboot(uint32_t handle, LivoxLidarRebootCallback cb,
    void* client_data) {
  LivoxLidarRebootRequest reboot_request;
  reboot_request.timeout = 100;
  return GeneralCommandHandler::GetInstance().SendCommand(handle,
      kCommandIDLidarRebootDevice, (uint8_t *)&reboot_request,
      sizeof(reboot_request), MakeCommandCallback<LivoxLidarRebootResponse>(cb,
      client_data));
}

livox_status CommandImpl::SendSingleControlCommand(uint32_t handle, 
                                             LivoxLidarAsyncControlCallback cb, 
                                             void* client_data,
                                             uint16_t command_key,
                                             uint8_t value) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t key_num = 1;
  uint16_t req_len = 0;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = command_key;
  kv->length = sizeof(uint8_t);
  kv->value[0] = value;
  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

}  // namespace livox
} // namespace lidar

