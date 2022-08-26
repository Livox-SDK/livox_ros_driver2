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

#include "command_impl.h"

#include "livox_lidar_def.h"
#include "general_command_handler.h"

#include "base/logging.h"
#include "comm/protocol.h"
#include "comm/generate_seq.h"

#include "build_request.h"

namespace livox {
namespace lidar {

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
  kv->key = static_cast<uint16_t>(kKeyDualEmitEnable);
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
  kv->key = static_cast<uint16_t>(kKeyPointSendEnable);
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
  kv->key = static_cast<uint16_t>(kKeyPointSendEnable);
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

livox_status CommandImpl::SetLivoxLidarRoiCfg0(uint32_t handle, const RoiCfg& roi_cfg0, LivoxLidarAsyncControlCallback cb,
    void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyRoiCfg0);
  kv->length = sizeof(RoiCfg);

  RoiCfg* roi_cfg = (RoiCfg*)&kv->value;
  memcpy(roi_cfg, &roi_cfg0, sizeof(RoiCfg));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(RoiCfg);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
  
}

livox_status CommandImpl::SetLivoxLidarRoiCfg1(uint32_t handle, const RoiCfg& roi_cfg1, LivoxLidarAsyncControlCallback cb,
    void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyRoiCfg1);
  kv->length = sizeof(RoiCfg);

  RoiCfg* roi_cfg = (RoiCfg*)&kv->value;
  memcpy(roi_cfg, &roi_cfg1, sizeof(RoiCfg));
  req_len += sizeof(LivoxLidarKeyValueParam) - 1 + sizeof(RoiCfg);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                     kCommandIDLidarWorkModeControl,
                     req_buff,
                     req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::EnableLivoxLidarRoi(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyRoiEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01;

  req_len += sizeof(LivoxLidarKeyValueParam);

  return GeneralCommandHandler::GetInstance().SendCommand(handle,
                    kCommandIDLidarWorkModeControl,
                    req_buff,
                    req_len,
                    MakeCommandCallback<LivoxLidarAsyncControlResponse>(cb, client_data));
}

livox_status CommandImpl::DisableLivoxLidarRoi(uint32_t handle, LivoxLidarAsyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  uint16_t key_num = 1;
  memcpy(&req_buff[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyRoiEn);
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
  kv->key = static_cast<uint16_t>(kKeyFuncIOCfg);
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

}  // namespace livox
} // namespace lidar

