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

#include <vector>
#include <set>
#include "command_impl.h"
#include "data_handler/data_handler.h"
#include "command_handler/command_handler.h"
#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"
#include "base/logging.h"

using std::pair;
using namespace livox::vehicle;

static bool is_initialized = false;

bool LivoxVehicleInit(const char* net_if) {
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
  InitLogger();
  is_initialized = true;
  if (!data_handler().Init()) {
    return false;
  }
  if (!command_handler().Init(std::string(net_if))) {
    return false;
  }
  return true;
}

void LivoxVehicleUninit() {
  if (!is_initialized) {
    return;
  }
#ifdef WIN32
    WSACleanup();
#endif // WIN32
  UninitLogger();
  is_initialized = false;
  command_handler().Uninit();
  data_handler().Uninit();
}

void RegisterLidarInfo(const LidarRegisterInfo* lidar_info, int number) {
  std::vector<LidarRegisterInfo> lidars_info;
  for (int i = 0; i < number; i++) {
    lidars_info.push_back(lidar_info[i]);
  }
  command_handler().AddDevice(lidars_info);
  return;
}

void SetVehicleLidarPointCloudCallback(OnVehicleLidarPointCloudCallback cb, void* client_data) {
  data_handler().AddPointCloudCallback(cb, client_data);
}

void SetVehicleLidarImuDataCallback(OnVehicleLidarImuDataCallback cb, void* client_data) {
  data_handler().AddImuDataCallback(cb, client_data);
}

uint16_t VehicleLidarAddPointCloudObserver(OnVehicleLidarPointCloudCallback cb, void* client_data) {
  return data_handler().AddPointCloudObserver(cb, client_data);
}

void VehicleLidarRemovePointCloudObserver(uint16_t listen_id) {
  data_handler().RemovePointCloudObserver(listen_id);
}

void SetVehicleLidarInfoChangeCallback(OnVehicleLidarInfoChangeCallback cb, void* client_data) {
  command_handler().SetLidarInfoChangeCallback(cb, client_data);
  return;
}

void SetExceptionDetailCallback(OnExceptionDetailCallback cb, void* client_data) {
  command_handler().SetExceptionDetailCallback(cb, client_data);
  return;
}

void SetExceptionInfoCallback(OnExceptionInfoCallback cb, void* client_data) {
  command_handler().SetExceptionInfoCallback(cb, client_data);
  return;
}

void SetVehicleLidarLogCallback(OnVehicleLidarLogInfoCallback cb, void* client_data) {
  command_handler().SetLidarLogCallback(cb, client_data);
  return;
}

void SetLidarSafetyInfoCallback(OnVehicleLidarSafetyInfoCallback cb, void* client_data) {
  command_handler().SetLidarSafetyInfoCallback(cb, client_data);
  return;
}

livox_vehicle_status VehicleLidarSyncControl(uint8_t slot, SyncControlInfo *info,
    OnSyncControlCallback cb, void* client_data) {

  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //work mode
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyWorkMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = info->work_mode;
  req_len += sizeof(VehicleKeyValueParam);

  //vehicle speed
  kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeySpeed);
  kv->length = sizeof(int32_t);
  int32_t* vehicle_speed = (int32_t*)&kv->value[0];
  *vehicle_speed = info->vehicle_speed;
  req_len += sizeof(VehicleKeyValueParam) - 1 + sizeof(int32_t);

  //ev temp
  kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyEnvironmentTemp);
  kv->length = sizeof(int32_t);
  int32_t* ev_temp = (int32_t*)&kv->value[0];
  *ev_temp = info->ev_temp;
  req_len += sizeof(VehicleKeyValueParam) - 1 + sizeof(int32_t);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarSetSlotNumber(uint8_t slot, uint8_t dst_slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //slot
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeySlotNum);
  kv->length = sizeof(uint8_t);
  kv->value[0] = dst_slot;
  req_len += sizeof(VehicleKeyValueParam) - 1 + sizeof(uint8_t);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarSetIp(uint8_t slot, LidarIpConfig ip_config, OnCommonCallback cb, void* client_data) {
  LidarPreConfigParam req;
  req.gw_addr = ip_config.gw_addr;
  req.ip_addr = ip_config.ip_addr;
  req.net_mask = ip_config.net_mask;
  req.slot_id = slot;

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarPreconfig,
                                      (uint8_t*)&req,
                                      sizeof(LidarPreConfigParam),
                                      MakeCommandCallback<uint8_t>(cb, client_data));
}

livox_vehicle_status VehicleLidarEnableHighResolutionPointType(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //point type
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyPointType);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(kHighResolutionPointData); //32bit point type
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarDisableHighResolutionPointType(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //point type
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyPointType);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(kLowResolutionPointData); //16bit point type
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarEnableGlassHeat(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //glass heat
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01; //enable glass heat
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarDisableGlassHeat(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //glass heat
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00; //disable glass heat
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

// just for industrial hap lidar
livox_vehicle_status VehicleLidarSetGlassHeat(uint8_t slot, VehicleGlassHeat glass_heat, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //glass heat
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyGlassHeat);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(glass_heat);
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarSetScanPattern(uint8_t slot, VehicleScanPattern scan_pattern, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //scan pattern
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyScanPattern);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(scan_pattern);
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarSetBlindSpot(uint8_t slot, uint32_t blind_spot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //blind spot
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyBlindSpotSet);
  kv->length = sizeof(uint32_t);
  uint32_t* blind_spot_set = reinterpret_cast<uint32_t*>(&kv->value[0]);
  *blind_spot_set = blind_spot;
  req_len += sizeof(VehicleKeyValueParam) - 1 + sizeof(uint32_t);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarQueryInternalInfo(uint8_t slot, OnQueryInternalInfoCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  std::set<VehicleDeviceParamKeyName> key_sets {
    kVehicleKeyWorkMode,
    kVehicleKeySlotNum,
    kVehicleKeyAddr,
    kVehicleKeyDeviceVer,
    kVehicleKeyProductInfo,
    kVehicleKeyGlassHeat,
    kVehicleKeyScanPattern,
    kVehicleKeyBlindSpotSet,
    kVehicleKeyGlassTemp,
    kVehicleKeyPointType,
    kVehicleKeyDualEmitEnable,
    //kKeySafetyInfo,
  };
  req_len++;
  for (const auto &key : key_sets) {
    req_buff[0]++;
    VehicleKeyListParam* kList = (VehicleKeyListParam*)&req_buff[req_len];
    kList->key = static_cast<uint16_t>(key);
    req_len += sizeof(uint16_t);
  }

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarGetInternalInfo,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarDiagInternalInfoResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarRegisterFaultInfo(uint8_t slot, LidarFaultInfo *info, CommonCommandCallback cb, void* client_data) {
  return command_handler().SendCommand(slot,
                                      kCommandIDLidarRegisterFaultInfo,
                                      (uint8_t*)info,
                                      sizeof(LidarFaultInfo),
                                      MakeCommandCallback<uint8_t>(cb, client_data));
}

livox_vehicle_status LidarDualEmitEnable(uint8_t slot, bool enable, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //dual emit enable
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyDualEmitEnable);
  kv->length = sizeof(uint8_t);
  if (enable) {
    kv->value[0] = 0x01;
  } else {
    kv->value[0] = 0x00;
  }
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

// just for industrial hap lidar
livox_vehicle_status VehicleLidarFusaFunciontEnable(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //dual emit enable
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyFusaFunciont);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x00;
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

// just for industrial hap lidar
livox_vehicle_status VehicleLidarFusaFunciontDisable(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //dual emit enable
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyFusaFunciont);
  kv->length = sizeof(uint8_t);
  kv->value[0] = 0x01;
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

// just for industrial hap lidar
livox_vehicle_status VehicleLidarPointCloudMulticastIpEnable(uint8_t slot, SetPointCloudMulticastIpRequest* req, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //dual emit enable
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyPointCloudMulticastIp);
  kv->length = sizeof(SetPointCloudMulticastIpRequest);
  SetPointCloudMulticastIpRequest* multicast_ip = (SetPointCloudMulticastIpRequest *)&kv->value[0];
  memcpy(multicast_ip->byte, req->byte, sizeof(SetPointCloudMulticastIpRequest));

  req_len += sizeof(VehicleKeyValueParam) - sizeof(uint8_t) + sizeof(SetPointCloudMulticastIpRequest);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

// just for industrial hap lidar
livox_vehicle_status VehicleLidarPointCloudMulticastIpDisable(uint8_t slot, OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //dual emit enable
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyPointCloudMulticastIp);
  kv->length = sizeof(SetPointCloudMulticastIpRequest);
  SetPointCloudMulticastIpRequest* multicast_ip = (SetPointCloudMulticastIpRequest *)&kv->value[0];
  memset(multicast_ip->byte, 0, sizeof(SetPointCloudMulticastIpRequest));

  req_len += sizeof(VehicleKeyValueParam) - sizeof(uint8_t) + sizeof(SetPointCloudMulticastIpRequest);
  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarSetFrameRate(uint8_t slot, LivoxVehiclePointFrameRate rate, 
  OnSyncControlCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;

  //frame rate set
  VehicleKeyValueParam * kv = (VehicleKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kVehicleKeyFrameRate);
  kv->length = sizeof(uint8_t);
  kv->value[0] = static_cast<uint8_t>(rate);
  req_len += sizeof(VehicleKeyValueParam);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarWorkModeControl,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarSyncControlResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarGetFrameRate(uint8_t slot, OnQueryInternalInfoCallback cb, void* client_data) {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  req_len++;
  req_buff[0]++;
  VehicleKeyListParam* kList = (VehicleKeyListParam*)&req_buff[req_len];
  kList->key = static_cast<uint16_t>(kVehicleKeyFrameRate);
  req_len += sizeof(uint16_t);

  return command_handler().SendCommand(slot,
                                      kCommandIDLidarGetInternalInfo,
                                      req_buff,
                                      req_len,
                                      MakeCommandCallback<LidarDiagInternalInfoResponse>(cb, client_data));  
}

// Upgrade
livox_vehicle_status VehicleLidarStartUpgrade(uint8_t slot, uint8_t *data, uint16_t length,
    VehicleStartUpgradeCallback cb, void* client_data) {
  return command_handler().SendCommand(slot, 
                                      kCommandIDGeneralRequestUpgrade,
                                      data, 
                                      length,
                                      MakeCommandCallback<VehicleStartUpgradeResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarXferFirmware(uint8_t slot, uint8_t *data, uint16_t length,
    VehicleXferFirmwareCallback cb, void* client_data) {
  return command_handler().SendCommand(slot, 
                                      kCommandIDGeneralXferFirmware,
                                      data, 
                                      length,
                                      MakeCommandCallback<VehicleXferFirmwareResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarCompleteXferFirmware(uint8_t slot, uint8_t *data, uint16_t length,
    VehicleCompleteXferFirmwareCallback cb, void* client_data) {
  return command_handler().SendCommand(slot,
                                      kCommandIDGeneralCompleteXferFirmware,
                                      data, 
                                      length,
                                      MakeCommandCallback<VehicleCompleteXferFirmwareResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarGetUpgradeProgress(uint8_t slot, uint8_t *data,
    uint16_t length, VehicleGetUpgradeProgressCallback cb, void* client_data) {
  return command_handler().SendCommand(slot,
                                      kCommandIDGeneralRequestUpgradeProgress,
                                      data, 
                                      length,
                                      MakeCommandCallback<VehicleGetUpgradeProgressResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarRequestFirmwareInfo(uint8_t slot,
    VehicleRequestFirmwareInfoCallback cb, void* client_data) {
  return command_handler().SendCommand(slot,
                                      kCommandIDGeneralRequestFirmwareInfo, 
                                      nullptr, 
                                      0,
                                      MakeCommandCallback<VehicleRequestFirmwareInfoResponse>(cb, client_data));
}

livox_vehicle_status VehicleLidarRequestReboot(uint8_t slot, VehicleRebootCallback cb,
    void* client_data) {
  VehicleRebootRequest reboot_request;
  reboot_request.timeout = 100;
  return command_handler().SendCommand(slot,
      kCommandIDGeneralRebootDevice, (uint8_t *)&reboot_request,
      sizeof(reboot_request), MakeCommandCallback<VehicleRebootResponse>(cb,
      client_data));
}
