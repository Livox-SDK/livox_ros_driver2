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

#ifndef LIVOX_SDK_VEHICLE_H_
#define LIVOX_SDK_VEHICLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "livox_def_vehicle.h"

void GetLivoxVehicleSdkVersion(LivoxVehicleSdkVersion *version);

bool LivoxPreConfigInit(const char* net_if, LidarPreConfigParam* param);
void LivoxPreConfigUninit();

typedef void (*OnPreConfigCallback)(livox_vehicle_status status, const char* broadcast_code, void* client_data);

void SetLidarPreConfigParamCallback(OnPreConfigCallback cb, void* client_data);

bool LivoxVehicleInit(const char* net_if);

void LivoxVehicleUninit();

void RegisterLidarInfo(const LidarRegisterInfo* lidar_info, int number);

typedef void (*OnVehicleLidarPointCloudCallback)(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data);
void SetVehicleLidarPointCloudCallback(OnVehicleLidarPointCloudCallback cb,void* client_data);

typedef void (*OnVehicleLidarImuDataCallback)(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data);
void SetVehicleLidarImuDataCallback(OnVehicleLidarImuDataCallback cb, void* client_data);

uint16_t VehicleLidarAddPointCloudObserver(OnVehicleLidarPointCloudCallback cb, void* client_data);

void VehicleLidarRemovePointCloudObserver(uint16_t listen_id);

typedef void (*OnExceptionDetailCallback)(uint8_t slot,
  LivoxDetailExceptionInfo* info, void* client_data);

void SetExceptionDetailCallback(OnExceptionDetailCallback cb, void* client_data);

typedef void (*OnExceptionInfoCallback)(uint8_t slot,
  LidarStatusCode* info, void* client_data);

void SetExceptionInfoCallback(OnExceptionInfoCallback cb, void* client_data);

//typedef void (*OnSyncControlCallback)(livox_status status,
//                                    uint8_t slot,
//                                    LidarSyncControlResponse *response,
//                                    void *client_data);
typedef void (*OnSyncControlCallback)(livox_vehicle_status status,
                                    uint8_t slot,
                                    LidarSyncControlResponse *response,
                                    void *client_data);

//livox__status LidarSyncControl(uint8_t slot, SyncControlInfo *info, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarSyncControl(uint8_t slot, SyncControlInfo *info, OnSyncControlCallback cb, void* client_data);

//livox__status LidarSetSlotNumber(uint8_t slot, uint8_t dst_slot, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarSetSlotNumber(uint8_t slot, uint8_t dst_slot, OnSyncControlCallback cb, void* client_data);

//typedef void (*OnCommonCallback)(livox_status status,
//                                    uint8_t slot,
//                                    uint8_t ret_code,
//                                    void *client_data);
typedef void (*OnCommonCallback)(livox_vehicle_status status,
                                    uint8_t slot,
                                    uint8_t ret_code,
                                    void *client_data);

//livox_status LidarSetIp(uint8_t slot, LidarIpConfig ip_config, OnCommonCallback cb, void* client_data);
livox_vehicle_status VehicleLidarSetIp(uint8_t slot, LidarIpConfig ip_config, OnCommonCallback cb, void* client_data);

//livox_status LidarEnableHighResolutionPointType(uint8_t slot, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarEnableHighResolutionPointType(uint8_t slot, OnSyncControlCallback cb, void* client_data);

//livox_status LidarDisableHighResolutionPointType(uint8_t slot, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarDisableHighResolutionPointType(uint8_t slot, OnSyncControlCallback cb, void* client_data);

//typedef void (*OnLidarInfoChangeCallback)(DeviceInfo *info, void *client_data);
typedef void (*OnVehicleLidarInfoChangeCallback)(VehicleDeviceInfo *info, void *client_data);

//void SetLidarInfoChangeCallback(OnLidarInfoChangeCallback cb, void* client_data);
void SetVehicleLidarInfoChangeCallback(OnVehicleLidarInfoChangeCallback cb, void* client_data);

//typedef void (*OnQueryInternalInfoCallback)(livox_status status, uint8_t slot, LidarDiagInternalInfoResponse* response,
//                                      void* client_data);
typedef void (*OnQueryInternalInfoCallback)(livox_vehicle_status status, uint8_t slot, LidarDiagInternalInfoResponse* response,
                                      void* client_data);

//livox_status LidarQueryInternalInfo(uint8_t slot, OnQueryInternalInfoCallback cb, void* client_data);
livox_vehicle_status VehicleLidarQueryInternalInfo(uint8_t slot, OnQueryInternalInfoCallback cb, void* client_data);


//typedef void (*OnLidarLogInfoCallback)(uint8_t slot, LidarLogInfo* response, void* client_data);
typedef void (*OnVehicleLidarLogInfoCallback)(uint8_t slot, LidarLogInfo* response,
                                      void* client_data);

//void SetLidarLogCallback(OnLidarLogInfoCallback cb, void* client_data);
void SetVehicleLidarLogCallback(OnVehicleLidarLogInfoCallback cb, void* client_data);

//livox_statusLidarEnableGlassHeat(uint8_t slot, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarEnableGlassHeat(uint8_t slot, OnSyncControlCallback cb, void* client_data);

livox_vehicle_status VehicleLidarSetGlassHeat(uint8_t slot, VehicleGlassHeat glass_heat, OnSyncControlCallback cb,
		void* client_data);

//livox_status LidarDisableGlassHeat(uint8_t slot, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarDisableGlassHeat(uint8_t slot, OnSyncControlCallback cb, void* client_data);

//livox_status LidarSetScanPattern(uint8_t slot, VehicleScanPattern scan_pattern, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarSetScanPattern(uint8_t slot, VehicleScanPattern scan_pattern,
		OnSyncControlCallback cb, void* client_data);

//livox_status LidarSetBlindSpot(uint8_t slot, uint32_t blind_spot, OnSyncControlCallback cb, void* client_data);
livox_vehicle_status VehicleLidarSetBlindSpot(uint8_t slot, uint32_t blind_spot, OnSyncControlCallback cb, void* client_data);

//typedef void (*CommonCommandCallback)(livox_status status, uint8_t slot, uint8_t response, void* client_data);
typedef void (*CommonCommandCallback)(livox_vehicle_status status, uint8_t slot, uint8_t response, void* client_data);

//livox_status LidarRegisterFaultInfo(uint8_t slot, LidarFaultInfo* info, CommonCommandCallback cb, void* client_data);
livox_vehicle_status VehicleLidarRegisterFaultInfo(uint8_t slot, LidarFaultInfo* info, CommonCommandCallback cb, void* client_data);

//typedef void (*OnLidarSafetyInfoCallback)(uint8_t slot, LidarSafetyInfo* response, void* client_data);
typedef void (*OnVehicleLidarSafetyInfoCallback)(uint8_t slot, LidarSafetyInfo* response, void* client_data);

//void SetLidarSafetyInfoCallback(OnLidarSafetyInfoCallback cb, void* client_data);
void SetVehicleLidarSafetyInfoCallback(OnVehicleLidarSafetyInfoCallback cb, void* client_data);

livox_vehicle_status LidarDualEmitEnable(uint8_t slot, bool enable, OnSyncControlCallback cb, void* client_data);

livox_vehicle_status VehicleLidarFusaFunciontEnable(uint8_t slot, OnSyncControlCallback cb, void* client_data);

livox_vehicle_status VehicleLidarFusaFunciontDisable(uint8_t slot, OnSyncControlCallback cb, void* client_data);

livox_vehicle_status VehicleLidarPointCloudMulticastIpEnable(uint8_t slot, SetPointCloudMulticastIpRequest* req,
		OnSyncControlCallback cb, void* client_data);

livox_vehicle_status VehicleLidarPointCloudMulticastIpDisable(uint8_t slot, OnSyncControlCallback cb, void* client_data);



livox_vehicle_status VehicleLidarSetFrameRate(uint8_t slot, LivoxVehiclePointFrameRate rate,
		OnSyncControlCallback cb, void* client_data);

livox_vehicle_status VehicleLidarGetFrameRate(uint8_t slot, OnQueryInternalInfoCallback cb, void* client_data);

/*******Upgrade Module***********/
typedef void (*VehicleRebootCallback)(livox_vehicle_status status, uint8_t slot,
    VehicleRebootResponse* response, void* client_data);
    
livox_vehicle_status VehicleLidarRequestReboot(uint8_t slot, VehicleRebootCallback cb,
    void* client_data);
/**
 * Upgrade related command
 */
typedef void (*VehicleStartUpgradeCallback)(livox_vehicle_status status, uint8_t slot,
    VehicleStartUpgradeResponse* response, void* client_data);
livox_vehicle_status VehicleLidarStartUpgrade(uint8_t slot,  uint8_t *data, uint16_t length,
    VehicleStartUpgradeCallback cb, void* client_data);

typedef void (*VehicleXferFirmwareCallback)(livox_vehicle_status status, uint8_t slot,
    VehicleXferFirmwareResponse* response, void* client_data);
livox_vehicle_status VehicleLidarXferFirmware(uint8_t slot, uint8_t *data, uint16_t length,
    VehicleXferFirmwareCallback cb, void* client_data);

typedef void (*VehicleCompleteXferFirmwareCallback)(livox_vehicle_status status, uint8_t slot,
    VehicleCompleteXferFirmwareResponse* response, void* client_data);
livox_vehicle_status VehicleLidarCompleteXferFirmware(uint8_t slot, uint8_t *data,
    uint16_t length, VehicleCompleteXferFirmwareCallback cb, void* client_data);

typedef void (*VehicleGetUpgradeProgressCallback)(livox_vehicle_status status,
    uint8_t slot, VehicleGetUpgradeProgressResponse* response, void* client_data);
livox_vehicle_status VehicleLidarGetUpgradeProgress(uint8_t slot, uint8_t *data,
    uint16_t length, VehicleGetUpgradeProgressCallback cb, void* client_data);

typedef void (*VehicleRequestFirmwareInfoCallback)(livox_vehicle_status status,
    uint8_t slot, VehicleRequestFirmwareInfoResponse* response, void* client_data);
livox_vehicle_status VehicleLidarRequestFirmwareInfo(uint8_t slot,
    VehicleRequestFirmwareInfoCallback cb, void* client_data);

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_SDK_VEHICLE_H_
