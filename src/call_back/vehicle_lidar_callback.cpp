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
#include "vehicle_lidar_callback.h"

#include <string>
#include <thread>

namespace livox_ros {

extern LdsLidar *g_lds_ldiar;

void VehicleLidarCallback::LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data) {
  LidarDevice* lidar_device = GetLidarDevice(info->slot, client_data);
  if (lidar_device == nullptr) {
    printf("Lidar info change call back failed, can not get lidar device.\n");
    return;
  }

  for (int i = 0; i < 16; ++i) {
    lidar_device->info.broadcast_code[i] = info->broadcast_code[i];
  }

  UserVehicleConfig* vehicle_config = &(lidar_device->vehicle_config);

  if (vehicle_config->data_type == 1) {
    VehicleLidarEnableHighResolutionPointType(info->slot, EnableVehicleLidarHighResolutionPointType, g_lds_ldiar);
    vehicle_config->set_bits |= kConfigDataType;
  } else if (vehicle_config->data_type == 2) {
    VehicleLidarDisableHighResolutionPointType(info->slot, DisableVehicleLidarHighResolutionPointType, g_lds_ldiar);
    vehicle_config->set_bits |= kConfigDataType;
  }

  if (vehicle_config->scan_pattern == kScanPatternNoneRepetive) {
    VehicleLidarSetScanPattern(info->slot, kScanPatternNoneRepetive, SetVehicleLidarScanPattern, g_lds_ldiar);
    vehicle_config->set_bits |= kConfigScanPattern;
  } else if (vehicle_config->scan_pattern == kScanPatternRepetive) {
    VehicleLidarSetScanPattern(info->slot, kScanPatternRepetive, SetVehicleLidarScanPattern, g_lds_ldiar);
    vehicle_config->set_bits |= kConfigScanPattern;
  }

  LidarDualEmitEnable(vehicle_config->slot, vehicle_config->dual_emit_enable, EnableVehicleLidarDualEmit, g_lds_ldiar);
  vehicle_config->set_bits |= kConfigDualEmit;

  VehicleLidarSetBlindSpot(vehicle_config->slot, vehicle_config->blind_spot_set, SetVehicleLidarBlindSpot, g_lds_ldiar);
  vehicle_config->set_bits |= kConfigBlindSpot;
}

void VehicleLidarCallback::EnableVehicleLidarHighResolutionPointType(livox_vehicle_status status, uint8_t slot,
        LidarSyncControlResponse *rsp, void *client_data) {
  if (status == kVehicleStatusSuccess) {
    printf("Vehicle lidar enable high resolution point type status: %d, slot: %d, ret_code: %d error_key: %d\n",
      status, slot, rsp->ret_code, rsp->error_key);

    LidarDevice* p_lidar_device =  GetLidarDevice(slot, client_data);
    if (p_lidar_device == nullptr) {
      printf("Enable vehicle lidar high resolution point type failed, can not get lidar device.\n");
    }

    p_lidar_device->vehicle_config.set_bits &= ~((uint32_t)(kConfigDataType));
    if (!p_lidar_device->vehicle_config.set_bits) {
      p_lidar_device->connect_state = kConnectStateSampling;
    }
  } else if (status == kVehicleStatusTimeout) {
    printf("Vehicle lidar enable high resolution point type  timeout\n");
    VehicleLidarEnableHighResolutionPointType(slot, EnableVehicleLidarHighResolutionPointType, g_lds_ldiar);
  }
}

void VehicleLidarCallback::DisableVehicleLidarHighResolutionPointType(livox_vehicle_status status, uint8_t slot,
        LidarSyncControlResponse *rsp, void *client_data) {
  if (status == kVehicleStatusSuccess) {
    printf("Vehicle lidar disable high resolution point type status: %d, slot: %d, ret_code: %d error_key: %d\n",
      status, slot, rsp->ret_code, rsp->error_key);

    LidarDevice* p_lidar_device =  GetLidarDevice(slot, client_data);
    if (p_lidar_device == nullptr) {
      printf("Enable vehicle lidar high resolution point type failed, can not get lidar device.\n");
    }

    p_lidar_device->vehicle_config.set_bits &= ~((uint32_t)(kConfigDataType));
    if (!p_lidar_device->vehicle_config.set_bits) {
      p_lidar_device->connect_state = kConnectStateSampling;
    }
  } else if (status == kVehicleStatusTimeout) {
    printf("Vehicle lidar disable high resolution point type  timeout\n");
    VehicleLidarDisableHighResolutionPointType(slot, DisableVehicleLidarHighResolutionPointType, g_lds_ldiar);
  }
}

void VehicleLidarCallback::SetVehicleLidarScanPattern(livox_vehicle_status status, uint8_t slot,
        LidarSyncControlResponse *rsp, void *client_data) {

  LidarDevice* p_lidar_device =  GetLidarDevice(slot, client_data);
  if (p_lidar_device == nullptr) {
    printf("Enable vehicle lidar high resolution point type failed, can not get lidar device.\n");
  }

  if (status == kVehicleStatusSuccess) {
    printf("Vehicle lidar set scan pattern status: %d, slot: %d, ret_code: %d error_key: %d\n",
      status, slot, rsp->ret_code, rsp->error_key);
    p_lidar_device->vehicle_config.set_bits &= ~((uint32_t)(kConfigScanPattern));
    if (!p_lidar_device->vehicle_config.set_bits) {
      p_lidar_device->connect_state = kConnectStateSampling;
    }
  } else if (status == kVehicleStatusTimeout) {
    printf("Vehicle lidar set scan pattern timeout\n");

    if (p_lidar_device->vehicle_config.scan_pattern == kScanPatternNoneRepetive) {
      VehicleLidarSetScanPattern(slot, kScanPatternNoneRepetive, SetVehicleLidarScanPattern, g_lds_ldiar);
    } else if (p_lidar_device->vehicle_config.scan_pattern == kScanPatternRepetive) {
      VehicleLidarSetScanPattern(slot, kScanPatternRepetive, SetVehicleLidarScanPattern, g_lds_ldiar);
    }
  }
}

void VehicleLidarCallback::EnableVehicleLidarDualEmit(livox_vehicle_status status, uint8_t slot,
        LidarSyncControlResponse *rsp, void *client_data) {
  LidarDevice* p_lidar_device =  GetLidarDevice(slot, client_data);
  if (p_lidar_device == nullptr) {
    printf("Enable vehicle lidar high resolution point type failed, can not get lidar device.\n");
  }

  if (status == kVehicleStatusSuccess) {
    printf("Vehicle lidar enable dual emit status: %d, slot: %d, ret_code: %d error_key: %d\n", status, slot, rsp->ret_code, rsp->error_key);
    p_lidar_device->vehicle_config.set_bits &= ~((uint32_t)(kConfigDualEmit));
    if (!p_lidar_device->vehicle_config.set_bits) {
      p_lidar_device->connect_state = kConnectStateSampling;
    }
  } else if (status == kVehicleStatusTimeout) {
    printf("Vehicle lidar enable dual emit timeout\n");
    LidarDualEmitEnable(slot, p_lidar_device->vehicle_config.dual_emit_enable, EnableVehicleLidarDualEmit, g_lds_ldiar);
  }
}

void VehicleLidarCallback::SetVehicleLidarBlindSpot(livox_vehicle_status status, uint8_t slot,
        LidarSyncControlResponse *rsp, void *client_data) {
  LidarDevice* p_lidar_device =  GetLidarDevice(slot, client_data);
  if (p_lidar_device == nullptr) {
    printf("Enable vehicle lidar high resolution point type failed, can not get lidar device.\n");
  }

  if (status == kVehicleStatusSuccess) {
    printf("Vehicle lidar set blind spot status: %d, slot: %d, ret_code: %d error_key: %d\n", status, slot, rsp->ret_code, rsp->error_key);
    p_lidar_device->vehicle_config.set_bits &= ~((uint32_t)(kConfigBlindSpot));
    if (!p_lidar_device->vehicle_config.set_bits) {
      p_lidar_device->connect_state = kConnectStateSampling;
    }
  } else if (status == kVehicleStatusTimeout) {
    printf("Vehicle lidar set blind spot timeout\n");
    VehicleLidarSetBlindSpot(slot, p_lidar_device->vehicle_config.blind_spot_set, SetVehicleLidarBlindSpot, g_lds_ldiar);
  }
}

// 1Hz
void VehicleLidarCallback::LidarWorkModeControl() {
  printf("lidar work mode control\n");
  for (uint8_t index = 0; index < kMaxLidarCount; ++index) {
    const LidarDevice* p_lidar = &(g_lds_ldiar->lidars_[index]);
    if (p_lidar->lidar_type == kVehicleLidarType) {
      SyncControlInfo info;
      info.slot = p_lidar->vehicle_config.slot;
      info.work_mode = 0x01;
      info.vehicle_speed = 11;
      info.ev_temp = 30;
      VehicleLidarSyncControl(info.slot, &info, LidarWorkModeControlCallback, g_lds_ldiar);
    }
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void VehicleLidarCallback::LidarWorkModeControlCallback(livox_vehicle_status status, uint8_t slot,
  LidarSyncControlResponse *rsp, void* client_data) {
  if (status == kVehicleStatusSuccess) {
    printf("lidar work mode control status: %d, slot: %d, ret_code: %d error_key: %d\n",
      status, slot, rsp->ret_code, rsp->error_key);
  } else if (status == kVehicleStatusTimeout) {
    printf("lidar work mode control timeout\n");
  }
}

LidarDevice* VehicleLidarCallback::GetLidarDevice(const uint8_t slot, void* client_data) {
  if (client_data == nullptr) {
    printf("Get lidar device failed, the client data is nullptr.\n");
    return nullptr;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (lds_lidar == nullptr) {
    printf("Get lidar device failed, the lds lidar is nullptr.\n");
    return nullptr;
  }

  uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kVehicleLidarType, slot, index);
  if (ret != 0) {
    printf("Get vehicle config failed, can not get index, the slot:%u\n", slot);
    return nullptr;
  }

  return &(lds_lidar->lidars_[index]);
}

} // namespace livox_ros
