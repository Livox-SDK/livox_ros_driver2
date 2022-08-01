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
#include "industrial_lidar_callback.h"

#include <stdio.h>
#include <mutex>

namespace livox_ros {

extern LdsLidar *g_lds_ldiar;

LidarDevice* IndustrialLidarCallback::GetLidarDevice(const uint8_t handle) {
  uint8_t index = 0;
  int8_t ret = g_lds_ldiar->cache_index_.GetIndex(kIndustryLidarType, handle, index);
  if (ret == -1) {
    printf("Get industry lidar device failed, can not get free index, the handle:%u\n", handle);
    return nullptr;
  }
  return &(g_lds_ldiar->lidars_[index]);
}

LidarDevice* IndustrialLidarCallback::GetLidarDevice(const uint8_t handle, void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  if (lds_lidar == nullptr) {
    return nullptr;
  }

	uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kIndustryLidarType, handle, index);
  if (ret == -1) {
    printf("Get instry lidar device failed, can not get free index, the handle:%u\n", handle);
    return nullptr;
  }
  return &(lds_lidar->lidars_[index]);
}

void IndustrialLidarCallback::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n",
           info->broadcast_code);
    return;
  }

  if (g_lds_ldiar->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n",
           info->broadcast_code);
  } else {
    if (!g_lds_ldiar->IsBroadcastCodeExistInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n",
             info->broadcast_code);
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kIndustryLidarType, handle, index);
    if (ret != 0) {
      printf("Industrial lidar broadcast failed, can not get free index.\n");
      return;
    }
    LidarDevice* p_lidar = &(g_lds_ldiar->lidars_[index]);
    if (p_lidar == nullptr) {
      printf("Add industrial lidar to connect is failed, can not get free index, the handle:%u\n", handle);
      return;
    }
    
    p_lidar->handle = handle;
    p_lidar->lidar_type = kIndustryLidarType;
    p_lidar->connect_state = kConnectStateOff;

    UserRawConfig config;
    if (g_lds_ldiar->GetRawIndustrialConfig(info->broadcast_code, config)) {
      printf("Could not find raw config, set config to default!\n");
      config.enable_fan = 1;
      config.return_mode = kFirstReturn;
      config.coordinate = kCoordinateCartesian;
      config.imu_rate = kImuFreq200Hz;
      //config.extrinsic_parameter_source = kNoneExtrinsicParameter;
      config.extrinsic_parameter_source = kExtrinsicParameterFromLidar;
      config.enable_high_sensitivity = false;
    }

    p_lidar->config.enable_fan = config.enable_fan;
    p_lidar->config.return_mode = config.return_mode;
    p_lidar->config.coordinate = config.coordinate;
    p_lidar->config.imu_rate = config.imu_rate;
    p_lidar->config.extrinsic_parameter_source = config.extrinsic_parameter_source;
    p_lidar->config.enable_high_sensitivity = config.enable_high_sensitivity;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void IndustrialLidarCallback::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = GetLidarDevice(handle);
  if (p_lidar == nullptr) {
    printf("Device change failed, can not get lidar device.\n");
    return;
  }

  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lds_ldiar);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
  } else if (type == kEventDisconnect) {
    printf("Lidar[%s] disconnect!\n", info->broadcast_code);
    g_lds_ldiar->ResetLidar(p_lidar, kSourceRawLidar);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    printf("Lidar[%s] status_code[%d] working state[%d] feature[%d]\n",
           p_lidar->info.broadcast_code,
           p_lidar->info.status.status_code.error_code, p_lidar->info.state,
           p_lidar->info.feature);
    SetErrorMessageCallback(handle, LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      /** Ensure the thread safety for set_bits and connect_state */
      std::lock_guard<std::mutex> lock(g_lds_ldiar->config_mutex_);
      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      } else {
        SetCartesianCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(
            handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
            SetPointCloudReturnModeCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if ((kDeviceTypeLidarMid70 != info->type) &&
          (kDeviceTypeLidarMid40 != info->type)) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 SetImuRatePushFrequencyCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      if (p_lidar->config.extrinsic_parameter_source ==
          kExtrinsicParameterFromLidar) {
        LidarGetExtrinsicParameter(handle, GetLidarExtrinsicParameterCb,
                                   g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigGetExtrinsicParameter;
      }

      if (kDeviceTypeLidarTele == info->type) {
        if (p_lidar->config.enable_high_sensitivity) {
          LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
          printf("Enable high sensitivity\n");
        } else {
          LidarDisableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
          printf("Disable high sensitivity\n");
        }
        p_lidar->config.set_bits |= kConfigSetHighSensitivity;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Receiving point cloud data from Livox LiDAR. */
void IndustrialLidarCallback::OnLidarDataCb(PointCloudFrame* frame, void *client_data) {
  if (frame->lidar_num == 0) {
    return;
  }
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  lds_lidar->StoragePointData(frame);
}

/** Query the firmware version of Livox LiDAR. */
void IndustrialLidarCallback::DeviceInformationCb(livox_status status, uint8_t handle,
                                   DeviceInformationResponse *ack, void *client_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("firmware version: %d.%d.%d.%d\n", ack->firmware_version[0],
           ack->firmware_version[1], ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void IndustrialLidarCallback::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

void IndustrialLidarCallback::ControlFanCb(livox_status status, uint8_t handle,
                            uint8_t response, void *client_data) {}

void IndustrialLidarCallback::SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = GetLidarDevice(handle, client_data);
  if (p_lidar == nullptr) {
    printf("Set point cloud return mode failed, can not get lidar device, the handle:%u\n", handle);
    return;
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  if (lds_lidar == nullptr) {
    return;
  }

  if (status == kStatusSuccess) {
    printf("Set return mode success!\n");

    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(
        handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
        SetPointCloudReturnModeCb, lds_lidar);
    printf("Set return mode fail, try again!\n");
  }
}

void IndustrialLidarCallback::SetCoordinateCb(livox_status status, uint8_t handle,
                               uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = GetLidarDevice(handle, client_data);
  if (p_lidar == nullptr) {
    printf("Set industrial lidar coordinate failed, can not get free index, the handle:%u\n", handle);
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  if (lds_lidar == nullptr) {
    return;
  }

  if (status == kStatusSuccess) {
    printf("Set coordinate success!\n");

    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, SetCoordinateCb, lds_lidar);
    }

    printf("Set coordinate fail, try again!\n");
  }
}

void IndustrialLidarCallback::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = GetLidarDevice(handle, client_data);
  if (p_lidar == nullptr) {
    printf("Set industrial lidar imu rate push frequency failed, can not get free index, the handle:%u\n", handle);
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  if (lds_lidar == nullptr) {
    return;
  }

  if (status == kStatusSuccess) {
    printf("Set imu rate success!\n");

    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                             SetImuRatePushFrequencyCb, g_lds_ldiar);
    printf("Set imu rate fail, try again!\n");
  }
}

/** Callback function of get LiDARs' extrinsic parameter. */
void IndustrialLidarCallback::GetLidarExtrinsicParameterCb(
    livox_status status, uint8_t handle,
    LidarGetExtrinsicParameterResponse *response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (status == kStatusSuccess) {
    if (response != nullptr) {
      printf("Lidar[%d] get ExtrinsicParameter status[%d] response[%d]\n", handle, status, response->ret_code);

      LidarExtrinsicParameters extrinsic;
      extrinsic.lidar_type = kIndustryLidarType;
      extrinsic.handle = handle;
      extrinsic.roll = response->roll;
      extrinsic.pitch = response->pitch;
      extrinsic.yaw = response->yaw;
      extrinsic.x = response->x;
      extrinsic.y = response->y;
      extrinsic.z = response->z;
      printf("GetLidarExtrinsicParameterCb,handle:%u, x:%d, y:%d, z:%d, roll:%f, pitch:%f, yar:%f\n",handle, extrinsic.x, extrinsic.y, extrinsic.z,
          extrinsic.roll, extrinsic.pitch, extrinsic.yaw);

      AddLidarsExtrinsicParams(extrinsic);

      printf("Lidar[%d] get ExtrinsicParameter success!\n", handle);

      LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
      if (lds_lidar == nullptr) {
        return;
      }

      LidarDevice* p_lidar = GetLidarDevice(handle, client_data);
      if (p_lidar == nullptr) {
        printf("Add industrial lidar to connect is failed, can not get free index, the handle:%u", handle);
        return;
      }

      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      p_lidar->config.set_bits &= ~((uint32_t)(kConfigGetExtrinsicParameter));
      if (!p_lidar->config.set_bits) {
        LidarStartSampling(handle, StartSampleCb, lds_lidar);
        p_lidar->connect_state = kConnectStateSampling;
      }
    } else {
      printf("Lidar[%d] get ExtrinsicParameter fail!\n", handle);
    }
  } else if (status == kStatusTimeout) {
    printf("Lidar[%d] get ExtrinsicParameter timeout!\n", handle);
    LidarGetExtrinsicParameter(handle, GetLidarExtrinsicParameterCb, g_lds_ldiar);
  }
}

void IndustrialLidarCallback::SetHighSensitivityCb(livox_status status, uint8_t handle,
                                    DeviceParameterResponse *response,
                                    void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = GetLidarDevice(handle, client_data);
  if (p_lidar == nullptr) {
    printf("Set industrial lidar high sensitivity failed, can not get free index, the handle:%u\n", handle);
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  if (lds_lidar == nullptr) {
    return;
  }

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigSetHighSensitivity));
    printf("Set high sensitivity success!\n");

    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.enable_high_sensitivity) {
      LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    } else {
      LidarDisableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    }
    printf("Set high sensitivity fail, try again!\n");
  }
}

/** Callback function of starting sampling. */
void IndustrialLidarCallback::StartSampleCb(livox_status status, uint8_t handle,
                             uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = GetLidarDevice(handle, client_data);
  if (p_lidar == nullptr) {
    printf("Start industrial lidar sample failed, can not get free index, the handle:%u\n", handle);
  }

  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", status,
             handle, response);
    } else {
      printf("Lidar start sample success\n");
    }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n",
           status, handle, response);
  }
}

/** Callback function of stopping sampling. */
void IndustrialLidarCallback::StopSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *client_data) {}

void IndustrialLidarCallback::SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                                uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  printf("Set lidar[%d] sync time status[%d] response[%d]\n", handle, status, response);
}

void IndustrialLidarCallback::ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length, void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  // std::unique_lock<std::mutex> lock(mtx);
  //LidarDevice *p_lidar = nullptr;
  for (uint8_t index = 0; index < kMaxLidarCount; ++index) {
    const LidarDevice* p_lidar = &(lds_lidar->lidars_[index]);
    if (p_lidar->lidar_type == kIndustryLidarType) {
      if (p_lidar->connect_state == kConnectStateSampling && p_lidar->info.state == kLidarStateNormal) {
        livox_status status = LidarSetRmcSyncTime(p_lidar->handle, rmc, rmc_length, SetRmcSyncTimeCb, lds_lidar);
        if (status != kStatusSuccess) {
          printf("Set GPRMC synchronization time error code: %d.\n", status);
        }
      }
    }
  }
}

} // namespace

