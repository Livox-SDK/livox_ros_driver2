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
#include "hub_callback.h"
#include "../lds_hub.h"
#include "../lds.h"

#include <string.h>
#include <stdio.h>
#include <mutex>

namespace livox_ros {

extern LdsHub *g_lds_hub;

/** Static function in LdsLidar for callback */
void HubCallback::OnHubDataCb(PointCloudFrame* frame, void *client_data) {
  if (frame->lidar_num == 0) {
    printf("Hub get data failed, the lidar num is zero.\n");
    return;
  }

  LdsHub *lds_hub = static_cast<LdsHub *>(client_data);
  if (lds_hub == nullptr) {
    printf("Hub get data failed, the client data is nullptr.\n");
    return;
  }

  lds_hub->StoragePointData(frame);
}

void HubCallback::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL) {
    return;
  }

  if (info->dev_type != kDeviceTypeHub) {
    printf("It's not a hub : %s\n", info->broadcast_code);
    return;
  }

  if (g_lds_hub->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
  } else {
    if (!g_lds_hub->IsBroadcastCodeExistInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n", info->broadcast_code);
      return;
    }
  }

  LidarDevice *p_hub = &g_lds_hub->hub_;
  if (p_hub->connect_state == kConnectStateOff) {
    bool result = false;
    uint8_t handle = 0;
    result = AddHubToConnect(info->broadcast_code, &handle);
    if (result == kStatusSuccess && handle < kMaxLidarCount) {
      //SetDataCallback(handle, HubCallback::OnHubDataCb, (void *)g_lds_hub);
      p_hub->handle = handle;
      p_hub->connect_state = kConnectStateOff;

      printf("add to connect\n");

      UserRawConfig config;
      if (strncmp(info->broadcast_code, g_lds_hub->hub_raw_config_.broadcast_code,
          sizeof(info->broadcast_code)) != 0) {
        printf("Could not find hub raw config, set config to default!\n");
        config.enable_fan = 1;
        config.return_mode = kFirstReturn;
        config.coordinate = kCoordinateCartesian;
        config.imu_rate = kImuFreq200Hz;
      } else {
        config = g_lds_hub->hub_raw_config_;
      }

      p_hub->config.enable_fan = config.enable_fan;
      p_hub->config.return_mode = config.return_mode;
      p_hub->config.coordinate = config.coordinate;
      p_hub->config.imu_rate = config.imu_rate;
    } else {
      printf("Add Hub to connect is failed : %d %d \n", result, handle);
    }
  }
}

/** Callback function of changing of device state. */
void HubCallback::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  if (info->handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_hub = &g_lds_hub->hub_;
  if (type == kEventHubConnectionChange) {
    if (p_hub->connect_state == kConnectStateOff) {
      p_hub->connect_state = kConnectStateOn;
      p_hub->info = *info;
      printf("Hub[%s] connect on\n", p_hub->info.broadcast_code);
    }
  } else if (type == kEventDisconnect) {
    g_lds_hub->ResetLds(0);
    g_lds_hub->ResetLidar(p_hub, 0);
    printf("Hub[%s] disconnect!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    p_hub->info = *info;
    printf("Hub[%s] StateChange\n", info->broadcast_code);
  }

  if (p_hub->connect_state == kConnectStateOn) {
    printf("Hub[%s] status_code[%d] working state[%d] feature[%d]\n",
           p_hub->info.broadcast_code,
           p_hub->info.status.status_code.error_code, p_hub->info.state,
           p_hub->info.feature);
    SetErrorMessageCallback(p_hub->handle, HubCallback::HubErrorStatusCb);
    if (p_hub->info.state == kLidarStateNormal) {
      HubQueryLidarInformation(HubQueryLidarInfoCb, g_lds_hub);
    }
  }
}

void HubCallback::HubQueryLidarInfoCb(livox_status status, uint8_t handle,
                                 HubQueryLidarInformationResponse *response,
                                 void *client_data) {
  LdsHub *lds_hub = static_cast<LdsHub *>(client_data);
  if ((handle >= kMaxLidarCount) || !response) {
    return;
  }

  if ((status == kStatusSuccess) && !response->ret_code) {
    if (response->count) {
      printf("Hub have %d lidars:\n", response->count);
      for (int i = 0; i < response->count; i++) {
        uint32_t index = HubGetLidarHandle(response->device_info_list[i].slot,
                                           response->device_info_list[i].id);
        if (index < kMaxLidarCount) {
          LidarDevice *p_lidar = &lds_hub->lidars_[index];
          p_lidar->handle = index;
          p_lidar->info.handle = index;
          p_lidar->info.slot = response->device_info_list[i].slot;
          p_lidar->info.id = response->device_info_list[i].id;
          p_lidar->info.type = response->device_info_list[i].dev_type;
          p_lidar->connect_state = kConnectStateSampling;
          strncpy(p_lidar->info.broadcast_code,
                  response->device_info_list[i].broadcast_code,
                  sizeof(p_lidar->info.broadcast_code));
          printf("[%d]%s DeviceType[%d] Slot[%d] Ver[%d.%d.%d.%d]\n", index,
                 p_lidar->info.broadcast_code, p_lidar->info.type,
                 p_lidar->info.slot, response->device_info_list[i].version[0],
                 response->device_info_list[i].version[1],
                 response->device_info_list[i].version[2],
                 response->device_info_list[i].version[3]);
        }
      }
      ConfigLidarsOfHub(lds_hub);
    } else {
      printf("Hub have no lidar, will not start sample!\n");
      HubQueryLidarInformation(HubQueryLidarInfoCb, lds_hub);
    }
  } else {
    printf("Device Query Informations Failed %d\n", status);
    HubQueryLidarInformation(HubQueryLidarInfoCb, lds_hub);
  }
}

/** Callback function of hub error message. */
void HubCallback::HubErrorStatusCb(livox_status status, uint8_t handle,
                              ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("sync_status : %u\n", message->hub_error_code.sync_status);
      printf("temp_status : %u\n", message->hub_error_code.temp_status);
      printf("lidar_status :%u\n", message->hub_error_code.lidar_status);
      printf("lidar_link_status : %u\n",
             message->hub_error_code.lidar_link_status);
      printf("firmware_err : %u\n", message->hub_error_code.firmware_err);
      printf("system_status : %u\n", message->hub_error_code.system_status);
    }
  }
}

void HubCallback::ControlFanCb(livox_status status, uint8_t handle, uint8_t response,
                          void *client_data) {}

void HubCallback::HubSetPointCloudReturnModeCb(
    livox_status status, uint8_t handle,
    HubSetPointCloudReturnModeResponse *response, void *client_data) {
  LdsHub *lds_hub = static_cast<LdsHub *>(client_data);
  if ((handle >= kMaxLidarCount) || !response) {
    return;
  }

  if ((status != kStatusSuccess) || (response->ret_code)) {
    printf("Hub set return mode fail!\n");
    ConfigPointCloudReturnMode(lds_hub);
  } else {
    printf("Hub set return mode success!\n");
    lds_hub->hub_.config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!lds_hub->hub_.config.set_bits) {
      HubStartSampling(HubCallback::StartSampleCb, lds_hub);
      lds_hub->hub_.connect_state = kConnectStateSampling;
    }
  }
}

void HubCallback::SetCoordinateCb(livox_status status, uint8_t handle,
                             uint8_t response, void *client_data) {
  LdsHub *lds_hub = static_cast<LdsHub *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_hub = &(lds_hub->hub_);
  if (status == kStatusSuccess) {
    p_hub->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    printf("Set coordinate success!\n");

    if (!p_hub->config.set_bits) {
      HubStartSampling(HubCallback::StartSampleCb, lds_hub);
      p_hub->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_hub->config.coordinate != 0) {
      SetSphericalCoordinate(handle, HubCallback::SetCoordinateCb, lds_hub);
    } else {
      SetCartesianCoordinate(handle, HubCallback::SetCoordinateCb, lds_hub);
    }
    printf("Set coordinate fail, try again!\n");
  }
}

void HubCallback::HubSetImuRatePushFrequencyCb(
    livox_status status, uint8_t handle,
    HubSetImuPushFrequencyResponse *response, void *client_data) {
  LdsHub *lds_hub = static_cast<LdsHub *>(client_data);
  if ((handle >= kMaxLidarCount) || !response) {
    return;
  }

  if ((status != kStatusSuccess) || (response->ret_code)) {
    printf("Hub set imu freq fail [%d]!\n", response->ret_code);
    ConfigImuPushFrequency(lds_hub);
  } else {
    printf("Hub set imu frequency success!\n");
    lds_hub->hub_.config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!lds_hub->hub_.config.set_bits) {
      HubStartSampling(HubCallback::StartSampleCb, lds_hub);
      lds_hub->hub_.connect_state = kConnectStateSampling;
    }
  }
}

/** Callback function of starting sampling. */
void HubCallback::StartSampleCb(livox_status status, uint8_t handle,
                           uint8_t response, void *client_data) {
  LdsHub *lds_hub = static_cast<LdsHub *>(client_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_hub = &lds_hub->hub_;
  if ((status != kStatusSuccess) || (response != 0)) {
    p_hub->connect_state = kConnectStateOn;
    printf("Hub start sample fail : state[%d] handle[%d] res[%d]\n", status, handle, response);

    for (int i = 0; i < kMaxLidarCount; i++) {
      LidarDevice *p_lidar = &(lds_hub->lidars_[i]);
      if (p_lidar->connect_state == kConnectStateSampling) {
        p_lidar->connect_state = kConnectStateOn;
      }
    }
  } else {
    printf("Hub start sample success!\n");
  }
}

/** Callback function of stopping sampling. */
void HubCallback::StopSampleCb(livox_status status, uint8_t handle, uint8_t response,
                          void *client_data) {}

void HubCallback::ConfigPointCloudReturnMode(LdsHub *lds_hub) {
  uint8_t req_buf[1024];
  HubSetPointCloudReturnModeRequest *req = (HubSetPointCloudReturnModeRequest *)req_buf;
  req->count = 0;
  for (int i = 0; i < kMaxLidarCount; i++) {
    LidarDevice *p_lidar = &(lds_hub->lidars_[i]);

    if ((p_lidar->info.type != kDeviceTypeLidarMid40) &&
        (p_lidar->connect_state == kConnectStateSampling)) {
      UserRawConfig config;
      if (lds_hub->GetRawConfig(p_lidar->info.broadcast_code, config)) {
        printf("Could not find raw config, set config to default!\n");
        config.enable_fan = 1;
        config.return_mode = kFirstReturn;
        config.coordinate = kCoordinateCartesian;
        config.imu_rate = kImuFreq200Hz;
      }
      strncpy(req->lidar_cfg_list[req->count].broadcast_code,
              p_lidar->info.broadcast_code,
              sizeof(req->lidar_cfg_list[req->count].broadcast_code));
      req->lidar_cfg_list[req->count].mode = config.return_mode;
      req->count++;
    }
  }

  if (req->count) {
    uint32_t length =
        1 + sizeof(SetPointCloudReturnModeRequestItem) * req->count;
    HubSetPointCloudReturnMode(req, length,
                               HubCallback::HubSetPointCloudReturnModeCb, lds_hub);
    lds_hub->hub_.config.set_bits |= kConfigReturnMode;
  }
}

void HubCallback::ConfigImuPushFrequency(LdsHub *lds_hub) {
  uint8_t req_buf[1024];
  HubSetImuPushFrequencyRequest *req = (HubSetImuPushFrequencyRequest *)req_buf;
  req->count = 0;
  for (int i = 0; i < kMaxLidarCount; i++) {
    LidarDevice *p_lidar = &(lds_hub->lidars_[i]);

    if ((p_lidar->info.type != kDeviceTypeLidarMid40) &&
        (p_lidar->info.type != kDeviceTypeLidarMid70) &&
        (p_lidar->connect_state == kConnectStateSampling)) {
      UserRawConfig config;
      if (lds_hub->GetRawConfig(p_lidar->info.broadcast_code, config)) {
        printf("Could not find raw config, set config to default!\n");
        config.enable_fan = 1;
        config.return_mode = kFirstReturn;
        config.coordinate = kCoordinateCartesian;
        config.imu_rate = kImuFreq200Hz;
      }
      strncpy(req->lidar_cfg_list[req->count].broadcast_code,
              p_lidar->info.broadcast_code,
              sizeof(req->lidar_cfg_list[req->count].broadcast_code));
      req->lidar_cfg_list[req->count].freq = config.imu_rate;
      req->count++;
    }
  }

  if (req->count) {
    uint32_t length = 1 + sizeof(SetImuPushFrequencyRequestItem) * req->count;
    HubSetImuPushFrequency(req, length, HubCallback::HubSetImuRatePushFrequencyCb,
                           lds_hub);
    lds_hub->hub_.config.set_bits |= kConfigImuRate;
  }
}

void HubCallback::ConfigLidarsOfHub(LdsHub *lds_hub) {
  ConfigPointCloudReturnMode(lds_hub);
  ConfigImuPushFrequency(lds_hub);

  if (lds_hub->hub_.config.coordinate != 0) {
    SetSphericalCoordinate(lds_hub->hub_.handle, HubCallback::SetCoordinateCb,
                           lds_hub);
    printf("Hub set coordinate spherical\n");
  } else {
    printf("Hub set coordinate cartesian\n");
    SetCartesianCoordinate(lds_hub->hub_.handle, HubCallback::SetCoordinateCb,
                           lds_hub);
  }
  lds_hub->hub_.config.set_bits |= kConfigCoordinate;

  lds_hub->hub_.connect_state = kConnectStateConfig;
}

} // namespace livox_ros
