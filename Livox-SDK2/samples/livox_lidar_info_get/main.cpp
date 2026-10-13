//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//

// Sample: actively query the lidar's current state info.
// Call QueryLivoxLidarInternalInfo when the lidar comes online; SDK sends
// a query for the dev_type's registered key set (Avia2 has 19 keys, see
// GeneralCommandHandler::GetQueryLidarInternalInfoKeys) and returns them
// as a packed KV list. This demo decodes every key into a readable value.

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

static std::atomic<bool> g_done{false};

static void PrintIpPortPair(const char* label, const uint8_t* value) {
  uint8_t  ip[4]     = {value[0], value[1], value[2], value[3]};
  uint16_t host_port = 0;
  uint16_t lidar_port = 0;
  memcpy(&host_port,  value + 4, sizeof(uint16_t));
  memcpy(&lidar_port, value + 6, sizeof(uint16_t));
  printf("  %-32s host_ip:%u.%u.%u.%u host_port:%u lidar_port:%u\n",
         label, ip[0], ip[1], ip[2], ip[3], host_port, lidar_port);
}

static void PrintKey(const LivoxLidarKeyValueParam* kv) {
  const uint8_t* v = &kv->value[0];
  switch (kv->key) {
    case kKeyPclDataType:
      printf("  %-32s %u\n", "pcl_data_type (0x0000)", v[0]);
      break;
    case kKeyLidarIpCfg:
      printf("  %-32s ip:%u.%u.%u.%u mask:%u.%u.%u.%u gw:%u.%u.%u.%u\n",
             "lidar_ip_cfg (0x0004)",
             v[0], v[1], v[2],  v[3],
             v[4], v[5], v[6],  v[7],
             v[8], v[9], v[10], v[11]);
      break;
    case kKeyStateInfoHostIpCfg:
      PrintIpPortPair("state_info_host_ip_cfg (0x0005)", v);
      break;
    case kKeyLidarPointDataHostIpCfg:
      PrintIpPortPair("pointcloud_host_ip_cfg (0x0006)", v);
      break;
    case kKeyLidarImuHostIpCfg:
      PrintIpPortPair("imu_host_ip_cfg (0x0007)", v);
      break;
    case kKeyWorkMode:
      printf("  %-32s %u\n", "work_mode (0x001A)", v[0]);
      break;
    case kKeyImuDataEn:
      printf("  %-32s %u\n", "imu_data_en (0x001C)", v[0]);
      break;
    case kKeySetFovMode:
      printf("  %-32s %u\n", "fov_mode (0x0022)", v[0]);
      break;
    case kKeySetEchoMode:
      printf("  %-32s %u\n", "echo_mode (0x0024)", v[0]);
      break;
    case kKeySetNTPServerIp:
      printf("  %-32s %u.%u.%u.%u\n", "ntp_server_ip (0x0025)", v[0], v[1], v[2], v[3]);
      break;
    case kKeySetITOCtrl:
      printf("  %-32s %u\n", "ito_mode (0x0027)", v[0]);
      break;
    case kKeySetFogNoiseFilter:
      printf("  %-32s %u\n", "fog_noise_filter (0x0028)", v[0]);
      break;
    case kKeySn: {
      char sn[17] = {0};
      memcpy(sn, v, kv->length < 16 ? kv->length : 16);
      printf("  %-32s %s\n", "sn (0x8000)", sn);
      break;
    }
    case kKeyVersionApp:
      printf("  %-32s %u.%u.%u.%u\n", "version_app (0x8002)", v[0], v[1], v[2], v[3]);
      break;
    case kKeyVersionLoader:
      printf("  %-32s %u.%u.%u.%u\n", "version_loader (0x8003)", v[0], v[1], v[2], v[3]);
      break;
    case kKeyCurWorkState:
      printf("  %-32s %u\n", "cur_work_state (0x8006)", v[0]);
      break;
    case kKeyCoreTemp: {
      int32_t core_temp = 0;
      memcpy(&core_temp, v, sizeof(int32_t));
      printf("  %-32s %d\n", "core_temp (0x8007)", core_temp);
      break;
    }
    case kKeyFwType:
      printf("  %-32s %u\n", "fw_type (0x8010)", v[0]);
      break;
    case kKeyHmsCode: {
      uint32_t hms[8] = {0};
      memcpy(hms, v, sizeof(hms));
      printf("  %-32s [%u,%u,%u,%u,%u,%u,%u,%u]\n", "hms_code (0x8011)",
             hms[0], hms[1], hms[2], hms[3], hms[4], hms[5], hms[6], hms[7]);
      break;
    }
    default:
      printf("  UNKNOWN key:0x%04X len:%u\n", kv->key, kv->length);
      break;
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse* response, void*) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query failed, status:%u handle:%u\n", status, handle);
    g_done.store(true);
    return;
  }
  if (response == nullptr) {
    printf("Query response is null.\n");
    g_done.store(true);
    return;
  }
  if (response->ret_code != 0) {
    printf("Query ret_code:%u\n", response->ret_code);
    g_done.store(true);
    return;
  }

  printf("Query OK, param_num:%u\n", response->param_num);

  uint16_t off = 0;
  for (uint16_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    PrintKey(kv);
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }
  g_done.store(true);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void*) {
  if (info == nullptr) {
    printf("LidarInfoChangeCallback: info is null.\n");
    return;
  }
  struct in_addr addr;
  addr.s_addr = handle;
  printf("Lidar online, handle:%u ip:%s sn:%s dev_type:%u\n",
         handle, inet_ntoa(addr), info->sn, info->dev_type);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    printf("Usage: %s <config.json>\n", argv[0]);
    return -1;
  }
  const std::string path = argv[1];

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  const int timeout_sec = 30;
  for (int i = 0; i < timeout_sec && !g_done.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  LivoxLidarSdkUninit();
  printf("info_get demo end.\n");
  return 0;
}
