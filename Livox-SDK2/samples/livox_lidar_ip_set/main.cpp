//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//

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

// Target lidar IP config to apply. Override via CLI: <config> <ip> <mask> <gw>
static char g_target_ip[16]   = "192.168.1.10";
static char g_target_mask[16] = "255.255.255.0";
static char g_target_gw[16]   = "192.168.1.1";

static std::atomic<bool> g_done{false};

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void*) {
  if (response == nullptr) {
    printf("Reboot response is null, status:%u handle:%u\n", status, handle);
    return;
  }
  printf("Reboot ack, status:%u handle:%u ret_code:%u\n", status, handle, response->ret_code);
  g_done.store(true);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void*) {
  if (response == nullptr) {
    printf("SetIp response is null, status:%u handle:%u\n", status, handle);
    return;
  }
  printf("SetLivoxLidarIp ack, status:%u handle:%u ret_code:%u error_key:%u\n",
         status, handle, response->ret_code, response->error_key);

  if (status == kLivoxLidarStatusSuccess && response->ret_code == 0 && response->error_key == 0) {
    printf("SetLivoxLidarIp OK, rebooting lidar to apply new IP...\n");
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  } else {
    printf("SetLivoxLidarIp failed, skip reboot.\n");
    g_done.store(true);
  }
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

  LivoxLidarIpInfo lidar_ip_info;
  memset(&lidar_ip_info, 0, sizeof(lidar_ip_info));
  strncpy(lidar_ip_info.ip_addr,  g_target_ip,   sizeof(lidar_ip_info.ip_addr)  - 1);
  strncpy(lidar_ip_info.net_mask, g_target_mask, sizeof(lidar_ip_info.net_mask) - 1);
  strncpy(lidar_ip_info.gw_addr,  g_target_gw,   sizeof(lidar_ip_info.gw_addr)  - 1);

  printf("Apply lidar ip -> %s / mask %s / gw %s\n",
         lidar_ip_info.ip_addr, lidar_ip_info.net_mask, lidar_ip_info.gw_addr);

  SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

int main(int argc, const char* argv[]) {
  if (argc < 2 || argc > 5) {
    printf("Usage: %s <config.json> [lidar_ip] [subnet_mask] [gateway]\n", argv[0]);
    printf("  defaults: %s %s %s\n", g_target_ip, g_target_mask, g_target_gw);
    return -1;
  }
  const std::string path = argv[1];
  if (argc >= 3) strncpy(g_target_ip,   argv[2], sizeof(g_target_ip)   - 1);
  if (argc >= 4) strncpy(g_target_mask, argv[3], sizeof(g_target_mask) - 1);
  if (argc >= 5) strncpy(g_target_gw,   argv[4], sizeof(g_target_gw)   - 1);

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  const int timeout_sec = 60;
  for (int i = 0; i < timeout_sec && !g_done.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  LivoxLidarSdkUninit();
  printf("set_lidar_ip demo end.\n");
  return 0;
}
