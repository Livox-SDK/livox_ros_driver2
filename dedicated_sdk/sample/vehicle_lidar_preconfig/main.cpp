//
// The MIT License (MIT)
//
// Copyright (c) 2021 Livox. All rights reserved.
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

#include "livox_sdk_vehicle.h"
#include "livox_def_vehicle.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>

//Lidar IP and slot configuration demo

void LidarPreConfigCallback(livox_vehicle_status status, const char* broadcast_code, void* client_data) {
  printf("LiDAR SN: %s Config Success.\n", broadcast_code);
}

int main(int argc, const char *argv[]) {
  const char* net_if = "192.168.1.35";
  LidarPreConfigParam param = {};
  param.slot_id = 2;
  uint8_t ip_addr[4] = {192, 168, 1, 55};
  memcpy(&param.ip_addr, ip_addr, 4);
  uint8_t gw_addr[4] = {192, 168, 1, 1};
  memcpy(&param.gw_addr, gw_addr, 4);
  uint8_t net_mask[4] = {255, 255, 255, 0};
  memcpy(&param.net_mask, net_mask, 4);
  if (LivoxPreConfigInit(net_if, &param)) {
    printf("preconfig init success\n");
  }
  SetLidarPreConfigParamCallback(LidarPreConfigCallback, nullptr);

  std::this_thread::sleep_for(std::chrono::minutes(5));
  LivoxPreConfigUninit();
  printf("Livox PreConifparam Demo End!\n");
}
