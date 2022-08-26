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

#include "livox_sdk_common.h"
#include "livox_def_common.h"
// #include "livox_def.h"

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include <iostream>
#include <string.h>
#include <map>
#include <memory>

//#define kMaxLidarCount 30
#define kMaxPacketNumber 100


int main(int argc, const char *argv[]) {
  if (!SetLvxParseDir("./test.lvx2")) {
    std::cout << "file not found! " << std::endl;
    return -1;
  }

  if (!StartParseLvxFile()) {
    return -1;
  }
  uint32_t total_frame_count = GetLvxFrameCount();
  uint32_t frame_durantion = GetLvxFrameDuration();
  std::cout << "lvx record total frame count: " << total_frame_count << ", frame duration:" << frame_durantion << std::endl;
  std::cout << "lvx record total time: " << static_cast<uint64_t>(total_frame_count) * static_cast<uint64_t>(frame_durantion) 
    << " ms" << std::endl;

  std::unique_ptr<LvxDeviceInfo[]> device_list(new LvxDeviceInfo[kMaxLidarCount]);
  uint8_t device_count = kMaxLidarCount;
  GetLvxDeviceInfoList(device_list.get(), &device_count);
  std::map<uint32_t, LvxDeviceInfo> device_info_map;
  for (int count = 0; count < device_count; count++) {
    uint32_t slot = device_list[count].lidar_id;
    std::cout << "lvx file slot: " << slot << std::endl;
    device_info_map[slot] = device_list[count];
  }
  std::cout << "get lvx device info count: " << static_cast<int32_t>(device_count) << std::endl;
 
  // Get all 32bit data
  uint32_t packet_num = 0;
  uint64_t total_point_number = 0;
  for (uint32_t frame_index = 0; frame_index < total_frame_count; frame_index++) {
    const LvxBasePackDetail* packet = GetLvxPointCloud(frame_index, &packet_num);
    for (uint32_t i = 0; i < packet_num; i++) {
      uint32_t slot = packet[i].header.lidar_id;
      auto info = device_info_map[slot];
      // printf("Packet header info, pitch:%f, roll:%f, yaw:%f, x:%f, y:%f, z:%f.\n",
      //     info.pitch, info.roll, info.yaw, info.x, info.y, info.z);
      int point_number = packet[i].header.length / sizeof(LivoxLidarExtendRawPoint);
      LivoxLidarExtendRawPoint *point = (LivoxLidarExtendRawPoint *)packet[i].raw_point;
      // for (int j = 0; j < point_number; j++) {
      //   printf("Packet raw point, x:%u, y:%u, z:%u, reflectivity:%u, tag:%u.\n",
      //       point[j].x, point[j].y, point[j].z, point[j].reflectivity, point[j].tag);
      // }
      total_point_number += point_number;
    }
  }

  std::cout << "total point number: " << total_point_number << std::endl;
  std::cout << "lvx record total frame count: " << total_frame_count << std::endl;
  StopParseLvxFile();
  std::cout << "Livox Parse Lvx File Demo End!" << std::endl;
}
