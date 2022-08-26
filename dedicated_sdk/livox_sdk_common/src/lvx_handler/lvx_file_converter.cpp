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

#include "lvx_file_converter.h"

#include <iostream>
#include <string>
#include <cstring>

#include "livox_def.h"

namespace livox {
namespace common {

LvxFileConverter &lvx_file_converter() {
  static LvxFileConverter converter;
  return converter;
}

LvxFileConverter::LvxFileConverter() {}

bool LvxFileConverter::Init() {
  bool expected = false;
  if (!is_working_.compare_exchange_strong(expected, true)) {
    return false;
  }
  return true;
}

void LvxFileConverter::Reset() {
  lvx_file_record_handler().CloseLvxFile();
  lvx_file_record_handler().Clear();
  lvx_file_parse_handler().StopParseLvxFile();
  infos_.clear();
  is_ready_.store(false);
  is_working_.store(false);
  return;
}

bool LvxFileConverter::ParseLvxFile(const char *lvx_file_path) {
  if (!lvx_file_parse_handler().SetLvxParseDir(std::string(lvx_file_path))) {
    std::cout << "Failed to load the file" << std::endl;
    return false;
  }
  if (!lvx_file_parse_handler().StartParseLvxFile()) {
    std::cout << "Failed to parse the file" << std::endl;
    return false;
  }
  std::unique_ptr<LvxDeviceInfo[]> device_list(new LvxDeviceInfo[kMaxLidarCount]);
  lvx_file_parse_handler().GetLvxDeviceInfoList(infos_);
  return true;
}

bool LvxFileConverter::InitFileHeader(const char *output_dir, LvxFileType file_type) {
  if (!lvx_file_record_handler().SetLvxRecordDir(output_dir)) {
    // std::cout << "invalid dir" << std::endl;
    return false;
  }
  lvx_file_record_handler().AddDeviceInfo(std::move(infos_));

  if (!lvx_file_record_handler().InitLvxFile(file_type)) {
    // std::cout << "failed to init lvx file" << std::endl;
    return false;
  }
  lvx_file_record_handler().InitLvxFileHeader();
  return true;
}

bool LvxFileConverter::RecordLvx3Frames() {
  // if (!is_ready_.load()) {
  //   return false;
  // }

  // uint32_t total_frame_count = lvx_file_parse_handler().GetFrameCount();
  // uint32_t packet_num = 0;
  // uint32_t half_packet_num = 0;
  // for (uint32_t frame_index = 0; frame_index < total_frame_count; frame_index++) {
  //   LvxBasePackDetail* packet = nullptr;
  //   LvxBaseHalfPackDetail* half_packet = nullptr;
  //   if (!(lvx_file_parse_handler().GetPointData(frame_index, packet, packet_num, half_packet, half_packet_num))) {
  //     std::cout << "Get point cloud failed, the index:" << frame_index << std::endl;
  //     continue;
  //   }

  //   std::vector<LvxPackageDetail> packages_vec;
  //   packages_vec.reserve(packet_num + half_packet_num);

  //   for (uint32_t i = 0; i < packet_num; i++) {
  //     int point_number = packet[i].header.length / sizeof(LivoxVehicleExtendRawPoint);
  //     LvxPackageDetail package;
  //     package.header = packet[i].header;
  //     package.packets.resize(point_number);
  //     LivoxVehicleExtendRawPoint *point = (LivoxVehicleExtendRawPoint *)packet[i].raw_point;
  //     for (int j = 0; j < point_number; j++) {
  //       package.packets[j].x = point[j].x;
  //       package.packets[j].y = point[j].y;
  //       package.packets[j].z = point[j].z;
  //       package.packets[j].reflectances = point[j].reflectivity;
  //       package.packets[j].tag = point[j].tag;
  //     }
  //     packages_vec.push_back(std::move(package));
  //   }

  //   for (uint32_t i = 0; i < half_packet_num; i++) {
  //     int point_number = half_packet[i].header.length / sizeof(LivoxVehicleExtendHalfRawPoint);
  //     LvxPackageDetail half_package;
  //     half_package.header = half_packet[i].header;
  //     half_package.packets.resize(point_number);
  //     LivoxVehicleExtendHalfRawPoint *point = (LivoxVehicleExtendHalfRawPoint *)half_packet[i].raw_point;
  //     for (int j = 0; j < point_number; j++) {
  //       half_package.packets[j].x = point[j].x;
  //       half_package.packets[j].y = point[j].y;
  //       half_package.packets[j].z = point[j].z;
  //       half_package.packets[j].reflectances = point[j].reflectivity;
  //       half_package.packets[j].tag = point[j].tag;
  //     }
  //     packages_vec.push_back(std::move(half_package));
  //   }
  //   lvx_file_record_handler().SaveFrameToLvx3File(packages_vec);
  // }
  return true;
}

bool LvxFileConverter::RecordLvx2Frames() {
  if (!is_ready_.load()) {
    return false;
  }

  uint32_t total_frame_count = lvx_file_parse_handler().GetFrameCount();
  // uint32_t frame_durantion = lvx_file_parse_handler().GetFrameDuration();
  // printf ("lvx record total time is: %d ms\n", (int)total_frame_count * (int)frame_durantion);
  uint32_t packet_num = 0;
  uint32_t half_packet_num = 0;
  for (uint32_t frame_index = 0; frame_index < total_frame_count; frame_index++) {
    LvxBasePackDetail* packet = nullptr;
    LvxBaseHalfPackDetail* half_packet = nullptr;
    if (!(lvx_file_parse_handler().GetPointData(frame_index, packet, packet_num, half_packet, half_packet_num))) {
      std::cout << "Get point cloud failed, the index:" << frame_index << std::endl;
      continue;
    }

    std::vector<LvxBasePackDetail> base_pack_vec;
    base_pack_vec.reserve(packet_num);
    for (uint32_t i = 0; i < packet_num; i++) {
      LvxBasePackDetail base_pack;
      base_pack.header = packet[i].header;
      if (base_pack.header.length > sizeof(LivoxVehicleExtendRawPoint)) {
        memcpy((char*)base_pack.raw_point, (char*)packet[i].raw_point, base_pack.header.length);
      }
      base_pack_vec.push_back(std::move(base_pack));
    }

    std::vector<LvxBaseHalfPackDetail> base_half_pack_vec;
    base_half_pack_vec.reserve(half_packet_num);
    for (uint32_t i = 0; i < half_packet_num; ++i) {
      LvxBaseHalfPackDetail base_half_pack;
      base_half_pack.header = half_packet[i].header;
      if (base_half_pack.header.length > sizeof(LivoxVehicleExtendHalfRawPoint)) {
        memcpy((char*)base_half_pack.raw_point, (char*)packet[i].raw_point, base_half_pack.header.length);
      }
    }

    lvx_file_record_handler().SaveFrameToLvx2File(base_pack_vec, base_half_pack_vec);
  }
  return true;
}

bool LvxFileConverter::Lvx2ToLvx3(const char *lvx2_file_path, const char *output_dir) {
  return ConvertFileType(lvx2_file_path, output_dir, LIVOX_FILE_LVX3_PPT);
}

bool LvxFileConverter::Lvx3ToLvx2(const char *lvx3_file_path, const char *output_dir) {
  return ConvertFileType(lvx3_file_path, output_dir, LIVOX_FILE_LVX2);
}

bool LvxFileConverter::ConvertFileType(const char *file_path, const char *output_dir, LvxFileType target_file_type) {
  // if (file_path == nullptr || output_dir == nullptr) {
  //   return false;
  // }
  // if (!Init()) {
  //   std::cout << "FileConverter: failed to init!" << std::endl;
  //   return false;
  // }
  // if (!ParseLvxFile(file_path)) {
  //   std::cout << "FileConverter: failed to parse the file: " << file_path << std::endl;
  //   return false;
  // }
  // if (!InitFileHeader(output_dir, target_file_type)) {
  //   std::cout << "FileConverter: failed to record file header!" << std::endl;
  //   return false;
  // }
  // is_ready_.store(true);
  // /*if (target_file_type == LIVOX_FILE_LVX3_PPT) {
  //   if (!RecordLvx3Frames()) {
  //     return false;
  //   }
  // } else {*/
  //   if (!RecordLvx2Frames()) {
  //     return false;
  //   }
  // //}
  // Reset();
  return true;
}

} // namespace common
} // namespace livox



