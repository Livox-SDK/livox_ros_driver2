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

#include <cstring>
#include <iostream>
#include <time.h>
//#include "base/logging.h"
#ifdef WIN32
#include <direct.h>
#include <io.h>
#else
#include <dirent.h>
#include <unistd.h>
#endif // WIN32

#include "lvx_file_parse_handler.h"
#include "lvx_file_manager.h"

#define WRITE_BUFFER_LEN 1024 * 1024

namespace livox {
namespace common {

LvxFileParseHandler &lvx_file_parse_handler() {
  static LvxFileParseHandler handler;
  return handler;
}

LvxFileParseHandler::LvxFileParseHandler() {}

bool LvxFileParseHandler::SetLvxParseDir(const std::string& dir) {
  lvx_file_.open(dir.c_str(), std::ios::in | std::ifstream::binary | std::ios_base::ate);
  if (!lvx_file_.is_open()) {
    std::cout << "Failed to open file: " << dir << std::endl;
    return false;
  }
  if (!ParseLvxFileHeader()) {
    Clear();
    return false;
  }
  return true;
}

bool LvxFileParseHandler::ParseLvxFileHeader() {
  //LOG_INFO("Lvx Parse Init File Header");
  LvxFilePublicHeader public_header = {};
  LvxFilePrivateHeader private_header = {};
  lvx_file_size_ = lvx_file_.tellg();
  //LOG_INFO("Lvx Parse Total File Size: {}", (int)lvx_file_size_);

  lvx_file_.seekg(0, std::ios::beg);
  lvx_file_.read((char*)&public_header, sizeof(public_header));
  lvx_file_.read((char*)&private_header, sizeof(private_header));
  const std::string signature_str = std::string((char*)public_header.signature);
  if (signature_str != kFileSignature) {
    std::cout << "File verification failed (1)" << std::endl;
    return false;
  }
  if (public_header.magic_code != MAGIC_CODE) {
    std::cout << "File verification failed (2)" << std::endl;
    return false;
  }

  file_version_.file_ver_major = public_header.version[0];
  file_version_.file_ver_minor = public_header.version[1];
  file_version_.file_ver_patch = public_header.version[2];
  file_version_.file_ver_build = public_header.version[3];
  printf("File version, major:%u, minor::%u, patch:%u, build:%u\n", file_version_.file_ver_major,
      file_version_.file_ver_minor, file_version_.file_ver_patch, file_version_.file_ver_build);

  file_type_ = LvxFileManager::CheckFileType(file_version_);
  if (LIVOX_FILE_UNKNOWN == file_type_) {
    std::cout << "Unsupported file version!" << std::endl;
    return false;
  }

  device_count_ = private_header.device_count;
  frame_duration_ = private_header.frame_duration;
  std::cout << "Lvx Parse Device count: " << static_cast<int>(device_count_)
    << ", Frame duration: " << frame_duration_ << std::endl;

  device_infos_.clear();
  LvxDeviceInfo cur_info = {};
  for (int i = 0; i < device_count_; i++) {
    lvx_file_.read((char*)&cur_info, sizeof(LvxDeviceInfo));
    device_infos_.push_back(cur_info);
  }
  return true;
}

bool LvxFileParseHandler::StartParseLvxFile() {
  if (!InitFrameList()) {
    return false;
  }
  return true;
}

bool LvxFileParseHandler::InitFrameList() {
  /*if (file_type_ == LIVOX_FILE_LVX3_PPT) {
    return ReadPptFrameInfos();
  } else {*/
    return ReadLvxFrameInfos();
  //}
}

bool LvxFileParseHandler::ReadLvxFrameInfos() {
  if (!lvx_file_.is_open() || (file_type_ != LIVOX_FILE_LVX2) ) {
    std::cout << "error in reading lvx frame: file type mismatch!" << std::endl;
    return false;
  }
  Lvx2FrameHeader frame_header = {};
  frame_header.next_offset = 0;
  while (frame_header.next_offset != lvx_file_size_) {
    lvx_file_.read((char*)&frame_header, sizeof(Lvx2FrameHeader));
    lvx_file_.seekg(frame_header.next_offset, std::ios::beg);
    if (frame_header.current_offset >= lvx_file_size_
      || static_cast<int64_t>(frame_header.next_offset - frame_header.current_offset) < 0) {
      break;
    }
    FrameDetail detail = {};
    detail.start_offset = frame_header.current_offset;
    detail.data_len = frame_header.next_offset - frame_header.current_offset;
    frame_detail_list_.push_back(detail);
  }
  return true;
}

uint32_t LvxFileParseHandler::GetFrameCount() {
  //LOG_INFO("Lvx Parse Frame count: {}", (int)frame_detail_list_.size());
  return frame_detail_list_.size();
}

uint32_t LvxFileParseHandler::GetFrameDuration() {
  return frame_duration_;
}

void LvxFileParseHandler::GetLvxDeviceInfoList(std::vector<LvxDeviceInfo>& infos) {
  infos = device_infos_;
  return;
}


bool LvxFileParseHandler::GetPointData(const int frame_index,
                                       LvxBasePackDetail* &lvx_pack_detail,
                                       uint32_t& packet_num,
                                       LvxBaseHalfPackDetail* &lvx_half_pack_detail,
                                       uint32_t& half_packet_num) {
  if(!UpdatePacketList(frame_index)) {
    // std::out warning
    return false;
  }

  packet_num = packet_list_.size();
  half_packet_num = half_packet_list_.size();
  lvx_pack_detail = packet_list_.data();
  lvx_half_pack_detail = half_packet_list_.data();
  std::cout << "Lvx Get Frame index: " << frame_index << ", packet size: " << packet_num
            << ", half packet size: " << half_packet_num << std::endl;
  return true;
}

bool LvxFileParseHandler::UpdatePacketList(int32_t frame_index) {
  if (frame_index >= (int)frame_detail_list_.size()) {
    return false;
  }
  ResetPacketList();

  const FrameDetail& frame_detail = frame_detail_list_[frame_index];
  //LOG_INFO("Lvx Get Frame index: {} Start offset: {} data len: {}", frame_index, (int)frame_detail.start_offset, (int)frame_detail.data_len);

  lvx_file_.seekg(frame_detail.start_offset, std::ios::beg);
  std::unique_ptr<char[]> buf;
  buf.reset(new char[frame_detail.data_len]);
  lvx_file_.read((char*)buf.get(), frame_detail.data_len);

  /*if (file_type_ == LIVOX_FILE_LVX3_PPT) {
    LvxBasePackDetail pack_detail = {};
    memset(&pack_detail, 0, sizeof(pack_detail));

    Lvx3FrameHeader *header = reinterpret_cast<Lvx3FrameHeader *>(buf.get());
    std::vector<std::vector<PACKAGE_POINT>> output_points;
    std::vector<uint64_t> timeStamps;   // unused
    LvxBasePackHeader *package_header_base = reinterpret_cast<LvxBasePackHeader *>((unsigned char*)buf.get() + sizeof(Lvx3FrameHeader));
    uint64_t encoded_data_offset = sizeof(Lvx3FrameHeader) + header->packets_num * sizeof(LvxBasePackHeader);
    uint64_t encoded_data_size = header->data_size - encoded_data_offset;
    //PPTDecode((unsigned char*)buf.get() + encoded_data_offset, encoded_data_size, output_points, timeStamps);
    for (uint32_t i = 0; i < header->packets_num; ++i) {
      pack_detail.header = package_header_base[i];
      for (uint32_t j = 0; j < output_points[i].size(); ++j) {
        pack_detail.raw_point[j].x = output_points[i][j].x;
        pack_detail.raw_point[j].y = output_points[i][j].y;
        pack_detail.raw_point[j].z = output_points[i][j].z;
        pack_detail.raw_point[j].reflectivity = output_points[i][j].reflectances;
        pack_detail.raw_point[j].tag = output_points[i][j].tag;
      }
      packet_list_.emplace_back(pack_detail);
      memset(&pack_detail, 0, sizeof(pack_detail));
    }
  } else {*/
    uint64_t current_offset = sizeof(Lvx2FrameHeader);
    while (current_offset < frame_detail.data_len) {
      LvxBasePackHeader* header = (LvxBasePackHeader*) &buf[current_offset];
      /*if (header->lidar_type == static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType)) {
        if (header->data_type == static_cast<uint8_t>(VehiclePointDataType::kLowResolutionPointData)) {
          LvxBaseHalfPackDetail* half_pack = (LvxBaseHalfPackDetail*) &buf[current_offset];
          if (header->length > sizeof(LivoxVehicleExtendHalfRawPoint)) {
            half_packet_list_.emplace_back(*half_pack);
          }
        } else if (header->data_type == static_cast<uint8_t>(VehiclePointDataType::kHighResolutionPointData)) {
          LvxBasePackDetail* pack = (LvxBasePackDetail*)&buf[current_offset];
          if (header->length > sizeof(LivoxVehicleExtendRawPoint)) {
            packet_list_.emplace_back(*pack);
          }
        }
        current_offset += header->length + sizeof(LvxBasePackHeader);
      } else*/ 
      if (header->lidar_type == static_cast<uint8_t>(LivoxLidarType::kLivoxLidarType)) {
        if (header->data_type == static_cast<uint8_t>(LivoxLidarPointDataType::kLivoxLidarCartesianCoordinateLowData)) {
          LvxBaseHalfPackDetail* half_pack = (LvxBaseHalfPackDetail*) &buf[current_offset];
          if (header->length > sizeof(LivoxLidarCartesianLowRawPoint)) {
            half_packet_list_.emplace_back(*half_pack);
          }
        } else if (header->data_type == static_cast<uint8_t>(LivoxLidarPointDataType::kLivoxLidarCartesianCoordinateHighData)) {
          LvxBasePackDetail* pack = (LvxBasePackDetail*)&buf[current_offset];
          if (header->length > sizeof(LivoxLidarCartesianHighRawPoint)) {
            packet_list_.emplace_back(*pack);
          }
        }
      }
      current_offset += header->length + sizeof(LvxBasePackHeader);
    }
  //}
  return true;
}

void LvxFileParseHandler::ResetPacketList() {
  packet_list_.clear();
  half_packet_list_.clear();
}

const LvxBasePackDetail* LvxFileParseHandler::GetHighResPointData(const int frame_index, uint32_t& packet_num) {
  if(!UpdatePacketList(frame_index)) {
    // std::cout warning
    packet_num = 0;
    return nullptr;
  }
  if (!ConvertToHighResPackets()) {
    // std::cout warning
    // don't return
  }

  packet_num = packet_list_.size();
  std::cout << "Lvx Get Frame index: " << frame_index << ", packet size: " << packet_num << std::endl;
  return packet_list_.data();
}

bool LvxFileParseHandler::ConvertToHighResPackets() {
  if (half_packet_list_.size() == 0) {
    return false;
  }

  LvxBasePackDetail base_pack;
  for (auto &half_pack : half_packet_list_) {
    base_pack.header = half_pack.header;
    uint32_t len = base_pack.header.length;
    uint32_t offset = 0;
    int i = 0;
    while (offset < len) {
      base_pack.raw_point[i].x = half_pack.raw_point[i].x;
      base_pack.raw_point[i].y = half_pack.raw_point[i].y;
      base_pack.raw_point[i].z = half_pack.raw_point[i].z;
      base_pack.raw_point[i].reflectivity = half_pack.raw_point[i].reflectivity;
      base_pack.raw_point[i].tag = half_pack.raw_point[i].tag;
      offset += sizeof(LivoxVehicleExtendHalfRawPoint);
      ++i;
    }
    packet_list_.emplace_back(base_pack);
  }
  return true;
}

bool LvxFileParseHandler::IsFileSupported() {
  return LIVOX_FILE_UNKNOWN != file_type_;
}

LvxFileVersion LvxFileParseHandler::GetFileVersion() {
  return file_version_;
}

bool LvxFileParseHandler::ReadPptFrameInfos() {
  if (!lvx_file_.is_open() || file_type_ != LIVOX_FILE_LVX3_PPT) {
    std::cout << "error in reading Ppt frame: file type mismatch!" << std::endl;
    return false;
  }
  Lvx3FrameHeader frame_header = {};
  frame_header.current_offset = 0;
  frame_header.data_size = 0;
  while((frame_header.current_offset + frame_header.data_size) < lvx_file_size_) {
    lvx_file_.read((char*)&frame_header, sizeof(Lvx3FrameHeader));
    FrameDetail detail = {};
    detail.start_offset = frame_header.current_offset;
    detail.data_len = frame_header.data_size;
    frame_detail_list_.push_back(detail);
    lvx_file_.seekg(frame_header.current_offset + frame_header.data_size, std::ios::beg);
  }
  return true;
}

bool LvxFileParseHandler::CutLvxFile(uint64_t start_frame_index, uint64_t end_frame_index, const std::string &dir) {
  if (start_frame_index >= end_frame_index ||
    end_frame_index >= frame_detail_list_.size()) {
    return false;
  }
  if (!lvx_file_.is_open()) {
    return false;
  }

#ifdef WIN32
    if (_access(dir.c_str(), 0) != 0) {
        return false;
#else
    if (access(dir.c_str(), 0) != 0) {
        return false;
#endif
    }

  time_t curtime = time(nullptr);
  char filename[30] = { 0 };
  tm* local_time = localtime(&curtime);
  if (file_type_ == LIVOX_FILE_LVX3_PPT) {
    strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx3", local_time);
  } else {
    strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx2", local_time);
  }
  std::string file_path = dir + (dir.back() == '/' ? "" : "/") + std::string(filename);
  re_lvx_file_.open(file_path.c_str(), std::ios::out | std::ios::binary);
  if (!re_lvx_file_.is_open()) {
    return false;
  }

  //copy header
  uint64_t header_size = sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader) +
    device_count_ * sizeof(LvxDeviceInfo);
  std::unique_ptr<char[]> write_buffer(new char[header_size]);
  lvx_file_.seekg(0, std::ios::beg);
  lvx_file_.read(write_buffer.get(), header_size);
  re_lvx_file_.write((char*)write_buffer.get(), header_size);

  //copy frames
  uint64_t frame_offset = frame_detail_list_[start_frame_index].start_offset -
    frame_detail_list_[0].start_offset;
  lvx_file_.seekg(frame_detail_list_[start_frame_index].start_offset, std::ios::beg);
  for (uint64_t index = start_frame_index; index < end_frame_index; index++) {
    const auto &frame_detail = frame_detail_list_[index];
    write_buffer.reset(new char[frame_detail.data_len]);
    lvx_file_.read((char*)write_buffer.get(), frame_detail.data_len);
    if (file_type_ == LIVOX_FILE_LVX3_PPT) {
      Lvx3FrameHeader* frame_header = reinterpret_cast<Lvx3FrameHeader*>(write_buffer.get());
      frame_header->current_offset -= frame_offset;
    } else {
      Lvx2FrameHeader* frame_header = reinterpret_cast<Lvx2FrameHeader*>(write_buffer.get());
      frame_header->current_offset -= frame_offset;
      frame_header->frame_index -= start_frame_index;
      frame_header->next_offset -= frame_offset;
    }
    re_lvx_file_.write((char*)write_buffer.get(), frame_detail.data_len);
  }
  re_lvx_file_.close();
  return true;
}

void LvxFileParseHandler::StopParseLvxFile() {
  Clear();
  return;
}

void LvxFileParseHandler::Clear() {
  if (lvx_file_.is_open()) {
    lvx_file_.close();
  }
  device_count_ = 0;
  frame_duration_ = 0;
  lvx_file_size_ = 0;
  memset(&file_version_, 0, sizeof(file_version_));
  file_type_ = LIVOX_FILE_UNKNOWN;
  frame_detail_list_.clear();
  device_infos_.clear();
  ResetPacketList();
}

}
}  // namespace livox
