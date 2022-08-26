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

#include "lvx_file_record_handler.h"

#include <time.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

#include "livox_def_common.h"
#include "livox_sdk_vehicle.h"
#include "livox_sdk.h"
#include "livox_lidar_api.h"
#include "lvx_file_manager.h"

#ifdef WIN32
#include <direct.h>
#include <io.h>
#else
#include <dirent.h>
#include <unistd.h>
#endif // WIN32

#define WRITE_BUFFER_LEN 1024 * 1024
#define MAGIC_CODE       (0xac0ea767)
static constexpr double kPI = 3.14159265358979323846;
constexpr uint32_t kEncodingThreadNum = 6;

namespace livox {
namespace common {

LvxFileRecordHandler &lvx_file_record_handler() {
  static LvxFileRecordHandler handler;
  return handler;
}

LvxFileRecordHandler::LvxFileRecordHandler() {}

bool LvxFileRecordHandler::SetLvxRecordDir(const std::string& dir) {
#ifdef WIN32
  if (_access(dir.c_str(), 0) != 0) {
    return false;
#else
  if (access(dir.c_str(), 0) != 0) {
    return false;
#endif
  }
  file_dir_ = dir;
  return true;
}

bool LvxFileRecordHandler::InitLvxFile(LvxFileType file_type) {
  time_t curtime = time(nullptr);
  char filename[30] = { 0 };
  tm* local_time = localtime(&curtime);
  if (file_type == LIVOX_FILE_LVX3_PPT) {
    strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx3", local_time);
  } else if (file_type == LIVOX_FILE_LVX2){
    strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx2", local_time);
  } else {
    std::cout << "Unsupported file type (" << file_type << ")!" << std::endl;
    return false;
  }
  std::string file_path = file_dir_ + (file_dir_.back() == '/' ? "" : "/") + std::string(filename);

  lvx_file_.open(file_path.c_str(), std::ios::out | std::ios::binary);
  if (!lvx_file_.is_open()) {
    std::cout << "failed to create the file: " << file_path << std::endl;
    return false;
  }
  cur_offset_ = 0;
  total_point_num_ = 0;
  file_type_ = file_type;
  return true;
}

void LvxFileRecordHandler::AddDeviceInfo(std::vector<LvxDeviceInfo> info) {
  Clear();
  device_info_list_ = std::move(info);
  uint8_t lidar_num = device_info_list_.size();
  //Todo jerry.lin hardcode max cache packet number 5000
  point_packet_list_.reserve(lidar_num * 5000);
  packages_vec_.reserve(lidar_num * 5000);
}

void LvxFileRecordHandler::InitLvxFileHeader() {
  uint64_t current_write_offset = 0;
  LvxFilePublicHeader lvx_file_public_header = {};
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  std::string signature = kFileSignature;
  if (file_type_ != LIVOX_FILE_LVX2 && file_type_ != LIVOX_FILE_LVX3_PPT) {
    std::cout << "unsupported file type (" << file_type_ << ")" << std::endl;
  }
  lvx_file_public_header.version[0] = LvxFileManager::GetFileTypeMapping()[file_type_].file_ver_major;
  lvx_file_public_header.version[1] = LvxFileManager::GetFileTypeMapping()[file_type_].file_ver_minor;
  lvx_file_public_header.version[2] = LvxFileManager::GetFileTypeMapping()[file_type_].file_ver_patch;
  lvx_file_public_header.version[3] = LvxFileManager::GetFileTypeMapping()[file_type_].file_ver_build;

  memcpy(lvx_file_public_header.signature, signature.c_str(), signature.size());
  lvx_file_public_header.magic_code = MAGIC_CODE;
  memcpy(write_buffer.get() + current_write_offset, (void *)&lvx_file_public_header, sizeof(LvxFilePublicHeader));
  current_write_offset += sizeof(LvxFilePublicHeader);

  uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
  LvxFilePrivateHeader lvx_file_private_header = { 0 };
  lvx_file_private_header.frame_duration = frame_duration_;
  lvx_file_private_header.device_count = device_count;
  memcpy(write_buffer.get() + current_write_offset, (void *)&lvx_file_private_header, sizeof(LvxFilePrivateHeader));
  current_write_offset += sizeof(LvxFilePrivateHeader);

  for (int i = 0; i < device_count; i++) {
    memcpy(write_buffer.get() + current_write_offset, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
    current_write_offset += sizeof(LvxDeviceInfo);
  }

  lvx_file_.write((char *)write_buffer.get(), current_write_offset);
  cur_offset_ += current_write_offset;
}

void LvxFileRecordHandler::StartRecordLvxFile(const LvxFileType file_type) {
  if (is_stopping_.load()) {
    // mustn't interrupt the stopping procedure
    return;
  }
  bool expected = false;
  if (!is_recording_.compare_exchange_strong(expected, true)) {
    return;
  }
  if (!InitLvxFile(file_type)) {
    return;
  }
  InitLvxFileHeader();

  lvx_save_thread_ = std::make_shared<std::thread>([this]() {
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    is_quit_.store(false);
    uint32_t lidar_num = device_info_list_.size();
    std::vector<LvxBasePackDetail> temp_point_packet_list;
    std::vector<LvxBaseHalfPackDetail> temp_point_half_packet_list;
    std::vector<LvxPackageDetail> temp_package_list;
    while (!is_quit_.load()) {
      {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait_for(lock,
          std::chrono::milliseconds(kDefaultFrameDurationTime) - (std::chrono::steady_clock::now() - last_time));
        last_time = std::chrono::steady_clock::now();
        if (file_type_ == LIVOX_FILE_LVX3_PPT) {
          if (packages_vec_.empty()) {
            continue;
          }
          temp_package_list = std::move(packages_vec_);
          packages_vec_.reserve(lidar_num * 5000);
        } else {
          if (point_packet_list_.empty() && point_half_packet_list_.empty()) {
            continue;
          }
          temp_point_packet_list = std::move(point_packet_list_);
          temp_point_half_packet_list = std::move(point_half_packet_list_);
          point_packet_list_.reserve(lidar_num * 5000);
          point_half_packet_list_.reserve(lidar_num * 5000);
        }
      }
      /*if (file_type_ == LIVOX_FILE_LVX3_PPT) {
        SaveFrameToLvx3File(temp_package_list);
        temp_package_list.clear();
      } else {*/
        SaveFrameToLvx2File(temp_point_packet_list, temp_point_half_packet_list);
        temp_point_packet_list.clear();
        temp_point_half_packet_list.clear();
      //}
    }
  });
  livox_lidar_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this); 
  vehicle_listen_id_ = VehicleLidarAddPointCloudObserver(OnVehicleLidarPointCloudCallback, this);
  listen_id_ = LidarAddPointCloudObserver(OnLidarPointCloudCallback, this);
  
}

void LvxFileRecordHandler::StopRecordLvxFile() {
  if (!is_recording_.load()) {
    return;
  }
  bool expected = false;
  if (!is_stopping_.compare_exchange_strong(expected, true)) {
    return;
  }

  is_quit_.store(true);
  cv_.notify_one();
  if (lvx_save_thread_) {
    lvx_save_thread_->join();
    lvx_save_thread_ = nullptr;
  }

  if (livox_lidar_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(livox_lidar_id_);
    livox_lidar_id_ = 0;
  }

  if (vehicle_listen_id_ > 0) {
    VehicleLidarRemovePointCloudObserver(vehicle_listen_id_);
    vehicle_listen_id_ = 0;
  }
  if (listen_id_ > 0) {
    LidarRemovePointCloudObserver(listen_id_);
    listen_id_ = 0;
  }
  printf("[Livox_SDK_INFO] total frame num: %u, total point num: %lu \n",
    cur_frame_index_, total_point_num_);
  CloseLvxFile();
  Clear();
  is_stopping_.store(false);
  is_recording_.store(false);
}


void LvxFileRecordHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type, 
    LivoxLidarEthernetPacket* data, void* client_data) {
  LvxFileRecordHandler* self = (LvxFileRecordHandler*)client_data;
  if (self->file_type_ == LIVOX_FILE_LVX3_PPT) {
    std::vector<LvxPackageDetail> package_vec;
    self->PackagePointHandle(handle, data, data->dot_num, package_vec);
    {
      std::unique_lock<std::mutex> lock(self->mtx_);
      for (const auto &package : package_vec) {
        self->packages_vec_.push_back(std::move(package));
      }
    }
  } else {
    std::vector<LvxBasePackDetail> packets;
    std::vector<LvxBaseHalfPackDetail> half_packets;
    self->BasePointsHandle(handle, data, data->dot_num, packets, half_packets);
    {
      std::unique_lock<std::mutex> lock(self->mtx_);
      if (data->data_type == VehiclePointDataType::kHighResolutionPointData) {
        for (const auto& packet : packets) {
          self->point_packet_list_.push_back(packet);
        }
      } else if (data->data_type == VehiclePointDataType::kLowResolutionPointData) {
        for (const auto& half_packet : half_packets) {
          self->point_half_packet_list_.push_back(half_packet);
        }
      }
    }
  }
}

// for livox lidar
void LvxFileRecordHandler::PackagePointHandle(uint32_t handle,
    const LivoxLidarEthernetPacket *data, uint32_t data_num, std::vector<LvxPackageDetail> &package_vec) {
  if ((data == nullptr) || (data_num == 0)) {
    // std::cout << "Invalid Eth Packet!" << std::endl;
    return;
  }
  LvxPackageDetail package;
  CopyPackHeader(handle, *data, data_num, package.header);

  package_vec.reserve(data_num / kMaxPointNumber + 1);
  package.packets.reserve(kMaxPointNumber);
  switch (data->data_type) {
    case LivoxLidarPointDataType::kLivoxLidarCartesianCoordinateHighData:
    {
      LivoxLidarCartesianHighRawPoint* point = (LivoxLidarCartesianHighRawPoint*)data->data;
      uint32_t index = 0;
      PACKAGE_POINT pp;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        pp.x = point[i].x;
        pp.y = point[i].y;
        pp.z = point[i].z;
        pp.reflectances = point[i].reflectivity;
        pp.tag = point[i].tag;
        package.packets.push_back(pp);
        if (index == kMaxPointNumber - 1) {
          package.header.length = kMaxPointNumber * sizeof(LivoxLidarCartesianHighRawPoint);
          package_vec.push_back(package);
          package.packets.clear();
        }
      }
      if (index != kMaxPointNumber - 1) {
        package.header.length = (index + 1) * sizeof(LivoxLidarCartesianHighRawPoint);
        package_vec.push_back(package);
      }
      break;
    }
    case LivoxLidarPointDataType::kLivoxLidarCartesianCoordinateLowData:
    {
      LivoxLidarCartesianLowRawPoint* half_point = (LivoxLidarCartesianLowRawPoint*)data->data;
      uint32_t index = 0;
      PACKAGE_POINT pp;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        pp.x = half_point[i].x * 10;
        pp.y = half_point[i].y * 10;
        pp.z = half_point[i].z * 10;
        pp.reflectances = half_point[i].reflectivity;
        pp.tag = half_point[i].tag;
        package.packets.push_back(pp);
        if (index == kMaxPointNumber - 1) {
          package.header.length = kMaxPointNumber * sizeof(LivoxLidarCartesianHighRawPoint);
          package_vec.push_back(package);
          package.packets.clear();
        }
      }
      if (index != kMaxPointNumber - 1) {
        package.header.length = (index + 1) * sizeof(LivoxLidarCartesianHighRawPoint);
        package_vec.push_back(package);
      }
      break;
    }
    default:
      break;
  }
}

// for livox lidar
void LvxFileRecordHandler::BasePointsHandle(uint32_t handle, LivoxLidarEthernetPacket *data, uint32_t data_num,
    std::vector<LvxBasePackDetail> &packets, std::vector<LvxBaseHalfPackDetail>& half_packets) {
  if ((data == nullptr) || (data_num == 0)) {
    // std::cout << "Invalid Eth Packet!" << std::endl;
    return;
  }

  switch (data->data_type) {
    case LivoxLidarPointDataType::kLivoxLidarCartesianCoordinateHighData:
    {
      LvxBasePackDetail packet;
      CopyPackHeader(handle, *data, data_num, packet.header);

      LivoxLidarCartesianHighRawPoint* point = (LivoxLidarCartesianHighRawPoint*)data->data;
      uint32_t index = 0;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        packet.raw_point[index].x = point[i].x;
        packet.raw_point[index].y = point[i].y;
        packet.raw_point[index].z = point[i].z;
        packet.raw_point[index].reflectivity = point[i].reflectivity;
        packet.raw_point[index].tag = point[i].tag;
        if (index == kMaxPointNumber - 1) {
          packet.header.length = kMaxPointNumber * sizeof(LivoxLidarCartesianHighRawPoint);
          packets.push_back(packet);
        }
      }
      if (index != kMaxPointNumber - 1) {
        packet.header.length = (index + 1) * sizeof(LivoxLidarCartesianHighRawPoint);
        packets.push_back(packet);
      }
      break;
    }
    case  LivoxLidarPointDataType::kLivoxLidarCartesianCoordinateLowData:
    {
      LvxBaseHalfPackDetail half_packet;
      CopyPackHeader(handle, *data, data_num, half_packet.header);
      LivoxLidarCartesianLowRawPoint* half_point = (LivoxLidarCartesianLowRawPoint*)data->data;
      uint32_t index = 0;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        half_packet.raw_point[index].x = half_point[i].x * 10;
        half_packet.raw_point[index].y = half_point[i].y * 10;
        half_packet.raw_point[index].z = half_point[i].z * 10;
        half_packet.raw_point[index].reflectivity = half_point[i].reflectivity;
        half_packet.raw_point[index].tag = half_point[i].tag;
        if (index == kMaxPointNumber - 1) {
          half_packet.header.length = kMaxPointNumber * sizeof(LivoxLidarCartesianLowRawPoint);
          half_packets.push_back(half_packet);
        }
      }
      if (index != kMaxPointNumber - 1) {
        half_packet.header.length = (index + 1) * sizeof(LivoxLidarCartesianLowRawPoint);
        half_packets.push_back(half_packet);
      }
      break;
    }
    default:
      break;
  }
}

// for livox lidar
void LvxFileRecordHandler::CopyPackHeader(uint32_t handle,
                                          const LivoxLidarEthernetPacket &eth_packet,
                                          uint32_t data_num,
                                          LvxBasePackHeader &header) {
  header.version = eth_packet.version;
  header.lidar_id = handle;
  header.timestamp_type = eth_packet.time_type;
  header.udp_counter = eth_packet.udp_cnt;
  header.frame_counter = eth_packet.frame_cnt;
  header.data_type = eth_packet.data_type;  // 32bit or 16bit
  header.lidar_type = static_cast<uint8_t>(LivoxLidarType::kLivoxLidarType);
  memcpy(header.timestamp, eth_packet.timestamp, 8 * sizeof(uint8_t));
  return;
}

void LvxFileRecordHandler::OnVehicleLidarPointCloudCallback(uint8_t slot,
                                                            LivoxVehicleEthPacket* data,
                                                            uint32_t data_num,
                                                            void* client_data) {
  LvxFileRecordHandler* self = (LvxFileRecordHandler*)client_data;

  if (self->file_type_ == LIVOX_FILE_LVX3_PPT) {
    std::vector<LvxPackageDetail> package_vec;
    self->PackagePointHandle(slot, data, data_num, package_vec);
    {
      std::unique_lock<std::mutex> lock(self->mtx_);
      for (const auto &package : package_vec) {
        self->packages_vec_.push_back(std::move(package));
      }
    }
  } else {
    std::vector<LvxBasePackDetail> packets;
    std::vector<LvxBaseHalfPackDetail> half_packets;
    self->BasePointsHandle(slot, data, data_num, packets, half_packets);
    {
      std::unique_lock<std::mutex> lock(self->mtx_);
      if (data->data_type == VehiclePointDataType::kHighResolutionPointData) {
        for (const auto& packet : packets) {
          self->point_packet_list_.push_back(packet);
        }
      } else if (data->data_type == VehiclePointDataType::kLowResolutionPointData) {
        for (const auto& half_packet : half_packets) {
          self->point_half_packet_list_.push_back(half_packet);
        }
      }
    }
  }
}

// for automotive lidar
void LvxFileRecordHandler::PackagePointHandle(uint8_t slot,
                                              const LivoxVehicleEthPacket *data,
                                              uint32_t data_num,
                                              std::vector<LvxPackageDetail> &package_vec) {
  if ((data == nullptr) || (data_num == 0)) {
    // std::cout << "Invalid Eth Packet!" << std::endl;
    return;
  }
  LvxPackageDetail package;
  CopyPackHeader(slot, *data, data_num, package.header);

  package_vec.reserve(data_num / kMaxPointNumber + 1);
  package.packets.reserve(kMaxPointNumber);
  switch (data->data_type) {
    case VehiclePointDataType::kHighResolutionPointData:
    {
      LivoxVehicleExtendRawPoint* point = (LivoxVehicleExtendRawPoint*)data->data;
      uint32_t index = 0;
      PACKAGE_POINT pp;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        pp.x = point[i].x;
        pp.y = point[i].y;
        pp.z = point[i].z;
        pp.reflectances = point[i].reflectivity;
        pp.tag = point[i].tag;
        package.packets.push_back(pp);
        if (index == kMaxPointNumber - 1) {
          package.header.length = kMaxPointNumber * sizeof(LivoxVehicleExtendRawPoint);
          package_vec.push_back(package);
          package.packets.clear();
        }
      }
      if (index != kMaxPointNumber - 1) {
        package.header.length = (index + 1) * sizeof(LivoxVehicleExtendRawPoint);
        package_vec.push_back(package);
      }
      break;
    }
    case VehiclePointDataType::kLowResolutionPointData:
    {
      LivoxVehicleExtendHalfRawPoint* half_point = (LivoxVehicleExtendHalfRawPoint*)data->data;
      uint32_t index = 0;
      PACKAGE_POINT pp;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        pp.x = half_point[i].x * 10;
        pp.y = half_point[i].y * 10;
        pp.z = half_point[i].z * 10;
        pp.reflectances = half_point[i].reflectivity;
        pp.tag = half_point[i].tag;
        package.packets.push_back(pp);
        if (index == kMaxPointNumber - 1) {
          package.header.length = kMaxPointNumber * sizeof(LivoxVehicleExtendRawPoint);
          package_vec.push_back(package);
          package.packets.clear();
        }
      }
      if (index != kMaxPointNumber - 1) {
        package.header.length = (index + 1) * sizeof(LivoxVehicleExtendRawPoint);
        package_vec.push_back(package);
      }
      break;
    }
    default:
      break;
  }
}

// for automotive lidar
void LvxFileRecordHandler::BasePointsHandle(uint8_t slot, LivoxVehicleEthPacket *data, uint32_t data_num,
    std::vector<LvxBasePackDetail> &packets, std::vector<LvxBaseHalfPackDetail>& half_packets) {
  if ((data == nullptr) || (data_num == 0)) {
    // std::cout << "Invalid Eth Packet!" << std::endl;
    return;
  }

  switch (data->data_type) {
    case VehiclePointDataType::kHighResolutionPointData:
    {
      LvxBasePackDetail packet;
      CopyPackHeader(slot, *data, data_num, packet.header);

      LivoxVehicleExtendRawPoint* point = (LivoxVehicleExtendRawPoint*)data->data;
      uint32_t index = 0;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        packet.raw_point[index].x = point[i].x;
        packet.raw_point[index].y = point[i].y;
        packet.raw_point[index].z = point[i].z;
        packet.raw_point[index].reflectivity = point[i].reflectivity;
        packet.raw_point[index].tag = point[i].tag;
        if (index == kMaxPointNumber - 1) {
          packet.header.length = kMaxPointNumber * sizeof(LivoxVehicleExtendRawPoint);
          packets.push_back(packet);
        }
      }
      if (index != kMaxPointNumber - 1) {
        packet.header.length = (index + 1) * sizeof(LivoxVehicleExtendRawPoint);
        packets.push_back(packet);
      }
      break;
    }
    case VehiclePointDataType::kLowResolutionPointData:
    {
      LvxBaseHalfPackDetail half_packet;
      CopyPackHeader(slot, *data, data_num, half_packet.header);
      LivoxVehicleExtendHalfRawPoint* half_point = (LivoxVehicleExtendHalfRawPoint*)data->data;
      uint32_t index = 0;
      for (uint32_t i = 0; i < data_num; i++) {
        index = i % kMaxPointNumber;
        half_packet.raw_point[index].x = half_point[i].x * 10;
        half_packet.raw_point[index].y = half_point[i].y * 10;
        half_packet.raw_point[index].z = half_point[i].z * 10;
        half_packet.raw_point[index].reflectivity = half_point[i].reflectivity;
        half_packet.raw_point[index].tag = half_point[i].tag;
        if (index == kMaxPointNumber - 1) {
          half_packet.header.length = kMaxPointNumber * sizeof(LivoxVehicleExtendHalfRawPoint);
          half_packets.push_back(half_packet);
        }
      }
      if (index != kMaxPointNumber - 1) {
        half_packet.header.length = (index + 1) * sizeof(LivoxVehicleExtendHalfRawPoint);
        half_packets.push_back(half_packet);
      }
      break;
    }
    default:
      break;
  }
}

// for automotive lidar
void LvxFileRecordHandler::CopyPackHeader(uint8_t slot,
                                          const LivoxVehicleEthPacket &eth_packet,
                                          uint32_t data_num,
                                          LvxBasePackHeader &header) {
  header.version = eth_packet.version;
  header.lidar_id = slot;
  header.timestamp_type = eth_packet.timestamp_type;
  header.udp_counter = 0;//eth_packet.udp_counter;
  header.frame_counter = eth_packet.frame_counter;
  header.data_type = eth_packet.data_type;  // 32bit or 16bit
  header.lidar_type = static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType);
  memcpy(header.timestamp, eth_packet.timestamp, 8 * sizeof(uint8_t));
  return;
}

void LvxFileRecordHandler::OnLidarPointCloudCallback(uint8_t handle,
                                                     LivoxEthPacket* data,
                                                     uint32_t data_num,
                                                     void* client_data) {
  LvxFileRecordHandler* self = (LvxFileRecordHandler*)client_data;
  if (self->file_type_ == LIVOX_FILE_LVX3_PPT) {
    LvxPackageDetail package;
    self->PackagePointHandle(handle, data, data_num, package);
    {
      std::unique_lock<std::mutex> lock(self->mtx_);
      if (package.packets.size() > 0) {
        self->packages_vec_.push_back(std::move(package));
      }
    }
  } else {
    std::vector<LvxBasePackDetail> packets;
    self->BasePointsHandle(handle, data, data_num, packets);
    {
      std::unique_lock<std::mutex> lock(self->mtx_);
      for (const auto& packet : packets) {
        self->point_packet_list_.push_back(std::move(packet));
      }
    }
  }
}

// for industrial lidar
void LvxFileRecordHandler::PackagePointHandle(uint8_t handle,
                                              const LivoxEthPacket *data,
                                              uint32_t data_num,
                                              LvxPackageDetail &package) {
  if ((data == nullptr) || (data_num == 0)) {
    // std::cout << "Invalid Eth Packet!" << std::endl;
    return;
  }
  CopyPackHeader(handle, *data, data_num, package.header);

  package.packets.clear();
  package.packets.reserve(data_num);
  switch (data->data_type) {
    case PointDataType::kCartesian:
    {
      package.header.data_type = 1;
      LivoxRawPoint* point = (LivoxRawPoint*)data->data;
      package.packets.resize(data_num);
      for (uint32_t i = 0; i < data_num; i++) {
        package.packets[i].x = point[i].x;
        package.packets[i].y = point[i].y;
        package.packets[i].z = point[i].z;
        package.packets[i].reflectances = point[i].reflectivity;
        package.packets[i].tag = 0;
      }
      break;
    }
    case PointDataType::kSpherical:
    {
      package.header.data_type = 1;
      LivoxSpherPoint* point = (LivoxSpherPoint*)data->data;
      package.packets.resize(data_num);
      for (uint32_t i = 0; i < data_num; i++) {
        double radius = point[i].depth;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        package.packets[i].x = radius * sin(theta) * cos(phi);
        package.packets[i].y = radius * sin(theta) * sin(phi);
        package.packets[i].z = radius * cos(theta);
        package.packets[i].reflectances = point[i].reflectivity;
        package.packets[i].tag = 0;
      }
      break;
    }
    case PointDataType::kExtendCartesian:
    {
      package.header.data_type = 1;
      LivoxExtendRawPoint* point = (LivoxExtendRawPoint*)data->data;
      package.packets.resize(data_num);
      for (uint32_t i = 0; i < data_num; i++) {
        package.packets[i].x = point[i].x;
        package.packets[i].y = point[i].y;
        package.packets[i].z = point[i].z;
        package.packets[i].reflectances = point[i].reflectivity;
        package.packets[i].tag = point[i].tag;
      }
      break;
    }
    case PointDataType::kExtendSpherical:
    {
      package.header.data_type = 1;
      LivoxExtendSpherPoint* point = (LivoxExtendSpherPoint*)data->data;
      package.packets.resize(data_num);
      for (uint32_t i = 0; i < data_num; i++) {
        double radius = point[i].depth;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        package.packets[i].x = radius * sin(theta) * cos(phi);
        package.packets[i].y = radius * sin(theta) * sin(phi);
        package.packets[i].z = radius * cos(theta);
        package.packets[i].reflectances = point[i].reflectivity;
        package.packets[i].tag = point[i].tag;
      }
      break;
    }
    case PointDataType::kDualExtendCartesian:
    {
      package.header.data_type = 2;
      LivoxDualExtendRawPoint* point = (LivoxDualExtendRawPoint*)data->data;
      package.packets.resize(data_num * 2);
      for (uint32_t i = 0; i < data_num; i++) {
        package.packets[2*i].x = point[i].x1;
        package.packets[2*i].y = point[i].y1;
        package.packets[2*i].z = point[i].z1;
        package.packets[2*i].reflectances = point[i].reflectivity1;
        package.packets[2*i].tag = point[i].tag1;
        package.packets[2*i+1].x = point[i].x2;
        package.packets[2*i+1].y = point[i].y2;
        package.packets[2*i+1].z = point[i].z2;
        package.packets[2*i+1].reflectances = point[i].reflectivity2;
        package.packets[2*i+1].tag = point[i].tag2;
      }
      break;
    }
    case PointDataType::kDualExtendSpherical:
    {
      package.header.data_type = 2;
      LivoxDualExtendSpherPoint* point = (LivoxDualExtendSpherPoint*)data->data;
      package.packets.resize(data_num * 2);
      for (uint32_t i = 0; i < data_num; i++) {
        double radius1 = point[i].depth1;
        double radius2 = point[i].depth2;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        package.packets[2*i].x = radius1 * sin(theta) * cos(phi);
        package.packets[2*i].y = radius1 * sin(theta) * sin(phi);
        package.packets[2*i].z = radius1 * cos(theta);
        package.packets[2*i].reflectances = point[i].reflectivity1;
        package.packets[2*i].tag = point[i].tag1;
        package.packets[2*i+1].x = radius2 * sin(theta) * cos(phi);
        package.packets[2*i+1].y = radius2 * sin(theta) * sin(phi);
        package.packets[2*i+1].z = radius2 * cos(theta);
        package.packets[2*i+1].reflectances = point[i].reflectivity2;
        package.packets[2*i+1].tag = point[i].tag2;
      }
      break;
    }
    case PointDataType::kTripleExtendCartesian:
    {
      package.header.data_type = 3;
      LivoxTripleExtendRawPoint* point = (LivoxTripleExtendRawPoint*)data->data;
      package.packets.resize(data_num * 3);
      for (uint32_t i = 0; i < data_num; i++) {
        package.packets[3*i].x = point[i].x1;
        package.packets[3*i].y = point[i].y1;
        package.packets[3*i].z = point[i].z1;
        package.packets[3*i].reflectances = point[i].reflectivity1;
        package.packets[3*i].tag = point[i].tag1;
        package.packets[3*i+1].x = point[i].x2;
        package.packets[3*i+1].y = point[i].y2;
        package.packets[3*i+1].z = point[i].z2;
        package.packets[3*i+1].reflectances = point[i].reflectivity2;
        package.packets[3*i+1].tag = point[i].tag2;
        package.packets[3*i+2].x = point[i].x3;
        package.packets[3*i+2].y = point[i].y3;
        package.packets[3*i+2].z = point[i].z3;
        package.packets[3*i+2].reflectances = point[i].reflectivity3;
        package.packets[3*i+2].tag = point[i].tag3;
      }
      break;
    }
    case PointDataType::kTripleExtendSpherical:
    {
      package.header.data_type = 3;
      LivoxTripleExtendSpherPoint* point = (LivoxTripleExtendSpherPoint*)data->data;
      package.packets.resize(data_num * 3);
      for (uint32_t i = 0; i < data_num; i++) {
        double radius1 = point[i].depth1;
        double radius2 = point[i].depth2;
        double radius3 = point[i].depth3;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        package.packets[3*i].x = radius1 * sin(theta) * cos(phi);
        package.packets[3*i].y = radius1 * sin(theta) * sin(phi);
        package.packets[3*i].z = radius1 * cos(theta);
        package.packets[3*i].reflectances = point[i].reflectivity1;
        package.packets[3*i].tag = point[i].tag1;
        package.packets[3*i+1].x = radius2 * sin(theta) * cos(phi);
        package.packets[3*i+1].y = radius2 * sin(theta) * sin(phi);
        package.packets[3*i+1].z = radius2 * cos(theta);
        package.packets[3*i+1].reflectances = point[i].reflectivity2;
        package.packets[3*i+1].tag = point[i].tag2;
        package.packets[3*i+2].x = radius3 * sin(theta) * cos(phi);
        package.packets[3*i+2].y = radius3 * sin(theta) * sin(phi);
        package.packets[3*i+2].z = radius3 * cos(theta);
        package.packets[3*i+2].reflectances = point[i].reflectivity3;
        package.packets[3*i+2].tag = point[i].tag3;
      }
      break;
    }
    default:
      break;
  }
}

// for industrial lidar
void LvxFileRecordHandler::BasePointsHandle(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, std::vector<LvxBasePackDetail> &packets) {
  if ((data == nullptr) || (data_num == 0)) {
    // std::cout << "Invalid Eth Packet!" << std::endl;
    return;
  }
  LvxBasePackDetail packet;
  CopyPackHeader(handle, *data, data_num, packet.header);

  switch (data->data_type) {
    case PointDataType::kCartesian:
    {
      packet.header.data_type = 1;
      LivoxRawPoint* point = (LivoxRawPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        packet.raw_point[i].x = point[i].x;
        packet.raw_point[i].y = point[i].y;
        packet.raw_point[i].z = point[i].z;
        packet.raw_point[i].reflectivity = point[i].reflectivity;
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kSpherical:
    {
      packet.header.data_type = 1;
      LivoxSpherPoint* point = (LivoxSpherPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        double radius = point[i].depth;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        packet.raw_point[i].x = radius * sin(theta) * cos(phi);
        packet.raw_point[i].y = radius * sin(theta) * sin(phi);
        packet.raw_point[i].z = radius * cos(theta);
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kExtendCartesian:
    {
      packet.header.data_type = 1;
      LivoxExtendRawPoint* point = (LivoxExtendRawPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        packet.raw_point[i].x = point[i].x;
        packet.raw_point[i].y = point[i].y;
        packet.raw_point[i].z = point[i].z;
        packet.raw_point[i].reflectivity = point[i].reflectivity;
        packet.raw_point[i].tag = point[i].tag;
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kExtendSpherical:
    {
      packet.header.data_type = 1;
      LivoxExtendSpherPoint* point = (LivoxExtendSpherPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        double radius = point[i].depth;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        packet.raw_point[i].x = radius * sin(theta) * cos(phi);
        packet.raw_point[i].y = radius * sin(theta) * sin(phi);
        packet.raw_point[i].z = radius * cos(theta);
        packet.raw_point[i].reflectivity = point[i].reflectivity;
        packet.raw_point[i].tag = point[i].tag;
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kDualExtendCartesian:
    {
      packet.header.data_type = 2;
      LivoxDualExtendRawPoint* point = (LivoxDualExtendRawPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        packet.raw_point[2*i].x = point[i].x1;
        packet.raw_point[2*i].y = point[i].y1;
        packet.raw_point[2*i].z = point[i].z1;
        packet.raw_point[2*i].reflectivity = point[i].reflectivity1;
        packet.raw_point[2*i].tag = point[i].tag1;
        packet.raw_point[2*i+1].x = point[i].x2;
        packet.raw_point[2*i+1].y = point[i].y2;
        packet.raw_point[2*i+1].z = point[i].z2;
        packet.raw_point[2*i+1].reflectivity = point[i].reflectivity2;
        packet.raw_point[2*i+1].tag = point[i].tag2;
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kDualExtendSpherical:
    {
      packet.header.data_type = 2;
      LivoxDualExtendSpherPoint* point = (LivoxDualExtendSpherPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        double radius1 = point[i].depth1;
        double radius2 = point[i].depth2;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        packet.raw_point[2*i].x = radius1 * sin(theta) * cos(phi);
        packet.raw_point[2*i].y = radius1 * sin(theta) * sin(phi);
        packet.raw_point[2*i].z = radius1 * cos(theta);
        packet.raw_point[2*i].reflectivity = point[i].reflectivity1;
        packet.raw_point[2*i].tag = point[i].tag1;
        packet.raw_point[2*i+1].x = radius2 * sin(theta) * cos(phi);
        packet.raw_point[2*i+1].y = radius2 * sin(theta) * sin(phi);
        packet.raw_point[2*i+1].z = radius2 * cos(theta);
        packet.raw_point[2*i+1].reflectivity = point[i].reflectivity2;
        packet.raw_point[2*i+1].tag = point[i].tag2;
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kTripleExtendCartesian:
    {
      packet.header.data_type = 3;
      LivoxTripleExtendRawPoint* point = (LivoxTripleExtendRawPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        packet.raw_point[3*i].x = point[i].x1;
        packet.raw_point[3*i].y = point[i].y1;
        packet.raw_point[3*i].z = point[i].z1;
        packet.raw_point[3*i].reflectivity = point[i].reflectivity1;
        packet.raw_point[3*i].tag = point[i].tag1;
        packet.raw_point[3*i+1].x = point[i].x2;
        packet.raw_point[3*i+1].y = point[i].y2;
        packet.raw_point[3*i+1].z = point[i].z2;
        packet.raw_point[3*i+1].reflectivity = point[i].reflectivity2;
        packet.raw_point[3*i+1].tag = point[i].tag2;
        packet.raw_point[3*i+2].x = point[i].x3;
        packet.raw_point[3*i+2].y = point[i].y3;
        packet.raw_point[3*i+2].z = point[i].z3;
        packet.raw_point[3*i+2].reflectivity = point[i].reflectivity3;
        packet.raw_point[3*i+2].tag = point[i].tag3;
      }
      packets.push_back(packet);
      break;
    }
    case PointDataType::kTripleExtendSpherical:
    {
      packet.header.data_type = 3;
      LivoxTripleExtendSpherPoint* point = (LivoxTripleExtendSpherPoint*)data->data;
      for (uint32_t i = 0; i < data_num; i++) {
        double radius1 = point[i].depth1;
        double radius2 = point[i].depth2;
        double radius3 = point[i].depth3;
        double theta = point[i].theta / 100.0 / 180 * kPI;
        double phi = point[i].phi / 100.0 / 180 * kPI;
        packet.raw_point[3*i].x = radius1 * sin(theta) * cos(phi);
        packet.raw_point[3*i].y = radius1 * sin(theta) * sin(phi);
        packet.raw_point[3*i].z = radius1 * cos(theta);
        packet.raw_point[3*i].reflectivity = point[i].reflectivity1;
        packet.raw_point[3*i].tag = point[i].tag1;
        packet.raw_point[3*i+1].x = radius2 * sin(theta) * cos(phi);
        packet.raw_point[3*i+1].y = radius2 * sin(theta) * sin(phi);
        packet.raw_point[3*i+1].z = radius2 * cos(theta);
        packet.raw_point[3*i+1].reflectivity = point[i].reflectivity2;
        packet.raw_point[3*i+1].tag = point[i].tag2;
        packet.raw_point[3*i+2].x = radius3 * sin(theta) * cos(phi);
        packet.raw_point[3*i+2].y = radius3 * sin(theta) * sin(phi);
        packet.raw_point[3*i+2].z = radius3 * cos(theta);
        packet.raw_point[3*i+2].reflectivity = point[i].reflectivity3;
        packet.raw_point[3*i+2].tag = point[i].tag3;
      }
      packets.push_back(packet);
      break;
    }
    default:
      break;
  }
}

// for industial lidar
void LvxFileRecordHandler::CopyPackHeader(uint8_t handle,
                                          const LivoxEthPacket &eth_packet,
                                          uint32_t data_num,
                                          LvxBasePackHeader &header) {
  header.version = eth_packet.version;
  header.lidar_id = handle;
  header.timestamp_type = eth_packet.timestamp_type;
  header.frame_counter = 0;
  header.length = data_num * sizeof(LivoxVehicleExtendRawPoint);
  header.lidar_type = static_cast<uint8_t>(LivoxLidarType::kIndustryLidarType);
  memcpy(header.timestamp, eth_packet.timestamp, 8 * sizeof(uint8_t));
  return;
}

void LvxFileRecordHandler::SaveFrameToLvx2File(const std::vector<LvxBasePackDetail>& point_packet_list,
    const std::vector<LvxBaseHalfPackDetail>& point_half_packet_list) {
  Lvx2FrameHeader frame_header = {};
  frame_header.current_offset = cur_offset_;
  frame_header.next_offset = cur_offset_ + sizeof(Lvx2FrameHeader);

  for (auto iterator = point_packet_list.begin(); iterator != point_packet_list.end(); iterator++) {
    total_point_num_ += static_cast<uint64_t>(iterator->header.length / sizeof(LivoxVehicleExtendRawPoint));
    uint64_t length = iterator->header.length + sizeof(LvxBasePackHeader);
    frame_header.next_offset += length;
  }

  for (auto it = point_half_packet_list.begin(); it != point_half_packet_list.end(); ++it) {
    total_point_num_ += static_cast<uint64_t>(it->header.length / sizeof(LivoxVehicleExtendHalfRawPoint));
    uint64_t length = it->header.length + sizeof(LvxBasePackHeader);
    frame_header.next_offset += length;
  }
  frame_header.frame_index = cur_frame_index_;

  uint64_t cur_pos = 0;
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

  memcpy(write_buffer.get() + cur_pos, (void*)&frame_header, sizeof(Lvx2FrameHeader));
  cur_pos += sizeof(Lvx2FrameHeader);

  for (auto iter = point_packet_list.begin(); iter != point_packet_list.end(); iter++) {
    uint64_t length = iter->header.length + sizeof(LvxBasePackHeader);
    if (cur_pos + length >= WRITE_BUFFER_LEN) {
      lvx_file_.write((char*)write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), length);
      cur_pos += length;
    } else {
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), length);
      cur_pos += length;
    }
  }
  lvx_file_.write((char*)write_buffer.get(), cur_pos);

  cur_pos = 0;
  for (auto iter = point_half_packet_list.begin(); iter != point_half_packet_list.end(); iter++) {
    uint64_t length = iter->header.length + sizeof(LvxBasePackHeader);
    if (cur_pos + length >= WRITE_BUFFER_LEN) {
      lvx_file_.write((char*)write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), length);
      cur_pos += length;
    } else {
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), length);
      cur_pos += length;
    }
  }
  lvx_file_.write((char*)write_buffer.get(), cur_pos);

  cur_offset_ = frame_header.next_offset;
  cur_frame_index_++;
}

void LvxFileRecordHandler::SaveFrameToLvx3File(const std::vector<std::vector<PACKAGE_POINT>> &package_vec,
                                               const std::vector<LvxBasePackHeader> &header_vec) {
  if (package_vec.size() != header_vec.size()) {
    // std::cout << "mismatching of the packages size and the headers size!" << std::endl;
    return;
  }

  uint32_t packages_num = package_vec.size();
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  uint64_t cur_pos = 0; // indicate the buffer write pointer
  // construct timestamp vec
  std::vector<uint64_t> timestamp_vec;
  timestamp_vec.reserve(packages_num);
  uint64_t timestamp_tmp;
  for (const auto& header : header_vec) {
    memcpy(&timestamp_tmp, header.timestamp, sizeof(header.timestamp));
    timestamp_vec.push_back(timestamp_tmp);
  }

  // encoding
  unsigned char* encoded_data = nullptr;
  uint64_t encoded_size = 0;
  //PPTEncode(package_vec, timestamp_vec, encoded_data, encoded_size, kEncodingThreadNum);
  if (encoded_size == 0) {
    std::cout << "failed to encode!" << std::endl;
    return;
  }

  // construct the frame header
  Lvx3FrameHeader frame_header = {};
  frame_header.current_offset = cur_offset_;
  frame_header.packets_num = packages_num;
  frame_header.data_size = sizeof(Lvx3FrameHeader) + packages_num * sizeof(LvxBasePackHeader) + encoded_size;
  memcpy(write_buffer.get() + cur_pos, (void*)&frame_header, sizeof(Lvx3FrameHeader));
  cur_pos += sizeof(Lvx3FrameHeader);
  // write the headers
  for (const auto& package_header : header_vec) {
    if (cur_pos + sizeof(LvxBasePackHeader) >= WRITE_BUFFER_LEN) {
      lvx_file_.write(write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void*)&package_header, sizeof(LvxBasePackHeader));
      cur_pos += sizeof(LvxBasePackHeader);
    } else {
      memcpy(write_buffer.get() + cur_pos, (void*)&package_header, sizeof(LvxBasePackHeader));
      cur_pos += sizeof(LvxBasePackHeader);
    }
  }
  lvx_file_.write(write_buffer.get(), cur_pos);
  cur_pos = 0;
  // write encoded data
  uint64_t data_cur_pos = 0;
  while (data_cur_pos < encoded_size) {
    if ((encoded_size - data_cur_pos) > WRITE_BUFFER_LEN) {
      lvx_file_.write((char*)(encoded_data) + data_cur_pos, WRITE_BUFFER_LEN);
      data_cur_pos += WRITE_BUFFER_LEN;
    } else {
      lvx_file_.write((char*)(encoded_data) + data_cur_pos, encoded_size - data_cur_pos);
      data_cur_pos = encoded_size;
    }
  }
  free(encoded_data);

  cur_offset_ += frame_header.data_size;
  cur_frame_index_++;
}

void LvxFileRecordHandler::SaveFrameToLvx3File(const std::vector<LvxPackageDetail> &packages_detail_vec) {
  uint64_t package_num = packages_detail_vec.size();
  std::vector<std::vector<PACKAGE_POINT>> packages_vec;
  std::vector<LvxBasePackHeader> headers_vec;
  packages_vec.reserve(package_num);
  headers_vec.reserve(package_num);

  for (auto & package_detail : packages_detail_vec) {
    packages_vec.push_back(std::move(package_detail.packets));
    headers_vec.push_back(package_detail.header);
    total_point_num_ += package_detail.packets.size();
  }
  SaveFrameToLvx3File(packages_vec, headers_vec);
}

void LvxFileRecordHandler::Clear() {
  cur_frame_index_ = 0;
  frame_duration_ = kDefaultFrameDurationTime;
  cur_offset_ = 0;
  total_point_num_ = 0;
  device_info_list_.clear();
  point_packet_list_.clear();
  packages_vec_.clear();
}

void LvxFileRecordHandler::CloseLvxFile() {
  if (lvx_file_.is_open()) {
    lvx_file_.close();
  }
}

} // namespace common
} // namespace livox