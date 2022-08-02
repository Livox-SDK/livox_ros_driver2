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

#include <time.h>
#include <math.h>
#include <map>
#include <cstring>
#include "lvx_to_point_cloud_frame.h"
#include "livox_sdk_common_util.h"
#include "../lvx_handler/lvx_file_parse_handler.h"

namespace livox {
namespace common {

LvxToPointCloudFrameConverter &lvx_to_point_cloud_frame_converter() {
  static LvxToPointCloudFrameConverter handler;
  return handler;
}  

bool LvxToPointCloudFrameConverter::SetLvxConvertDir(const std::string &dir) {
    return lvx_file_parse_handler().SetLvxParseDir(dir);
}

void LvxToPointCloudFrameConverter::UpdateExtrinsic() {
    for (auto& device_info : device_info_map_) {
        auto id = device_info.first;
        auto param = device_info.second;
        extrinsics_[id] = ExtrinsicParameters();
        auto& extrinsic = extrinsics_[id];
        extrinsic.trans[0] = param.x;
        extrinsic.trans[1] = param.y;
        extrinsic.trans[2] = param.z;

        double cos_roll = cos(static_cast<double>(param.roll * PI / 180.0));
        double cos_pitch = cos(static_cast<double>(param.pitch * PI / 180.0));
        double cos_yaw = cos(static_cast<double>(param.yaw * PI / 180.0));
        double sin_roll = sin(static_cast<double>(param.roll * PI / 180.0));
        double sin_pitch = sin(static_cast<double>(param.pitch * PI / 180.0));
        double sin_yaw = sin(static_cast<double>(param.yaw * PI / 180.0));

        extrinsic.rotation[0][0] = cos_pitch * cos_yaw;
        extrinsic.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
        extrinsic.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

        extrinsic.rotation[1][0] = cos_pitch * sin_yaw;
        extrinsic.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
        extrinsic.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

        extrinsic.rotation[2][0] = -sin_pitch;
        extrinsic.rotation[2][1] = sin_roll * cos_pitch;
        extrinsic.rotation[2][2] = cos_roll * cos_pitch;
    }
}

void LvxToPointCloudFrameConverter::PointCloudsConvert(const LvxBasePackDetail& packet) {
    auto header = packet.header;
    uint8_t slot = header.lidar_id;
    uint8_t lidar_type = header.lidar_type;
    uint8_t data_type = header.data_type;
    
    uint32_t id = 0;
    GetLidarId(lidar_type, slot, id);
    uint8_t product_type = device_info_map_[id].device_type;
    uint64_t time_stamp;
    if (lidar_type == static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType)) {
        time_stamp = 
            GetVehicleEthPacketTimestamp(header.timestamp_type, header.timestamp, sizeof(header.timestamp));
    } else {
        time_stamp = 
            GetVehicleEthPacketTimestamp(header.timestamp_type, header.timestamp, sizeof(header.timestamp));
    }
    uint8_t line_num = GetLaserLineNumber(product_type);
    uint64_t point_interval = GetPointInterval(product_type);
    uint32_t point_number = packet.header.length / sizeof(LivoxExtendRawPoint);
    auto extrinsic = extrinsics_[id];
    LivoxExtendRawPoint *raw = (LivoxExtendRawPoint *)packet.raw_point;
    PointCloudXyzlt point = {};
    for (uint32_t i = 0; i < point_number; i++) {
        point.x = (raw[i].x * extrinsic.rotation[0][0] +
                raw[i].y * extrinsic.rotation[0][1] +
                raw[i].z * extrinsic.rotation[0][2] + extrinsic.trans[0]) / 1000.0;
        point.y = (raw[i].x* extrinsic.rotation[1][0] +
                raw[i].y * extrinsic.rotation[1][1] +
                raw[i].z * extrinsic.rotation[1][2] + extrinsic.trans[1]) / 1000.0;
        point.z = (raw[i].x * extrinsic.rotation[2][0] +
                raw[i].y * extrinsic.rotation[2][1] +
                raw[i].z * extrinsic.rotation[2][2] + extrinsic.trans[2]) / 1000.0;
        point.intensity = raw[i].reflectivity;
        
        point.line = i % line_num % data_type;
        point.tag = raw[i].tag;
        point.offset_time = time_stamp + i * point_interval % data_type;
        point_clouds_[id].push_back(point);
    }
}


void LvxToPointCloudFrameConverter::HalfPointCloudsConvert(const LvxBaseHalfPackDetail& packet) {
    auto header = packet.header;
    uint8_t slot = header.lidar_id;
    uint8_t lidar_type = header.lidar_type;
    uint8_t data_type = header.data_type;
    
    uint32_t id = 0;
    GetLidarId(lidar_type, slot, id);
    uint8_t product_type = device_info_map_[id].device_type;
    uint64_t time_stamp;
    if (lidar_type == static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType)) {
        time_stamp = 
            GetVehicleEthPacketTimestamp(header.timestamp_type, header.timestamp, sizeof(header.timestamp));
    } else {
        time_stamp = 
            GetVehicleEthPacketTimestamp(header.timestamp_type, header.timestamp, sizeof(header.timestamp));
    }
    uint8_t line_num = GetLaserLineNumber(product_type);
    uint64_t point_interval = GetPointInterval(product_type);
    uint32_t point_number = packet.header.length / sizeof(LivoxExtendRawPoint);
    auto extrinsic = extrinsics_[id];
    LivoxExtendRawPoint *raw = (LivoxExtendRawPoint *)packet.raw_point;
    PointCloudXyzlt point = {};
    for (uint32_t i = 0; i < point_number; i++) {
        point.x = (raw[i].x * extrinsic.rotation[0][0] +
                raw[i].y * extrinsic.rotation[0][1] +
                raw[i].z * extrinsic.rotation[0][2] + extrinsic.trans[0]) / 1000.0;
        point.y = (raw[i].x* extrinsic.rotation[1][0] +
                raw[i].y * extrinsic.rotation[1][1] +
                raw[i].z * extrinsic.rotation[1][2] + extrinsic.trans[1]) / 1000.0;
        point.z = (raw[i].x * extrinsic.rotation[2][0] +
                raw[i].y * extrinsic.rotation[2][1] +
                raw[i].z * extrinsic.rotation[2][2] + extrinsic.trans[2]) / 1000.0;
        point.intensity = raw[i].reflectivity;
        
        point.line = i % line_num % data_type;
        point.tag = raw[i].tag;
        point.offset_time = time_stamp + i * point_interval % data_type;
        point_clouds_[id].push_back(point);
    }
}

void LvxToPointCloudFrameConverter::ConvertToPointFrame(uint8_t publish_frequency, 
    PointCloudsFrameCallback cb, void * client_data) {
    callback_ =  cb;
    client_data_ = client_data;
    converter_thread_ = std::make_shared<std::thread>([this, publish_frequency] {
        if (!lvx_file_parse_handler().StartParseLvxFile()) {
            if (callback_) {
                callback_(0, 0, nullptr, client_data_);
            }
            return;
        }
        std::vector<LvxDeviceInfo> info;
        //Get lidar info
        lvx_file_parse_handler().GetLvxDeviceInfoList(info);
        device_info_map_.clear();
        for (size_t count = 0; count < info.size(); count++) {
            uint8_t lidar_id = info[count].lidar_id;
            uint8_t lidar_type = info[count].lidar_type;
            uint32_t id = 0;
            GetLidarId(lidar_type, lidar_id, id);
            device_info_map_[id] = info[count];
        }
        //Update Extrinsic
        UpdateExtrinsic();
        //Get frame detail of lvx file
        uint32_t total_frame_count = lvx_file_parse_handler().GetFrameCount();
        point_clouds_.clear();
        //uint32_t packet_num = 0;
        for (uint64_t frame_index = 0; frame_index < total_frame_count; frame_index++) {
            LvxBasePackDetail* lvx_pack_detail = nullptr;
            uint32_t packet_num = 0;
            LvxBaseHalfPackDetail* lvx_half_pack_detail = nullptr;
            uint32_t half_packet_num = 0;
            lvx_file_parse_handler().GetPointData(frame_index, lvx_pack_detail, packet_num, lvx_half_pack_detail, half_packet_num);
            for (uint64_t i = 0; i < packet_num; i++) {
                PointCloudsConvert(lvx_pack_detail[i]);
            }
            for (uint64_t i = 0; i < half_packet_num; i++) {
                HalfPointCloudsConvert(lvx_half_pack_detail[i]);
            }

            //1 frame = 50ms = 20Hz
            uint8_t times = 20 / publish_frequency;
            if ((frame_index + 1) % times == 0) {
                PublishPointCloudFrame(frame_index / times, total_frame_count / times);
            } else if (frame_index == total_frame_count - 1) {
                PublishPointCloudFrame(total_frame_count / times, total_frame_count / times);
            }
        }
        lvx_file_parse_handler().StopParseLvxFile();
    });
}

void LvxToPointCloudFrameConverter::SetLidarsOffsetTime(uint64_t base_time) {
    for (auto& lidar_point : point_clouds_) {
        for (auto& point_cloud : lidar_point.second) {
            point_cloud.offset_time -= base_time;
        }
    }
}

uint64_t LvxToPointCloudFrameConverter::GetLidarBaseTime(uint8_t id) {
  if (point_clouds_[id].empty()) {
    return 0;
  }
  return point_clouds_[id].at(0).offset_time;
}

void LvxToPointCloudFrameConverter::PublishPointCloudFrame(uint32_t frame_index, 
    uint32_t total_frame_count) {
    frame_.lidar_num = 0;  
    frame_.base_time = std::numeric_limits<uint64_t>::max();
    for (auto &lidar_points : point_clouds_) {
        auto id = lidar_points.first;
        uint64_t base_time = GetLidarBaseTime(id);
        if (base_time != 0 && base_time < frame_.base_time) {
            frame_.base_time = base_time;
        }
    }
    SetLidarsOffsetTime(frame_.base_time);
    // Get Lidar Point
    for (auto &lidar_points : point_clouds_) {
        if (lidar_points.second.empty()) {
            continue;
        }
        uint16_t id = lidar_points.first;
        auto &lidar_point = frame_.lidar_point[frame_.lidar_num];

        lidar_point.lidar_type = GetLidarType(id);
        lidar_point.handle = GetLidarHandle(id);

        lidar_point.points_num = lidar_points.second.size();
        lidar_point.points = lidar_points.second.data();
        frame_.lidar_num++;
    }
    if (callback_) {
        callback_(frame_index, total_frame_count, &frame_, client_data_);
    }
    point_clouds_.clear();
}

void LvxToPointCloudFrameConverter::StopConvert() {
    if (converter_thread_ && converter_thread_->joinable()) {
        converter_thread_->join();
        converter_thread_ = nullptr;
    }
    point_clouds_.clear();
}

}
}  // namespace livox
