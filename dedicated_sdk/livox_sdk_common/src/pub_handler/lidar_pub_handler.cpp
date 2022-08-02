#include "lidar_pub_handler.h"
#include <iostream>
#include <cmath>

namespace livox {

namespace common {

LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {}

uint64_t LidarPubHandler::GetLidarBaseTime() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

void LidarPubHandler::SetLidarOffsetTime(uint64_t base_time) {
  for (auto& point_cloud : points_clouds_) {
    point_cloud.offset_time -= base_time;
  }
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointCloudXyzlt>& points_clouds) {
  std::lock_guard<std::mutex> lock(mutex_);
  points_clouds.swap(points_clouds_);
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_clouds_.size();
}

void LidarPubHandler::PointCloudProcess(StoragePacket & pkt) {
   //convert to standard format and extrinsic compensate
  if (pkt.lidar_type == static_cast<uint8_t>(LivoxLidarType::kIndustryLidarType)) {
    IndustryLidarPointCloudProcess(pkt);
  } else if (pkt.lidar_type == static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType)) {
    VehicleLidarPointCloudProcess(pkt);
  } else if (pkt.lidar_type == static_cast<uint8_t>(LivoxLidarType::kDirectLidarType)) {
    DirectLidarPointCloudProcess(pkt);
  } else if (pkt.lidar_type == static_cast<uint8_t>(LivoxLidarType::kLivoxLidarType)) {
    LivoxLidarPointCloudProcess(pkt);
  }
}

void LidarPubHandler::IndustryLidarPointCloudProcess(StoragePacket& pkt) {
  switch (pkt.data_type) {
      case kCartesian:
        CartesianPointProcess(pkt);
        break;
      case kSpherical:
        SphericalPointProcess(pkt);
        break;
      case kExtendCartesian:
        ExtendCartesianProcess(pkt);
        break;
      case kExtendSpherical:
        ExtendSphericalProcess(pkt);
        break;
      case kDualExtendCartesian:
        DualExtendCartesianProcess(pkt);
        break;
      case kDualExtendSpherical:
        DualExtendSphericalProcess(pkt);
        break;
      default:
        break;
    }
}

void LidarPubHandler::VehicleLidarPointCloudProcess(StoragePacket & pkt) {
  switch (pkt.data_type) {
    case kHighResolutionPointData:
      VehicleExtendRawPointCloudProcess(pkt);
      break;
    case kLowResolutionPointData:
      VehicleExtendHalfRawPointCloudProcess(pkt);
      break;
    default:
      break;
  }
}

void LidarPubHandler::DirectLidarPointCloudProcess(StoragePacket & pkt) {
  switch (pkt.data_type) {
    case kCartesianCoordinateHighData:
      DirectExtendRawPointCloudProcess(pkt);
      break;
    case kCartesianCoordinateLowData:
      DirectExtendHalfRawPointCloudProcess(pkt);
      break;
    case kSphericalCoordinateData:
      DirectSphericalPointProcess(pkt);
      break;
    default:
      break;
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(StoragePacket & pkt) {
  switch (pkt.data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type)
                << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::CartesianPointProcess(StoragePacket& pkt) {
  LivoxRawPoint* raw = (LivoxRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x * extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = 0;
    point.tag = 0;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::SphericalPointProcess(StoragePacket& pkt) {
  LivoxSpherPoint* raw = (LivoxSpherPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = 0;
    point.tag = 0;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ExtendCartesianProcess(StoragePacket& pkt) {
  LivoxExtendRawPoint* raw = (LivoxExtendRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ExtendSphericalProcess(StoragePacket& pkt) {
  LivoxExtendSpherPoint* raw = (LivoxExtendSpherPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::DualExtendCartesianProcess(StoragePacket& pkt) {
  LivoxDualExtendRawPoint* raw = (LivoxDualExtendRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point[2] = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point[0].x = raw[i].x1 / 1000.0;
      point[0].y = raw[i].y1 / 1000.0;
      point[0].z = raw[i].z1 / 1000.0;
      point[1].x = raw[i].x2 / 1000.0;
      point[1].y = raw[i].y2 / 1000.0;
      point[1].z = raw[i].z2 / 1000.0;
    } else {
      point[0].x = (raw[i].x1 * extrinsic_.rotation[0][0] +
                raw[i].y1 * extrinsic_.rotation[0][1] +
                raw[i].z1 * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point[0].y = (raw[i].x1 * extrinsic_.rotation[1][0] +
                raw[i].y1 * extrinsic_.rotation[1][1] +
                raw[i].z1 * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point[0].z = (raw[i].x1 * extrinsic_.rotation[2][0] +
                raw[i].y1 * extrinsic_.rotation[2][1] +
                raw[i].z1 * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
      point[1].x = (raw[i].x2 * extrinsic_.rotation[0][0] +
                raw[i].y2 * extrinsic_.rotation[0][1] +
                raw[i].z2 * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point[1].y = (raw[i].x2 * extrinsic_.rotation[1][0] +
                raw[i].y2 * extrinsic_.rotation[1][1] +
                raw[i].z2 * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point[1].z = (raw[i].x2 * extrinsic_.rotation[2][0] +
                raw[i].y2 * extrinsic_.rotation[2][1] +
                raw[i].z2 * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point[0].intensity = raw[i].reflectivity1;
    point[1].intensity  = raw[i].reflectivity2;
    point[0].tag = raw[i].tag1;
    point[1].tag = raw[i].tag2;
    point[0].line = point[1].line = i % pkt.line_num;
    point[0].offset_time = point[1].offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point[0]);
    points_clouds_.push_back(point[1]);
  }
}

void LidarPubHandler::DualExtendSphericalProcess(StoragePacket& pkt) {
  LivoxDualExtendSpherPoint* raw = (LivoxDualExtendSpherPoint*)pkt.raw_data.data();
  PointCloudXyzlt point[2] = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius1 = raw[i].depth1 / 1000.0;
    double radius2 = raw[i].depth2 / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x1 = radius1 * sin(theta) * cos(phi);
    double src_y1 = radius1 * sin(theta) * sin(phi);
    double src_z1 = radius1 * cos(theta);
    double src_x2 = radius2 * sin(theta) * cos(phi);
    double src_y2 = radius2 * sin(theta) * sin(phi);
    double src_z2 = radius2 * cos(theta);
    if (pkt.extrinsic_enable) {
      point[0].x = src_x1;
      point[0].y = src_y1;
      point[0].z = src_z1;
      point[1].x = src_x2;
      point[1].y = src_y2;
      point[1].z = src_z2;
    } else {
      point[0].x = src_x1 * extrinsic_.rotation[0][0] +
                src_y1 * extrinsic_.rotation[0][1] +
                src_z1 * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point[0].y = src_x1 * extrinsic_.rotation[1][0] +
                src_y1 * extrinsic_.rotation[1][1] +
                src_z1 * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point[0].z = src_x1 * extrinsic_.rotation[2][0] +
                src_y1 * extrinsic_.rotation[2][1] +
                src_z1 * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
      point[1].x = src_x2 * extrinsic_.rotation[0][0] +
                src_y2 * extrinsic_.rotation[0][1] +
                src_z2 * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point[1].y = src_x2 * extrinsic_.rotation[1][0] +
                src_y2 * extrinsic_.rotation[1][1] +
                src_z2 * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point[1].z = src_x2 * extrinsic_.rotation[2][0] +
                src_y2 * extrinsic_.rotation[2][1] +
                src_z2 * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point[0].intensity = raw[i].reflectivity1;
    point[1].intensity  = raw[i].reflectivity2;
    point[0].tag = raw[i].tag1;
    point[1].tag = raw[i].tag2;
    point[0].line = point[1].line = i % pkt.line_num;
    point[0].offset_time = point[1].offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point[0]);
    points_clouds_.push_back(point[1]);
  }
}


void LidarPubHandler::SetLidarsExtrinsicParams(LidarExtrinsicParameters param) {
  if (is_set_extrinsic_params_) {
    return;
  }
  extrinsic_.trans[0] = param.x;
  extrinsic_.trans[1] = param.y;
  extrinsic_.trans[2] = param.z;

  double cos_roll = cos(static_cast<double>(param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(param.pitch * PI / 180.0));
  double cos_yaw = cos(static_cast<double>(param.yaw * PI / 180.0));
  double sin_roll = sin(static_cast<double>(param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(param.pitch * PI / 180.0));
  double sin_yaw = sin(static_cast<double>(param.yaw * PI / 180.0));

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true;
}

void LidarPubHandler::VehicleExtendRawPointCloudProcess(StoragePacket& pkt) {
  pkt.point_num = pkt.raw_data.size()/ sizeof(LivoxVehicleExtendRawPoint);
  LivoxVehicleExtendRawPoint* raw = (LivoxVehicleExtendRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::VehicleExtendHalfRawPointCloudProcess(StoragePacket& pkt) {
  pkt.point_num = pkt.raw_data.size()/ sizeof(LivoxVehicleExtendHalfRawPoint);
  LivoxVehicleExtendHalfRawPoint* raw = (LivoxVehicleExtendHalfRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::DirectExtendRawPointCloudProcess(StoragePacket& pkt) {
  pkt.point_num = pkt.raw_data.size()/ sizeof(LivoxDirectCartesianHighRawPoint);
  LivoxDirectCartesianHighRawPoint* raw = (LivoxDirectCartesianHighRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;

      point.y = (raw[i].x * extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;

      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = 0;
    point.tag = 0;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::DirectExtendHalfRawPointCloudProcess(StoragePacket& pkt) {
  pkt.point_num = pkt.raw_data.size()/ sizeof(LivoxDirectCartesianLowRawPoint);
  LivoxDirectCartesianLowRawPoint* raw = (LivoxDirectCartesianLowRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::DirectSphericalPointProcess(StoragePacket& pkt) {
  pkt.point_num = pkt.raw_data.size()/ sizeof(LivoxDirectSpherPoint);
  LivoxDirectSpherPoint* raw = (LivoxDirectSpherPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {

    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = 0;
    point.tag = 0;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianHighPoint(StoragePacket & pkt) {
  LivoxLidarCartesianHighRawPoint* raw = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(StoragePacket & pkt) {
  LivoxLidarCartesianLowRawPoint* raw = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessSphericalPoint(StoragePacket& pkt) {
  LivoxLidarSpherPoint* raw = (LivoxLidarSpherPoint*)pkt.raw_data.data();
  PointCloudXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

} // namespace vehicle
} // namespace livox