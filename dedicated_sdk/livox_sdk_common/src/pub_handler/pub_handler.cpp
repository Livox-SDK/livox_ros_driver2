#include"pub_handler.h"

#include <iostream>
#include <limits>

namespace livox {

namespace common {

PubHandler &pub_handler() {
  static PubHandler handler;
  return handler;
}

void PubHandler::Init() {

}

void PubHandler::Uninit() {
  if (vehicle_listen_id_ > 0) {
    VehicleLidarRemovePointCloudObserver(vehicle_listen_id_);
    vehicle_listen_id_ = 0;
  }
  if (listen_id_ > 0) {
    LidarRemovePointCloudObserver(listen_id_);
    listen_id_ = 0;
  }
  if (direct_listen_id_ > 0) {
    DirectLidarRemovePointCloudObserver(direct_listen_id_);
    direct_listen_id_ = 0;
  }
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  is_quit_.store(true);
  if (point_process_thread_ &&
    point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  }
}

void PubHandler::SetPointCloudConfig(PointCloudConfig config) {
  publish_interval_ = kSeconds / config.publish_freq;
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) {
  imu_client_data_ = client_data;
  imu_callback_ = cb;
}

void PubHandler::AddLidarsExtrinsicParams(LidarExtrinsicParameters& param) {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  uint32_t id = 0;
  GetLidarId(param.lidar_type, param.handle, id);
  lidar_extrinsics_[id] = param;
}

void PubHandler::ClearAllLidarsExtrinsicParams() {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  lidar_extrinsics_.clear();
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) {
  pub_client_data_ = client_data;
  points_callback_ = cb;
  vehicle_listen_id_ = VehicleLidarAddPointCloudObserver(OnVehicleLidarPointCloudCallback, this);
  listen_id_ = LidarAddPointCloudObserver(OnLidarPointCloudCallback, this);
  direct_listen_id_ = DirectLidarAddPointCloudObserver(OnDirectLidarPointCloudCallback, this);
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

void PubHandler::OnVehicleLidarPointCloudCallback(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }

  if (data->data_type == kVehicleLidarImuData) {
    if (self->imu_callback_) {
      LivoxVehicleImuRawPoint* imu = (LivoxVehicleImuRawPoint*)data->data;
      LidarImuPoint imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType);
      imu_data.slot = slot;
      imu_data.time_stamp = GetEthPacketTimestamp(data->timestamp_type, data->timestamp, sizeof (data->timestamp));
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }

  StoragePacket packet = {};
  packet.handle = slot;
  packet.lidar_type = static_cast<uint8_t>(LivoxLidarType::kVehicleLidarType);
  packet.extrinsic_enable = false;
  //Todo jerry.lin, vehicle lidar line_num 先 hardcode
  packet.line_num = 1;
  packet.data_type = data->data_type;
  packet.point_interval = data->time_interval * 1000 / data_num; //ns

  packet.time_stamp = GetVehicleEthPacketTimestamp(data->timestamp_type, data->timestamp, sizeof(data->timestamp));

  uint32_t length = data->length;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_data_queues_.push_back(packet);
    self->packet_condition_.notify_one();
  }
}

void PubHandler::OnDirectLidarPointCloudCallback(uint32_t handle, LivoxDirectEthPacket* data, uint32_t data_num, void* client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }

  // if (data->data_type == kDirectLidarImuData) {
  //   if (self->imu_callback_) {
  //     LivoxDirectImuRawPoint* imu = (LivoxDirectImuRawPoint*) data->data;
  //     LidarImuPoint imu_data;
  //     imu_data.lidar_type = static_cast<uint8_t>(LivoxLidarType::kDirectLidarType);
  //     imu_data.handle = handle;
  //     imu_data.time_stamp = GetEthPacketTimestamp(data->timestamp_type, data->timestamp, sizeof (data->timestamp));
  //     imu_data.gyro_x = imu->gyro_x;
  //     imu_data.gyro_y = imu->gyro_y;
  //     imu_data.gyro_z = imu->gyro_z;
  //     imu_data.acc_x = imu->acc_x;
  //     imu_data.acc_y = imu->acc_y;
  //     imu_data.acc_z = imu->acc_z;
  //     self->imu_callback_(&imu_data, client_data);
  //   }
  //   return;
  // }

  StoragePacket packet = {};
  packet.handle = handle;
  packet.lidar_type = static_cast<uint8_t>(LivoxLidarType::kDirectLidarType);
  packet.extrinsic_enable = false;
  packet.line_num = 1;
  packet.data_type = data->data_type;
  packet.point_interval = data->time_interval * 1000 / data_num; //ns

  packet.time_stamp = GetDirectEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp));

  uint32_t length = data->length - sizeof(LivoxDirectEthPacket) + 1;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_data_queues_.push_back(packet);
    self->packet_condition_.notify_one();
  }
}

void PubHandler::OnLidarPointCloudCallback(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }
  if (data->data_type == kImu) {
    if (self->imu_callback_) {
      LivoxImuPoint* imu = (LivoxImuPoint*) data->data;
      LidarImuPoint imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LivoxLidarType::kIndustryLidarType);
      imu_data.handle = handle;
      imu_data.time_stamp = GetEthPacketTimestamp(data->timestamp_type, data->timestamp, sizeof (data->timestamp));
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }

  StoragePacket packet = {};
  if (handle == kMaxLidarCount - 1) {
    packet.lidar_type = static_cast<uint8_t>(LivoxLidarType::kIndustryLidarType);
    packet.handle = HubGetLidarHandle(data->slot, data->id);
    packet.extrinsic_enable = true;
  } else {
    packet.lidar_type = static_cast<uint8_t>(LivoxLidarType::kIndustryLidarType);
    packet.handle = handle;
    packet.extrinsic_enable = false;
  }
  uint8_t product_type = self->GetProductType(packet.handle);
  packet.line_num = GetLaserLineNumber(product_type);
  packet.data_type = data->data_type;
  packet.point_num = data_num;

  packet.point_interval = GetPointInterval(product_type);

  packet.time_stamp = GetEthPacketTimestamp(data->timestamp_type, data->timestamp, sizeof(data->timestamp));
  uint32_t length = GetEthPacketLen(data->data_type);
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_data_queues_.push_back(packet);
    self->packet_condition_.notify_one();
  }
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, void *client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }

  if (data->data_type == kLivoxLidarImuData) {
    if (self->imu_callback_) {
      LivoxImuPoint* imu = (LivoxImuPoint*) data->data;
      LidarImuPoint imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LivoxLidarType::kLivoxLidarType);
      imu_data.handle = handle;
      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type,
                                                  data->timestamp, sizeof(data->timestamp));
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }

  StoragePacket packet = {};
  packet.handle = handle;
  packet.lidar_type = static_cast<uint8_t>(LivoxLidarType::kLivoxLidarType);
  packet.extrinsic_enable = false;
  if (dev_type == kDeviceTypeLidarHAP) {
    packet.line_num = kLineNumberHAP;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type = data->data_type;
  packet.point_num = data->dot_num;
  packet.point_interval = data->time_interval * 100 / data->dot_num;  //ns
  packet.time_stamp = GetDirectEthPacketTimestamp(data->time_type,
                                                  data->timestamp, sizeof(data->timestamp));
  uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_data_queues_.push_back(packet);
    self->packet_condition_.notify_one();
  }

  return;
}

uint8_t PubHandler::GetProductType(uint8_t handle) {
  if (devices_info_.find(handle) == devices_info_.end()) {
    DeviceInfo device_info;
    GetLidarDeviceInfo(handle, &device_info);
    devices_info_[handle] = device_info;
  }
  return devices_info_[handle].type;
}

void PubHandler::PublishPointCloud() {
  frame_.base_time = std::numeric_limits<uint64_t>::max();
  frame_.lidar_num = 0;

  // Calculate Base Time
  for (auto &process_handler : lidar_process_handlers_) {
    uint64_t base_time = process_handler.second->GetLidarBaseTime();
    if (base_time != 0 && base_time < frame_.base_time) {
      frame_.base_time = base_time;
    }
  }
  // Get Lidar Point
  for (auto &process_handler : lidar_process_handlers_) {
    process_handler.second->SetLidarOffsetTime(frame_.base_time);
    uint32_t id = process_handler.first;
    points_[id].clear();
    process_handler.second->GetLidarPointClouds(points_[id]);
    if (points_[id].empty()) {
      continue;
    }
    LidarPoint& lidar_point = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type = GetLidarType(id);  // TODO:
    lidar_point.handle = GetLidarHandle(id);

    lidar_point.points_num = points_[id].size();
    lidar_point.points = points_[id].data();
    frame_.lidar_num++;
  }
  //publish point
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
  return;
}

void PubHandler::CheckTimer() {
  auto now_time = std::chrono::high_resolution_clock::now();
  //First Set
  if (last_pub_time_ == std::chrono::high_resolution_clock::time_point()) {
    last_pub_time_ = now_time;
    return;
  } else if (now_time - last_pub_time_ <
      std::chrono::nanoseconds(publish_interval_)) {
    return;
  }
  last_pub_time_ += std::chrono::nanoseconds(publish_interval_);

  PublishPointCloud();

  return;
}

void PubHandler::RawDataProcess() {
  StoragePacket raw_data;
  while (!is_quit_) {
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (raw_data_queues_.empty()) {
        packet_condition_.wait(lock);
      }
      raw_data = raw_data_queues_.front();
      raw_data_queues_.pop_front();
    }

    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
      lidar_process_handlers_[id].reset(new LidarPubHandler());
    }
    auto &process_handler = lidar_process_handlers_[id];
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
        lidar_process_handlers_[id]->SetLidarsExtrinsicParams(lidar_extrinsics_[id]);
    }
    process_handler->PointCloudProcess(raw_data);
    CheckTimer();
  }
}

}
}  // namespace livox
