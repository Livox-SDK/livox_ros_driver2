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

#include "data_handler.h"
#include <base/logging.h>

#include "livox_def_direct.h"

namespace livox {

namespace direct {

static const size_t kPrefixDataSize = 18;

DataHandler &data_handler() {
  static DataHandler handler;
  return handler;
}

bool DataHandler::Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr) {
  if (direct_host_ipinfo_ptr == nullptr && direct_lidars_info_ptr == nullptr) {
    LOG_ERROR("Data handler init failed, pointers to input parameters are null");
    return false;
  }

  impl_.reset(new DataHandlerImpl(this));
  if (impl_ == nullptr || (!impl_->Init(direct_host_ipinfo_ptr, direct_lidars_info_ptr))) {
    LOG_ERROR("Create data handler impl failed, or data handler impl init failed.");
    impl_.reset(nullptr);
    return false;
  }

  LOG_INFO("Data Handler Init Succ.");
  return true;
}

bool DataHandler::Start() {
  return impl_->Start();
}

uint16_t DataHandler::AddPointCloudObserver(const DataCallback &cb, void *client_data) {
  uint16_t observer_id = GenerateObserverId();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    observers_[observer_id] = std::make_pair(cb, client_data);
  }
  return observer_id;
}

void DataHandler::RemovePointCloudObserver(uint16_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (observers_.find(id) != observers_.end()) {
    observers_.erase(id);
  }
}

uint16_t DataHandler::GenerateObserverId() {
  static std::atomic<std::uint16_t> observer_id(1);
  uint16_t value = observer_id.load();
  uint16_t desired = 0;
  do {
    if (value == UINT16_MAX) {
      desired = 1;
    } else {
      desired = value + 1;
    }
  } while (!observer_id.compare_exchange_weak(value, desired));
  return desired;
}

void DataHandler::SetPointDataCallback(const DataCallback& cb, void *client_data) {
  point_data_callbacks_ = cb;
  point_client_data_ = client_data;
}

void DataHandler::SetImuDataCallback(const DataCallback& cb, void* client_data) {
  imu_data_callbacks_ = cb;
  imu_client_data_ = client_data;
}

void DataHandler::OnDataCallback(const uint32_t handle, void *data, uint16_t size) {
  LivoxDirectEthPacket *lidar_data = (LivoxDirectEthPacket *)data;
  if (lidar_data == NULL) {
    return;
  }

  if (lidar_data->data_type == kImuData) {
    uint32_t imu_data_length = lidar_data->length - sizeof(LivoxDirectEthPacket) + 1;
    size = imu_data_length / sizeof(LivoxDirectImuRawPoint);
    if (imu_data_callbacks_) {
      imu_data_callbacks_(handle, lidar_data, size, imu_client_data_);
    }
    return;
  }

  uint32_t point_data_length = lidar_data->length - sizeof(LivoxDirectEthPacket) + 1;
  if (lidar_data->data_type == kCartesianCoordinateHighData) {
    size = point_data_length / sizeof(LivoxDirectCartesianHighRawPoint);
  } else if (lidar_data->data_type == kCartesianCoordinateLowData) {
    size = point_data_length / sizeof(LivoxDirectCartesianLowRawPoint);
  } else if (lidar_data->data_type == kSphericalCoordinateData) {
    size = point_data_length / sizeof(LivoxDirectSpherPoint);
  } else {
    //LOG_INFO("Unknow data type:{}", lidar_data->data_type);
    return;
  }
 
 
  if (point_data_callbacks_) {
    point_data_callbacks_(handle, lidar_data, size, point_client_data_);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& observer : observers_) {
      auto callback = observer.second.first;
      auto client_data = observer.second.second;
      if (callback) {
        callback(handle, lidar_data, size, client_data);
      }
    }
  }
}

void DataHandler::Destory() {
  if (impl_) {
    impl_.reset(NULL);
  }
}

} // namespace direct
}  // namespace livox
