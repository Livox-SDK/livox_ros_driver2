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
#include "lidar_data_handler.h"
#include "command_handler/command_impl.h"
#include "base/network/network_util.h"

namespace livox {
namespace vehicle {

//static const size_t kPrefixDataSize = 24;

DataHandler &data_handler() {
  static DataHandler handler;
  return handler;
}

void DataHandler::AddPointCloudCallback(const DataCallback &cb, void *client_data) {
  point_client_data_ = client_data;
  point_callback_ = cb;
  return;
}

void DataHandler::AddImuDataCallback(const DataCallback &cb, void *client_data) {
  imu_client_data_ = client_data;
  imu_callback_ = cb;
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

uint16_t DataHandler::AddPointCloudObserver(const DataCallback &cb, void *client_data) {
  uint16_t observer_id = GenerateObserverId();
  if (!loop_.expired()) {
      loop_.lock()->PostTask([this, cb, observer_id, client_data]{
        observers_[observer_id] = std::make_pair(cb, client_data);
      });
  }
  return observer_id;
}

void DataHandler::RemovePointCloudObserver(uint16_t id) {
    if (!loop_.expired()) {
        loop_.lock()->PostTask([this, id] {
          if (observers_.find(id) != observers_.end()) {
            observers_.erase(id);
          }
        });
    }
}

bool DataHandler::Init() {
  if (!impl_) {
    impl_.reset(new LidarDataHandlerImpl(this));
  }
  thread_.reset(new IOThread());
  thread_->Init(false, false);
  loop_ = thread_->loop();
  // create point sock
  point_sock_ = util::CreateSocket(kPointCloudPort);
  if (point_sock_ == -1) {
    return false;
  }
  loop_.lock()->AddDelegate(point_sock_, impl_.get());

  // create imu data sock
  imu_sock_ = util::CreateSocket(kImuDataPort);
  if (imu_sock_ == -1) {
    return false;
  }
  loop_.lock()->AddDelegate(imu_sock_, impl_.get());

  return thread_->Start();
}

void DataHandler::Uninit() {
  if (point_sock_ != -1) {
    loop_.lock()->RemoveDelegate(point_sock_, impl_.get());
  }
  thread_->Quit();
  thread_->Join();
  thread_->Uninit();
  if (point_sock_ != -1) {
    util::CloseSock(point_sock_);
    point_sock_ = -1;
  }
  impl_.reset(nullptr);

  point_callback_ = nullptr;
  point_client_data_ = nullptr;
  return;
}

void DataHandler::OnDataCallback(socket_t sock, void *data, uint16_t size) {
  LivoxVehicleEthPacket *lidar_data = (LivoxVehicleEthPacket *)data;
  if (lidar_data == nullptr) {
    return;
  }

  if (sock == point_sock_) {
    if (lidar_data->data_type == kHighResolutionPointData) {
      size = lidar_data->length / sizeof(LivoxVehicleExtendRawPoint);
    } else if (lidar_data->data_type == kLowResolutionPointData) {
      size = lidar_data->length / sizeof(LivoxVehicleExtendHalfRawPoint);
    }

    if (point_callback_) {
      point_callback_(lidar_data->slot, lidar_data, size, point_client_data_);
    }
  } else if (sock == imu_sock_) {
    size = lidar_data->length / sizeof(LivoxVehicleImuRawPoint);
    if (imu_callback_) {
      imu_callback_(lidar_data->slot, lidar_data, size, imu_client_data_);
    }
  } else {
    return;
  }

  for (const auto& observer : observers_) {
    auto callback = observer.second.first;
    auto client_data = observer.second.second;
    if (callback) {
      callback(lidar_data->slot, lidar_data, size, client_data);
    }
  }
}

}
}  // namespace livox
