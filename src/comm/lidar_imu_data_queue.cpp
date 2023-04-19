//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#include "lidar_imu_data_queue.h"

namespace livox_ros {

void LidarImuDataQueue::Push(ImuData *imu_data,
                             const ExtParameterDetailed &detail) {
  ImuData data;
  data.lidar_type = imu_data->lidar_type;
  data.handle = imu_data->handle;
  data.time_stamp = imu_data->time_stamp;

  data.acc_x = imu_data->acc_x * detail.rotation[0][0] +
               imu_data->acc_y * detail.rotation[0][1] +
               imu_data->acc_z * detail.rotation[0][2] +
               detail.trans[0] * 1000.0;
  data.acc_y = imu_data->acc_x * detail.rotation[1][0] +
               imu_data->acc_y * detail.rotation[1][1] +
               imu_data->acc_z * detail.rotation[1][2] +
               detail.trans[1] * 1000.0;
  data.acc_z = imu_data->acc_x * detail.rotation[2][0] +
               imu_data->acc_y * detail.rotation[2][1] +
               imu_data->acc_z * detail.rotation[2][2] +
               detail.trans[2] * 1000.0;

  data.gyro_x = imu_data->gyro_x * detail.rotation[0][0] +
                imu_data->gyro_y * detail.rotation[0][1] +
                imu_data->gyro_z * detail.rotation[0][2] +
                detail.trans[0] * 1000.0;
  data.gyro_y = imu_data->gyro_x * detail.rotation[1][0] +
                imu_data->gyro_y * detail.rotation[1][1] +
                imu_data->gyro_z * detail.rotation[1][2] +
                detail.trans[1] * 1000.0;
  data.gyro_z = imu_data->gyro_x * detail.rotation[2][0] +
                imu_data->gyro_y * detail.rotation[2][1] +
                imu_data->gyro_z * detail.rotation[2][2] +
                detail.trans[2] * 1000.0;

  std::lock_guard<std::mutex> lock(mutex_);
  imu_data_queue_.push_back(std::move(data));
}

bool LidarImuDataQueue::Pop(ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_data_queue_.empty()) {
    return false;
  }
  imu_data = imu_data_queue_.front();
  imu_data_queue_.pop_front();
  return true;
}

bool LidarImuDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_data_queue_.empty();
}

void LidarImuDataQueue::Clear() {
  std::list<ImuData> tmp_imu_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.swap(tmp_imu_data_queue);
  }
}

} // namespace livox_ros