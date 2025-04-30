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

#include "lidar_info_data_queue.h"

namespace livox_ros {

void LidarInfoDataQueue::Push(const LidarInfoData* lidar_info_data) {
  LidarInfoData data(*lidar_info_data);
  std::lock_guard<std::mutex> lock(mutex_);
  lidar_info_data_queue_.push_back(std::move(data));
}

bool LidarInfoDataQueue::Pop(LidarInfoData& lidar_info_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (lidar_info_data_queue_.empty()) {
    return false;
  }
  lidar_info_data = lidar_info_data_queue_.front();
  lidar_info_data_queue_.pop_front();
  return true;
}

bool LidarInfoDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return lidar_info_data_queue_.empty();
}

void LidarInfoDataQueue::Clear() {
  std::list<LidarInfoData> tmp_lidar_info_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_info_data_queue_.swap(tmp_lidar_info_data_queue);
  }
}

} // namespace livox_ros