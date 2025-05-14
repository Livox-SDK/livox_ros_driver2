//
// The MIT License (MIT)
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

#ifndef LIVOX_ROS_DRIVER_LIDAR_INFO_DATA_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_INFO_DATA_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>

namespace livox_ros {

// Based on the IMU Data Type in Livox communication protocol
// TODO: add a link to the protocol

typedef struct LidarInfoData {
  uint8_t lidar_type;
  uint32_t handle;
  uint64_t time_stamp;
  LidarInfoData(): lidar_type(0), handle(0), time_stamp(0) {}

} LidarInfoData;

class LidarInfoDataQueue {
 public:
  void Push(const LidarInfoData* imu_data);
  bool Pop(LidarInfoData& imu_data);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  std::list<LidarInfoData> lidar_info_data_queue_;
};

} // namespace

#endif // #define LIVOX_ROS_DRIVER_LIDAR_INFO_DATA_QUEUE_H_


