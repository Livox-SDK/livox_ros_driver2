#include "lidar_imu_data_queue.h"

namespace livox_ros {

void LidarImuDataQueue::Push(LidarImuPoint* imu_data) {
  LidarImuPoint data;
  data.lidar_type = imu_data->lidar_type;
  if (data.lidar_type == kVehicleLidarType) {
    data.slot = imu_data->slot;
  } else if (data.lidar_type == kIndustryLidarType) {
    data.handle = imu_data->handle;
  }
  data.time_stamp = imu_data->time_stamp;

  data.gyro_x = imu_data->gyro_x;
  data.gyro_y = imu_data->gyro_y;
  data.gyro_z = imu_data->gyro_z;

  data.acc_x = imu_data->acc_x;
  data.acc_y = imu_data->acc_y;
  data.acc_z = imu_data->acc_z;
  

  std::lock_guard<std::mutex> lock(mutex_);
  imu_data_queue_.push_back(std::move(data));
}

bool LidarImuDataQueue::Pop(LidarImuPoint& imu_data) {
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
  std::list<LidarImuPoint> tmp_imu_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.swap(tmp_imu_data_queue);
  }
}

} // namespace