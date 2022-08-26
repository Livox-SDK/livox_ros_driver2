#ifndef LIVOX_PUB_HANDLER_H_
#define LIVOX_PUB_HANDLER_H_

#include <functional>
#include <memory>
#include <deque>
#include <mutex>
#include <cstring>
#include <condition_variable>
#include <map>
#include <atomic>
#include <thread>
#include <atomic>

#include "lidar_pub_handler.h"
#include"livox_def.h"
#include"livox_sdk.h"
#include "livox_sdk_vehicle.h"
#include "livox_def_vehicle.h"
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

namespace livox {

namespace common {

class PubHandler {
public:
  using PointCloudsCallback = std::function<void(PointCloudFrame*, void *)>;
  using ImuDataCallback = std::function<void(LidarImuPoint*, void*)>;
  using TimePoint = std::chrono::high_resolution_clock::time_point;

  PubHandler() {}

  ~ PubHandler() { Uninit(); }

  void Uninit();
  void Init();

  void SetPointCloudConfig(PointCloudConfig config);
  void SetPointCloudsCallback(PointCloudsCallback cb, void* client_data);
  void AddLidarsExtrinsicParams(LidarExtrinsicParameters& extrinsic_params);
  void ClearAllLidarsExtrinsicParams();
  void SetImuDataCallback(ImuDataCallback cb, void* client_data);

private:

  //thread to process raw data
  void RawDataProcess();
  std::atomic<bool> is_quit_{false};
  std::shared_ptr<std::thread> point_process_thread_;

  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;

  //publish callback
  void CheckTimer();
  void PublishPointCloud();

  static void OnVehicleLidarPointCloudCallback(uint8_t slot, LivoxVehicleEthPacket* data,
                                               uint32_t data_num, void* client_data);
  static void OnLidarPointCloudCallback(uint8_t handle, LivoxEthPacket* data, uint32_t data_num,
                                        void* client_data);
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                             LivoxLidarEthernetPacket *data, void *client_data);

  uint8_t GetProductType(uint8_t handle);

  PointCloudsCallback points_callback_;
  void* pub_client_data_ = nullptr;

  ImuDataCallback imu_callback_;
  void* imu_client_data_ = nullptr;

  PointCloudFrame frame_;

  std::deque<StoragePacket> raw_data_queues_;

  //pub config
  uint32_t publish_interval_ = 100000000; //100 ms
  TimePoint last_pub_time_;

  std::map<uint32_t, std::unique_ptr<LidarPubHandler>> lidar_process_handlers_;
  std::map<uint32_t, std::vector<PointCloudXyzlt>> points_;
  std::map<uint32_t, LidarExtrinsicParameters> lidar_extrinsics_;
  std::map<uint32_t, DeviceInfo> devices_info_;
  uint16_t vehicle_listen_id_ = 0;
  uint16_t listen_id_ = 0;
  uint16_t lidar_listen_id_ = 0;
};

PubHandler &pub_handler();

}

}  // namespace livox

#endif  // LIVOX_PUB_HANDLER_H_