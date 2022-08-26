#include <vector>
#include <mutex>
#include <cstring>
#include <atomic>
#include "livox_def.h"
#include "livox_def_common.h"
#include "livox_def_vehicle.h"
#include "livox_sdk_common.h"
#include "livox_sdk_common_util.h"
#include "livox_lidar_def.h"

namespace livox {

namespace common {

class LidarPubHandler {

public:
  LidarPubHandler();
  ~ LidarPubHandler() {}

  void PointCloudProcess(StoragePacket& pkt);
  void SetLidarsExtrinsicParams(LidarExtrinsicParameters param);
  void GetLidarPointClouds(std::vector<PointCloudXyzlt>& points_clouds);

  uint32_t GetLidarPointCloudsSize();

  uint64_t GetLidarBaseTime();
  void SetLidarOffsetTime(uint64_t base_time);

private:
  void IndustryLidarPointCloudProcess(StoragePacket& pkt);
  void CartesianPointProcess(StoragePacket& pkt);
  void SphericalPointProcess(StoragePacket& pkt);
  void ExtendCartesianProcess(StoragePacket& pkt);
  void ExtendSphericalProcess(StoragePacket& pkt);
  void DualExtendCartesianProcess(StoragePacket& pkt);
  void DualExtendSphericalProcess(StoragePacket& pkt);

  void VehicleLidarPointCloudProcess(StoragePacket& pkt);
  void VehicleExtendRawPointCloudProcess(StoragePacket& pkt);
  void VehicleExtendHalfRawPointCloudProcess(StoragePacket& pkt);

  void LivoxLidarPointCloudProcess(StoragePacket & pkt);
  void ProcessCartesianHighPoint(StoragePacket & pkt);
  void ProcessCartesianLowPoint(StoragePacket & pkt);
  void ProcessSphericalPoint(StoragePacket & pkt);
  std::vector<PointCloudXyzlt> points_clouds_;
  ExtrinsicParameters extrinsic_ = {
    {0, 0, 0},
    {
      {1, 0, 0},
      {0, 1, 1},
      {0, 0, 1}
    }
  };
  std::mutex mutex_;
  std::atomic_bool is_set_extrinsic_params_;

};

}
}  // namespace livox