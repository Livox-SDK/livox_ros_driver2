#include "livox_sdk_common.h"

#include "livox_sdk.h"
#include "livox_sdk_vehicle.h"
#include "livox_def.h"
#include "livox_def_vehicle.h"

#include "lvx_handler/lvx_file_parse_handler.h"
#include "lvx_handler/lvx_file_record_handler.h"
#include "lvx_handler/lvx_file_converter.h"

#include "converter/lvx_to_point_cloud_frame.h"

#include "upgrader/upgrade_manager.h"
#include "pub_handler/pub_handler.h"
#include "algorithm/obstacle_zone_converter/obstacle_zone_converter.h"

#include <vector>

using namespace livox::common;

// void GetLivoxSdkVersion(LivoxSdkVersion *version) {
//   if (version != NULL) {
//     version->major = LIVOX_SDK_MAJOR_VERSION;
//     version->minor = LIVOX_SDK_MINOR_VERSION;
//     version->patch = LIVOX_SDK_PATCH_VERSION;
//   }
// }

/*******Record Lvx File Module***********/
// Save Lvx File
bool SetLvxRecordDir(const char* dir) {
  return lvx_file_record_handler().SetLvxRecordDir(std::string(dir));
}

// extrinsic need to be set
void AddDeviceInfo(LvxDeviceInfo *info, uint8_t size) {
  std::vector<LvxDeviceInfo> device_infos;
  for (uint8_t i = 0; i < size; i++) {
    device_infos.push_back(*(info + i));
  }
  lvx_file_record_handler().AddDeviceInfo(std::move(device_infos));
}

void StartRecordFile(LvxFileType file_type) {
  lvx_file_record_handler().StartRecordLvxFile(LIVOX_FILE_LVX2);
}

void StartRecordLvxFile() {
  StartRecordFile(LIVOX_FILE_LVX2);
}

void StopRecordLvxFile() {
  lvx_file_record_handler().StopRecordLvxFile();
}

/*******Parse Lvx File Module***********/
bool SetLvxParseDir(const char *dir) {
  bool ret = lvx_file_parse_handler().SetLvxParseDir(std::string(dir));
  return ret;
}

bool IsFileVersionSupported(LvxFileVersion& minimum_version_required) {
  minimum_version_required = lvx_file_parse_handler().GetFileVersion();
  if (!lvx_file_parse_handler().IsFileSupported()) {
    return false;
  }
  return true;
}

bool StartParseLvxFile() {
  return lvx_file_parse_handler().StartParseLvxFile();
}

//Lvx file's total frame count
uint32_t GetLvxFrameCount() {
  return lvx_file_parse_handler().GetFrameCount();
}

//Frame duration between frame index: default 50ms
uint32_t GetLvxFrameDuration() {
  return lvx_file_parse_handler().GetFrameDuration();
}

//Get Device's extrinsic and sn code.
void GetLvxDeviceInfoList(LvxDeviceInfo *info, uint8_t *size) {
  std::vector<LvxDeviceInfo> infos;
  lvx_file_parse_handler().GetLvxDeviceInfoList(infos);
  *size = (uint8_t)infos.size();
  for (uint8_t i = 0; i < infos.size(); i++) {
    info[i] = infos[i];
  }
  return;
}

// Get 32bit point data for a specific frame, deprecated
const LvxBasePackDetail* GetLvxPointCloud(int frame_index, uint32_t *packet_num) {
  if (packet_num == nullptr) {
    return nullptr;
  }
  return lvx_file_parse_handler().GetHighResPointData(frame_index, *packet_num);
}

// Get 32bit & 16bit point data for a specific frame
LvxPointsPackInfo GetLvxPointData(int frame_index) {
  LvxPointsPackInfo points_packet;
  bool ret = lvx_file_parse_handler().GetPointData(frame_index,
                                                   points_packet.lvx_pack_detail,
                                                   points_packet.pack_num,
                                                   points_packet.lvx_half_pack_detail,
                                                   points_packet.half_pack_num);
  if (ret) {
    return points_packet;
  }
  return {};
}

bool CutLvxFile(uint64_t start_frame_index, uint64_t end_frame_index, const char* dir) {
  return lvx_file_parse_handler().CutLvxFile(start_frame_index, end_frame_index, std::string(dir));
}

void StopParseLvxFile() {
  return lvx_file_parse_handler().StopParseLvxFile();
}

/*******Convert Lvx File Module***********/
bool Lvx2ToLvx3(const char *lvx2_file_path, const char *output_dir) {
  return lvx_file_converter().Lvx2ToLvx3(lvx2_file_path, output_dir);
}

bool Lvx3ToLvx2(const char *lvx3_file_path, const char *output_dir) {
  return lvx_file_converter().Lvx3ToLvx2(lvx3_file_path, output_dir);
}

// Livox lidar upgrade
bool SetVehicleUpgradeFirmwarePath(const char* firmware_path) {
  return upgrade_manager().SetVehicleUpgradeFirmwarePath(firmware_path);
}

void SetUpgradeProgressCallback(OnUpgradeProgressCallbackCallback cb, void* client_data) {
  upgrade_manager().SetUpgradeProgressCallback(cb, client_data);
  return;
}

void UpgradeVehicleLidar(const uint8_t slot) {
  upgrade_manager().UpgradeVehicleLidar(slot);
}

void UpgradeVehicleLidars(const uint8_t* slot, const uint8_t lidar_num) {
  upgrade_manager().UpgradeVehicleLidars(slot, lidar_num);
}

bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path) {
  return upgrade_manager().SetLivoxLidarUpgradeFirmwarePath(firmware_path);
}

void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data) {
  upgrade_manager().SetLivoxLidarUpgradeProgressCallback(cb, client_data);
}

void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num) {
  upgrade_manager().UpgradeLivoxLidars(handle, lidar_num);
}

// Direct lidar upgrade
bool SetDirectUpgradeFirmwarePath(const char* firmware_path) {
  return upgrade_manager().SetDirectUpgradeFirmwarePath(firmware_path);
}

void SetDirectLidarUpgradeProgressCallback(OnDirectLidarUpgradeProgressCallback cb, void* client_data) {
  upgrade_manager().SetDirectLidarUpgradeProgressCallback(cb, client_data);
}
void UpgradeDirectLidars(const uint32_t* handle, const uint8_t lidar_num) {
  upgrade_manager().UpgradeDirectLidars(handle, lidar_num);
}


bool SetUpgradeFirmwarePath(const char *dir) {
  return upgrade_manager().SetUpgradeFirmwarePath(std::string(dir));
}

void StartUpgradeLidar(LivoxLidarType type, uint8_t slot) {
  upgrade_manager().StartUpgradeLidar(type, slot);
  return;
}

void StopUpgradeLidar(LivoxLidarType type, uint8_t slot) {
  upgrade_manager().StopUpgradeLidar(type, slot);
  return;
}

void SetObstacleAvoidanceCubeParams(ObstacleZoneCubeParam* param, uint8_t para_num) {
  std::vector< ObstacleZoneCubeParam> cube_array(param, param + para_num);
  obstacle_zone_converter().SetCubeArray(cube_array);
  return;
}

void SetObstacleAvoidanceCylinderParams(ObstacleZoneCylinderParam* param, uint8_t para_num) {
  std::vector< ObstacleZoneCylinderParam> cylinder_array(param, param + para_num);
  obstacle_zone_converter().SetCylinderArray(cylinder_array);
  return;
}

bool AvoidanceObstacleZoneFileConvert(const char* file_name, ObstacleZoneConvertProgressCallback cb, void* client_data) {
  obstacle_zone_converter().Convert(std::string(file_name), cb, client_data);
  return true;
}

int AvoidanceObstacleTestPoint(int x, int y, int z) {
  return obstacle_zone_converter().TestPoint(x, y, z);
}

void ResetAllObstacleAvoidanceParams() {
  obstacle_zone_converter().Clear();
  return;
}

void AddLidarsExtrinsicParams(LidarExtrinsicParameters params) {
  pub_handler().AddLidarsExtrinsicParams(params);
}

void SetPointCloudConfig(PointCloudConfig config) {
  pub_handler().SetPointCloudConfig(config);
}

void SetPointCloudCallback(PubPointCloudCallback cb, void *client_data) {
  pub_handler().SetPointCloudsCallback(cb, client_data);
}

void SetLidarImuDataCallback(LidarImuDataCallback cb, void* client_data) {
  pub_handler().SetImuDataCallback(cb, client_data);
}

//const PointCloudFrame* GetPointCloudFrame() {
//  return pub_handler().GetPointCloudFrame();
//}

bool SetLvxConvertPointFrameDir(const char* dir) {
  return lvx_to_point_cloud_frame_converter().SetLvxConvertDir(std::string(dir));
}

void ConvertToPointFrame(uint8_t publish_frequency, PointCloudsFrameCallback cb, void * client_data) {
  lvx_to_point_cloud_frame_converter().ConvertToPointFrame(publish_frequency, cb, client_data);
  return;
}

void StopLvxConvertToPointFrame() {
  lvx_to_point_cloud_frame_converter().StopConvert();
  return;
}

