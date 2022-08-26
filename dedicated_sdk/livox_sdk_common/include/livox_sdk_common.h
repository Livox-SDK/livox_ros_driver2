#ifndef LIVOX_SDK_COMMON_H_
#define LIVOX_SDK_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "livox_def_common.h"

// Get SDK version number
// void GetLivoxSdkVersion(LivoxSdkVersion *version);

/*******Record Lvx File Module***********/
// Save Lvx File
bool SetLvxRecordDir(const char *dir);

// extrinsic need to be set
void AddDeviceInfo(LvxDeviceInfo *info, uint8_t size);

void StartRecordLvxFile();  // deprecated
void StartRecordFile(LvxFileType file_type);  // preferable

void StopRecordLvxFile();

/*******Parse Lvx File Module***********/
bool SetLvxParseDir(const char *dir);

bool IsFileVersionSupported(LvxFileVersion& minimum_version_required);

bool StartParseLvxFile();

//Lvx file's total frame count
uint32_t GetLvxFrameCount();

//Frame duration between frame index: default 50ms
uint32_t GetLvxFrameDuration();

//Get Device's extrinsic param slot etc.
void GetLvxDeviceInfoList(LvxDeviceInfo *info, uint8_t *size);

// Get 32bit point data for a specific frame (to be deprecated)
const LvxBasePackDetail* GetLvxPointCloud(int frame_index, uint32_t *packet_num);

// Get 32bit & 16bit point data for a specific frame
LvxPointsPackInfo GetLvxPointData(int frame_index);

bool CutLvxFile(uint64_t start_frame_index, uint64_t end_frame_index, const char* dir);

void StopParseLvxFile();

/*******Convert Lvx File Module***********/
// Convert a .lvx2 file to a .lvx3 file
bool Lvx2ToLvx3(const char *lvx_file_path, const char *output_dir);

// Convert a .lvx3 file to a .lvx2 file
bool Lvx3ToLvx2(const char *lvx2_file_path, const char *output_dir);

/*******Upgrade Module***********/
bool SetVehicleUpgradeFirmwarePath(const char* firmware_path);

void UpgradeVehicleLidar(const uint8_t slot);

void UpgradeVehicleLidars(const uint8_t* slot_vec, const uint8_t lidar_num);

// Livox lidar upgrade
bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path);
typedef void (*OnLivoxLidarUpgradeProgressCallback)(LivoxLidarType type,
                                               uint32_t handle,
                                               LidarUpgradeState state,
                                               void *client_data);
void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data);
void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num);

bool SetUpgradeFirmwarePath(const char *dir);

typedef void (*OnUpgradeProgressCallbackCallback)(LivoxLidarType type,
                                               uint8_t slot,
                                               LidarUpgradeState state,
                                               void *client_data);

void SetUpgradeProgressCallback(OnUpgradeProgressCallbackCallback, void* client_data);

void StartUpgradeLidar(LivoxLidarType type, uint8_t slot);

void StopUpgradeLidar(LivoxLidarType type, uint8_t slot);

/******** Point Publish Module ******/
typedef void (*PubPointCloudCallback)(PointCloudFrame *point_cloud_frame, void *client_data);

void SetPointCloudCallback(PubPointCloudCallback cb, void *client_data);

void SetPointCloudConfig(PointCloudConfig config);

void AddLidarsExtrinsicParams(LidarExtrinsicParameters params);

typedef void (*LidarImuDataCallback)(LidarImuPoint* imu_data, void *client_data);
void SetLidarImuDataCallback(LidarImuDataCallback cb, void* client_data);

bool SetLvxConvertPointFrameDir(const char* dir);

typedef void (*PointCloudsFrameCallback)(uint32_t frame_index, uint32_t total_frame, PointCloudFrame *point_cloud_frame, void *client_data);
void ConvertToPointFrame(uint8_t publish_frequency, PointCloudsFrameCallback cb, void * client_data);

void StopLvxConvertToPointFrame();


/******* Obstacle Avoidance *********/
void SetObstacleAvoidanceCubeParams(ObstacleZoneCubeParam* param, uint8_t para_num);

void SetObstacleAvoidanceCylinderParams(ObstacleZoneCylinderParam* param, uint8_t para_num);

typedef void (*ObstacleZoneConvertProgressCallback)(uint8_t progress, void* client_data);
bool AvoidanceObstacleZoneFileConvert(const char* file_name, ObstacleZoneConvertProgressCallback cb, void* client_data);

void ResetAllObstacleAvoidanceParams();

int AvoidanceObstacleTestPoint(int x, int y, int z);

#ifdef __cplusplus
}
#endif

#endif //LIVOX_SDK_COMMON_H_
