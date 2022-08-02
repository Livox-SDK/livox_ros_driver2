#ifndef LIVOX_DEF_COMMON_H_
#define LIVOX_DEF_COMMON_H_

#include "livox_def_vehicle.h"

#define kMaxPointNumber 100
#define kDefaultFrameDurationTime 50
#define kMaxLidarNumber 32

/** The numeric version information struct.  */
// typedef struct {
//   int major{0};       /**< major number */
//   int minor{0};       /**< minor number */
//   int patch{0};       /**< patch number */
//   int build{0};       /**< build number used to distinguish between industrial and automotive SDK version*/
// } LivoxSdkVersion;

typedef enum {
  kIndustryLidarType = 1,
  kVehicleLidarType = 2,
  kDirectLidarType = 4,
  kLivoxLidarType = 8
} LivoxLidarType;

typedef enum {
  kPublishFrequency10Hz = 0,
  kPublishFrequency20Hz = 1,
} LivoxPublishFrequency;

typedef enum {
  kUpgradeIdle = 0,
  kUpgradeRequest = 1,
  kUpgradeXferFirmware = 2,
  kUpgradeCompleteXferFirmware = 3,
  kUpgradeGetUpgradeProgress = 4,
  kUpgradeComplete = 5,
  kUpgradeTimeout = 6,
  kUpgradeErr = 7,
  kUpgradeUndef = 8
} FsmState;

typedef enum {
  kEventRequestUpgrade = 0,
  kEventXferFirmware = 1,
  kEventCompleteXferFirmware = 2,
  kEventGetUpgradeProgress = 3,
  kEventComplete = 4,
  kEventReinit = 5,
  kEventTimeout = 6,
  kEventErr = 7,
  kEventUndef = 8
} FsmEvent;

typedef enum {
  LIVOX_FILE_LVX,        // 1.1.0.0
  LIVOX_FILE_LVX2,       // 2.0.0.0
  LIVOX_FILE_LVX3_PPT,   // 2.4.0.0
  LIVOX_FILE_UNKNOWN
} LvxFileType;

typedef struct {
  uint8_t file_ver_major;
  uint8_t file_ver_minor;
  uint8_t file_ver_patch;
  uint8_t file_ver_build;
} LvxFileVersion;

#pragma pack(1)
typedef struct {
  uint8_t signature[16];
  uint8_t version[4];
  uint32_t magic_code;
} LvxFilePublicHeader;

typedef struct {
  uint32_t frame_duration;
  uint8_t device_count;
} LvxFilePrivateHeader;

// typedef struct {
//   uint8_t lidar_broadcast_code[16];
//   uint8_t hub_broadcast_code[16];
//   //slot for LivoxLidarType::kVehicleLidarType
//   // handle for LivoxLidarType::kIndustryLidarType
//   uint8_t lidar_id;
//   uint8_t lidar_type; //refer to LivoxLidarType
//   uint8_t device_type;
//   uint8_t extrinsic_enable;
//   float roll;
//   float pitch;
//   float yaw;
//   float x;
//   float y;
//   float z;
// } LvxDeviceInfo;

typedef struct {
  uint8_t lidar_broadcast_code[16];
  uint8_t hub_broadcast_code[16];
  uint32_t lidar_id;
  uint8_t lidar_type; //refer to LivoxLidarType
  uint8_t device_type;
  uint8_t extrinsic_enable;
  float roll;
  float pitch;
  float yaw;
  float x;
  float y;
  float z;
} LvxDeviceInfo;

// typedef struct {
//   uint8_t version;    // protocol version
//   //slot for LivoxLidarType::kVehicleLidarType
//   //handle for LivoxLidarType::kIndustryLidarType
//   uint8_t lidar_id;
//   uint8_t lidar_type; //refer to LivoxLidarType
//   uint8_t timestamp_type;
//   uint8_t timestamp[8];
//   uint16_t udp_counter;
//   uint8_t data_type;
//   uint32_t length;   // equal to point_number * sizeof(LivoxExtendRawPoint)
//   uint8_t frame_counter;
//   uint8_t reserve[4];
// } LvxBasePackHeader;

typedef struct {
  uint8_t version;    // protocol version
  //slot for LivoxLidarType::kVehicleLidarType
  //handle for LivoxLidarType::kIndustryLidarType
  uint32_t lidar_id;
  uint8_t lidar_type; //refer to LivoxLidarType
  uint8_t timestamp_type;
  uint8_t timestamp[8];
  uint16_t udp_counter;
  uint8_t data_type;
  uint32_t length;   // equal to point_number * sizeof(LivoxExtendRawPoint)
  uint8_t frame_counter;
  uint8_t reserve[4];
} LvxBasePackHeader;

typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarExtendRawPoint;

typedef struct {
  int16_t x;            /**< X axis, Unit:cm */
  int16_t y;            /**< Y axis, Unit:cm */
  int16_t z;            /**< Z axis, Unit:cm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarExtendHalfRawPoint;

typedef struct {
  LvxBasePackHeader header;
  LivoxLidarExtendRawPoint raw_point[kMaxPointNumber];
} LvxBasePackDetail;

typedef struct {
  LvxBasePackHeader header;
  LivoxLidarExtendHalfRawPoint raw_point[kMaxPointNumber];
} LvxBaseHalfPackDetail;

typedef struct {
  LvxBasePackDetail*      lvx_pack_detail;
  uint32_t                pack_num;
  LvxBaseHalfPackDetail*  lvx_half_pack_detail;
  uint32_t                half_pack_num;
} LvxPointsPackInfo;

typedef struct {
  FsmEvent state;
  uint8_t progress;
} LidarUpgradeState;

typedef struct {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  uint64_t offset_time;
} PointCloudXyzlt;

typedef struct {
  uint32_t handle;
  // union {
  //   uint8_t handle;
  //   uint8_t slot;
  // };
  uint8_t lidar_type; ////refer to LivoxLidarType
  uint32_t points_num;
  PointCloudXyzlt* points;
} LidarPoint;

typedef struct {
  uint64_t base_time;
  uint8_t lidar_num;
  LidarPoint lidar_point[kMaxLidarNumber];
} PointCloudFrame;

typedef union {
  uint8_t publish_freq;     // point cloud callback frequency. eg, 10 Hz.
  uint64_t max_cache_size;  // max point cloud cache size. eg,  200 MB
} CacheConfigUnion;

typedef struct {
// point cloud callback frequency. eg, 10 Hz. refer to LivoxPublishFrequency
  uint8_t publish_freq;
} PointCloudConfig;

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  // union {
  //   uint8_t handle;
  //   uint8_t slot;
  // };
  float roll;  /**< Roll angle, unit: degree. */
  float pitch; /**< Pitch angle, unit: degree. */
  float yaw;   /**< Yaw angle, unit: degree. */
  int32_t x;   /**< X translation, unit: mm. */
  int32_t y;   /**< Y translation, unit: mm. */
  int32_t z;   /**< Z translation, unit: mm. */
} LidarExtrinsicParameters;

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  uint8_t slot;
  // union {
  //   uint8_t handle;
  //   uint8_t slot;
  // };
  uint64_t time_stamp;
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} LidarImuPoint;

typedef struct {
 int x;
 int y;
 int z;
 int l;
 int w;
 int h;
 float roll;
 float pitch;
 float yaw;
 int mode;
 int priority;
} ObstacleZoneCubeParam;

typedef struct {
 int x;
 int y;
 int z;
 int h;
 float r1;
 float r2;
 float theta;
 float roll;
 float pitch;
 float yaw;
 int mode;
 int priority;
} ObstacleZoneCylinderParam;

#pragma pack()

#endif //LIVOX_DEF_COMMON_H_
