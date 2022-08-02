#ifndef LIVOX_SDK_COMMON_UTIL_H_
#define LIVOX_SDK_COMMON_UTIL_H_


#include <chrono>
#include <vector>
#include <livox_def.h>
#include <string.h>

namespace livox {
namespace common {

typedef float TranslationVector[3]; /**< x, y, z translation, unit: mm. */
typedef float RotationMatrix[3][3];

typedef struct {
  TranslationVector trans;
  RotationMatrix rotation;
} ExtrinsicParameters;

const double PI = 3.14159265358979323846;

typedef struct {
  uint32_t points_per_packet;
  uint32_t points_per_second;
  uint32_t point_interval;  /**< unit:ns */
  uint32_t packet_interval; /**< unit:ns */
  uint32_t data_length;
} PacketInfoPair;

const uint32_t KEthPacketMaxLength = 1500;
const uint32_t KEthPacketHeaderLength = 18; /**< (sizeof(LivoxEthPacket) - 1) */
constexpr uint32_t kMaxProductType = 17;
const int kSeconds = 1000000000; // 1s = 1000000000 ns

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  bool extrinsic_enable;
  uint32_t point_num;
  uint8_t data_type;
  uint8_t line_num;
  uint64_t time_stamp;
  uint64_t point_interval;
  std::vector<uint8_t> raw_data;
} StoragePacket;

typedef struct {
  uint32_t points_per_second; /**< number of points per second */
  uint32_t point_interval;    /**< unit:ns */
  uint32_t line_num;          /**< laser line number */
} ProductTypePointInfoPair;

typedef struct {
  uint32_t points_per_packet; /**< number of points every packet */
  uint32_t packet_length;     /**< length of raw ethenet packet unit:bytes */
  uint32_t raw_point_length;  /**< length of point uint:bytes */
  uint32_t echo_num;          /**< echo number of current data */
} DataTypePointInfoPair;

constexpr DataTypePointInfoPair data_type_info_pair_table[kMaxPointDataType] = {
    {100, KEthPacketHeaderLength + 100 * sizeof(LivoxRawPoint), sizeof(LivoxRawPoint), 1},
    {100, KEthPacketHeaderLength + 100 * sizeof(LivoxSpherPoint),  sizeof(LivoxSpherPoint),  1},
    {96,  KEthPacketHeaderLength + 96  * sizeof(LivoxExtendRawPoint), sizeof(LivoxExtendRawPoint), 1},
    {96,  KEthPacketHeaderLength + 96  * sizeof(LivoxExtendSpherPoint),  sizeof(LivoxExtendSpherPoint),  1},
    {48,  KEthPacketHeaderLength + 48  * sizeof(LivoxDualExtendRawPoint), sizeof(LivoxDualExtendRawPoint), 2},
    {48,  KEthPacketHeaderLength + 48  * sizeof(LivoxDualExtendSpherPoint),  sizeof(LivoxDualExtendSpherPoint), 2},
    {1,   KEthPacketHeaderLength + 1   * sizeof(LivoxImuPoint),   sizeof(LivoxImuPoint), 1},
    {30,  KEthPacketHeaderLength + 30  * sizeof(LivoxTripleExtendRawPoint), sizeof(LivoxTripleExtendRawPoint), 3},
    {30,  KEthPacketHeaderLength + 30  * sizeof(LivoxTripleExtendSpherPoint),  sizeof(LivoxTripleExtendSpherPoint), 3}
};

const PacketInfoPair packet_info_pair_table[kMaxPointDataType] = {
    {100, 100000, 10000, 1000000, 1318},
    {100, 100000, 10000, 1000000, 918},
    {96, 240000, 4167, 400000, 1362},
    {96, 240000, 4167, 400000, 978},
    {48, 480000, 4167, 400000, 1362},
    {48, 480000, 4167, 400000, 978},
    {1, 200, 10000000, 10000000, 42},
};

constexpr ProductTypePointInfoPair product_type_info_pair_table[kMaxProductType] = {
    {100000, 10000, 1},
    {100000, 10000, 1},
    {240000, 4167 , 6}, /**< tele */
    {240000, 4167 , 6},
    {100000, 10000, 1},
    {100000, 10000, 1},
    {100000, 10000, 1}, /**< mid70 */
    {240000, 4167,  6},
    {240000, 4167,  6},
    {200000, 5000,  4}, /**< mid360 */
    {240000, 4167,  6},
    {240000, 4167,  6},
    {240000, 4167,  6},
    {240000, 4167,  6},
    {240000, 4167,  6}, /**< default */
    {270000, 3703,  6}, /**< hap */
    {1800000,555,  16}, /**< pa */
};

inline uint32_t GetPointInterval(uint32_t product_type) {
  return product_type_info_pair_table[product_type].point_interval;
}

inline uint32_t GetLaserLineNumber(uint32_t product_type) {
  return product_type_info_pair_table[product_type].line_num;
}

inline uint32_t GetEthPacketLen(uint32_t data_type) {
  return data_type_info_pair_table[data_type].packet_length;
}

/** 8bytes stamp to uint64_t stamp */
typedef union {
  struct {
    uint32_t low;
    uint32_t high;
  } stamp_word;

  uint8_t stamp_bytes[8];
  int64_t stamp;
} LdsStamp;

inline uint64_t GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);

  if (timestamp_type == kTimestampTypePtp) {
    return time.stamp;
  } else if (timestamp_type == kTimestampTypePpsGps) {
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    time_utc.tm_year = time_stamp[0] + 100; // map 2000 to 1990
    time_utc.tm_mon  = time_stamp[1] - 1;   // map 1~12 to 0~11
    time_utc.tm_mday = time_stamp[2];
    time_utc.tm_hour = time_stamp[3];
    time_utc.tm_min = 0;
    time_utc.tm_sec = 0;

#ifdef WIN32
    uint64_t time_epoch = _mkgmtime(&time_utc);
#else
    uint64_t time_epoch = timegm(&time_utc); // no timezone
#endif
    time_epoch = time_epoch * 1000000 + time.stamp_word.high; // to us
    time_epoch = time_epoch * 1000;                           // to ns

    return time_epoch;
  }
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

inline uint64_t GetVehicleEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);
  if (timestamp_type == kTimestampTypePtp) {
    return time.stamp;
  }
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

inline uint64_t GetDirectEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);
  if (timestamp_type == kTimestampTypePtp) {
    return time.stamp;
  }
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

inline bool GetLidarId(uint8_t lidar_type, uint32_t handle, uint32_t& id) {
  if (lidar_type == kIndustryLidarType) {
    id = handle;
    return true;
  } else if (lidar_type == kVehicleLidarType) {
    id = 1 << 8 | handle;
    return true;
  } else if (lidar_type == kDirectLidarType) {
    id = handle;
    return true;
  } else if (lidar_type == kLivoxLidarType) {
    id = handle;
    return true;
  }
  return false;
}

inline uint8_t GetLidarType(const uint32_t id) {
  if (id < 256) {
    return kIndustryLidarType;
  } else if (id < 512) {
    return kVehicleLidarType;
  } else {
    return kLivoxLidarType;
  }
}

inline uint32_t GetLidarHandle(const uint32_t id) {
  if (id < 256) {
    return id;
  }
  if (id < 512) {
    uint32_t handle = id - (1 << 8);
    return handle;
  }
  return id;
}


}
}

#endif
