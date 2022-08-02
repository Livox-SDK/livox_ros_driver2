//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
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

#ifndef LIVOX_UPGRADE_FIRMWARE_H_
#define LIVOX_UPGRADE_FIRMWARE_H_

#include <fstream>
#include <ios>

#include "FastCRC.h"

namespace livox {
namespace common {

const uint32_t kMd5SignatureLength = 16;
const uint32_t kEnlFileVersionV2 = 0x02000000;
const uint32_t kEnlFileVersionV3 = 0x03000000;

typedef enum {
  kFirmwareMultiApp = 0,
  kFirmwareApp = 1,
  kFirmwareLoader = 2,
  kFirmwareUnknown = 3,
} FirmwareType;

typedef enum {
  kFirmwareDeviceTypeHub = 0,          /**< Livox Hub. */
  kFirmwareDeviceTypeLidarMid40 = 1,   /**< Mid-40. */
  kFirmwareDeviceTypeLidarTele = 2,    /**< Tele. */
  kFirmwareDeviceTypeLidarHorizon = 3, /**< Horizon. */
  kFirmwareDeviceTypeLidarHubV2 = 4,   /**< HubV2. */
  kFirmwareDeviceTypeLidarMidLite = 5, /**< Mid-Lite. */
  kFirmwareDeviceTypeLidarMid70 = 6,   /**< Mid-70. */
  kFirmwareDeviceTypeLidarAvia = 7,    /**< Avia. */
  kFirmwareDeviceTypeLidarXxx1 = 8,    /**< xxx1. */
  kFirmwareDeviceTypeLidarXxx2 = 9,    /**< xxx2. */
  kFirmwareDeviceTypeLidarHap = 10,    /**< Hap */
  kFirmwareDeviceUnkown
} FirmwareDeviceType;

#pragma pack(1)

// typedef struct {
//   uint32_t file_version;
//   uint32_t firmware_version;
//   uint32_t firmware_length;
//   uint8_t firmware_type;
//   uint8_t device_type;
//   uint8_t encrypt_type;
//   uint8_t rsvd[2];
//   uint8_t checksum_type;
//   uint16_t checksum_length;
//   uint8_t checksum[256];
//   uint64_t modify_time;
//   uint16_t header_checksum;
// } LivoxEncryptFirmwareHeader;


typedef struct {
  uint32_t file_version;
  uint32_t firmware_version;
  uint32_t firmware_length;
  uint8_t firmware_type;
  uint8_t device_type;
  uint8_t encrypt_type;
  uint8_t rsvd[2];
  uint8_t checksum_type;
  uint16_t checksum_length;
  uint8_t checksum[128];
  uint8_t hw_whitelist[128];
  uint64_t modify_time;
  uint16_t header_checksum;
} LivoxEncryptFirmwareHeader;

typedef struct {
  uint8_t overall_signature[kMd5SignatureLength];
} LivoxEncryptFirmwareTail;

/** Lidar feature. */
typedef enum {
  kEverythingIsOk = 0,
  kFirmwareOutOfLength = 1,
  kSystemIsNotReady = 2,
  kFirmwareTypeMismatch = 3,
  kUpgradeStateMismatch = 4,
} RequestUpgradeReturnCode;

const uint32_t kGeneralTryCountLimit = 10;
const uint32_t kGetProcessTryCountLimit = 30;
const uint32_t kGetProgressTryCountLimit = 10;

#pragma pack()

class Firmware {
 public:
  Firmware();
  ~Firmware();
  bool Open(const char *firmware_path);
  void Close();

  const uint32_t FirmwarePackageVersion() const { return header_.file_version; }


  LivoxEncryptFirmwareHeader header_;
  uint8_t *data_;
  LivoxEncryptFirmwareTail tail_;
  uint64_t file_size_;

 private:
  uint64_t MiniFileSize();
  bool ReadAndCheckHeader();
  std::ifstream file_;

  FastCRC16 crc16_;
};

}
}

#endif //LIVOX_UPGRADE_FIRMWARE_H_
