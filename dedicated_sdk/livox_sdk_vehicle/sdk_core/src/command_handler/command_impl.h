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

#ifndef LIVOX_SDK_COMMAND_IMPL_H
#define LIVOX_SDK_COMMAND_IMPL_H

#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"

namespace livox {
namespace vehicle {

/** The maximum buffer size of command */
static const uint32_t kMaxCommandBufferSize = 1536;
static const uint16_t KDefaultTimeOut = 500;

static const uint16_t kCommandPort = 56000;
static const uint16_t kPointCloudPort = 57000;
static const uint16_t kImuDataPort = 58000;
static const uint16_t kLogPort = 59000;


/**  Enum that represents the command id. */
typedef enum {
  /**
   * Lidar command set, set the working mode and sub working mode of a LiDAR.
   */
  kCommandIDLidarSearch = 0x00,
  kCommandIDLidarPreconfig = 0x01,
  kCommandIDLidarWorkModeControl = 0x02,
  kCommandIDLidarGetDiagnosisInfo = 0x03,
  kCommandIDLidarGetInternalInfo = 0x04,
  kCommandIDLidarGetExceptionInfo = 0x05,
  kCommandIDLidarSafetyInfo = 0x06,
  kCommandIDLidarRegisterFaultInfo = 0x07,
  kCommandIDGeneralRebootDevice = 0x0a,
  kCommandIDLidarLogInfo = 0x10,
  kCommandIDGeneralRequestUpgrade = 0x20,
  kCommandIDGeneralXferFirmware = 0x21,
  kCommandIDGeneralCompleteXferFirmware = 0x22,
  kCommandIDGeneralRequestUpgradeProgress = 0x23,
  kCommandIDGeneralRequestFirmwareInfo = 0xFF,
  /**
   * ******************************************************
   * Don't add command id after kCommandIDLidarCommandCount.
   */
  kCommandIDLidarCommandCount
} LidarCommandID;

/**
 * Enum that represents the command type.
 */
typedef enum {
  /** command type, which requires response from the receiver. */
  kCommandTypeCmd = 0,
  /** acknowledge type, which is the response of command type. */
  kCommandTypeAck = 1,
} CommandType;

#pragma pack(1)

typedef struct {
  uint8_t ret_code;
  uint8_t broadcast_code[16];
  uint32_t ip_addr;
} SearchLidarResponse;

#pragma pack()

}
}  // namespace livox
#endif  // LIVOX_SDK_COMMAND_IMPL_H
