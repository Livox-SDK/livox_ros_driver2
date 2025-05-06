//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#ifndef LIVOX_ROS_DRIVER_LIDAR_DIAGN_DATA_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_DIAGN_DATA_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <utility>

namespace livox_ros {

typedef enum {
  HmsDiagnAbnormalLevelOk = 0x00,  // added value, not defined in Livox documentation
  HmsDiagnAbnormalLevelInfo = 0x01,
  HmsDiagnAbnormalLevelWarning = 0x02,
  HmsDiagnAbnormalLevelError = 0x03,
  HmsDiagnAbnormalLevelFatal = 0x04,
  HmsDiagnAbnormalLevelUnkown = 0xFF,  // added value, not defined in Livox documentation

} HmsDiagnAbnormalLevel;

typedef enum {
  LidarDiagStatusLevelNormal = 0,
  LidarDiagStatusLevelWarning = 1,
  LidarDiagStatusLevelError = 2,
  LidarDiagStatusLevelSafetyErr = 3,
  LidarDiagStatusLevelUnknow = 0xF,  // added value, not defined in Livox documentation
} LidarDiagStatusLevel;

// HMS(health management system) codes
// Diagnostic Trouble Code, Each non-0 value represents a piece of diagnostic information.
// When the radar does not work normally, the cause of the problem can be confirmed through the diagnostic code

typedef std::pair<uint8_t, std::string> StatusCodeInfo; // <LidarDiagStatusLevel, LidarDiagStatusLevelString>
typedef std::tuple<std::string, std::string> HmsDiagnInfo;  // <abnormal description, suggested solution>
typedef std::tuple<uint16_t, uint8_t, HmsDiagnInfo> HmsDiagnCodeInfo;  // <HMS Abnormal ID, HMS Abnormal LEVEL, HMS Diagnostic Info>

static std::map<uint16_t, HmsDiagnInfo> hms_diagn_codes { // <HMS abnormal ID, HmsDiagnInfo>
 { 0x0000, { "No errors, no warnings.", "" } },
 { 0x0102, { "Environment temperature is slightly high.", "Please check the environment temperature and losing heat measures." } },
 { 0x0103, { "Environment temperature is relatively high.", "Please check the environment temperature and losing heat measures." } },
 { 0x0104, { "The window is dirty, which will influence the reliability of the point cloud.", "Please clean the window." } },
 { 0x0105, { "An error occurred during device upgrade process.", "Please restart the upgrade process." } },
 { 0x0111, { "Abnormal temperature of internal components of the device.", "Please check the environment temperature and losing heat measures." } },
 { 0x0112, { "Abnormal temperature of internal components of the device.", "Please check the environment temperature and losing heat measures." } },
 { 0x0113, { "IMU stopped working.", "Please try to restart the device to restore." } },
 { 0x0114, { "Environment temperature is high.", "Please check the environment temperature and losing heat measures." } },
 { 0x0115, { "Environment temperature beyond the limit, the device has stopped working.", "Please check the environment temperature and losing heat measures." } },
 { 0x0116, { "Abnormal external voltage.", "Please check the external voltage." } },
 { 0x0117, { "Abnormal lidar parameters.", "Please try to restart the device to restore." } },
 { 0x0201, { "Scan module is heating.", "Please wait for the scan module heating." } },
 { 0x0210, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0211, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0212, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0213, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0214, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0215, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0216, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0217, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0218, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0219, { "Scan module is abnormal, the system is trying to recover.", "Please wait, if it lasts too long, please try restarting the device to restore." } },
 { 0x0401, { "PPS time synchronization fails because of loss of GPS signal.", "Please check the GPS signal." } },
 { 0x0402, { "PTP time synchronization stop or time gap is too big.", "Please check the PTP time source." } },
 { 0x0403, { "The version of PTP is 1588-v2.1, device don’t support this version.", "Please replace 1588-v2.1 version with 1588.2.0 version." } },
 { 0x0404, { "PPS time synchronization abnormal.", "Please check the PPS and GPS signal." } },
 { 0x0405, { "There was an exception in time synchronization.", "Please check the exception reason." } },
 { 0x0406, { "Time synchronization accuracy is low.", "Please check the time source." } },
 { 0x0407, { "PPS time synchronization fails because of loss of GPS signal.", "Please check the GPS signal." } },
 { 0x0408, { "PPS time synchronization fails because of loss of PPS signal.", "Please check the PPS signal." } },
 { 0x0409, { "GPS signal is abnormal.", "Please check the GPS time source." } },
 { 0x040A, { "The PTP and GPTP signals exist at the same time.", "Please check the network topology, use PTP or GPTP alone to synchronize." } },
 { 0xFFFF, { "Health Management System (HMS) codes not received.", "" } },  // added value, not defined in Livox documentation
};

typedef struct LidarDiagnData {
  uint8_t lidar_type;
  uint32_t handle;
  uint64_t time_stamp;
  // LiDAR diag status code
  struct StatusCode {
    StatusCodeInfo global;
    StatusCodeInfo system_module;
    StatusCodeInfo scan_module;
    StatusCodeInfo ranging_module;
    StatusCodeInfo communication_module;
  } status_code;
  std::vector<HmsDiagnCodeInfo> hms_diagn;

  LidarDiagnData(): lidar_type(0), handle(0), time_stamp(0) {}

  bool empty() { return time_stamp == 0.0; }
} LidarDiagnData;

void CreateDiagnStatusCode(uint16_t lidar_diag_status, LidarDiagnData::StatusCode& status_code);
void CreateDiagnCodeInfo(uint32_t hms_code_full, HmsDiagnCodeInfo& hms_diagn_code_info);

class LidarDiagnDataShare {
 public:
  void Push(const LidarDiagnData* lidar_diagn_data);
  bool Pop(LidarDiagnData& lidar_diagn_data);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  LidarDiagnData lidar_diagn_data_share_;
};

} // namespace

#endif // #define LIVOX_ROS_DRIVER_LIDAR_DIAGN_DATA_QUEUE_H_


