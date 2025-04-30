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

#include "lidar_diagn_data_share.h"
#include <vector>
#include<algorithm>

namespace livox_ros {

static std::string TranslateDiagnStatusCode2String(uint8_t code) {
  switch(code) {
    case LidarDiagStatusLevelNormal:
      return std::string("Normal.");
    case LidarDiagStatusLevelWarning:
      return std::string("Warning.");
    case LidarDiagStatusLevelError:
      return std::string("Error.");
    case LidarDiagStatusLevelSafertyErr:
      return std::string("SafertyErr.");
    default:
      return std::string("Undefined.");
  }
}

void CreateDiagnStatusCode(uint16_t lidar_diag_status, LidarDiagnData::StatusCode& status_code) {
  uint8_t system_module = lidar_diag_status && 0x000f;
  uint8_t scan_module = (lidar_diag_status >> 4) && 0x000f;
  uint8_t ranging_module = (lidar_diag_status >> 8) && 0x000f;
  uint8_t communication_module = (lidar_diag_status >> 12) && 0x000f;
  std::vector<uint8_t> status_codes_all { system_module, scan_module, ranging_module, communication_module };
  uint8_t global = *std::max_element(status_codes_all.begin(), status_codes_all.begin());

  status_code.system_module = StatusCodeInfo(system_module, TranslateDiagnStatusCode2String(system_module));
  status_code.scan_module = StatusCodeInfo(scan_module, TranslateDiagnStatusCode2String(scan_module));
  status_code.ranging_module = StatusCodeInfo(ranging_module, TranslateDiagnStatusCode2String(ranging_module));
  status_code.communication_module = StatusCodeInfo(communication_module, TranslateDiagnStatusCode2String(communication_module));
  status_code.global = StatusCodeInfo(global, TranslateDiagnStatusCode2String(global));
}

void CreateDiagnCodeInfo(uint32_t hms_code_full, HmsDiagnCodeInfo& hms_diagn_code_info) {
  uint16_t hms_abnormal_id = (hms_code_full & 0xffff0000) >> 16;
  uint8_t hms_abnormal_level = hms_code_full & 0xff;

  std::string hms_abnormal_description = "This is undescribed HMS code.";
  std::string hms_suggested_solution = "";

  if (hms_diagn_codes.find(hms_abnormal_id) != hms_diagn_codes.end()) {
    hms_abnormal_description = std::get<0>(hms_diagn_codes[hms_abnormal_id]);
    hms_suggested_solution = std::get<1>(hms_diagn_codes[hms_abnormal_id]);
  }

  switch (hms_abnormal_level) {
    case HmsDiagnAbnormalLevelOk:
      hms_diagn_code_info = { hms_abnormal_id, HmsDiagnAbnormalLevelOk,
        { std::string("OK - ") + hms_abnormal_description, hms_suggested_solution } };
      break;
    case HmsDiagnAbnormalLevelInfo:
      hms_diagn_code_info = { hms_abnormal_id, HmsDiagnAbnormalLevelInfo,
        { std::string("INFO - ") + hms_abnormal_description, hms_suggested_solution } };
      break;
    case HmsDiagnAbnormalLevelWarning:
      hms_diagn_code_info = { hms_abnormal_id, HmsDiagnAbnormalLevelWarning,
        { std::string("WARN - ") + hms_abnormal_description, hms_suggested_solution } };
      break;
    case HmsDiagnAbnormalLevelError:
      hms_diagn_code_info = { hms_abnormal_id, HmsDiagnAbnormalLevelError,
        { std::string("ERROR - ") + hms_abnormal_description, hms_suggested_solution } };
      break;
    case HmsDiagnAbnormalLevelFatal:
      hms_diagn_code_info = { hms_abnormal_id, HmsDiagnAbnormalLevelFatal,
        { std::string("FATAL - ") + hms_abnormal_description, hms_suggested_solution } };
      break;
    default:
      hms_diagn_code_info = { hms_abnormal_id, HmsDiagnAbnormalLevelUnkown,
        { std::string("UNKOWN - ") + hms_abnormal_description, hms_suggested_solution } };
      break;
  }
}

void LidarDiagnDataShare::Push(const LidarDiagnData* lidar_diagn_data) {
  // LidarDiagnData data(*lidar_diagn_data);
  std::lock_guard<std::mutex> lock(mutex_);
  lidar_diagn_data_share_ = *lidar_diagn_data;
}

bool LidarDiagnDataShare::Pop(LidarDiagnData& lidar_diagn_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (lidar_diagn_data_share_.empty()) {
    return false;
  }
  lidar_diagn_data = lidar_diagn_data_share_;
  return true;
}

bool LidarDiagnDataShare::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return lidar_diagn_data_share_.empty();
}

void LidarDiagnDataShare::Clear() {
  LidarDiagnData tmp_lidar_diagn_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_diagn_data_share_ = tmp_lidar_diagn_data_queue;
  }
}

} // namespace livox_ros