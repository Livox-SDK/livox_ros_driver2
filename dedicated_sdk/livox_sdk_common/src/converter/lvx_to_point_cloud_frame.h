//
// The MIT License (MIT)
//
// Copyright (c) 2021 Livox. All rights reserved.
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

#ifndef LIVOX_LVX_TO_POINT_CLOUD_FRAME_CONVERTER_H_
#define LIVOX_LVX_TO_POINT_CLOUD_FRAME_CONVERTER_H_

#include <livox_def_common.h>
#include <livox_def_vehicle.h>
#include "livox_sdk_common_util.h"
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <atomic>

namespace livox {
namespace common {

class LvxToPointCloudFrameConverter {
public:
  using PointCloudsFrameCallback = std::function<void(uint32_t, uint32_t, PointCloudFrame*, void *)>;
  LvxToPointCloudFrameConverter() = default;
  bool SetLvxConvertDir(const std::string &dir);
  void ConvertToPointFrame(uint8_t publish_frequency, PointCloudsFrameCallback cb, void * client_data);
  void StopConvert();

private:
  void UpdateExtrinsic();
  void PointCloudsConvert(const LvxBasePackDetail& packet);
  void HalfPointCloudsConvert(const LvxBaseHalfPackDetail& packet);
  void SetLidarsOffsetTime(uint64_t base_time);
  uint64_t GetLidarBaseTime(uint8_t id);
  void PublishPointCloudFrame(uint32_t frame_index, uint32_t total_frame_count);

  std::shared_ptr<std::thread> converter_thread_;
  std::map<uint16_t, std::vector<PointCloudXyzlt>> point_clouds_;
  std::map<uint16_t, ExtrinsicParameters> extrinsics_;
  PointCloudFrame frame_;

  std::map<uint32_t, LvxDeviceInfo> device_info_map_;
  PointCloudsFrameCallback callback_;
  void* client_data_ = nullptr;
};

LvxToPointCloudFrameConverter &lvx_to_point_cloud_frame_converter();

}
}  // namespace livox

#endif  // LIVOX_LVX_TO_POINT_CLOUD_FRAME_CONVERTER_H_