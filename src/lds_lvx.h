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

// livox lidar lvx data source

#ifndef LIVOX_ROS_DRIVER_LDS_LVX_H_
#define LIVOX_ROS_DRIVER_LDS_LVX_H_

#include <memory>
#include <atomic>

#include "lds.h"
#include "comm/comm.h"

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace livox_ros {
/**
 * Lidar data source abstract.
 */
class LdsLvx final : public Lds {
 public:
  static LdsLvx *GetInstance(double publish_freq) {
    static LdsLvx lds_lvx(publish_freq);
    return &lds_lvx;
  }

  int Init(const char *lvx_path);
  
  void ReadLvxFile();
 
 private:
  LdsLvx(double publish_freq);
  LdsLvx(const LdsLvx &) = delete;
  ~LdsLvx();
  LdsLvx &operator=(const LdsLvx &) = delete;

  static void OnPointCloudsFrameCallback(uint32_t frame_index, uint32_t total_frame, PointFrame *point_cloud_frame, void *client_data);

 private:
  volatile bool is_initialized_;
  static std::atomic_bool is_file_end_;
};

}  // namespace livox_ros
#endif
