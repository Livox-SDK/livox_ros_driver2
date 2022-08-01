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

#include "lds_lvx.h"

#include "livox_sdk_common.h"
#include "livox_def_common.h"
#include "livox_def.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <functional>
#include <memory>
#include <thread>
#include <condition_variable>
#include <mutex>

namespace livox_ros {

// std::condition_variable LdsLvx::cv_;
// std::mutex LdsLvx::mtx_;
std::atomic_bool LdsLvx::is_file_end_(false);

LdsLvx::LdsLvx(double publish_freq) : Lds(publish_freq, kSourceLvxFile), is_initialized_(false) {
}

LdsLvx::~LdsLvx() {
}

int LdsLvx::Init(const char *lvx_path) {
  if (is_initialized_) {
    printf("Livox file data source is already inited!\n");
    return -1;
  }

#ifdef BUILDING_ROS2
  DisableConsoleLogger();
#endif

  printf("Lds lvx init lvx_path:%s.\n", lvx_path);

  if (!SetLvxConvertPointFrameDir(lvx_path)) {
    printf("Set lvx convert point frame dir failed, the path:%s.\n", lvx_path);
    return -1;
  }
 
  is_initialized_ = true;
  return 0;
}

void LdsLvx::OnPointCloudsFrameCallback(uint32_t frame_index, uint32_t total_frame, PointCloudFrame *point_cloud_frame, void *client_data) {
  if (!point_cloud_frame) {
    printf("Point clouds frame call back failed, point cloud frame is nullptr.\n");
    return;
  }

  LdsLvx* lds = static_cast<LdsLvx*>(client_data);
  if (lds == nullptr) {
    printf("Point clouds frame call back failed, client data is nullptr.\n");
  }

  lds->StorageLvxPointData(point_cloud_frame);

  if (frame_index == total_frame) {
    is_file_end_.store(true);
  }
}

void LdsLvx::ReadLvxFile() {
  ConvertToPointFrame(publish_freq_, LdsLvx::OnPointCloudsFrameCallback, this);
  while(!is_file_end_.load());
  StopLvxConvertToPointFrame();
}

}  // namespace livox_ros


