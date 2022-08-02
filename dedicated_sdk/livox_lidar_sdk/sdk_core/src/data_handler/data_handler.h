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

#ifndef LIVOX_DATA_HANDLER_H_
#define LIVOX_DATA_HANDLER_H_

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "comm/define.h"
#include "base/io_loop.h"

namespace livox {
namespace lidar {

class DataHandler : public noncopyable {
 private:
  DataHandler() {}
  DataHandler(const DataHandler& other) = delete;
  DataHandler& operator=(const DataHandler& other) = delete;
 public:
  void Destory();
  ~DataHandler();
  static DataHandler& GetInstance();

  bool Init();

  void Handle(const uint8_t dev_type, const uint32_t handle, uint8_t *buf, uint32_t buf_size);

  uint16_t AddPointCloudObserver(const DataCallback &cb, void *client_data);
  void RemovePointCloudObserver(uint16_t id);

  void SetPointDataCallback(const DataCallback& cb, void *client_data);
  void SetImuDataCallback(const DataCallback& cb, void* client_data);

 private:
  uint16_t GenerateObserverId();
 private:
  DataCallback point_data_callbacks_;
  void* point_client_data_;

  DataCallback imu_data_callbacks_;
  void* imu_client_data_;

  std::map<uint16_t, std::pair<DataCallback, void*>> observers_;
  std::mutex mutex_;
};

} // namespace lidar
}  // namespace livox

#endif  // LIVOX_DATA_HANDLER_H_















