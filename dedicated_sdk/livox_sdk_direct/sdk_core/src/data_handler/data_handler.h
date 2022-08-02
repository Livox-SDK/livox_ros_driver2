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

#include "data_handler_impl.h"
#include "base/io_loop.h"


namespace livox {

namespace direct {

class DataHandlerImpl;

class DataHandler : public noncopyable {
 public:
  DataHandler() {}
  bool Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr);
  bool Start();
  uint16_t AddPointCloudObserver(const DataCallback &cb, void *client_data);
  void RemovePointCloudObserver(uint16_t id);
  void SetPointDataCallback(const DataCallback& cb, void *client_data);
  void SetImuDataCallback(const DataCallback& cb, void* client_data); 
  void OnDataCallback(const uint32_t handle, void *data, uint16_t size);
  void Destory();

 private:
  uint16_t GenerateObserverId();
 private:
  DataCallback point_data_callbacks_;
  void* point_client_data_;

  DataCallback imu_data_callbacks_;
  void* imu_client_data_;

  std::unique_ptr<DataHandlerImpl> impl_;
  std::map<uint16_t, std::pair<DataCallback, void*>> observers_;
  std::mutex mutex_;
};


DataHandler &data_handler();

} // namespace direct
}  // namespace livox

#endif  // LIVOX_DATA_HANDLER_H_















