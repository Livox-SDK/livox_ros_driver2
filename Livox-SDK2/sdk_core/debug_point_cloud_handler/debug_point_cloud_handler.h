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

#ifndef DEBUG_POINT_CLOUD_HANDLER_H_
#define DEBUG_POINT_CLOUD_HANDLER_H_

#include "base/io_thread.h"
#include "base/noncopyable.h"
#include "base/logging.h"

#include "command_handler/command_impl.h"
#include "comm/define.h"

#include <string>
#include <cstdio>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

namespace livox {
namespace lidar {

class DebugPointCloudHandler {
 public:
  DebugPointCloudHandler(std::uint32_t handle, std::string sn, std::uint8_t dev_type, std::string path);
  ~DebugPointCloudHandler();
  bool StoreData(uint8_t* buf, uint32_t buf_size);
  void WriteData();
  bool Enable(bool enable);

 private:
  std::uint32_t         handle_;
  std::string           sn_;
  std::uint8_t          dev_type_;
  std::uint64_t         file_size_{0};
  std::string           file_path_{""};
  std::string           file_name_;
  std::atomic<bool>     enable_{false};

  std::vector<uint8_t>  data_;
  std::mutex            data_mutex_;
  std::condition_variable cv_;

  std::shared_ptr<std::thread>  thread_ptr_{nullptr};
  FILE*                         file_handle_{nullptr};

  static constexpr uint64_t max_file_size_ = {4ULL * 1024 * 1024 * 1024};
};

} // namespace lidar
}  // namespace livox

#endif  // DEBUG_POINT_CLOUD_HANDLER_H_
