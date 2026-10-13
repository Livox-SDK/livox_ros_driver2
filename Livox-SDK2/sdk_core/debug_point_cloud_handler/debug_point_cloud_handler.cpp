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

#include "debug_point_cloud_handler.h"

#include "FastCRC/FastCRC.h"
#include "spdlog/fmt/fmt.h"

#include <algorithm>
#include <iostream>
#include <chrono>
#ifdef WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace livox {
namespace lidar {

DebugPointCloudHandler::DebugPointCloudHandler(std::uint32_t handle, std::string sn, std::uint8_t dev_type, std::string path)
                        : handle_(handle), sn_(sn), dev_type_(dev_type), file_size_(0) {
  if (path.back() == '/') path.pop_back();
  file_path_ = path;
}

DebugPointCloudHandler::~DebugPointCloudHandler() {
  if (thread_ptr_) {
    enable_.store(false);
    cv_.notify_all();
    thread_ptr_->join();
    thread_ptr_ = nullptr;
  }
  if (file_handle_) {
    std::fclose(file_handle_);
    file_handle_ = nullptr;
  }
}

const std::string GetCurrentSystemTime() {
	auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::tm now_tm;
  #ifdef WIN32
  localtime_s(&now_tm, &now);
  #else
  localtime_r(&now, &now_tm);
  #endif
  char buffer[128];
  strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", &now_tm);
	return buffer;
}

bool DebugPointCloudHandler::StoreData(uint8_t* buf, uint32_t buf_size) {
  if (buf) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    std::copy(buf, buf + buf_size, std::back_inserter(data_));
    lock.unlock();
    cv_.notify_one();
    return true;
  }
  return false;
}

void DebugPointCloudHandler::WriteData() {
  while (enable_.load()) {
    std::vector<uint8_t> local_data;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      cv_.wait_for(lock, std::chrono::milliseconds(100),
                   [this] { return !data_.empty() || !enable_.load(); });
      if (!enable_.load()) break;
      local_data.swap(data_);
    }
    // 锁已释放，在锁外执行文件 I/O

    if (local_data.empty()) continue;

    if (file_size_ >= max_file_size_) {
      LOG_WARN("{} file size over 4 GB", file_name_);
      break;
    }
    if (!file_handle_) {
      char name_buf[256];
      snprintf(name_buf, sizeof(name_buf), "lidar_%u_%s.LivoxDebugPointCloudData",
               handle_, GetCurrentSystemTime().c_str());
      file_name_ = name_buf;
      std::string full_path = file_path_ + "/" + file_name_;
      file_handle_ = std::fopen(full_path.c_str(), "ab");
      if (!file_handle_) {
        LOG_ERROR("Failed to open {} for writing", full_path);
        break;
      }
      // write file header
      FastCRC16 crc_16;
      LivoxLidarDebugPointCloudFileHeader file_header;
      file_header.file_ver  = {0x01};
      file_header.dev_type  = {dev_type_};
      file_header.data_type = {0x01};
      memcpy(file_header.sn, sn_.c_str(), sizeof(file_header.sn));
      memset(file_header.rsvd, 0, sizeof(file_header.rsvd));
      file_header.crc16 = crc_16.ccitt(
          reinterpret_cast<const uint8_t*>(&file_header),
          offsetof(LivoxLidarDebugPointCloudFileHeader, crc16));
      std::fwrite(&file_header, 1, sizeof(file_header), file_handle_);
      file_size_ += sizeof(file_header);
    }

    std::fwrite(local_data.data(), 1, local_data.size(), file_handle_);
    file_size_ += local_data.size();
    std::fflush(file_handle_);
#ifdef WIN32
    _commit(_fileno(file_handle_));
#else
    fsync(fileno(file_handle_));
#endif
  }
}

bool DebugPointCloudHandler::Enable(bool enable) {
  enable_.store(enable);
  if (enable) {
    if (thread_ptr_) {
      cv_.notify_all();
      thread_ptr_->join();
      thread_ptr_ = nullptr;
    }
    thread_ptr_ = std::make_shared<std::thread>(&DebugPointCloudHandler::WriteData, this);
  } else {
    cv_.notify_all();
  }
  return true;
}

} // namespace lidar
}  // namespace livox
