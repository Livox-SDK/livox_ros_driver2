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

#include "logger_handler.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#ifdef WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include "base/logging.h"
#include "command_handler/command_impl.h"
#include "device_manager.h"
#include "livox_lidar_def.h"
#include "file_manager.h"

namespace livox {
namespace lidar {

std::string GetCurFormatTime() {
  std::time_t t = std::time(nullptr);
  std::stringstream format_time;
  format_time << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S");
  return format_time.str();
}

void LoggerHandler::Init() {
  is_stop_write_.store(false);
  thread_ptr_ = std::make_shared<std::thread>(&LoggerHandler::SaveToFile, this);
}

void LoggerHandler::Destory() {
  if (thread_ptr_) {
    is_stop_write_.store(true);
    thread_ptr_->join();
    thread_ptr_ = nullptr;
  }

  for (auto &file: current_files_) {
   auto& fp = file.second.fp;
   if (fp) {
    std::fclose(fp);
    fp = nullptr;
   }
  } 
}

void LoggerHandler::SaveToFile() {
  while (!is_stop_write_.load()) {
    Write();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void LoggerHandler::StoreLogBag(DeviceLoggerFilePushRequest* req, uint8_t flag) {
  LOG_INFO("Transform Data Length : {}", req->data_length);
  WriteBuffer write_buff {};
  write_buff.log_type = req->log_type;
  write_buff.flag = flag;
  write_buff.file_index = req->file_index;
  write_buff.data_length = req->data_length;
  write_buff.trans_index = req->trans_index;
  write_buff.data_ptr.reset(new uint8_t[req->data_length], [] (uint8_t * buff) { delete[] buff;});
  memcpy(write_buff.data_ptr.get(), req->data, write_buff.data_length);

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push(std::move(write_buff));
  }
}

void LoggerHandler::CreateFile(const WriteBuffer& write_buff) {
  std::string current_time_str = GetCurFormatTime();
  uint8_t log_type = write_buff.log_type;
  log_branch_path_[log_type] = log_root_path_ + (log_root_path_.back() == '/' ? "" : "/") + "type_" + std::to_string(log_type);
  if (!IsDirectoryExits(log_branch_path_[log_type])) {
    if (!MakeDirecotory(log_branch_path_[log_type])) {
      LOG_ERROR("Can't Create Dir {}", log_branch_path_[log_type]);
      return;
    }
  }

  // The handling of the four given scenarios has already been included.
  if (current_files_[log_type].fp) {
    if ((current_files_[log_type].trans_index + 1) != write_buff.trans_index) {
      LOG_WARN("The terminal command to end the {}rd log file has been lost.", current_files_[log_type].file_index);
    }
    std::fclose(current_files_[log_type].fp);
    current_files_[log_type].fp = nullptr;
  }

  std::string file_path =  log_branch_path_[log_type] + "/" +
                            current_time_str + "_" + serial_num_ +
                            "_" + std::to_string(log_type) + "_" + std::to_string(write_buff.file_index) + ".dat";
  LOG_INFO("file path : {}", file_path);
  current_files_[log_type].fp = std::fopen(file_path.c_str(), "ab");
  if (current_files_[log_type].fp) {
    std::fwrite(write_buff.data_ptr.get(), 1, write_buff.data_length, current_files_[log_type].fp);
    std::fflush(current_files_[log_type].fp);
#ifdef WIN32
    _commit(_fileno(current_files_[log_type].fp));
#else
    fsync(fileno(current_files_[log_type].fp));
#endif
  }
  current_files_[log_type].flag = write_buff.flag;
  current_files_[log_type].file_index = write_buff.file_index;
  current_files_[log_type].trans_index = write_buff.trans_index;
  current_files_[log_type].file_name = current_time_str + "_" + serial_num_ +
                                      "_" + std::to_string(log_type) + "_" + std::to_string(write_buff.file_index) + ".dat";
  LOG_INFO("Create File index: {}", (int)write_buff.file_index);
}

void LoggerHandler::WriteFile(const WriteBuffer& write_buff) {
  uint8_t log_type = write_buff.log_type;

  //check file index is equal
  if (current_files_[log_type].file_index != write_buff.file_index) {
    LOG_WARN("Log Type: {}, File Index error: last file index: {}, current file index: {}",
            (int)log_type,
            current_files_[log_type].file_index,
            write_buff.file_index);
    return;
  }
  
  if (current_files_[log_type].trans_index + 1 != write_buff.trans_index && write_buff.trans_index != 1) {
    LOG_WARN("Log Type: {}, Trans Index error: last trans index: {}, current trans index: {}",
            (int)log_type,
            current_files_[log_type].trans_index,
            write_buff.trans_index);
  }

  if (current_files_[log_type].fp) {
    std::fwrite(write_buff.data_ptr.get(), 1, write_buff.data_length, current_files_[log_type].fp);
    std::fflush(current_files_[log_type].fp);
#ifdef WIN32
    _commit(_fileno(current_files_[log_type].fp));
#else
    fsync(fileno(current_files_[log_type].fp));
#endif
  } else {
    LOG_ERROR("There is an issue with the lidar firmware, the starting file command is not sent from lidar. trans_index: {}", write_buff.trans_index);
  }
  current_files_[log_type].flag = write_buff.flag;
  current_files_[log_type].trans_index = write_buff.trans_index;
}

void LoggerHandler::StopFile(const WriteBuffer& write_buff) {
  uint8_t log_type = write_buff.log_type;

  if (current_files_[log_type].flag == static_cast<uint8_t>(Flag::kEndFile) &&
      current_files_[log_type].trans_index + 1 != write_buff.trans_index) {
    LOG_ERROR("There is an issue with the lidar firmware. Multiple terminal commands to close the log files with discontinuous trans_index.");
  }

  if (current_files_[log_type].fp) {
    std::fclose(current_files_[log_type].fp);
    current_files_[log_type].fp = nullptr;
  }
  current_files_[log_type].fp = nullptr;
  current_files_[log_type].flag = write_buff.flag;
  current_files_[log_type].trans_index = write_buff.trans_index;
}

void LoggerHandler::Write() {
  std::queue<WriteBuffer> queue;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue.swap(queue_);
  }

  while (!queue.empty()) {
    WriteBuffer& write_buff = queue.front();

    if (write_buff.trans_index < current_files_[write_buff.log_type].trans_index &&
        write_buff.flag != static_cast<uint8_t>(Flag::kCreateFile)) {
      queue.pop();
      continue;
    }

    if(write_buff.flag == static_cast<uint8_t>(Flag::kCreateFile)) {
      CreateFile(write_buff);
    }

    if(write_buff.flag == static_cast<uint8_t>(Flag::kEndFile)) {
      StopFile(write_buff);
    }

    if(write_buff.flag == static_cast<uint8_t>(Flag::kTransferData)) {
      WriteFile(write_buff);
    }

    queue.pop();
  }
}

} // namespace lidar
}  // namespace livox
