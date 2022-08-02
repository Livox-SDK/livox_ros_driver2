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

#include "command_handler.h"
#include <mutex>
#include "base/logging.h"

#include "command_handler_impl.h"

#include "comm/protocol.h"

using std::lock_guard;
using std::mutex;
using std::shared_ptr;
using std::make_pair;
using std::multimap;
using std::pair;

namespace livox {

namespace direct {

CommandHandler &command_handler() {
  static CommandHandler handler;
  return handler;
}

bool CommandHandler::Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr,
    const uint8_t lidars_num) {
  if (direct_host_ipinfo_ptr == nullptr && direct_lidars_info_ptr == nullptr) {
    LOG_ERROR("Command handler init failed, pointers to input parameters are null");
    return false;
  }
  if (lidars_num == 0 || lidars_num >= kMaxLidarCount) {
    LOG_ERROR("Command handler init failed, lidar_num is zero or exceeds the maximum 32");
    return false;
  }

  impl_.reset(new CommandHandlerImpl(this));
  if (impl_ == nullptr || !(impl_->Init(direct_host_ipinfo_ptr, direct_lidars_info_ptr, lidars_num))) {
    return false;
  }

  LOG_INFO("Command Handler Init Succ.");
  return true;
}

bool CommandHandler::Start() {
  return impl_->Start();
}

void CommandHandler::LidarStateInfoCallback(const uint32_t handle, DirectLidarStateInfo* info) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_lidar_info_.find(handle) == handle_lidar_info_.end()) {
      handle_lidar_info_[handle] = *info;
    }
  }

  if (lidar_info_callback_) {
    lidar_info_callback_(handle, info, lidar_info_client_data_);
  }
}

void CommandHandler::DirectLidarCfgUpdateCallback(const uint32_t handle, DirectLidarCmdResInfo* res) {
  if (lidar_cfg_update_callback_) {
    lidar_cfg_update_callback_(handle, res, lidar_cfg_update_client_data_);
  }
}

livox_direct_status CommandHandler::UpdateDirectLidarCfg(std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr) {
  if (impl_ == NULL) {
    return kDirectStatusHandlerImplNotExist;
  }
  return impl_->UpdateDirectLidarCfg(direct_lidars_info_ptr);
}

livox_direct_status CommandHandler::SendCommand(uint32_t handle,
                                         uint16_t command_id,
                                         uint8_t *data,
                                         uint16_t length,
                                         const std::shared_ptr<CommandCallback> &cb) {
  uint8_t lidar_id = 0;
  std::string sn;
  std::string lidar_ip;
  if (!GetLidarInfo(handle, lidar_id, sn, lidar_ip)) {
    return kDirectStatusChannelNotExist;
  }

  if (command_id == kKeyReSetDevice) {
    lidar_ip = "255.255.255.255";
  }

  Command cmd(BuildRequest::GenerateSeq(), command_id, kCommandTypeCmd, lidar_id, sn, data, length, handle,
      lidar_ip, KDefaultTimeOut, cb);
  if (impl_ == nullptr) {
    return kDirectStatusHandlerImplNotExist; 
  }
  impl_->SendAsync(cmd);
  return kDirectStatusSuccess;
}

bool CommandHandler::GetLidarInfo(const uint32_t handle, uint8_t& lidar_id, std::string& sn, std::string& lidar_ip) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_lidar_info_.find(handle) != handle_lidar_info_.end()) {
    const DirectLidarStateInfo& lidar_info = handle_lidar_info_[handle];
    lidar_id = lidar_info.lidar_id;
    sn = lidar_info.sn;
    lidar_ip = lidar_info.lidar_ip;
    return true;
  }
  return false;
}

void CommandHandler::Destory() {
  if (impl_) {
    impl_.reset(nullptr);
  }
}

}  // namespace livox
} // namespace direct

