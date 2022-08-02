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

#include "params_check.h"

#include "livox_lidar_def.h"

#include "comm/define.h"
#include "base/logging.h"


#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <set>

namespace livox {
namespace lidar {

ParamsCheck::ParamsCheck(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) : lidars_cfg_ptr_(lidars_cfg_ptr),
    custom_lidars_cfg_ptr_(custom_lidars_cfg_ptr) {}

bool ParamsCheck::Check() {
  if (lidars_cfg_ptr_ == nullptr && custom_lidars_cfg_ptr_ == nullptr) {
    LOG_ERROR("Params check failed, all params is nullptr.");
    return false;
  }

  if (lidars_cfg_ptr_->empty() && custom_lidars_cfg_ptr_->empty()) {
    LOG_ERROR("Params check failed, all livox lidars config is empty.");
    return false;
  }


  return true;
}

bool ParamsCheck::CheckLidarIp() {
  std::set<std::string> lidars_ip;
  for (auto it = lidars_cfg_ptr_->begin(); it != lidars_cfg_ptr_->end(); ++it) {
    if (it->lidar_net_info.lidar_ipaddr.empty()) {
      continue;
    }
    if (lidars_ip.find(it->lidar_net_info.lidar_ipaddr) != lidars_ip.end()) {
      LOG_ERROR("Params check failed, lidar ip conflict, the liar ip:{}", it->lidar_net_info.lidar_ipaddr.c_str());
      return false;
    }
    lidars_ip.insert(it->lidar_net_info.lidar_ipaddr);
  }

  for (auto it = custom_lidars_cfg_ptr_->begin(); it != custom_lidars_cfg_ptr_->end(); ++it) {
    if (it->lidar_net_info.lidar_ipaddr.empty()) {
      LOG_ERROR("Params check failed, custom lidar ipaddr is empty.");
      return false;
    }
    if (lidars_ip.find(it->lidar_net_info.lidar_ipaddr) != lidars_ip.end()) {
      LOG_ERROR("Params check failed, lidar ip conflict the lidar ip:{}", it->lidar_net_info.lidar_ipaddr.c_str());
      return false;
    }
    lidars_ip.insert(it->lidar_net_info.lidar_ipaddr);
  }

  return true;
}


} // namespace lidar
} // namespace livox


