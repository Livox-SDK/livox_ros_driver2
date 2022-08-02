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

#ifndef LIVOX_PARSE_INPUT_PARAMS_H_
#define LIVOX_PARSE_INPUT_PARAMS_H_

#include "comm/define.h"
#include "livox_def_direct.h"


#include <string>
#include <memory>
#include <vector>
#include <map>

#include <stdio.h>
#include <string.h>

namespace livox {

namespace direct {
class ParseInputParams {
 public:
  ParseInputParams(DirectLidarHostCfg* direct_host_cfg_ptr, DirectLidarCfg* direct_lidar_cfg_ptr, const uint8_t lidars_num);
  bool Parse(std::shared_ptr<DirectHostIpInfo>& direct_host_ipinfo_ptr,
      std::shared_ptr<std::vector<DirectLidarInfo>>& direct_lidars_info_ptr);
 private:
  bool ParseHostIpInfo(const DirectLidarHostCfg* host_cfg_ptr, std::shared_ptr<DirectHostIpInfo>& direct_host_info_ptr);
  bool ParseLidarsInfo(std::shared_ptr<std::vector<DirectLidarInfo>>& direct_lidars_info_ptr);
  bool ParseLidarIpInfo(const DirectLidarIpCfg& lidar_ipinfo_cfg,
      std::shared_ptr<DirectLidarIpInfo>& direct_lidar_ipinfo_ptr);
 private:
  const DirectLidarHostCfg* direct_host_cfg_ptr_;
  const DirectLidarCfg* direct_lidar_cfg_ptr_;
  const uint8_t lidars_num_;
};

} // namespace livox
} // namesace direct

#endif // LIVOX_PARSE_INPUT_PARAMS_H_

