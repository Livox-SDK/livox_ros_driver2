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
#ifndef LIVOX_BUILD_REQUEST_H_
#define LIVOX_BUILD_REQUEST_H_

#include <memory>
#include <map>
#include <list>
#include <string>
#include <algorithm>

#include <string.h>

#include "base/io_loop.h"
#include "comm/comm_port.h"

#include "livox_lidar_def.h"
#include "comm/define.h"

#include <memory>

namespace livox {
namespace lidar {


class BuildRequest {
 public:
  static bool BuildUpdateViewLidarCfgRequest(const ViewLidarIpInfo& view_lidar_info, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildUpdateLidarCfgRequest(const LivoxLidarCfg& lidar_cfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildUpdateMid360LidarCfgRequest(const LivoxLidarCfg& lidar_cfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetLidarIPInfoRequest(const LivoxLidarIpInfo& ip_config, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetHostStateInfoIPCfgRequest(const HostStateInfoIpInfo& host_state_info_ipcfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetHostPointDataIPInfoRequest(const HostPointIPInfo& lidar_ip_config, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetHostImuDataIPInfoRequest(const HostImuDataIPInfo& host_imu_ipcfg, uint8_t* req_buf, uint16_t& req_len);
  static bool IpToU8(const std::string& src, const std::string& seq, std::vector<uint8_t>& result);
 private:
  static bool InitLidarIpinfoVal(const LivoxLidarIpInfo& lidar_ip_config, LivoxLidarIpInfoValue* lidar_ipinfo_val_ptr);
  static bool InitHostIpAddr(const std::string& host_ip, HostIpInfoValue* host_ipinfo_val_ptr);
};

} // namespace direct
} // namespace livox

# endif // LIVOX_BUILD_REQUEST_H_
