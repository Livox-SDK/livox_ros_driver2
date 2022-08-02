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


#include "comm/define.h"

#include <memory>

namespace livox {

namespace direct {

typedef struct {
  uint8_t host_push_msg_ip[4];
  uint8_t host_push_msg_port[2];

  uint8_t host_point_data_ip[4];
  uint8_t host_point_data_port[2];

  uint8_t host_imu_data_ip[4];
  uint8_t host_imu_data_port[2]; 
} HostIpInfoValue;

typedef struct {
  uint8_t lidar_ipaddr[4];
  uint8_t lidar_subnet_mask[4];
  uint8_t lidar_gateway[4];
} LidarIpInfoValue;

class BuildRequest {
 public:
  bool BuildBroadcastRequest(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr, uint8_t* req_buf, uint16_t& req_len);
  static uint16_t GenerateSeq();
  bool BuildSetUpLidarRequest(const DirectLidarInfo& direct_lidar_info, uint8_t* req_buff, uint16_t& req_len);
 private:
  bool InitHostIpinfoVal(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr, HostIpInfoValue* host_ipinfo_val_ptr);
  bool IpToU8(const std::string& src, const std::string& seq, std::vector<uint8_t>& result);
  bool InitLidarIpinfoVal(std::shared_ptr<DirectLidarIpInfo> lidar_ipinfo_ptr, LidarIpInfoValue* lidar_ipinfo_val_ptr);
};

} // namespace direct
} // namespace livox

# endif // LIVOX_BUILD_REQUEST_H_
