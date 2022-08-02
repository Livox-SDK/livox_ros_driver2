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

#include "build_request.h"
#include <functional>
#include <atomic>
#include "base/logging.h"

#include <stdio.h>

#include "comm/define.h"

#include <memory>

namespace livox {

namespace direct {

bool BuildRequest::BuildBroadcastRequest(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    uint8_t* req_buf, uint16_t& req_len) {
  if (direct_host_ipinfo_ptr == nullptr || req_buf == nullptr) {
    LOG_ERROR("Build broadcast request failed, wrong input params.");
    return false;
  }

  req_len = 0;
  uint16_t list_len = 1;
  memcpy(&req_buf[req_len], &list_len, sizeof(list_len));
  req_len += sizeof(uint16_t) + sizeof(uint16_t);

  DirectKeyValueParam* kv = (DirectKeyValueParam *)(&req_buf[req_len]);
  kv->key = static_cast<uint16_t>(kKyeHostIpAddr);
  kv->length = sizeof(uint8_t) * 18;
  HostIpInfoValue* host_ipinfo_val_ptr = (HostIpInfoValue *)&kv->value[0];
  if (!InitHostIpinfoVal(direct_host_ipinfo_ptr, host_ipinfo_val_ptr)) {
    LOG_ERROR("Build broadcast request failed, init host ip info val failed.");
    return false;
  }

  uint16_t port = 0;
  memcpy(&port, host_ipinfo_val_ptr->host_push_msg_port, sizeof(uint8_t) * 2);
  // LOG_INFO("host_push_msg_ip {} {} {} {}, port {}", host_ipinfo_val_ptr->host_push_msg_ip[0],
  //     host_ipinfo_val_ptr->host_push_msg_ip[1], host_ipinfo_val_ptr->host_push_msg_ip[2],
  //     host_ipinfo_val_ptr->host_push_msg_ip[3], port);


  req_len += sizeof(DirectKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  return true;
}

bool BuildRequest::InitHostIpinfoVal(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr, HostIpInfoValue* host_ipinfo_val_ptr) {
  if (direct_host_ipinfo_ptr== nullptr || host_ipinfo_val_ptr == nullptr) {
    LOG_ERROR("Build braodcast request failed, init host ip info val failed, wrong input params.");
    return false;
  }

  // Set host push msg ip info
  std::vector<uint8_t> vec_host_push_msg_ip;
  if (!IpToU8(direct_host_ipinfo_ptr->host_push_msg_ip, ".", vec_host_push_msg_ip)) {
    LOG_ERROR("Host push cmd msg ip fault, the ip:%s", direct_host_ipinfo_ptr->host_push_msg_ip.c_str());
    return false;
  }
  memcpy(host_ipinfo_val_ptr->host_push_msg_ip, vec_host_push_msg_ip.data(), sizeof(uint8_t) * 4);
  memcpy(host_ipinfo_val_ptr->host_push_msg_port, &(direct_host_ipinfo_ptr->host_push_cmd_port), sizeof(uint16_t));
  // LOG_INFO("InitHostIpinfoVal ip0 : {} {} {} {}, port:{}", vec_host_push_msg_ip[0], vec_host_push_msg_ip[1],
  //     vec_host_push_msg_ip[2], vec_host_push_msg_ip[3], direct_host_ipinfo_ptr->host_push_cmd_port);

  // Set host point data ip info
  std::vector<uint8_t> vec_host_point_data_ip;
  if (!IpToU8(direct_host_ipinfo_ptr->host_point_data_ip, ".", vec_host_point_data_ip)) {
    LOG_ERROR("Host point data ip fault, the ip:%s", direct_host_ipinfo_ptr->host_point_data_ip.c_str());
    return false;
  }
  memcpy(host_ipinfo_val_ptr->host_point_data_ip, vec_host_point_data_ip.data(), sizeof(uint8_t) * 4);
  memcpy(host_ipinfo_val_ptr->host_point_data_port, &(direct_host_ipinfo_ptr->host_point_data_port), sizeof(uint16_t));
  // LOG_INFO("InitHostIpinfoVal ip1 : {} {} {} {}, port:{}", vec_host_point_data_ip[0], vec_host_point_data_ip[1],
  //     vec_host_point_data_ip[2], vec_host_point_data_ip[3], direct_host_ipinfo_ptr->host_point_data_port);

  // Set host imu data ip info
  std::vector<uint8_t> vec_host_imu_data_ip;
  if (!IpToU8(direct_host_ipinfo_ptr->host_imu_data_ip, ".", vec_host_imu_data_ip)) {
     LOG_ERROR("Host point data ip fault, the ip:%s", direct_host_ipinfo_ptr->host_imu_data_ip.c_str());
     return false;  
  }
  memcpy(host_ipinfo_val_ptr->host_imu_data_ip, vec_host_imu_data_ip.data(), sizeof(uint8_t) * 4);
  memcpy(host_ipinfo_val_ptr->host_imu_data_port, &(direct_host_ipinfo_ptr->host_imu_data_port), sizeof(uint16_t));
  // LOG_INFO("InitHostIpinfoVal ip1 : {} {} {} {}, port:{}", vec_host_imu_data_ip[0], vec_host_imu_data_ip[1],
  //     vec_host_imu_data_ip[2], vec_host_imu_data_ip[3], direct_host_ipinfo_ptr->host_imu_data_port);

  return true;
}

bool BuildRequest::IpToU8(const std::string& src, const std::string& seq, std::vector<uint8_t>& result) {
  std::string::size_type pos1, pos2;
  pos2 = src.find(seq);
  pos1 = 0;
  while (std::string::npos != pos2) {
    int32_t val = std::stoi(src.substr(pos1, pos2-pos1));
    if (val < 0 || val > 256) {
      LOG_ERROR("Build broadcast request failed, ip to u8 failed, the ip:{}, the fault val:{}",
          src, val);
      return false;
    }

    result.push_back(val);
    pos1 = pos2 + seq.size();
    pos2 = src.find(seq, pos1);
  }

  if (pos1 != src.length()) {
    int32_t val = std::stoi(src.substr(pos1, pos2-pos1));
    if (val < 0 || val > 256) {
      LOG_ERROR("Build broadcast request failed, ip to u8 failed, the ip:{}, the fault val:{}",
          src, val);
      return false;
    }
    result.push_back(val);
  }

  if (result.size() != 4) {
    LOG_ERROR("Build broadcast request failed, ip to u8 failed, the ip:{}, the val size:{}",
        src.c_str(), result.size());
    return false;
  }
  return true;
}

uint16_t BuildRequest::GenerateSeq() {
  static std::atomic<std::uint16_t> seq(1);
  uint16_t value = seq.load();
  uint16_t desired = 0;
  do {
    if (value == UINT16_MAX) {
      desired = 1;
    } else {
      desired = value + 1;
    }
  } while (!seq.compare_exchange_weak(value, desired));
  return desired;
}

bool BuildRequest::BuildSetUpLidarRequest(const DirectLidarInfo& direct_lidar_info, uint8_t* req_buff, uint16_t& req_len) {
  if (req_buff == nullptr) {
    return false;
  }
  
  req_len = 0;
  uint16_t list_len = 10;

  memcpy(&req_buff[req_len], &list_len, sizeof(uint16_t));
  req_len += sizeof(uint16_t) + sizeof(uint16_t);

  // Set lidar id
  DirectKeyValueParam* kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyLidarID);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.lidar_id;
  req_len += sizeof(DirectKeyValueParam);

  // Set lidar ip_mode
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyLidarIpMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.lidar_ipmode;
  req_len += sizeof(DirectKeyValueParam);

  //Set lidar ipinfo
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyLidarIpAddr);
  kv->length = sizeof(uint8_t) * 12;
  LidarIpInfoValue* lidar_ipinfo_val_ptr = (LidarIpInfoValue *)&kv->value[0];
  if (!InitLidarIpinfoVal(direct_lidar_info.lidar_ipinfo_ptr, lidar_ipinfo_val_ptr)) {
    return false;
  }
  req_len += sizeof(DirectKeyValueParam) - sizeof(uint8_t) + sizeof(LidarIpInfoValue);

  //Set host ipinfo
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKyeHostIpAddr);
  kv->length = sizeof(uint8_t) * 18;
  HostIpInfoValue* host_ipinfo_val_ptr = (HostIpInfoValue *)&kv->value[0];
  if (!InitHostIpinfoVal(direct_lidar_info.host_ipinfo_ptr, host_ipinfo_val_ptr)) {
    return false;
  }
  req_len += sizeof(DirectKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);

  // Set lidar sample mode
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeySampleMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.sample_mode;
  req_len += sizeof(DirectKeyValueParam);


  // Set lidar pattern mode
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPatternMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.pattern_mode;
  req_len += sizeof(DirectKeyValueParam);

  // Set lidar pcl data type
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyPclDataType);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.pcl_data_type;
  req_len += sizeof(DirectKeyValueParam);

  // Set lidar imu data en
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyImuDataEn);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.imu_data_en;
  req_len += sizeof(DirectKeyValueParam);

  // Set lidar install attitude
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyInstallAttitude);
  kv->length = sizeof(InstallAttitude);
  InstallAttitude* install_attitude = (InstallAttitude*)&kv->value[0];
  memcpy(install_attitude, &direct_lidar_info.install_attitude, sizeof(InstallAttitude));
  req_len += sizeof(DirectKeyValueParam) - sizeof(uint8_t) + sizeof(InstallAttitude);

  // Set lidar work mode
  kv = (DirectKeyValueParam *)&req_buff[req_len];
  kv->key = static_cast<uint16_t>(kKeyWorkMode);
  kv->length = sizeof(uint8_t);
  kv->value[0] = direct_lidar_info.work_mode;
  req_len += sizeof(DirectKeyValueParam);

  return true;
}

bool BuildRequest::InitLidarIpinfoVal(std::shared_ptr<DirectLidarIpInfo> lidar_ipinfo_ptr, LidarIpInfoValue* lidar_ipinfo_val_ptr) {
  if (lidar_ipinfo_ptr == nullptr || lidar_ipinfo_val_ptr == nullptr) {
    return false;
  }

  // Set lidar ip info
  std::vector<uint8_t> vec_lidar_ip;
  if (!IpToU8(lidar_ipinfo_ptr->lidar_ipaddr, ".", vec_lidar_ip)) {
    return false;
  }
  memcpy(lidar_ipinfo_val_ptr->lidar_ipaddr, vec_lidar_ip.data(), sizeof(uint8_t) * 4);

  // Set lidar subnet mask
  std::vector<uint8_t> vec_lidar_subnet_mask;
  if (!IpToU8(lidar_ipinfo_ptr->lidar_subnet_mask, ".", vec_lidar_subnet_mask)) {
    return false;
  }
  memcpy(lidar_ipinfo_val_ptr->lidar_subnet_mask, vec_lidar_subnet_mask.data(), sizeof(uint8_t) * 4);

  // Set lidar gateway
  std::vector<uint8_t> vec_lidar_gateway;
  if (!IpToU8(lidar_ipinfo_ptr->lidar_gateway, ".", vec_lidar_gateway)) {
     return false;  
  }
  memcpy(lidar_ipinfo_val_ptr->lidar_gateway, vec_lidar_gateway.data(), sizeof(uint8_t) * 4);
  return true;
}

} // namespace direct

} // namespace livox
