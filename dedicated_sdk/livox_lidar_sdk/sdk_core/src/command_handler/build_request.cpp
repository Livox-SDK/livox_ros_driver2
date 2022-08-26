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
namespace lidar {

bool BuildRequest::BuildUpdateViewLidarCfgRequest(const ViewLidarIpInfo& view_lidar_info, uint8_t* req_buf, uint16_t& req_len) {
  req_len = 0;
  uint16_t key_num = 2;
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * point_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  point_kv->key = static_cast<uint16_t>(kKeyLidarPointDataHostIPCfg);
  point_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_point_ip_info_val = (HostIpInfoValue*)&point_kv->value;
  if (!InitHostIpAddr(view_lidar_info.host_ip, host_point_ip_info_val)) {
    LOG_ERROR("Build update view lidar cfg request failed, init host ip addr failed.");
    return false;
  }

  memcpy(&(host_point_ip_info_val->host_port), &view_lidar_info.host_point_port, sizeof(view_lidar_info.host_point_port));
  memcpy(&(host_point_ip_info_val->lidar_port), &view_lidar_info.lidar_point_port, sizeof(view_lidar_info.lidar_point_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);

  LivoxLidarKeyValueParam * imu_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  imu_kv->key = static_cast<uint16_t>(kKeyLidarImuHostIPCfg);
  imu_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_imu_ip_info_val = (HostIpInfoValue*)&imu_kv->value;
  if (!InitHostIpAddr(view_lidar_info.host_ip, host_imu_ip_info_val)) {
    LOG_ERROR("Build update view lidar cfg request failed, init imu host ip addr failed.");
    return false;
  }

  memcpy(&(host_imu_ip_info_val->host_port), &view_lidar_info.host_imu_data_port, sizeof(view_lidar_info.host_imu_data_port));
  memcpy(&(host_imu_ip_info_val->lidar_port), &view_lidar_info.lidar_imu_data_port, sizeof(view_lidar_info.lidar_imu_data_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  return true;
}

bool BuildRequest::BuildUpdateMid360LidarCfgRequest(const LivoxLidarCfg& lidar_cfg,
    uint8_t* req_buf, uint16_t& req_len) {
  
  uint16_t key_num = 3;
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * state_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  state_kv->key = static_cast<uint16_t>(kKeyStateInfoHostIPCfg);
  state_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_state_ip_info_val = (HostIpInfoValue*)&state_kv->value;
  if (!InitHostIpAddr(lidar_cfg.host_net_info.push_msg_ip, host_state_ip_info_val)) {
    LOG_ERROR("Build update lidar cfg request failed, init host ip addr failed.");
    return false;
  }

  uint16_t lidar_state_port = kMid360LidarPushMsgPort;
  memcpy(&(host_state_ip_info_val->host_port), &lidar_cfg.host_net_info.push_msg_port, sizeof(lidar_cfg.host_net_info.push_msg_port));
  memcpy(&(host_state_ip_info_val->lidar_port), &lidar_state_port, sizeof(lidar_state_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);

  LivoxLidarKeyValueParam * point_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  point_kv->key = static_cast<uint16_t>(kKeyLidarPointDataHostIPCfg);
  point_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_point_ip_info_val = (HostIpInfoValue*)&point_kv->value;
  if (!InitHostIpAddr(lidar_cfg.host_net_info.point_data_ip, host_point_ip_info_val)) {
    LOG_ERROR("Build update lidar cfg request failed, init host ip addr failed.");
    return false;
  }

  uint16_t lidar_point_port = kMid360LidarPointCloudPort;
  memcpy(&(host_point_ip_info_val->host_port), &lidar_cfg.host_net_info.point_data_port, sizeof(lidar_cfg.host_net_info.point_data_port));
  memcpy(&(host_point_ip_info_val->lidar_port), &lidar_point_port, sizeof(lidar_point_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);

  LivoxLidarKeyValueParam * imu_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  imu_kv->key = static_cast<uint16_t>(kKeyLidarImuHostIPCfg);
  imu_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_imu_ip_info_val = (HostIpInfoValue*)&imu_kv->value;
  if (!InitHostIpAddr(lidar_cfg.host_net_info.imu_data_ip, host_imu_ip_info_val)) {
    LOG_ERROR("Build update lidar cfg request failed, init imu host ip addr failed.");
    return false;
  }

  uint16_t lidar_imu_port = kMid360LidarImuDataPort;
  memcpy(&(host_imu_ip_info_val->host_port), &lidar_cfg.host_net_info.imu_data_port, sizeof(lidar_cfg.host_net_info.imu_data_port));
  memcpy(&(host_imu_ip_info_val->lidar_port), &lidar_imu_port, sizeof(lidar_imu_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  // LOG_ERROR("Build imu host ip:{}, host_port:{}, lidar_port:{}", lidar_cfg.host_net_info.imu_data_ip.c_str(),
  //     lidar_cfg.host_net_info.imu_data_port, lidar_imu_port);
  return true;

}

bool BuildRequest::BuildUpdateLidarCfgRequest(const LivoxLidarCfg& lidar_cfg,
    uint8_t* req_buf, uint16_t& req_len) {
  
  uint16_t key_num = 0;
  if(lidar_cfg.device_type == kLivoxLidarTypePA) {
    key_num = 1;
  } else {
    key_num = 2;
  }
  
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * point_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  point_kv->key = static_cast<uint16_t>(kKeyLidarPointDataHostIPCfg);
  point_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_point_ip_info_val = (HostIpInfoValue*)&point_kv->value;
  if (!InitHostIpAddr(lidar_cfg.host_net_info.point_data_ip, host_point_ip_info_val)) {
    LOG_ERROR("Build update lidar cfg request failed, init host ip addr failed.");
    return false;
  }

  uint16_t lidar_point_port = 0;
  if (lidar_cfg.device_type == kLivoxLidarTypeIndustrialHAP) {
    lidar_point_port = kHAPPointDataPort;
  } else if (lidar_cfg.device_type == kLivoxLidarTypeMid360) {
    lidar_point_port = kMid360LidarPointCloudPort;
  } else if (lidar_cfg.device_type == kLivoxLidarTypePA) {
    lidar_point_port = kPaLidarPointCloudPort;
  } else {
    LOG_ERROR("Build update lidar cfg request failed, unknown the dev_type:{}", lidar_cfg.device_type);
    return false;
  }
  memcpy(&(host_point_ip_info_val->host_port), &lidar_cfg.host_net_info.point_data_port, sizeof(lidar_cfg.host_net_info.point_data_port));
  memcpy(&(host_point_ip_info_val->lidar_port), &lidar_point_port, sizeof(lidar_point_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);

  if (lidar_cfg.device_type == kLivoxLidarTypePA) {
    return true;
  }

  LivoxLidarKeyValueParam * imu_kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  imu_kv->key = static_cast<uint16_t>(kKeyLidarImuHostIPCfg);
  imu_kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_imu_ip_info_val = (HostIpInfoValue*)&imu_kv->value;
  if (!InitHostIpAddr(lidar_cfg.host_net_info.imu_data_ip, host_imu_ip_info_val)) {
    LOG_ERROR("Build update lidar cfg request failed, init imu host ip addr failed.");
    return false;
  }

  uint16_t lidar_imu_port = 0;
  if (lidar_cfg.device_type == kLivoxLidarTypeIndustrialHAP) {
    lidar_imu_port = kHAPIMUPort;
  } else if (lidar_cfg.device_type == kLivoxLidarTypeMid360) {
    lidar_imu_port = kMid360LidarImuDataPort;
  } else {
    LOG_ERROR("Build update lidar cfg request failed, unknown the dev_type:{}", lidar_cfg.device_type);
    return false;
  }

  memcpy(&(host_imu_ip_info_val->host_port), &lidar_cfg.host_net_info.imu_data_port, sizeof(lidar_cfg.host_net_info.imu_data_port));
  memcpy(&(host_imu_ip_info_val->lidar_port), &lidar_imu_port, sizeof(lidar_imu_port));
  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  // LOG_ERROR("Build imu host ip:{}, host_port:{}, lidar_port:{}", lidar_cfg.host_net_info.imu_data_ip.c_str(),
  //     lidar_cfg.host_net_info.imu_data_port, lidar_imu_port);
  return true;
}


bool BuildRequest::BuildSetLidarIPInfoRequest(const LivoxLidarIpInfo& lidar_ip_config, uint8_t* req_buf, uint16_t& req_len) {
  uint16_t key_num = 1;
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  kv->key = static_cast<uint16_t>(kKeyLidarIPCfg);
  kv->length = sizeof(uint8_t) * 12;
  LivoxLidarIpInfoValue* lidar_ip_val = (LivoxLidarIpInfoValue*)&kv->value;
  if (!InitLidarIpinfoVal(lidar_ip_config, lidar_ip_val)) {
    LOG_ERROR("Build set lidar ip info request failed, init lidar ip addr failed.");
    return false;
  }

  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(LivoxLidarIpInfoValue);
  return true;
}

bool BuildRequest::BuildSetHostStateInfoIPCfgRequest(const HostStateInfoIpInfo& host_state_info_ipcfg,
    uint8_t* req_buf, uint16_t& req_len) {
  uint16_t key_num = 1;
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  kv->key = static_cast<uint16_t>(kKeyStateInfoHostIPCfg);
  kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_ip_info_val = (HostIpInfoValue*)&kv->value;

  if (!InitHostIpAddr(host_state_info_ipcfg.host_ip_addr, host_ip_info_val)) {
    LOG_ERROR("Build set host point data ip info request failed, init host ip addr failed.");
    return false;
  }

  memcpy(&(host_ip_info_val->host_port), &host_state_info_ipcfg.host_state_info_port, sizeof(host_state_info_ipcfg.host_state_info_port));
  memcpy(&(host_ip_info_val->lidar_port), &host_state_info_ipcfg.lidar_state_info_port, sizeof(host_state_info_ipcfg.lidar_state_info_port));

  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  return true;
}

bool BuildRequest::BuildSetHostPointDataIPInfoRequest(const HostPointIPInfo& host_point_ip_cfg, uint8_t* req_buf, uint16_t& req_len) {
  uint16_t key_num = 1;
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  kv->key = static_cast<uint16_t>(kKeyLidarPointDataHostIPCfg);
  kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_ip_info_val = (HostIpInfoValue*)&kv->value;

  if (!InitHostIpAddr(host_point_ip_cfg.host_ip_addr, host_ip_info_val)) {
    LOG_ERROR("Build set host point data ip info request failed, init host ip addr failed.");
    return false;
  }

  memcpy(&(host_ip_info_val->host_port), &host_point_ip_cfg.host_point_data_port, sizeof(host_point_ip_cfg.host_point_data_port));
  memcpy(&(host_ip_info_val->lidar_port), &host_point_ip_cfg.lidar_point_data_port, sizeof(host_point_ip_cfg.lidar_point_data_port));

  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  return true;
}

bool BuildRequest::BuildSetHostImuDataIPInfoRequest(const HostImuDataIPInfo& host_imu_ipcfg, uint8_t* req_buf, uint16_t& req_len) {
  uint16_t key_num = 1;
  memcpy(&req_buf[req_len], &key_num, sizeof(key_num));
  req_len = sizeof(key_num) + sizeof(uint16_t);

  LivoxLidarKeyValueParam * kv = (LivoxLidarKeyValueParam *)&req_buf[req_len];
  kv->key = static_cast<uint16_t>(kKeyLidarImuHostIPCfg);
  kv->length = sizeof(uint8_t) * 8;
  HostIpInfoValue* host_ip_info_val = (HostIpInfoValue*)&kv->value;

  if (!InitHostIpAddr(host_imu_ipcfg.host_ip_addr, host_ip_info_val)) {
    LOG_ERROR("Build set host imu data ip info request failed, init host ip addr failed.");
    return false;
  }

  memcpy(&(host_ip_info_val->host_port), &host_imu_ipcfg.host_imu_data_port, sizeof(host_imu_ipcfg.host_imu_data_port));
  memcpy(&(host_ip_info_val->lidar_port), &host_imu_ipcfg.lidar_imu_data_port, sizeof(host_imu_ipcfg.lidar_imu_data_port));

  req_len += sizeof(LivoxLidarKeyValueParam) - sizeof(uint8_t) + sizeof(HostIpInfoValue);
  return true;
}

bool BuildRequest::InitLidarIpinfoVal(const LivoxLidarIpInfo& lidar_ip_config, LivoxLidarIpInfoValue* lidar_ipinfo_val_ptr) {
  if (lidar_ipinfo_val_ptr == nullptr) {
    return false;
  }

  // Set lidar ip info
  std::vector<uint8_t> vec_lidar_ip;
  if (!IpToU8(lidar_ip_config.ip_addr, ".", vec_lidar_ip)) {
    return false;
  }
  memcpy(lidar_ipinfo_val_ptr->lidar_ipaddr, vec_lidar_ip.data(), sizeof(uint8_t) * 4);

  // Set lidar subnet mask
  std::vector<uint8_t> vec_lidar_subnet_mask;
  if (!IpToU8(lidar_ip_config.net_mask, ".", vec_lidar_subnet_mask)) {
    return false;
  }
  memcpy(lidar_ipinfo_val_ptr->lidar_subnet_mask, vec_lidar_subnet_mask.data(), sizeof(uint8_t) * 4);

  // Set lidar gateway
  std::vector<uint8_t> vec_lidar_gateway;
  if (!IpToU8(lidar_ip_config.gw_addr, ".", vec_lidar_gateway)) {
     return false;  
  }
  memcpy(lidar_ipinfo_val_ptr->lidar_gateway, vec_lidar_gateway.data(), sizeof(uint8_t) * 4);
  return true;
}

bool BuildRequest::InitHostIpAddr(const std::string& host_ip, HostIpInfoValue* host_ipinfo_val_ptr) {
  if (host_ipinfo_val_ptr == nullptr) {
    return false;
  }

  // Set lidar ip info
  std::vector<uint8_t> vec_host_ip;
  if (!IpToU8(host_ip, ".", vec_host_ip)) {
    return false;
  }
  memcpy(host_ipinfo_val_ptr->host_ip, vec_host_ip.data(), sizeof(uint8_t) * 4);
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

} // namespace lidar
} // namespace livox
