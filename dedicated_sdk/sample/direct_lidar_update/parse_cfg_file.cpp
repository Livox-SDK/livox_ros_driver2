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
#include "parse_cfg_file.h"

#include <iostream>

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}

bool ParseCfgFile::ParseHostConfig(DirectLidarHostCfg& direct_host_cfg) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout <<"Parse config file failed, can not open json config file, the path:" << path_ << std::endl;
    return false;
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  
  rapidjson::Document doc;
  if (doc.ParseStream(config_file).HasParseError()) {
    std::cout <<"Parse config file failed, rapidjson has parse error!" << std::endl;
    return false;
  }

  if (!doc.HasMember("direct_lidar_config") || !doc["direct_lidar_config"].IsObject()) {
    std::cout <<"Parse config file failed,  there is no direct_lidar_config member or the direct_lidar_config member"
        << " is not a object!" << std::endl;
    return false;
  }  
  const rapidjson::Value &object = doc["direct_lidar_config"];

  if (!object.HasMember("host_ipinfo_config") || !object["host_ipinfo_config"].IsObject()) {
    std::cout <<"Parse config file failed, there is no host_ipinfo_config member or the host_ipinfo_config member"
        << " is not a object!" << std::endl;
    return false;
  }

  // Parse industrial lidar config
  const rapidjson::Value &host_cfg_object = object["host_ipinfo_config"];
  return ParseHostCfg(host_cfg_object, direct_host_cfg);
}

bool ParseCfgFile::ParseHostCfg(const rapidjson::Value &object, DirectLidarHostCfg& direct_host_cfg) {
  if (object.HasMember("host_push_cmd_ip") && object["host_push_cmd_ip"].IsString()) {
    std::string host_push_cmd_ip = object["host_push_cmd_ip"].GetString();
    strcpy(direct_host_cfg.host_push_cmd_ip, host_push_cmd_ip.c_str());
  } else {
    std::cout <<"Parse config file failed, there is no host_push_cmd_ip member or the host_push_cmd_ip member"
        << " is not a string!" << std::endl;
    return false;
  }

  if (object.HasMember("host_point_data_ip") && object["host_point_data_ip"].IsString()) {
    std::string host_point_data_ip = object["host_point_data_ip"].GetString();
    strcpy(direct_host_cfg.host_point_data_ip, host_point_data_ip.c_str());
  } else {
    std::cout <<"Parse config file failed, there is no host_point_data_ip member or the host_push_cmd_ip member"
        << " is not a string!" << std::endl;
    return false;
  }

  if (object.HasMember("host_imu_data_ip") && object["host_imu_data_ip"].IsString()) {
    std::string host_imu_data_ip = object["host_imu_data_ip"].GetString();
    strcpy(direct_host_cfg.host_imu_data_ip, host_imu_data_ip.c_str());
  } else {
    std::cout <<"Parse config file failed, there is no host_imu_data_ip member or the host_imu_data_ip member"
        << " is not a string!" << std::endl;
    return false;
  }

  if (object.HasMember("host_push_cmd_port") && object["host_push_cmd_port"].IsUint()) {
    direct_host_cfg.host_push_cmd_port = static_cast<uint16_t>(object["host_push_cmd_port"].GetUint());
  } 

  if (object.HasMember("host_cmd_port") && object["host_cmd_port"].IsUint()) {
    direct_host_cfg.host_cmd_port = static_cast<uint16_t>(object["host_cmd_port"].GetUint());
  }

  if (object.HasMember("host_point_data_port") && object["host_point_data_port"].IsUint()) {
    direct_host_cfg.host_point_data_port = static_cast<uint16_t>(object["host_point_data_port"].GetUint());
  }

  if (object.HasMember("host_imu_data_port") && object["host_imu_data_port"].IsUint()) {
    direct_host_cfg.host_imu_data_port = static_cast<uint16_t>(object["host_imu_data_port"].GetUint());
  }

  if (object.HasMember("host_log_port") && object["host_log_port"].IsUint()) {
    direct_host_cfg.host_log_port = static_cast<uint16_t>(object["host_log_port"].GetUint());
  }
  
  printf("Parse host cfg, host_push_cmd_ip:%s, host_point_data_ip:%s, host_imu_data_ip:%s.\n",
      direct_host_cfg.host_push_cmd_ip, direct_host_cfg.host_point_data_ip, direct_host_cfg.host_imu_data_ip);
  printf("host_cmd_port:%u, host_push_cmd_port:%u, host_point_data_port:%u, "
         "host_imu_data_port:%u, host_log_port:%u.\n",
      direct_host_cfg.host_cmd_port, direct_host_cfg.host_push_cmd_port,
      direct_host_cfg.host_point_data_port, direct_host_cfg.host_imu_data_port,
      direct_host_cfg.host_log_port);
  return true;
}
