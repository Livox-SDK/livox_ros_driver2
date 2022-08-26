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
#include "base/logging.h"

namespace livox {
namespace lidar {

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}

bool ParseCfgFile::Parse(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    LOG_INFO("Parse industrial config failed, can not open json config file!");
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  
  rapidjson::Document doc;
  if (doc.ParseStream(config_file).HasParseError()) {
    if (raw_file) {
      std::fclose(raw_file);
    }
    LOG_INFO("Parse direct config failed, parse the config file has error!");
    return false;
  }

  lidars_cfg_ptr.reset(new std::vector<LivoxLidarCfg>());
  custom_lidars_cfg_ptr.reset(new std::vector<LivoxLidarCfg>());

  if (doc.HasMember("HAP") && doc["HAP"].IsObject()) {
    LivoxLidarCfg hap_lidar_cfg;
    if (!GetDevType("HAP", hap_lidar_cfg.device_type)) {
      LOG_ERROR("Parse hap object failed, the device_type is error.");
      return false;
    }

    const rapidjson::Value &hap_object = doc["HAP"];
    if (!ParseLidarCfg(hap_object, hap_lidar_cfg, false)) {
      if (raw_file) {
        std::fclose(raw_file);
      }
      LOG_ERROR("Parse hap lidar cfg failed.");
      return false;
    }
    lidars_cfg_ptr->push_back(std::move(hap_lidar_cfg));
  }

  if (doc.HasMember("MID360") && doc["MID360"].IsObject()) {
    LivoxLidarCfg lidar_cfg;
    if (!GetDevType("MID360", lidar_cfg.device_type)) {
      LOG_ERROR("Parse mid360 object failed, the device_type is error.");
      return false;
    }
    const rapidjson::Value &hap_object = doc["MID360"];
    if (!ParseLidarCfg(hap_object, lidar_cfg, false)) {
      if (raw_file) {
        std::fclose(raw_file);
      }
      LOG_ERROR("Parse mid360 lidar cfg failed.");
      return false;
    }
    lidars_cfg_ptr->push_back(std::move(lidar_cfg));
  }

  if (doc.HasMember("PA") && doc["PA"].IsObject()) {
    LivoxLidarCfg pa_lidar_cfg;
    if (!GetDevType("PA", pa_lidar_cfg.device_type)) {
      LOG_ERROR("Parse pa object failed, the device_type is error.");
      return false;
    }
    const rapidjson::Value &hap_object = doc["PA"];
    if (!ParseLidarCfg(hap_object, pa_lidar_cfg, false)) {
      if (raw_file) {
        std::fclose(raw_file);
      }
      LOG_ERROR("Parse pa lidar cfg failed.");
      return false;
    }
    lidars_cfg_ptr->push_back(std::move(pa_lidar_cfg));
  }

  if (doc.HasMember("custom_lidars") && doc["custom_lidars"].IsArray()) {
    const rapidjson::Value& custom_lidars = doc["custom_lidars"];
    size_t lidars_num = custom_lidars.Size();
    for (size_t i = 0; i < lidars_num; ++i) {
      LivoxLidarCfg lidar_cfg;
      const rapidjson::Value &object = custom_lidars[i];
      if (!ParseLidarCfg(object, lidar_cfg, true)) {
        if (raw_file) {
          std::fclose(raw_file);
        }
        LOG_ERROR("Parse custom lidars faield, the index:{}", i);
        return false;
      }
      custom_lidars_cfg_ptr->push_back(std::move(lidar_cfg));
    }
  }

  if (raw_file) {
    std::fclose(raw_file);
  }
  return true;
}

bool ParseCfgFile::ParseLidarCfg(const rapidjson::Value &object, LivoxLidarCfg& lidar_cfg, bool is_custom) {
  if (is_custom) {
    if (!object.HasMember("device_type") || !object["device_type"].IsString()) {
      LOG_ERROR("Parse hap object failed, has not device_type member or device_type is not string.");
      return false;
    }

    std::string device_type = object["device_type"].GetString();
    if (!GetDevType(device_type, lidar_cfg.device_type)) {
      LOG_ERROR("Parse hap object failed, the device_type is error, the val:{}", device_type.c_str());
      return false;
    }
  }

  if (!ParseLidarNetInfo(object, lidar_cfg.lidar_net_info, is_custom)) {
    LOG_ERROR("Parse hap lidar net info failed.");
    return false;
  }

  if (!ParseHostNetInfo(object, lidar_cfg.host_net_info)) {
    LOG_ERROR("Parse host net info failed.");
    return false;
  }

  if (!ParseGeneralCfgInfo(object, lidar_cfg.general_cfg_info)) {
    LOG_ERROR("Parse general cfg failed");
    return false;
  }

  return true;
}

bool ParseCfgFile::GetDevType(const std::string& type, uint8_t& device_type) {
  if (type == "HAP") {
    device_type = kLivoxLidarTypeIndustrialHAP;
    return true;
  } else if (type == "MID360") {
    device_type = kLivoxLidarTypeMid360;
    return true;
  } else if (type == "PA") {
    device_type = kLivoxLidarTypePA;
    return true;
  }
  return false;
}

bool ParseCfgFile::ParseLidarNetInfo(const rapidjson::Value &object, LivoxLidarNetInfo& lidar_net_info, bool is_custom) {
  if (is_custom) {
    if (!object.HasMember("lidar_ipaddr") || !object["lidar_ipaddr"].IsString()) {
      LOG_ERROR("Parse lidar net info failed, has not lidar_ipaddr or lidar_ipaddr is not str.");
      return false;
    }
    lidar_net_info.lidar_ipaddr = object["lidar_ipaddr"].GetString();
  } else {
    lidar_net_info.lidar_ipaddr = "";
  }

  if (!object.HasMember("lidar_net_info") || !object["lidar_net_info"].IsObject()) {
    LOG_ERROR("Parse lidar net info failed, has not lidar_net_info member or lidar_net_info is not object.");
    return false;
  }
  const rapidjson::Value &lidar_net_info_object = object["lidar_net_info"];

  if (!lidar_net_info_object.HasMember("cmd_data_port") || !lidar_net_info_object["cmd_data_port"].IsUint()) {
    LOG_ERROR("Parse lidar net info failed, has not cmd_data_port member or cmd_data_port is not uint.");
    return false;
  }
  lidar_net_info.cmd_data_port = lidar_net_info_object["cmd_data_port"].GetUint();

  if (!lidar_net_info_object.HasMember("push_msg_port") || !lidar_net_info_object["push_msg_port"].IsUint()) {
    LOG_ERROR("Parse lidar net info failed, has not push_msg_port member or push_msg_port is not uint.");
    return false;
  }
  lidar_net_info.push_msg_port = lidar_net_info_object["push_msg_port"].GetUint();

  if (!lidar_net_info_object.HasMember("point_data_port") || !lidar_net_info_object["point_data_port"].IsUint()) {
    LOG_ERROR("Parse lidar net info failed, has not point_data_port member or point_data_port is not uint.");
    return false;
  }
  lidar_net_info.point_data_port = lidar_net_info_object["point_data_port"].GetUint();

  if (!lidar_net_info_object.HasMember("imu_data_port") || !lidar_net_info_object["imu_data_port"].IsUint()) {
    LOG_ERROR("Parse lidar net info failed, has not imu_data_port member or imu_data_port is not uint.");
    return false;
  }
  lidar_net_info.imu_data_port = lidar_net_info_object["imu_data_port"].GetUint();

  if (!lidar_net_info_object.HasMember("log_data_port") || !lidar_net_info_object["log_data_port"].IsUint()) {
    LOG_ERROR("Parse lidar net info failed, has not log_data_port member or log_data_port is not uint.");
    return false;
  }
  lidar_net_info.log_data_port = lidar_net_info_object["log_data_port"].GetUint();
  return true;
}

bool ParseCfgFile::ParseHostNetInfo(const rapidjson::Value &object, HostNetInfo& host_net_info) {
  if (!object.HasMember("host_net_info") || !object["host_net_info"].IsObject()) {
    LOG_ERROR("Parse lidar net info failed, has not host_net_info member or host_net_info is not object.");
    return false;
  }
  const rapidjson::Value &host_net_info_object = object["host_net_info"];

  // parse cmd data info
  if (!host_net_info_object.HasMember("cmd_data_ip") || !host_net_info_object["cmd_data_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, has not cmd_data_ip or cmd_data_ip is not string.");
    return false;
  }
  host_net_info.cmd_data_ip = host_net_info_object["cmd_data_ip"].GetString();

  if (!host_net_info_object.HasMember("cmd_data_port") || !host_net_info_object["cmd_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not cmd_data_port or cmd_data_port is not uint.");
    return false;
  }
  host_net_info.cmd_data_port = host_net_info_object["cmd_data_port"].GetUint();

  // parse push msg info
  if (!host_net_info_object.HasMember("push_msg_ip") || !host_net_info_object["push_msg_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, has not push_msg_ip or push_msg_ip is not string.");
    return false;
  }
  host_net_info.push_msg_ip = host_net_info_object["push_msg_ip"].GetString();

  if (!host_net_info_object.HasMember("push_msg_port") || !host_net_info_object["push_msg_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not push_msg_port or push_msg_port is not uint.");
    return false;
  }
  host_net_info.push_msg_port = host_net_info_object["push_msg_port"].GetUint();

  // parse point data info
  if (!host_net_info_object.HasMember("point_data_ip") || !host_net_info_object["point_data_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, has not point_data_ip or point_data_ip is not string.");
    return false;
  }
  host_net_info.point_data_ip = host_net_info_object["point_data_ip"].GetString();

  if (!host_net_info_object.HasMember("point_data_port") || !host_net_info_object["point_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not point_data_port or point_data_port is not uint.");
    return false;
  }
  host_net_info.point_data_port = host_net_info_object["point_data_port"].GetUint();

  // parse imu data info
  if (!host_net_info_object.HasMember("imu_data_ip") || !host_net_info_object["imu_data_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, has not imu_data_ip or imu_data_ip is not string.");
    return false;
  }
  host_net_info.imu_data_ip = host_net_info_object["imu_data_ip"].GetString();

  if (!host_net_info_object.HasMember("imu_data_port") || !host_net_info_object["imu_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not imu_data_port or imu_data_port is not uint.");
    return false;
  }
  host_net_info.imu_data_port = host_net_info_object["imu_data_port"].GetUint();

  // parse log info
  if (!host_net_info_object.HasMember("log_data_ip") || !host_net_info_object["log_data_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, has not log_data_ip or log_data_ip is not string.");
    return false;
  }
  host_net_info.log_data_ip = host_net_info_object["log_data_ip"].GetString();

  if (!host_net_info_object.HasMember("log_data_port") || !host_net_info_object["log_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not cmd_data_port or cmd_data_port is not uint.");
    return false;
  }
  host_net_info.log_data_port = host_net_info_object["log_data_port"].GetUint();
  return true;
}

bool ParseCfgFile::ParseGeneralCfgInfo(const rapidjson::Value &object, GeneralCfgInfo& general_cfg_info) {
  return true;
}

} // namespace lidar
} // namespace livox
