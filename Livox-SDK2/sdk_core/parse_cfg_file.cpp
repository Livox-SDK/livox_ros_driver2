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
#include "parse_cfg_file.h"
#include "base/logging.h"

#include <map>
#include <string>

namespace livox {
namespace lidar {

const std::map<std::string, LivoxLidarDeviceType> dev_type_map = {
  {"HAP",     kLivoxLidarTypeIndustrialHAP},
  {"MID360",  kLivoxLidarTypeMid360},
  {"Mid360s", kLivoxLidarTypeMid360s},
  {"Avia2",   kLivoxLidarTypeAvia2},
  {"Mid360l", kLivoxLidarTypeMid360l}
};


ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}


bool ParseCfgFile::Parse(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
                         std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr,
                         std::shared_ptr<LivoxLidarLoggerCfg>& lidar_logger_cfg_ptr,
                         std::shared_ptr<LivoxLidarSdkFrameworkCfg>& sdk_framework_cfg_ptr) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    LOG_INFO("Parse lidar config failed, can not open json config file!");
  }
  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));  
  rapidjson::Document doc;
  if (doc.ParseStream(config_file).HasParseError()) {
    if (raw_file) {
      std::fclose(raw_file);
    }
    LOG_ERROR("Parse lidar config failed, parse the config file has error!");
    return false;
  }

  lidars_cfg_ptr.reset(new std::vector<LivoxLidarCfg>());
  custom_lidars_cfg_ptr.reset(new std::vector<LivoxLidarCfg>());
  lidar_logger_cfg_ptr.reset(new LivoxLidarLoggerCfg());
  sdk_framework_cfg_ptr.reset(new LivoxLidarSdkFrameworkCfg());

  if (doc.HasMember("master_sdk")) {
    if (doc["master_sdk"].IsBool()) {
      sdk_framework_cfg_ptr->master_sdk = doc["master_sdk"].GetBool();
      if (sdk_framework_cfg_ptr->master_sdk) {
        LOG_INFO("set master/slave sdk to master sdk");
      } else {
        LOG_INFO("set master/slave sdk to slave sdk");
      }
    } else {
      LOG_ERROR("set master/slave sdk error");
      if (raw_file) {
        std::fclose(raw_file);
      }
      return false;
    }
  } else {
    LOG_INFO("set master/slave sdk to master sdk by default");
    sdk_framework_cfg_ptr->master_sdk = true;
  }

  if (doc.HasMember("lidar_log_enable")) {
    if (doc["lidar_log_enable"].IsBool()) {
      lidar_logger_cfg_ptr->lidar_log_enable = doc["lidar_log_enable"].GetBool();
    } else {
      LOG_ERROR("Lidar log enable data type is error");
      if (raw_file) {
        std::fclose(raw_file);
      }
      return false;
    }

    if (doc.HasMember("lidar_log_cache_size_MB") && doc["lidar_log_cache_size_MB"].IsUint()) {
      lidar_logger_cfg_ptr->lidar_log_cache_size = doc["lidar_log_cache_size_MB"].GetUint();
    } else {
      LOG_ERROR("Parse json file failed, has not lidar_log_cache_size_MB member or lidar_log_cache_size_MB is uint");
      if (raw_file) {
        std::fclose(raw_file);
      }
      return false;
    }

    if (doc.HasMember("lidar_log_path") && doc["lidar_log_path"].IsString()) {
      lidar_logger_cfg_ptr->lidar_log_path = doc["lidar_log_path"].GetString();
    } else {
      LOG_ERROR("Parse json file failed, has not lidar_log_path member or lidar_log_path is uint");
      if (raw_file) {
        std::fclose(raw_file);
      }
      return false;
    }
    LOG_INFO("Lidar log cfg, lidar_log_enable:{}, lidar_log_cache_size_MB:{}, lidar_log_path:{}",
        lidar_logger_cfg_ptr->lidar_log_enable, lidar_logger_cfg_ptr->lidar_log_cache_size,
        lidar_logger_cfg_ptr->lidar_log_path.c_str());
  } else {
    lidar_logger_cfg_ptr->lidar_log_enable = false;
    lidar_logger_cfg_ptr->lidar_log_cache_size = 0;
    lidar_logger_cfg_ptr->lidar_log_path = "./"; // TODO Executable program path
    if (doc.HasMember("lidar_log_path") && doc["lidar_log_path"].IsString()) {
      lidar_logger_cfg_ptr->lidar_log_path = doc["lidar_log_path"].GetString();
    }
    LOG_INFO("Livox lidar logger disable.");
  }


  if (doc.HasMember("HAP") && doc["HAP"].IsObject()) {
    uint8_t device_type = dev_type_map.at("HAP");
    const rapidjson::Value &object = doc["HAP"];

    if (!ParseLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
      if (raw_file) {
        std::fclose(raw_file);
      }
      return false;
    }
  }

  if (doc.HasMember("MID360") && doc["MID360"].IsObject()) {
    uint8_t device_type = dev_type_map.at("MID360");    
    const rapidjson::Value &object = doc["MID360"];

    if (!ParseLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
      if (raw_file) {
        std::fclose(raw_file);
      }
      return false;
    }
  }
  
  if (doc.HasMember("Mid360s") && doc["Mid360s"].IsObject()) {
    
      uint8_t device_type = dev_type_map.at("Mid360s");    
      const rapidjson::Value &object = doc["Mid360s"];

      if (!ParseLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
        if (raw_file) {
          std::fclose(raw_file);
      }
      return false;
    }
  }

   if (doc.HasMember("Avia2") && doc["Avia2"].IsObject()) {
      uint8_t device_type = dev_type_map.at("Avia2");
      const rapidjson::Value &object = doc["Avia2"];

      if (!ParseLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
        if (raw_file) {
          std::fclose(raw_file);
      }
      return false;
    }
  }

    if (doc.HasMember("Mid360l") && doc["Mid360l"].IsObject()) {
      uint8_t device_type = dev_type_map.at("Mid360l");
      const rapidjson::Value &object = doc["Mid360l"];

      if (!ParseLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
        if (raw_file) {
          std::fclose(raw_file);
      }
      return false;
    }
  }

  if (raw_file) {
    std::fclose(raw_file);
  }
  return true;
}

bool ParseCfgFile::ParseLidarCfg(const rapidjson::Value &object, const uint8_t& device_type, std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr, std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {
  if (object.HasMember("host_net_info") && object["host_net_info"].IsArray()) {
    if (!ParseNewLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
      LOG_ERROR("Parse hap lidar new cfg failed.");
      return false;
    }
  } else if (object.HasMember("host_net_info") && object["host_net_info"].IsObject()) {
    if (!ParseOldLidarCfg(object, device_type, lidars_cfg_ptr)) {
      LOG_ERROR("Parse hap lidar old cfg failed.");
      return false;
    }
  } else {
    LOG_ERROR("Parse lidar net info failed, has not host_net_info member or host_net_info is not object or arry.");
    return false;
  }
  return true;
}

bool ParseCfgFile::ParseNewLidarCfg(const rapidjson::Value &object, const uint8_t& device_type, std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr, std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {
  const rapidjson::Value &host_net_info_object = object["host_net_info"];
  size_t config_num = host_net_info_object.Size();

  for (size_t i = 0; i < config_num; ++i) {
    if (!host_net_info_object[i].HasMember("lidar_ip") || !host_net_info_object[i]["lidar_ip"].IsArray()) {
      LivoxLidarCfg lidar_cfg;

      if (!ParseTypeLidarCfg(object, host_net_info_object[i], device_type, lidar_cfg)) {
        return false;
      }

      lidars_cfg_ptr->push_back(std::move(lidar_cfg));
      continue;
    }

    const rapidjson::Value &lidar_ip_arr = host_net_info_object[i]["lidar_ip"];
    size_t lidar_ip_num = lidar_ip_arr.Size();
    for (size_t j = 0; j < lidar_ip_num; ++j) {
      LivoxLidarCfg lidar_cfg;

      const rapidjson::Value &lidar_ip = lidar_ip_arr[j];
      if (!lidar_ip.IsString()) {
        LOG_ERROR("Parse lidar ip failed, has not lidar_ip member or lidar_ip is not object.");
        return false;
      }
      lidar_cfg.lidar_net_info.lidar_ipaddr = lidar_ip.GetString();

      if (!ParseTypeLidarCfg(object, host_net_info_object[i], device_type, lidar_cfg)) {
        return false;
      }

      custom_lidars_cfg_ptr->push_back(std::move(lidar_cfg));
    }
  }

  return true;
}

bool ParseCfgFile::ParseOldLidarCfg(const rapidjson::Value &object, const uint8_t& device_type, std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr) {
  const rapidjson::Value &host_net_info_object = object["host_net_info"];
  LivoxLidarCfg lidar_cfg;
  if (!ParseTypeLidarCfg(object, host_net_info_object, device_type, lidar_cfg)) {
    return false;
  }
  lidars_cfg_ptr->push_back(std::move(lidar_cfg));
  return true;
}

bool ParseCfgFile::ParseTypeLidarCfg(const rapidjson::Value &object, const rapidjson::Value &host_net_info_object, const uint8_t& device_type, LivoxLidarCfg& lidar_cfg) {
  lidar_cfg.device_type = device_type;
  if (!ParseLidarNetInfo(object, lidar_cfg.lidar_net_info)) {
    LOG_ERROR("Parse hap lidar net info failed.");
    return false;
  }
  if (!ParseHostNetInfo(host_net_info_object, lidar_cfg.host_net_info)) {
    LOG_ERROR("Parse host net info failed.");
    return false;
  }
  if (!ParseGeneralCfgInfo(object, lidar_cfg.general_cfg_info)) {
    LOG_ERROR("Parse general cfg failed");
    return false;
  }

  return true;
}

bool ParseCfgFile::ParseLidarNetInfo(const rapidjson::Value &object, LivoxLidarNetInfo& lidar_net_info) {
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


bool ParseCfgFile::ParseHostNetInfo(const rapidjson::Value &host_net_info_object, HostNetInfo& host_net_info) {
  if (!host_net_info_object.HasMember("host_ip") && !host_net_info_object.HasMember("cmd_data_ip")) {
    LOG_ERROR("Parse host net info failed, has not host_ip or cmd_data_ip.");
    return false;
  }
  if (host_net_info_object.HasMember("host_ip") && !host_net_info_object["host_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, host_ip is not string.");
    return false;
  }
  if (host_net_info_object.HasMember("cmd_data_ip") && !host_net_info_object["cmd_data_ip"].IsString()) {
    LOG_ERROR("Parse host net info failed, cmd_data_ip is not string.");
    return false;
  }

  // parse cmd ip info
  if (host_net_info_object.HasMember("cmd_data_ip") && host_net_info_object["cmd_data_ip"].IsString()) {
    host_net_info.host_ip = host_net_info_object["cmd_data_ip"].GetString();
  }

  // parse host ip info
  if (host_net_info_object.HasMember("host_ip") && host_net_info_object["host_ip"].IsString()) {
    host_net_info.host_ip = host_net_info_object["host_ip"].GetString();
  }

  // parse multicast ip info
  if (host_net_info_object.HasMember("multicast_ip")) {
    if (host_net_info_object["multicast_ip"].IsString()) {
      host_net_info.multicast_ip = host_net_info_object["multicast_ip"].GetString();
    } else {
      LOG_ERROR("Parse host net info failed, has not multicast_ip or multicast_ip is not string.");
      return false;
    }
  } else {
    host_net_info.multicast_ip = "";
  }

  // parse cmd port info
  if (!host_net_info_object.HasMember("cmd_data_port") || !host_net_info_object["cmd_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not cmd_data_port or cmd_data_port is not uint.");
    return false;
  }
  host_net_info.cmd_data_port = host_net_info_object["cmd_data_port"].GetUint();

  // parse push msg port
  if (!host_net_info_object.HasMember("push_msg_port") || !host_net_info_object["push_msg_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not push_msg_port or push_msg_port is not uint.");
    return false;
  }
  host_net_info.push_msg_port = host_net_info_object["push_msg_port"].GetUint();

  // parse point data port
  if (!host_net_info_object.HasMember("point_data_port") || !host_net_info_object["point_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not point_data_port or point_data_port is not uint.");
    return false;
  }
  host_net_info.point_data_port = host_net_info_object["point_data_port"].GetUint();

  // parse imu data port
  if (!host_net_info_object.HasMember("imu_data_port") || !host_net_info_object["imu_data_port"].IsUint()) {
    LOG_ERROR("Parse host net info failed, has not imu_data_port or imu_data_port is not uint.");
    return false;
  }
  host_net_info.imu_data_port = host_net_info_object["imu_data_port"].GetUint();

  // parse log port
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
