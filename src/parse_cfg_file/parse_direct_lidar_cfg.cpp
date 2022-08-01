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
#include "parse_direct_lidar_cfg.h"

namespace livox_ros {

ParseDirectLidarCfgFile::ParseDirectLidarCfgFile(const std::string& path) : path_(path) {}

bool ParseDirectLidarCfgFile::Parse(DirectLidarParam& direct_lidar_param) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout <<"Parse industrial config failed, can not open json config file!" << std::endl;
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  
  rapidjson::Document doc;
  if (doc.ParseStream(config_file).HasParseError()) {
    std::cout <<"Parse direct config failed, parse the config file has error!" << std::endl;
    return false;
  }

  if (!doc.HasMember("direct_lidar_config") || !doc["direct_lidar_config"].IsObject()) {
    std::cout << "Parse direct config failed, has not the direct_lidar_config member or is not object." << std::endl;
    return false;
  }
  const rapidjson::Value& object = doc["direct_lidar_config"];

  if (object.HasMember("host_config") && object["host_config"].IsObject()) {
    const rapidjson::Value &host_cfg_object = object["host_config"];
    direct_lidar_param.direct_host_cfg_ptr.reset(new DirectLidarHostCfg());
    return ParseHostCfg(host_cfg_object, *direct_lidar_param.direct_host_cfg_ptr);
  }

  if (object.HasMember("lidar_config") && object["lidar_config"].IsArray()) {
    const rapidjson::Value &array = object["lidar_config"];
    direct_lidar_param.direct_lidars_cfg_ptr.reset(new std::vector<DirectLidarCfg>());
    return ParseLidarsCfg(array, *(direct_lidar_param.direct_lidars_cfg_ptr));
  }
  return false;
}

bool ParseDirectLidarCfgFile::ParseHostCfg(const rapidjson::Value &object, DirectLidarHostCfg& direct_host_cfg) {
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

  if (object.HasMember("host_cmd_port") && object["host_cmd_port"].IsUint()) {
    direct_host_cfg.host_cmd_port = static_cast<uint16_t>(object["host_cmd_port"].GetUint());
  }

  if (object.HasMember("host_push_cmd_port") && object["host_push_cmd_port"].IsUint()) {
    direct_host_cfg.host_push_cmd_port = static_cast<uint16_t>(object["host_push_cmd_port"].GetUint());
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


bool ParseDirectLidarCfgFile::ParseLidarsCfg(const rapidjson::Value& array,
    std::vector<DirectLidarCfg>& vec_direct_lidar_cfg) {
  size_t len = array.Size();
  vec_direct_lidar_cfg.resize(len);
  for (size_t i = 0; i < len; i++) {
    printf("The index:%lu lidar config.\n", i);
    const rapidjson::Value &object = array[i];
    DirectLidarCfg& direct_lidar_cfg = vec_direct_lidar_cfg[i];
    if (!object.IsObject()) {
      std::cout << "Parse direct lidar config failed, the index:" << i << " is not object." << std::endl;
      return false;
    }

    if (object.HasMember("sn") && object["sn"].IsString()) {
      std::string sn = object["sn"].GetString();
      strcpy(direct_lidar_cfg.sn, sn.c_str());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no sn member or the sn member"
        << " is not a string!" << std::endl;   
      return false;
    }

    if (object.HasMember("lidar_id") && object["lidar_id"].IsUint()) {
      direct_lidar_cfg.lidar_id = static_cast<uint8_t>(object["lidar_id"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no lidar_id member or the lidar_id member"
        << " is not a uint8!" << std::endl; 
      return false;
    }

    if (object.HasMember("lidar_ipmode") && object["lidar_ipmode"].IsUint()) {
      direct_lidar_cfg.lidar_ipmode = static_cast<uint8_t>(object["lidar_ipmode"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no lidar_ipmode member or the lidar_ipmode member"
        << " is not a uint8!" << std::endl; 
      return false;
    }

    if (object.HasMember("lidar_ipinfo_config") && object["lidar_ipinfo_config"].IsObject()) {
      const rapidjson::Value& lidar_ipinfo_object = object["lidar_ipinfo_config"];
      if (!ParseLidarIpCfg(lidar_ipinfo_object, direct_lidar_cfg.lidar_ipinfo_cfg)) {
        return false;
      }
    } else {
      std::cout <<"Parse direct lidar config failed, there is no lidar_ipinfo_config member or the lidar_ipinfo_config member"
        << " is not a object!" << std::endl;
      return false;
    }

    if (object.HasMember("host_ipinfo_config") && object["host_ipinfo_config"].IsObject()) {
      const rapidjson::Value& host_ipinfo_object = object["host_ipinfo_config"];
      if (!ParseHostCfg(host_ipinfo_object, direct_lidar_cfg.host_cfg)) {
        return false;
      }
    } else {
      std::cout <<"Parse direct lidar config failed, there is no direct_host_cfg member or the direct_host_cfg member"
        << " is not a object!" << std::endl;
      return false;
    }

    if (object.HasMember("sample_mode") && object["sample_mode"].IsUint()) {
      direct_lidar_cfg.sample_mode = static_cast<uint8_t>(object["sample_mode"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no sample_mode member or the sample_mode member"
        << " is not a uint8!" << std::endl;
      return false;
    }

    if (object.HasMember("pattern_mode") && object["pattern_mode"].IsUint()) {
      direct_lidar_cfg.pattern_mode = static_cast<uint8_t>(object["pattern_mode"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no pattern_mode member or the pattern_mode member"
        << " is not a uint8!" << std::endl;
      return false;
    }

    if (object.HasMember("pcl_data_type") && object["pcl_data_type"].IsUint()) {
      direct_lidar_cfg.pcl_data_type = static_cast<uint8_t>(object["pcl_data_type"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no pcl_data_type member or the pcl_data_type member"
        << " is not a uint8!" << std::endl;
      return false;
    }

    if (object.HasMember("imu_data_en") && object["imu_data_en"].IsUint()) {
      direct_lidar_cfg.imu_data_en = static_cast<uint8_t>(object["imu_data_en"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no imu_data_en member or the imu_data_en member"
        << " is not a uint8!" << std::endl;
      return false;
    }

    if (object.HasMember("install_attitude") && object["install_attitude"].IsObject()) {
      const rapidjson::Value& install_attitude_object = object["install_attitude"];
      if (!ParseInstallAttitude(install_attitude_object, direct_lidar_cfg.install_attitude)) {
        return false;
      }
    } else {
      std::cout <<"Parse direct lidar config failed, there is no install_attitude member or the install_attitude member"
        << " is not a object!" << std::endl;
      return false;
    }

    if (object.HasMember("work_mode") && object["work_mode"].IsUint()) {
      direct_lidar_cfg.work_mode = static_cast<uint8_t>(object["work_mode"].GetUint());
    } else {
      std::cout <<"Parse direct lidar config failed, there is no work_mode member or the work_mode member"
        << " is not a object!" << std::endl;
      return false;
    }
    printf("Parse direct Lidar config, sn:%s, lidar_id:%u, lidar_ipmode:%u, sample_mode:%u, pattern_mode:%u, pcl_data_type:%u"
           ", imu_data_en:%u, work_mode:%u.\n", direct_lidar_cfg.sn, direct_lidar_cfg.lidar_id, direct_lidar_cfg.lidar_ipmode,
            direct_lidar_cfg.sample_mode, direct_lidar_cfg.pattern_mode, direct_lidar_cfg.pcl_data_type,
            direct_lidar_cfg.imu_data_en, direct_lidar_cfg.work_mode);
  }
  return true;
}

bool ParseDirectLidarCfgFile::ParseLidarIpCfg(const rapidjson::Value& object, DirectLidarIpCfg& direct_lidar_ip_cfg) {
  if (object.HasMember("lidar_ipaddr") && object["lidar_ipaddr"].IsString()) {
    std::string lidar_ipaddr = object["lidar_ipaddr"].GetString();
    strcpy(direct_lidar_ip_cfg.lidar_ipaddr, lidar_ipaddr.c_str());
  } else {
    std::cout <<"Parse direct lidar ip cfg failed, there is no lidar_ipaddr member or the lidar_ipaddr member"
        << " is not a string!" << std::endl;
    return false;
  }

  if (object.HasMember("lidar_subnet_mask") && object["lidar_subnet_mask"].IsString()) {
    std::string lidar_subnet_mask = object["lidar_subnet_mask"].GetString();
    strcpy(direct_lidar_ip_cfg.lidar_subnet_mask, lidar_subnet_mask.c_str());
  } else {
    std::cout <<"Parse direct lidar ip cfg failed, there is no lidar_subnet_mask member or the lidar_subnet_mask member"
        << " is not a string!" << std::endl;
    return false;
  }

  if (object.HasMember("lidar_gateway") && object["lidar_gateway"].IsString()) {
    std::string lidar_gateway = object["lidar_gateway"].GetString();
    strcpy(direct_lidar_ip_cfg.lidar_gateway, lidar_gateway.c_str());
  } else {
    std::cout <<"Parse direct lidar ip cfg failed, there is no lidar_gateway member or the lidar_gateway member"
        << " is not a string!" << std::endl;
    return false;
  }
  std::cout << "Parse direct lidar ipinfo cfg, lidar_ipaddr:" << direct_lidar_ip_cfg.lidar_ipaddr << ", lidar_subnet_mask:"
            << direct_lidar_ip_cfg.lidar_subnet_mask << ", lidar_gatway:" << direct_lidar_ip_cfg.lidar_gateway << std::endl;

  return true;
}

bool ParseDirectLidarCfgFile::ParseInstallAttitude(const rapidjson::Value& object, InstallAttitude& install_attitude) {
  if (object.HasMember("roll_deg") && object["roll_deg"].IsFloat()) {
    install_attitude.roll_deg = object["roll_deg"].GetFloat();
  } else {
    std::cout <<"Parse install attitude failed, there is no roll_deg member or the roll_deg member"
        << " is not a float!" << std::endl;
    return false;
  }

  if (object.HasMember("pitch_deg") && object["pitch_deg"].IsFloat()) {
    install_attitude.pitch_deg = object["pitch_deg"].GetFloat();
  } else {
    std::cout <<"Parse install attitude failed, there is no pitch_deg member or the pitch_deg member"
        << " is not a float!" << std::endl;
    return false;
  }

  if (object.HasMember("yaw_deg") && object["yaw_deg"].IsFloat()) {
    install_attitude.yaw_deg = object["yaw_deg"].GetFloat();
  } else {
    std::cout <<"Parse install attitude failed, there is no yaw_deg member or the yaw_deg member"
        << " is not a float!" << std::endl;
    return false;
  }

  if (object.HasMember("x") && object["x"].IsInt()) {
    install_attitude.x = object["x"].GetInt();
  } else {
    std::cout <<"Parse install attitude failed, there is no x member or the x member"
        << " is not a int!" << std::endl;
    return false;
  }

  if (object.HasMember("y") && object["y"].IsInt()) {
    install_attitude.y = static_cast<uint8_t>(object["y"].GetInt());
  } else {
    std::cout <<"Parse install attitude failed, there is no y member or the y member"
        << " is not a int!" << std::endl;
    return false;
  }

  if (object.HasMember("z") && object["z"].IsInt()) {
    install_attitude.z = static_cast<uint8_t>(object["z"].GetInt());
  } else {
    std::cout <<"Parse install attitude failed, there is no z member or the z member"
        << " is not a int!" << std::endl;
    return false;
  }

  printf("Parse install attitude, roll_deg:%f, pitch_deg:%f, yaw_deg::%f, x:%d, y:%d, z:%d.\n",
         install_attitude.roll_deg, install_attitude.pitch_deg, install_attitude.yaw_deg, install_attitude.x,
         install_attitude.y, install_attitude.z);
  return true;
}

} // namespace livox_ros
