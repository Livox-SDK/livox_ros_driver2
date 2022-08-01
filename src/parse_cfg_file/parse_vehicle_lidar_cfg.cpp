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
#include "parse_vehicle_lidar_cfg.h"

namespace livox_ros {

ParseVehicleLidarCfgFile::ParseVehicleLidarCfgFile(const std::string& path) : path_(path) {}


bool ParseVehicleLidarCfgFile::Parse(VehicleLidarOption& vehicle_lidar_option,
                                     std::vector<UserVehicleConfig>& raw_vehicle_config,
                                     std::vector<LidarRegisterInfo>& lidars_register_info,
                                     std::vector<LidarExtrinsicParameters>& lidar_extrins_params) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    printf("Parse vehicle config failed, can not open json config file!\n");
    return false;
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));

  rapidjson::Document doc;
  if (doc.ParseStream(config_file).HasParseError()) {
    printf("Vehicle config file parse failed, parse stream has error!\n");
    return false;
  } 
  
  // Parse vehicle lidar config
  if (!doc.HasMember("vehicle_lidar_config") || !doc["vehicle_lidar_config"].IsObject()) {
    printf("Vehicle config file parse failed, has not vehicle_lidar_config the member, or is not object.\n");
    return false;
  }

  const rapidjson::Value &vehicle_lidar_object = doc["vehicle_lidar_config"];
  if (!vehicle_lidar_object.HasMember("local_addr") || !vehicle_lidar_object["local_addr"].IsString()) {
    printf("Parse vehicle lidar config file failed, can not parse local addr.\n");
    return false;
  }
  vehicle_lidar_option.local_addr = vehicle_lidar_object["local_addr"].GetString();

  if (!vehicle_lidar_object.HasMember("product_type") || !vehicle_lidar_object["product_type"].IsInt()) {
    printf("Parse vehicle lidar config file failed, can not parse product type.\n");
    return false;
  }
  uint32_t product_type = vehicle_lidar_object["product_type"].GetInt();
  vehicle_lidar_option.product_type = static_cast<uint8_t>(product_type);

  if (!vehicle_lidar_object.HasMember("lidar_config") || !vehicle_lidar_object["lidar_config"].IsArray()) {
    printf("Parse vehicle lidar config file failed, can not parse lidar config.\n");
    return false;
  }
  const rapidjson::Value &array = vehicle_lidar_object["lidar_config"];

  size_t len = array.Size();
  for (size_t i = 0; i < len; i++) {
    const rapidjson::Value &object = array[i];
    if (object.IsObject()) {
      UserVehicleConfig config;
      memset(&config, 0, sizeof(config));
      
      if (!object.HasMember("slot") || !object["slot"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse slot.\n");
        return false;
      }
      uint32_t slot = object["slot"].GetInt();
      config.slot = static_cast<uint8_t>(slot);

      LidarRegisterInfo lidar_register_info;
      lidar_register_info.slot = config.slot;
      if (!object.HasMember("ip_addr") || !object["ip_addr"].IsString()) {
        printf("Parse vehicle lidar config file failed, can not parse ip addr.\n");
        return false;
      }
      std::string ip_addr = object["ip_addr"].GetString();
      std::strncpy(lidar_register_info.ip_addr, ip_addr.c_str(), sizeof(lidar_register_info.ip_addr));

      if (!object.HasMember("data_type") || !object["data_type"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse data type.\n");
        return false;
      } 
      uint32_t data_type = object["data_type"].GetInt();
      config.data_type = static_cast<uint8_t>(data_type);

      if (!object.HasMember("scan_pattern") || !object["scan_pattern"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse scan pattern.\n");
        return false;
      }
      uint32_t scan_pattern = object["scan_pattern"].GetInt();
      config.scan_pattern = static_cast<uint8_t>(scan_pattern);

      if (!object.HasMember("blind_spot_set") || !object["blind_spot_set"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse blind spot set.\n");
        return false;
      }
      config.blind_spot_set = object["blind_spot_set"].GetInt();

      if (!object.HasMember("dual_emit_enable") || !object["dual_emit_enable"].IsBool()) {
        printf("Parse vehicle lidar config file failed, can not parse dual emit enable.\n");
        return false;
      }
      config.dual_emit_enable = object["dual_emit_enable"].GetBool();

      LidarExtrinsicParameters lidar_extrins_param;
      lidar_extrins_param.lidar_type = kVehicleLidarType;
      lidar_extrins_param.handle = config.slot;

      if (!object.HasMember("roll") || !object["roll"].IsFloat()) {
        printf("Parse vehicle lidar config file failed, can not parse roll.\n");
        return false;
      }
      lidar_extrins_param.roll = object["roll"].GetFloat();

      if (!object.HasMember("pitch") || !object["pitch"].IsFloat()) {
        printf("Parse vehicle lidar config file failed, can not parse pitch.\n");
        return false;
      }
      lidar_extrins_param.pitch = object["pitch"].GetFloat();

      if (!object.HasMember("yaw") || !object["yaw"].IsFloat()) {
        printf("Parse vehicle lidar config file failed, can not parse yaw.\n");
        return false;
      }
      lidar_extrins_param.yaw = object["yaw"].GetFloat();

      if (!object.HasMember("x") || !object["x"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse x.\n");
        return false;
      }
      lidar_extrins_param.x = object["x"].GetInt();

      if (!object.HasMember("y") || !object["y"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse y.\n");
        return false;
      }
      lidar_extrins_param.y = object["y"].GetInt();

      if (!object.HasMember("z") || !object["z"].IsInt()) {
        printf("Parse vehicle lidar config file failed, can not parse z.\n");
        return false;
      }
      lidar_extrins_param.z = object["z"].GetInt();

      printf("Vehicle slot:%d, ip_addr:%s, local_addr:%s, data_type:%u, scan_pattern:%u, blind_spot:%u, dual_emit:%d, roll:%f, pitch:%f, yaw:%f, x:%d, y:%d, z:%d.\n",
              config.slot, lidar_register_info.ip_addr, vehicle_lidar_option.local_addr.c_str(), config.data_type, config.scan_pattern, config.blind_spot_set, config.dual_emit_enable,
              lidar_extrins_param.roll, lidar_extrins_param.pitch, lidar_extrins_param.yaw, lidar_extrins_param.x, lidar_extrins_param.y,
              lidar_extrins_param.z);

      lidars_register_info.push_back(std::move(lidar_register_info));
      lidar_extrins_params.push_back(std::move(lidar_extrins_param));
      raw_vehicle_config.push_back(std::move(config));
    }
  }

  if (raw_file) {
    std::fclose(raw_file);
  }
  return true;
}

} // namespace livox
