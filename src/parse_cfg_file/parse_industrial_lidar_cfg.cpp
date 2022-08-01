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
#include "parse_industrial_lidar_cfg.h"

namespace livox_ros {

ParseIndustrialLidarCfgFile::ParseIndustrialLidarCfgFile(const std::string& path) : path_(path) {}


bool ParseIndustrialLidarCfgFile::ParseIndustrialLidarConfig(std::vector<UserRawConfig>& raw_industrial_config,
    bool& enable_timesync, TimeSyncConfig& timesync_config) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    printf("Parse industrial config failed, can not open json config file!\n");
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  
  rapidjson::Document doc;
  if (doc.ParseStream(config_file).HasParseError()) {
    printf("Industrial config file parse failed, parse stream has error!\n");
    return false;
  }

  //industrial_lidar_config
  if (!doc.HasMember("industrial_lidar_config") || !doc["industrial_lidar_config"].IsObject()) {
    return false;
  }

  const rapidjson::Value& object = doc["industrial_lidar_config"].GetObject();
  if (!ParseLidarConfig(object, raw_industrial_config)) {
    return false;
  }

  if (!ParseTimesyncConfig(object, enable_timesync, timesync_config)) {
    printf("Parse timesync config fail\n");
    enable_timesync = false;
  }

  if (raw_file) {
    std::fclose(raw_file);
  }

  return true;
}

bool ParseIndustrialLidarCfgFile::ParseLidarConfig(const rapidjson::Value& object,
    std::vector<UserRawConfig>& raw_industrial_config) {
  if (!(object.HasMember("lidar_config")) || !(object["lidar_config"].IsArray())) {
    printf("Parse industrial lidar cfg file failed, the object has not lidar_confi member "
        "or the member of lidar_config is not array.\n");
    return false;
  }

  const rapidjson::Value &array = object["lidar_config"];
  size_t len = array.Size();
  for (size_t i = 0; i < len; i++) {
    const rapidjson::Value &object = array[i];
    if (object.IsObject()) {
      UserRawConfig config;
      memset(&config, 0, sizeof(config));
      if (object.HasMember("broadcast_code") && object["broadcast_code"].IsString()) {
        std::string broadcast_code = object["broadcast_code"].GetString();
        std::strncpy(config.broadcast_code, broadcast_code.c_str(), sizeof(config.broadcast_code));
      } else {
        printf("User config file parse error\n");
        continue;
      }

      if (object.HasMember("enable_connect") && object["enable_connect"].IsBool()) {
        config.enable_connect = object["enable_connect"].GetBool();
      }
      if (object.HasMember("enable_fan") && object["enable_fan"].IsBool()) {
        config.enable_fan = object["enable_fan"].GetBool();
      }
      if (object.HasMember("return_mode") && object["return_mode"].IsInt()) {
        config.return_mode = object["return_mode"].GetInt();
      }
      if (object.HasMember("coordinate") && object["coordinate"].IsInt()) {
        config.coordinate = object["coordinate"].GetInt();
      }
      if (object.HasMember("imu_rate") && object["imu_rate"].IsInt()) {
        config.imu_rate = object["imu_rate"].GetInt();
      }
      if (object.HasMember("extrinsic_parameter_source") && object["extrinsic_parameter_source"].IsInt()) {
        config.extrinsic_parameter_source = object["extrinsic_parameter_source"].GetInt();
      }
      if (object.HasMember("enable_high_sensitivity") && object["enable_high_sensitivity"].GetBool()) {
        config.enable_high_sensitivity = object["enable_high_sensitivity"].GetBool();
      }

      printf("Industrial broadcast code[%s] : %d %d %d %d %d %d\n",
              config.broadcast_code, config.enable_connect,
              config.enable_fan, config.return_mode, config.coordinate,
              config.imu_rate, config.extrinsic_parameter_source);
      
      raw_industrial_config.push_back(std::move(config));
    }
  }
  return true;
}

bool ParseIndustrialLidarCfgFile::ParseTimesyncConfig(const rapidjson::Value& object,
    bool& enable_timesync, TimeSyncConfig& timesync_config) {
  
  if (!object.HasMember("timesync_config") || !object["timesync_config"].IsObject()) {
    return false;
  }

  const rapidjson::Value &timesync_object = object["timesync_config"];
  if (!timesync_object.HasMember("enable_timesync") || !timesync_object["enable_timesync"].IsBool()) {
    return false;
  }
  enable_timesync = timesync_object["enable_timesync"].GetBool();

  if (!timesync_object.HasMember("device_name") || !timesync_object["device_name"].IsString()) {
    return false;
  }
  std::string device_name = timesync_object["device_name"].GetString();
  std::strncpy(timesync_config.dev_config.name, device_name.c_str(), sizeof(timesync_config.dev_config.name));

  if (!timesync_object.HasMember("comm_device_type") || !timesync_object["comm_device_type"].IsInt()) {
    return false;
  }
  timesync_config.dev_config.type = timesync_object["comm_device_type"].GetInt();


  if (timesync_config.dev_config.type == kCommDevUart) {
    if (!timesync_object.HasMember("baudrate_index") || !timesync_object["baudrate_index"].IsInt()) {
      return false;
    }
    timesync_config.dev_config.config.uart.baudrate = timesync_object["baudrate_index"].GetInt();

    if (!timesync_object.HasMember("parity_index") || !timesync_object["parity_index"].IsInt()) {
      return false;
    }
    timesync_config.dev_config.config.uart.parity = timesync_object["parity_index"].GetInt();
  }

  if (enable_timesync) {
    printf("Enable timesync : ");
    if (timesync_config.dev_config.type == kCommDevUart) {
      printf("Uart[%s],baudrate index[%d],parity index[%d]\n",
              timesync_config.dev_config.name,
              timesync_config.dev_config.config.uart.baudrate,
              timesync_config.dev_config.config.uart.parity);
    }
    printf("Parse config succ.\n");
  } else {
    printf("Disable timesync\n");
    printf("Parse config succ.\n");
  }
  return true;
}

/** Config file process */
void ParseIndustrialLidarCfgFile::ParseHubConfig(UserRawConfig& hub_config, std::vector<UserRawConfig>& configs) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    printf("Parse vehicle config failed, can not open json config file!\n");
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  
  rapidjson::Document doc;
  if (!doc.ParseStream(config_file).HasParseError()) {
    if (doc.HasMember("hub_config") && doc["hub_config"].IsObject()) {
      const rapidjson::Value &object = doc["hub_config"];
      //UserRawConfig hub_config;
      memset(&hub_config, 0, sizeof(hub_config));
      if (object.HasMember("broadcast_code") && object["broadcast_code"].IsString()) {
        std::string broadcast_code = object["broadcast_code"].GetString();
        std::memcpy(hub_config.broadcast_code, broadcast_code.c_str(), sizeof(hub_config.broadcast_code));

        if (object.HasMember("enable_connect") && object["enable_connect"].IsBool()) {
          hub_config.enable_connect = object["enable_connect"].GetBool();
        }
        if (object.HasMember("coordinate") && object["coordinate"].IsInt()) {
          hub_config.coordinate = object["coordinate"].GetInt();
        }

        printf("Hub[%s] : %d %d %d %d %d\n", hub_config.broadcast_code,
                hub_config.enable_connect, hub_config.enable_fan,
                hub_config.return_mode, hub_config.coordinate,
                hub_config.imu_rate);
      } else {
        printf("User hub config file parse error\n");
      }
    }

    if (doc.HasMember("lidar_config") && doc["lidar_config"].IsArray()) {
      const rapidjson::Value &array = doc["lidar_config"];
      size_t len = array.Size();
      for (size_t i = 0; i < len; i++) {
        const rapidjson::Value &object = array[i];
        if (object.IsObject()) {
          UserRawConfig config;
          memset(&config, 0, sizeof(config));
          if (object.HasMember("broadcast_code") && object["broadcast_code"].IsString()) {
            std::string broadcast_code = object["broadcast_code"].GetString();
            std::memcpy(config.broadcast_code, broadcast_code.c_str(), sizeof(config.broadcast_code));
          } else {
            printf("User config file parse error\n");
            continue;
          }

          if (object.HasMember("enable_fan") && object["enable_fan"].IsBool()) {
            config.enable_fan = object["enable_fan"].GetBool();
          }
          if (object.HasMember("return_mode") && object["return_mode"].IsInt()) {
            config.return_mode = object["return_mode"].GetInt();
          }
          if (object.HasMember("imu_rate") && object["imu_rate"].IsInt()) {
            config.imu_rate = object["imu_rate"].GetInt();
          }
          if (hub_config.enable_connect) {
            config.coordinate = hub_config.coordinate;
          } else {
            config.coordinate = 0;
          }
          printf("Lidar[%s] : %d %d %d %d %d\n", config.broadcast_code,
                  config.enable_connect, config.enable_fan, config.return_mode,
                  config.coordinate, config.imu_rate);

          configs.push_back(std::move(config));
        }
      }
    } 
  } else {
    printf("Vehicle config file parse failed, parse stream has error!\n");
  }

  if (raw_file) {
    std::fclose(raw_file);
  }
}


} // namespace livox
