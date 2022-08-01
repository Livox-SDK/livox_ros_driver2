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
#include "parse_livox_lidar_cfg.h"
#include <iostream>

namespace livox_ros {

bool LivoxLidarConfigParser::Parse(std::vector<UserLivoxLidarConfig> &lidar_configs,
                                   std::vector<LidarExtrinsicParameters> &extrinsic_params) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout << "failed to open config file: " << path_ << std::endl;
  }

  lidar_configs.clear();
  extrinsic_params.clear();
  char read_buffer[kMaxBufferSize];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  rapidjson::Document doc;

  do {
    if (doc.ParseStream(config_file).HasParseError()) {
      std::cout << "failed to parse config jason" << std::endl;
      break;
    }
    if (!doc.HasMember("lidar_configs") ||
        !doc["lidar_configs"].IsArray() ||
        0 == doc["lidar_configs"].Size()) {
      std::cout << "there is no user-defined config" << std::endl;
      break;
    }
    if (!ParseBasicConfigs(doc, lidar_configs)) {
      std::cout << "failed to parse basic configs" << std::endl;
      break;
    }
    if (!ParseExtrinsics(doc, extrinsic_params)) {
      std::cout << "failed to parse extrinsic parameters" << std::endl;
      break;
    }
    return true;
  } while (false);

  std::fclose(raw_file);
  return false;
}


bool LivoxLidarConfigParser::ParseBasicConfigs(const rapidjson::Document &doc,
                                              std::vector<UserLivoxLidarConfig> &user_configs) {
  const rapidjson::Value &lidar_configs = doc["lidar_configs"];
  for (auto &config : lidar_configs.GetArray()) {
    if (!config.HasMember("ip")) {
      continue;
    }
    UserLivoxLidarConfig user_config;

    // parse user configs
    user_config.handle = IpStringToNum(std::string(config["ip"].GetString()));
    if (!config.HasMember("data_type")) {
      user_config.data_type = -1;
    } else {
      user_config.data_type = static_cast<int8_t>(config["data_type"].GetInt());
    }
    if (!config.HasMember("pattern_mode")) {
      user_config.pattern_mode = -1;
    } else {
      user_config.pattern_mode = static_cast<int8_t>(config["pattern_mode"].GetInt());
    }
    if (!config.HasMember("blind_spot_set")) {
      user_config.blind_spot_set = -1;
    } else {
      user_config.blind_spot_set = static_cast<int8_t>(config["blind_spot_set"].GetInt());
    }
    if (!config.HasMember("dual_emit_en")) {
      user_config.dual_emit_en = -1;
    } else {
      user_config.dual_emit_en = static_cast<uint8_t>(config["dual_emit_en"].GetInt());
    }
    user_config.set_bits = 0;
    user_config.get_bits = 0;
    user_configs.push_back(user_config);
  }

  if (0 == user_configs.size()) {
    std::cout << "no valid base configs" << std::endl;
    return false;
  }
  std::cout << "successfully parse base config, counts: "
            << user_configs.size() << std::endl;
  return true;
}

bool LivoxLidarConfigParser::ParseExtrinsics(const rapidjson::Document &doc,
                                             std::vector<LidarExtrinsicParameters>& extrinsic_params) {
  const rapidjson::Value &lidar_configs = doc["lidar_configs"];
  for (auto &config : lidar_configs.GetArray()) {
    if (!config.HasMember("ip") || !config.HasMember("extrinsic_parameter")) {
      continue;
    }

    LidarExtrinsicParameters param;
    param.lidar_type = kLivoxLidarType;
    param.handle = IpStringToNum(config["ip"].GetString());
    auto &value = config["extrinsic_parameter"];
    if (!value.HasMember("roll") || !value.HasMember("pitch") ||
        !value.HasMember("yaw")  || !value.HasMember("x") ||
        !value.HasMember("y")    || !value.HasMember("z")){
      std::cout << "incomplete extrinsic parameters, skip..." << std::endl;
      continue;
    }
    param.roll  = static_cast<float>(value["roll"].GetFloat());
    param.pitch = static_cast<float>(value["pitch"].GetFloat());
    param.yaw   = static_cast<float>(value["yaw"].GetFloat());
    param.x     = static_cast<int32_t>(value["x"].GetInt());
    param.y     = static_cast<int32_t>(value["y"].GetInt());
    param.z     = static_cast<int32_t>(value["z"].GetInt());
    std::cout << "extrinsic parameter[" << extrinsic_params.size() << "], " << std::endl
              << "  roll: " << param.roll << ", pitch: " << param.pitch << ", yaw: " << param.yaw
              << "  x: " << param.x << ", y: " << param.y << ", z: " << param.z << std::endl;
    extrinsic_params.push_back(param);
  }

  if (0 == extrinsic_params.size()) {
    std::cout << "no valid extrinsic parameters" << std::endl;
    return false;
  }
  std::cout << "successfully parse extrinsic parameters, counts: "
            << extrinsic_params.size() << std::endl;
  return true;
}


} // namespace livox
