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

#ifndef PARSE_LIDAR_STATE_INFO_H_
#define PARSE_LIDAR_STATE_INFO_H_

#include <memory>
#include <map>
#include <mutex>

#include "comm/define.h"
#include "comm/protocol.h"

#include "livox_lidar_def.h"

#include "third_party/rapidjson/document.h"
#include "third_party/rapidjson/filereadstream.h"
#include "third_party/rapidjson/stringbuffer.h"
#include "third_party/rapidjson/prettywriter.h"

namespace livox {

namespace lidar {

class ParseLidarStateInfo {
 public:
  static bool Parse(const CommPacket& packet, std::string& info);
 private:
  static bool ParseStateInfo(const CommPacket& packet, DirectLidarStateInfo& info);
  static void ParseLidarIpAddr(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info);
  static void ParseStateInfoHostIPCfg(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info);
  static void ParsePointCloudHostIpCfg(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info);
  static void ParseImuDataHostIpCfg(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info);
  static void LivoxLidarStateInfoToJson(const DirectLidarStateInfo& info, std::string& lidar_info);
};

} // namespace livox
} // namespace lidar

# endif // PARSE_LIDAR_STATE_INFO_H_




















