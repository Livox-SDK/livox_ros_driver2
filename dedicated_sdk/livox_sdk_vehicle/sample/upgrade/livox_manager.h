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

#ifndef UPGRADE_LIVOX_MANAGER_H_
#define UPGRADE_LIVOX_MANAGER_H_

#include <stdint.h>
#include <string.h>
#include <mutex>
#include <vector>
#include <map>
#include <unordered_map>

#include "firmware.h"
#include "lidar.h"
#include "upgrade.h"

namespace livox {

void ParseCommandlineInputBroadcastCode(
    const char* cammandline_str, std::vector<std::string>& broadcast_code_list);
uint32_t SplitStr(char* str, const char* delim, char** strs, uint32_t m,
    uint32_t n);
int32_t ParseFirmwareInfoStr(char* str, std::map<std::string,
    std::string>& KeyValueItems);
char HexToXPengChar(uint8_t byte);
void IntVerToXPengVer(uint32_t ver, std::string& ver_str);

class LivoxManager {
 public:
  LivoxManager();
  ~LivoxManager();

  static void RequestFirmwareInfoResponseHandler(livox_vehicle_status status,
      uint8_t slot, RequestFirmwareInfoResponse* response, void* client_data);

  int32_t Init(std::vector<std::string>& broadcast_code_list);

  void AddNewLidar(DeviceInfo* info);
  bool IsExistInWhitelist(const char* broadcast_code);
  int32_t AddBroadcastCodeToWhitelist(const char* broadcast_code);
  std::shared_ptr<Lidar> CreateNewLidar(DeviceInfo* dev_info);
  std::shared_ptr<Lidar> FindLidar(const char* broadcast_code);
  int32_t GetLidarSlot(const char* broadcast_code);
  bool IsLidarConnected(const char* broadcast_code);
  bool IsLidarGetFirmwareInfo(const char* broadcast_code);

  void UpdateLidarFirmwareInfo(uint32_t slot,
    std::map<std::string, std::string>& KeyValuePair);
  int32_t OpenFirmware(const char* firmware_path);
  int32_t UpgradeLidar(const char* broadcast_code);
  livox_vehicle_status RequestFirmwareInfo(uint32_t slot);
  int32_t GetXPengStyleVersion(std::shared_ptr<Lidar>& lidar,
    XPengStyleVersion& xpeng_version);

 private:
  // comm::CommNode* server_node_;
  std::vector<std::string> lidar_whiltelist_;
  typedef std::unordered_map<uint8_t, std::shared_ptr<Lidar>> LidarDictType;
  LidarDictType lidars_;
  std::mutex mutex_;
  uint8_t handle_base_;
  std::shared_ptr<Firmware> firmware_;
  uint32_t try_count_;
};

}  // namespace livox
#endif
