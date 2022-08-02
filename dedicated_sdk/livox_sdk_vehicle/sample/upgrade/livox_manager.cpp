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

#include "livox_manager.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <thread>
#include <map>
#include <string>

//#include <sys/socket.h>
//#include <sys/types.h>
//#include <unistd.h>

// #include "network_utils/net_utils.h"
#include "upgrade.h"

namespace livox {

using namespace std;

LivoxManager::LivoxManager()
    : handle_base_(0) {}

LivoxManager::~LivoxManager() {
}

int32_t LivoxManager::Init(vector<string>& broadcast_code_list) {
  for (auto ite : broadcast_code_list) {
    AddBroadcastCodeToWhitelist(ite.c_str());
  }

  return 0;
}

/** Find in broadcastCode whitelist */
bool LivoxManager::IsExistInWhitelist(const char* broadcast_code) {
  if (!broadcast_code) {
    return false;
  }

  for (auto ite : lidar_whiltelist_) {
    if (strncmp(broadcast_code, ite.c_str(), kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

/** Add broadcast code to whitelist */
int32_t LivoxManager::AddBroadcastCodeToWhitelist(const char* broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize)) {
    return -1;
  }

  if (IsExistInWhitelist(broadcast_code)) {
    printf("%s is alrealy exist!\r\n", broadcast_code);
    return 0;
  }

  {
    lock_guard<mutex> lock(mutex_);
    lidar_whiltelist_.push_back(broadcast_code);
  }

  return 0;
}

/** Find in lidar list */
shared_ptr<Lidar> LivoxManager::FindLidar(const char* broadcast_code) {
  shared_ptr<Lidar> tmp;
  if (!broadcast_code) {
    return tmp;
  }

  for (auto ite : lidars_) {
    char* cur_broadcast_code = (char *)(ite.second->dev_info_.broadcast_code);
    if (strncmp(broadcast_code, cur_broadcast_code, kBroadcastCodeSize) == 0) {
      tmp = ite.second;
      break;
    }
  }

  return tmp;
}

/** Get lidar handle in lidar list */
int32_t LivoxManager::GetLidarSlot(const char* broadcast_code) {
  shared_ptr<Lidar> tmp;
  if (!broadcast_code) {
    return -1;
  }

  for (auto ite : lidars_) {
    char* cur_broadcast_code = (char *)(ite.second->dev_info_.broadcast_code);
    if (strncmp(broadcast_code, cur_broadcast_code, kBroadcastCodeSize) == 0) {
      return ite.second->dev_info_.slot;
    }
  }

  return -1;
}

/** Get lidar handle in lidar list */
bool LivoxManager::IsLidarConnected(const char* broadcast_code) {
  shared_ptr<Lidar> tmp;
  if (!broadcast_code) {
    return false;
  }

  int32_t slot = GetLidarSlot(broadcast_code);
  if (slot >= 0) {
    LidarDictType::iterator ite = lidars_.find(slot);
    if (ite != lidars_.end()) {
      shared_ptr<Lidar> lidar = ite->second;
      if (lidar) {
        if (lidar->connect_state_ > kConnectStateHandshake) {
          return true;
        }
      }
    }
  } else {
    printf("Lidar[%s] isn't connected\r\n", broadcast_code);
  }

  return false;
}

bool LivoxManager::IsLidarGetFirmwareInfo(const char* broadcast_code) {
  shared_ptr<Lidar> tmp;
  if (!broadcast_code) {
    return false;
  }

  int32_t slot = GetLidarSlot(broadcast_code);
  if (slot >= 0) {
    LidarDictType::iterator ite = lidars_.find(slot);
    if (ite != lidars_.end()) {
      shared_ptr<Lidar> lidar = ite->second;
      if (lidar) {
        if (lidar->connect_state_ == kConnectStateGetLidarInfo) {
          return true;
        }
      }
    }
  } else {
    printf("Lidar[%s] don't have firmware info\r\n", broadcast_code);
  }

  return false;
}

void LivoxManager::AddNewLidar(DeviceInfo* info) {
  shared_ptr<Lidar> new_lidar;
  bool handshake_request = false;
  do {
    if (!IsExistInWhitelist((char *)(info->broadcast_code))) {
      printf("Lidar[%s] is not in whitelist\r\n", info->broadcast_code);
      break;
    }

    new_lidar = FindLidar((char *)(info->broadcast_code));
    if (new_lidar) {
      printf("Lidar[%s] already exist\r\n", info->broadcast_code);
      if (new_lidar->connect_state_ < kConnectStateOn) {
        new_lidar->connect_state_ = kConnectStateOn;
        handshake_request = true;
      }
      break;
    }

    new_lidar = CreateNewLidar(info);
    if (new_lidar) {
      lidars_[new_lidar->dev_info_.slot] = new_lidar;
      handshake_request = true;
    }
  } while (0);

  /* Handshake not needed, but we can get the lidar firmware info */
  if (handshake_request) {
    printf("Lidar[%s] connect!\r\n", info->broadcast_code);
    /* Get Lidar firmware info by 0xFF command */
    RequestFirmwareInfo(new_lidar->dev_info_.slot);
  }
}

shared_ptr<Lidar> LivoxManager::CreateNewLidar(
    DeviceInfo* dev_info) {
  shared_ptr<Lidar> tmp;

  tmp.reset(new Lidar);
  memcpy(&tmp->dev_info_, dev_info, sizeof(DeviceInfo));
  tmp->connect_state_ = kConnectStateOn;

  printf("Create New Lidar[%s]\r\n", tmp->dev_info_.broadcast_code);
  return tmp;
}

int32_t LivoxManager::OpenFirmware(const char* firmware_path) {
  firmware_ = make_shared<Firmware>();
  if (firmware_->Open(firmware_path)) {
    printf("Open firmware_path fail\r\n");
    return -1;
  }

  return 0;
}

int32_t LivoxManager::UpgradeLidar(const char* broadcast_code) {
  if (!firmware_) {
    printf("Firmware is null\r\n");
    return 0;
  }

  shared_ptr<Upgrade> upgrade(new Upgrade(firmware_));
  shared_ptr<Lidar> lidar = FindLidar(broadcast_code);
  if (lidar) {
    upgrade->SetLidar(lidar);
    // lidar->SetUpgrader(upgrade_handler);
    upgrade->FsmEventHandler(kEventRequestUpgrade);
  }

  do {
    this_thread::sleep_for(chrono::milliseconds(100));
    if (upgrade->IsUpgradeError()) {
      printf("Upgrade error, try again please!\r\n");
      break;
    }

    if (upgrade->IsUpgradeComplete()) {
      printf("Upgrade successfully\r\n");
      break;
    }
  } while (1);

  return 0;
}

void LivoxManager::UpdateLidarFirmwareInfo(uint32_t slot,
    map<string, string> &KeyValuePair) {
  LidarDictType::iterator ite = lidars_.find(slot);
  if (ite != lidars_.end()) {
    shared_ptr<Lidar> lidar = ite->second;

    map<string, string>::iterator tmp_type = KeyValuePair.find("FmType");
    if (tmp_type != KeyValuePair.end()) {
        if (tmp_type->second == "App") {
          lidar->firmware_type = 1;
        } else {
          lidar->firmware_type = 0;
        }
    }

    map<string, string>::iterator tmp_app_ver = KeyValuePair.find("AppVer");
    if (tmp_app_ver != KeyValuePair.end()) {
      if (tmp_app_ver->second.size() > 4) { /* dji fm version */
        lidar->app_ver = stoi(tmp_app_ver->second);
      } else {/* Indicate ABCD style version */
        lidar->app_ver = 0xFFFF;
        lidar->xpeng_ver.app_ver.assign("V");
        lidar->xpeng_ver.app_ver.append(tmp_app_ver->second.substr(1,1));
        lidar->xpeng_ver.app_ver.append(".");
        lidar->xpeng_ver.app_ver.append(tmp_app_ver->second.substr(2,1));
        lidar->xpeng_ver.app_ver.append(".");
        lidar->xpeng_ver.app_ver.append(tmp_app_ver->second.substr(3,1));
        printf("[00]App version[%s]\r\n", lidar->xpeng_ver.app_ver.c_str());
      }
      printf("[0000]App version length[%ld]\r\n", tmp_app_ver->second.size());
      printf("[0]App version[%d]\r\n", lidar->app_ver);
    } else {
      if (!lidar->firmware_type) { /* in loader mode, init app_ver to 0 */
        lidar->app_ver = 0;
        printf("[1]App version[%d]\r\n", lidar->app_ver);
      } else {
        map<string, string>::iterator tmp_fm_ver = KeyValuePair.find("FmVer");
        if (tmp_fm_ver != KeyValuePair.end()) {
          lidar->app_ver = stoi(tmp_fm_ver->second);
          printf("[2]App version[%d]\r\n", lidar->app_ver);
        } else {
          lidar->app_ver = 0;
          printf("[3]App version[%d]\r\n", lidar->app_ver);
        }
      }
    }

    map<string, string>::iterator tmp_loader_ver =
        KeyValuePair.find("LoaderVer");
    if (tmp_loader_ver != KeyValuePair.end()) {
      if (tmp_loader_ver->second.size() > 4) { /* dji fm version */
        lidar->loader_ver = stoi(tmp_loader_ver->second);
      } else {/* Indicate ABCD style version */
        lidar->loader_ver = 0xFFFF;
        lidar->xpeng_ver.loader_ver.assign("V");
        lidar->xpeng_ver.loader_ver.append(tmp_loader_ver->second.substr(1,1));
        lidar->xpeng_ver.loader_ver.append(".");
        lidar->xpeng_ver.loader_ver.append(tmp_loader_ver->second.substr(2,1));
        lidar->xpeng_ver.loader_ver.append(".");
        lidar->xpeng_ver.loader_ver.append(tmp_loader_ver->second.substr(3,1));
        printf("[00]Loader version[%s]\r\n",
            lidar->xpeng_ver.loader_ver.c_str());
      }
      printf("Loader version[%d]\r\n", lidar->loader_ver);
    } else {
      lidar->loader_ver = 0;
    }

    map<string, string>::iterator hw_ver = KeyValuePair.find("HwVer");
    if (hw_ver != KeyValuePair.end()) {
      if (hw_ver->second.size() == 4) { /* "xxxx" strle version */
        lidar->hw_ver = stoi(hw_ver->second);
      } else if (hw_ver->second.size() > 4) { /* dji hw version : xx_xx */
        lidar->hw_ver = stoi(hw_ver->second.substr(3,2));
      } else {/* ileagal hw version */
        lidar->hw_ver = 0;
      }
      printf("Hw version[%d]\r\n", lidar->hw_ver);
    } else {
      lidar->hw_ver = 0;
    }

    /* Convert to xpeng style version */
    GetXPengStyleVersion(lidar, lidar->xpeng_ver);
    /* Update connect state */
    lidar->connect_state_ = kConnectStateGetLidarInfo;
  }
}

livox_vehicle_status LivoxManager::RequestFirmwareInfo(uint32_t slot) {
  return LidarRequestFirmwareInfo(slot, RequestFirmwareInfoResponseHandler,
      this);
}

void LivoxManager::RequestFirmwareInfoResponseHandler(livox_vehicle_status status,
    uint8_t slot, RequestFirmwareInfoResponse* response, void* client_data) {
  LivoxManager* manager = static_cast<LivoxManager *>(client_data);

  if (status == kStatusSuccess) {
    manager->try_count_ = 0;
    if (response->ret_code) {
      printf("Request firmware info Fail[%d]\r\n", response->ret_code);
    } else {
      printf("Request firmware info[%d][%s]\r\n", response->length,
          response->info);
      if (response->length) {
        map<string, string> KeyValuePair;
        ParseFirmwareInfoStr((char *)response->info, KeyValuePair);
        manager->UpdateLidarFirmwareInfo(slot, KeyValuePair);
      }
    }
  } else {
    printf("Request firmware info fail, try again[%d]!\r\n",
        manager->try_count_);
    ++manager->try_count_;
    if (manager->try_count_ < 3) {
      manager->RequestFirmwareInfo(slot);
    } else {
      manager->try_count_ = 0;
      printf("Request firmware info exceed limit, exit!\r\n");
    }
  }
}

int32_t LivoxManager::GetXPengStyleVersion(std::shared_ptr<Lidar>& lidar,
    XPengStyleVersion& xpeng_ver) {
    if (lidar) {
      if (lidar->firmware_type) {
        xpeng_ver.fw_type.assign("App");
      } else {
        xpeng_ver.fw_type.assign("Loader");
      }

      uint8_t byte = 0;
      char substr[2] = { 0 };
      xpeng_ver.hw_ver.assign("H.");

      if (lidar->hw_ver > 35) {
        byte = 35;
      } else {
        byte = lidar->hw_ver;
      }
      substr[0] = HexToXPengChar(byte);
      xpeng_ver.hw_ver.append(substr);

      if (lidar->app_ver != 0xFFFF) {
        IntVerToXPengVer(lidar->app_ver, xpeng_ver.app_ver);
      }
      if (lidar->loader_ver != 0xFFFF) {
        IntVerToXPengVer(lidar->loader_ver, xpeng_ver.loader_ver);
      }
      printf("FmType[%s] Hw[%s] AppVer[%s] LoaderVer[%s]\r\n",
          xpeng_ver.fw_type.c_str(), xpeng_ver.hw_ver.c_str(),
          xpeng_ver.app_ver.c_str(), xpeng_ver.loader_ver.c_str());
    }

    return 0;
}

/** Global function */
void ParseCommandlineInputBroadcastCode(
    const char* cammandline_str,
    vector<string>& broadcast_code_list) {
  char strs[kMaxSupportLidarNum][kMaxFirmwareInfoItemSize] = { 0 };

  char* str = new char[sizeof(cammandline_str)];
  strcpy(str, cammandline_str);

  uint32_t input_num = SplitStr(str, "&", (char **)strs, kMaxSupportLidarNum,
      kMaxFirmwareInfoItemSize);
  char invalid_broadcast_code[] = "000000000";
  printf("Commandline input broadcast code :\r\n");
  for (uint32_t i = 0; i < input_num; i++) {
    printf("[%s] length[%ld]\r\n", strs[i], strlen(strs[i]));
    if ((kBroadcastCodeSize > strlen(strs[i])) &&
        (NULL == strstr(strs[i], invalid_broadcast_code))) {
      broadcast_code_list.push_back(strs[i]);
    } else {
      printf("Invalid broadcast code:%s!\r\n", strs[i]);
    }
  }

  delete[] str;
}

/** Split string to strings by delim
 *  @str: input string will be split
 *  @delim: the delimiter string
 *  @strs: strs array supplyed by the caller,which storage the strs
 *  @strs_num: the strs array colomn
 *  @length: the strs array line
 *  @ret:the number of splited strs
 */
uint32_t SplitStr(char* str, const char* delim, char** strs, uint32_t m,
    uint32_t n) {
  uint32_t strs_num = 0;
  char *ptr;
  char *p;
  char *strs_header = (char *)strs;

#ifndef WIN32
  ptr = strtok_r(str, delim, &p);
  while(ptr != NULL){
    if (strs_num < m) { /* strorage str */
      strncpy(strs_header, ptr, n);
      strs_header += n;
    } else {
      break;
    }
    ++strs_num;
    ptr = strtok_r(NULL, delim, &p);
  }
#else
  ptr = strtok_s(str, delim, &p);
  while(ptr != NULL){
    if (strs_num < m) { /* strorage str */
      strncpy(strs_header, ptr, n);
      strs_header += n;
    } else {
      break;
    }
    ++strs_num;
    ptr = strtok_s(NULL, delim, &p);
  }
#endif

  return strs_num;
}

int32_t ParseFirmwareInfoStr(char* str, map<string, string> &KeyValueItems) {
  if (str == nullptr) {
    return -1;
  }

  /* Split info string by space, and storage all sub strs */
  char strs[kMaxFirmwareInfoItems][kMaxFirmwareInfoItemSize] = { 0 };
  uint32_t strs_num = SplitStr(str, " ", (char**)strs, kMaxFirmwareInfoItems,
      kMaxFirmwareInfoItemSize);

  /* Parse every substr and get the key-value pair */
  for (uint32_t i = 0; i < strs_num; i++) {
    char members[2][kMaxFirmwareInfoItemSize];
    memset(members, 0, sizeof(members));
    uint32_t members_num = SplitStr(strs[i], ":", (char**)members, 2,
      kMaxFirmwareInfoItemSize);
    if (members_num == 2) {
      KeyValueItems.insert(pair<string, string>(members[0], members[1]));
      printf("[%s][%s]\r\n", members[0], members[1]);
    }
  }

  return 0;
}

char HexToXPengChar(uint8_t byte) {
  static const char map_table[37] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  uint8_t tmp = 0;

  tmp = byte;
  if (byte > 35) {
    tmp = 35;
  }

  return map_table[tmp];
}

void IntVerToXPengVer(uint32_t ver, string& ver_str) {
  uint8_t byte = 0;
  char substr[2] = { 0 };

  ver_str.assign("V");

  byte = (ver >> 16) & 0xFF; /* Highest byte */
  substr[0] = HexToXPengChar(byte);
  ver_str.append(substr);
  ver_str.append(".");

  byte = (ver >> 8) & 0xFF; /* mid byte */
  substr[0] = HexToXPengChar(byte);
  ver_str.append(substr);
  ver_str.append(".");

  byte = ver & 0xFF; /* Lowest byte */
  substr[0] = HexToXPengChar(byte);
  ver_str.append(substr);
}

}  // namespace livox
