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
#include "livox_upgrade_tool.h"
#include <stdint.h>
#include <chrono>
#include <thread>
#include <map>

#include "livox_manager.h"
#include "upgrade.h"

using namespace std;
using namespace livox;

typedef struct {
  uint8_t WorkMode;
  uint8_t SlotId;
  uint8_t AddressData[18];
  uint8_t VersionInfo[12];
  uint8_t ProductInfo[19];
} LidarDiagInternalInfo;

const int kLidarNumber = 2;
const char* kNetIf = "192.168.1.2"; //local netcard's address
const LidarRegisterInfo lidar_info[kLidarNumber] = {
  { 1, "192.168.1.3" },
  { 2, "172.20.1.53" }
};

std::map<uint8_t, LidarDiagInternalInfo> DiagInternalInfo;

/*Log File Descriptor*/
// const std::string log_save_dir = "./";
// std::map<uint8_t, std::FILE*> log_fp;

// std::string GetCurFormatTime() {
//   std::time_t t = std::time(nullptr);
//   std::stringstream format_time;
//   format_time << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S");
//   return format_time.str();
// }

void LidarExceptionDetailCallback(uint8_t slot,
  LivoxDetailExceptionInfo* info, void* client_data) {
    printf("exception detail callback\n");
}

void LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data) {
  LivoxManager* lidar_manager = static_cast<LivoxManager *>(client_data);
  printf("Lidar Slot: %d SN: %s\n", info->slot, info->broadcast_code);
  lidar_manager->AddNewLidar(info);
}

int main(int argc, char** argv) {
  LivoxManager lidar_manager;

  printf("Livox Upgrade Tool Version: %s\n", LIVOX_UPGRADE_TOOL_VERSION_STRING);
  printf("Commandline input %d args : \n", argc);
  for (int i = 0; i < argc; i++) {
    printf("%s\n", argv[i]);
  }

  /** Init with host's network card's ip address. */
  if (!LivoxInit(kNetIf)) {
    printf("Livox Init Failed\n");
    LivoxUninit();
    return -1;
  }

  SetExceptionDetailCallback(LidarExceptionDetailCallback, nullptr); /** Set Exception Inforamtion Callback. */
  SetLidarInfoChangeCallback(LidarInfoChangeCallback, &lidar_manager); /** Set Lidar Inforamtion Change Callback. */
  RegisterLidarInfo(lidar_info, kLidarNumber); /** Register lidars slot and ip address to listen. */

  vector<string> broadcast_code_list;
  if (argc > 1) {
    ParseCommandlineInputBroadcastCode(argv[1], broadcast_code_list);
  } else {
    printf("Please input broadcast code!\n");
  }
  broadcast_code_list.push_back("broadcast_code_list");

  lidar_manager.Init(broadcast_code_list);
  if (argc > 2) {
    lidar_manager.OpenFirmware(argv[2]);
    while (true) {
      this_thread::sleep_for(chrono::milliseconds(500));
      if (lidar_manager.IsLidarGetFirmwareInfo(
          broadcast_code_list[0].c_str())) {
        printf("Start to upgrade %s\n", broadcast_code_list[0].c_str());
        lidar_manager.UpgradeLidar(broadcast_code_list[0].c_str());
        break;
      }
    }
  } else {
    printf("Please input firmware path!\n");
  }

  LivoxUninit();
  exit(0);
  return 0;
}
