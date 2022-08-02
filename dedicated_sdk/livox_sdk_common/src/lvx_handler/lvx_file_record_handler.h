//
// The MIT License (MIT)
//
// Copyright (c) 2021 Livox. All rights reserved.
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

#ifndef LIVOX_LVX_FILE_RECORD_HANDLER_H_
#define LIVOX_LVX_FILE_RECORD_HANDLER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>

#include "livox_def_common.h"
#include "livox_def_vehicle.h"
#include "livox_lidar_def.h"
#include "livox_def.h"
#include "lvx_file_def.h"

namespace livox {
namespace common {

class LvxFileConverter;

class LvxFileRecordHandler {
public:
  LvxFileRecordHandler();
  bool SetLvxRecordDir(const std::string &dir);
  void AddDeviceInfo(std::vector<LvxDeviceInfo> info);
  void StartRecordLvxFile(LvxFileType file_type);
  void StopRecordLvxFile();

  friend LvxFileConverter;

  // tmp for public;

private:
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
  static void OnVehicleLidarPointCloudCallback(uint8_t slot, LivoxVehicleEthPacket* data, uint32_t data_num, void* client_data);
  static void OnLidarPointCloudCallback(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data);

  void BasePointsHandle(uint32_t handle, LivoxLidarEthernetPacket *data, uint32_t data_num,
    std::vector<LvxBasePackDetail> &packets, std::vector<LvxBaseHalfPackDetail>& half_packets);
  void BasePointsHandle(uint8_t slot, LivoxVehicleEthPacket *data, uint32_t data_num,
      std::vector<LvxBasePackDetail> &packets, std::vector<LvxBaseHalfPackDetail>& half_packets);
  void BasePointsHandle(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, std::vector<LvxBasePackDetail> &packets);

  void PackagePointHandle(uint32_t handle, const LivoxLidarEthernetPacket *data, uint32_t data_num, std::vector<LvxPackageDetail> &package_vec);
  void PackagePointHandle(uint8_t handle, const LivoxEthPacket *data, uint32_t data_num, LvxPackageDetail &package);
  void PackagePointHandle(uint8_t slot, const LivoxVehicleEthPacket *data, uint32_t data_num, std::vector<LvxPackageDetail> &package_vec);

  void CopyPackHeader(uint32_t handle, const LivoxLidarEthernetPacket &eth_packet, uint32_t data_num, LvxBasePackHeader &header);
  void CopyPackHeader(uint8_t handle, const LivoxEthPacket &eth_packet, uint32_t data_num, LvxBasePackHeader &header);
  void CopyPackHeader(uint8_t slot, const LivoxVehicleEthPacket &eth_packet, uint32_t data_num, LvxBasePackHeader &header);

  void SaveFrameToLvx3File(const std::vector<std::vector<PACKAGE_POINT>> &package_vec,
                           const std::vector<LvxBasePackHeader> &header_vec);
  void SaveFrameToLvx3File(const std::vector<LvxPackageDetail> &packages_detail_vec);
  void SaveFrameToLvx2File(const std::vector<LvxBasePackDetail>& point_packet_list,
    const std::vector<LvxBaseHalfPackDetail>& point_half_packet_list);

  bool InitLvxFile(LvxFileType file_type);
  void InitLvxFileHeader();
  void CloseLvxFile();
  void Clear();

  //* file related
  std::string file_dir_;
  std::ofstream lvx_file_;
  LvxFileType file_type_;

  std::vector<LvxDeviceInfo> device_info_list_;
  uint32_t cur_frame_index_;
  uint64_t cur_offset_;
  uint32_t frame_duration_;
  uint64_t total_point_num_;

  uint16_t vehicle_listen_id_ = 0;
  uint16_t listen_id_ = 0;
  uint16_t livox_lidar_id_ = 0;
  std::vector<LvxBasePackDetail> point_packet_list_;
  std::vector<LvxBaseHalfPackDetail> point_half_packet_list_;
  std::vector<LvxPackageDetail> packages_vec_;

  std::condition_variable cv_;
  std::mutex mtx_;
  std::shared_ptr<std::thread> lvx_save_thread_;
  std::atomic<bool> is_quit_{false};
  std::atomic<bool> is_recording_{false};
  std::atomic<bool> is_stopping_{false};
};

LvxFileRecordHandler &lvx_file_record_handler();

}
}  // namespace livox

#endif  // LIVOX_LVX_FILE_RECORD_HANDLER_H_
