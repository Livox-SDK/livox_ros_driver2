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

#ifndef LIVOX_LVX_FILE_PARSE_HANDLER_H_
#define LIVOX_LVX_FILE_PARSE_HANDLER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <atomic>

#include "livox_def_common.h"
#include "livox_def_vehicle.h"
#include "lvx_file_def.h"
#include "livox_lidar_def.h"

namespace livox {
namespace common {

class LvxFileParseHandler {
public:
  struct FrameDetail {
    uint64_t start_offset;
    uint64_t data_len;
  };

  LvxFileParseHandler();
  bool SetLvxParseDir(const std::string& dir);
  void GetLvxDeviceInfoList(std::vector<LvxDeviceInfo>& info);

  // when return false, no arguments changed.
  bool GetPointData(const int frame_index,
                    LvxBasePackDetail* &lvx_pack_detail,
                    uint32_t& packet_num,
                    LvxBaseHalfPackDetail* &lvx_half_pack_detail,
                    uint32_t& half_packet_num);
  // convert 16-bit data to 32-bit data, and then output is all 32-bit data
  // minor performance loss compared to `GetPointData`
  const LvxBasePackDetail* GetHighResPointData(const int frame_index, uint32_t& packet);

  uint32_t GetFrameCount();
  uint32_t GetFrameDuration();
  bool StartParseLvxFile();
  void StopParseLvxFile();

  // bool IsFileValid();
  bool IsFileSupported();
  LvxFileVersion GetFileVersion();

  bool CutLvxFile(uint64_t start_frame_index, uint64_t end_frame_index, const std::string &dir);

private:
  bool ParseLvxFileHeader();
  bool InitFrameList();
  bool ReadLvxFrameInfos();
  bool ReadPptFrameInfos();
  bool UpdatePacketList(int32_t frame_index);
  bool ConvertToHighResPackets();
  void ResetPacketList();
  void Clear();

private:
  std::ifstream lvx_file_;
  std::ofstream re_lvx_file_;
  uint8_t device_count_ = 0;
  uint32_t frame_duration_ = 0;
  LvxFileVersion file_version_;
  LvxFileType file_type_ = LIVOX_FILE_UNKNOWN;

  std::vector<LvxDeviceInfo> device_infos_;
  uint64_t lvx_file_size_ = 0;
  std::vector<FrameDetail> frame_detail_list_;
  std::vector<LvxBasePackDetail> packet_list_;
  std::vector<LvxBaseHalfPackDetail> half_packet_list_;
};

LvxFileParseHandler &lvx_file_parse_handler();

}
}  // namespace livox

#endif  // LIVOX_LVX_FILE_RECORD_HANDLER_H_
