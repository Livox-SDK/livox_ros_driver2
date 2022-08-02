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

#ifndef LIVOX_LVX_FILE_CONVERTER_H_
#define LIVOX_LVX_FILE_CONVERTER_H_

#include <vector>
#include <atomic>

#include "livox_def_common.h"
#include "lvx_file_def.h"
#include "lvx_file_parse_handler.h"
#include "lvx_file_record_handler.h"

namespace livox {
namespace common {

class LvxFileConverter {
 public:
  LvxFileConverter();
  //* convert a .lvx2 file to a .lvx3 file
  bool Lvx2ToLvx3(const char *lvx2_file_path, const char *output_dir);
  //* convert a .lvx3 file to a .lvx2 file
  bool Lvx3ToLvx2(const char *lvx3_file_path, const char *output_dir);

 private:
  bool Init();
  bool ParseLvxFile(const char *lvx_file_path);
  bool InitFileHeader(const char *output_dir, LvxFileType file_type);
  bool RecordLvx2Frames();
  bool RecordLvx3Frames();
  void Reset();
  bool ConvertFileType(const char *file_path, const char *output_dir, LvxFileType target_file_type);

  std::vector<LvxDeviceInfo> infos_;
  std::atomic<bool> is_working_{false};
  std::atomic<bool> is_ready_{false};
};

LvxFileConverter &lvx_file_converter();

} // namespace common
} // namespace livox



#endif // LIVOX_LVX_FILE_CONVERTER_H_