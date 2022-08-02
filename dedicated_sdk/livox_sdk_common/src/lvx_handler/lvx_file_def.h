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

#ifndef LIVOX_LVX_FILE_H_
#define LIVOX_LVX_FILE_H_

#include <map>
#include <vector>
#include <string>

#include "livox_def_common.h"

#define MAGIC_CODE       (0xac0ea767)

namespace livox {
namespace common {

const std::string kFileSignature    = "livox_tech";

typedef struct _PACKAGE_POINT {
  int32_t x;
  int32_t y;
  int32_t z;
  unsigned char reflectances;
  unsigned char tag;
} PACKAGE_POINT;


typedef struct {
  LvxBasePackHeader header;
  std::vector<PACKAGE_POINT> packets;
} LvxPackageDetail;

typedef struct {
  uint64_t current_offset;
  uint64_t next_offset;
  uint64_t frame_index;
} Lvx2FrameHeader;

typedef struct {
  uint64_t current_offset;      // frame offset in .lvx2 file
  uint32_t packets_num;         // number of packets in the frame
  uint64_t data_size;           // sizeof(Lvx3FrameHeader) + sizeof(LvxBasePackHeader) * packets_num + encoded_data_size
} Lvx3FrameHeader;

} // namespace common
} // namespace livox

#endif // LIVOX_LVX_FILE_H_