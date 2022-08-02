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

#include "lvx_file_manager.h"

namespace livox {
namespace common {

// all supported file versions
namespace {
  LvxFileVersion kLvxFileVer{1, 1, 0, 0};
  LvxFileVersion kLvx2FileVer{2, 0, 0, 0};
  LvxFileVersion kLvx3PptFileVer{2, 4, 0, 0};
}

  std::map<LvxFileType, LvxFileVersion> LvxFileManager::FileTypeMapping_ = {
    {LIVOX_FILE_LVX, kLvxFileVer},
    {LIVOX_FILE_LVX2, kLvx2FileVer},
    {LIVOX_FILE_LVX3_PPT, kLvx3PptFileVer}
  };

std::map<LvxFileType, LvxFileVersion> LvxFileManager::GetFileTypeMapping() {
  return FileTypeMapping_;
}

LvxFileType LvxFileManager::CheckFileType(const LvxFileVersion& ver) {
  /*if (ver.file_ver_major == 1) {
    if (ver.file_ver_minor == 1 && ver.file_ver_patch == 0) {
      return LIVOX_FILE_LVX;
    } else {
      return LIVOX_FILE_UNKNOWN;
    }
  } else */if (ver.file_ver_major == 2) {
    if (ver.file_ver_minor == 0 && ver.file_ver_patch == 0) {
      return LIVOX_FILE_LVX2;
    } /*else if (ver.file_ver_minor == 4 && ver.file_ver_patch == 0) {
      return LIVOX_FILE_LVX3_PPT;
    } */else {
      return LIVOX_FILE_UNKNOWN;
    }
  } else {
    return LIVOX_FILE_UNKNOWN;
  }
}

} // namespace common
} // namespace livox