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

#include <string.h>

#include "firmware.h"

namespace livox {

Firmware::Firmware() : data_(nullptr), file_size_(0), crc16_(0) {
  memset((void *)&header_, 0, sizeof(header_));
  memset((void *)&tail_, 0, sizeof(tail_));
}

Firmware::~Firmware() {
  if (data_) {
    delete[] data_;
  }

  file_.close();
}

int32_t Firmware::Open(const char *firmware_path) {
  int32_t ret = -1;
  if (!firmware_path) {
    return ret;
  }

  file_.open(firmware_path,
             std::ios::in | std::ifstream::binary | std::ios_base::ate);
  if (!file_.is_open()) {
    printf("Open %s firmware file fail[%d]!\n", firmware_path, ret);
    return ret;
  }

  file_size_ = file_.tellg();
  file_.seekg(0, std::ios::beg);
  if (file_size_ < MiniFileSize()) {
    printf("Firmware file size is too small!\n");
    return ret;
  }

  if (!ReadAndCheckHeader()) {
    printf("Firmware file header is wrong!\n");
    return ret;
  }

  printf("Firmware file total size %lu\n", file_size_);
  ret = 0;
  return ret;
}

uint64_t Firmware::MiniFileSize() {
  return sizeof(header_) + sizeof(tail_) + 1;
}

bool Firmware::ReadAndCheckHeader() {
  file_.seekg(0, std::ios::beg);
  file_.read((char *)(&header_), sizeof(header_));
  printf("This firmware is used for device[%d].\n", header_.device_type);

  uint16_t crc = crc16_.mcrf4xx_calc(
      (uint8_t *)(&header_), (uint16_t)(sizeof(header_) - sizeof(uint16_t)));
  if (crc != header_.header_checksum) {
    printf("Header checksum[%4x %4x] error!\n", crc, header_.header_checksum);
    return false;
  }

  printf("Firmware raw data size : %d\n", header_.firmware_length);
  data_ = new uint8_t[header_.firmware_length];
  file_.read((char *)(data_), header_.firmware_length);
  file_.read((char *)(&tail_), sizeof(tail_));

  if (file_) {
    printf("All firmware data have be read successfully.\n");
  } else {
    printf("Read firmware fail[%ld]!\n", file_.gcount());
  }

  return true;
}
}
