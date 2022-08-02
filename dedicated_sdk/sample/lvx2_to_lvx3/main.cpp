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

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "livox_sdk_common.h"
#include "livox_def_common.h"
#include "livox_sdk.h"
#include "PPTAPI.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <map>
#include <mutex>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <vector>

constexpr int kSupportedLidarNumber = 1;
constexpr int kWriteBufferLength =  1024 * 1024;

bool ParseFile(std::string file) {
  if (!SetLvxParseDir(file.c_str())) {
    printf("Parse file failed.");
    return false;
  }
  if (!StartParseLvxFile()) {
    return false;
  }
  return true;
}

int main(int argc, const char *argv[]) {
  Lvx2ToLvx3("test.lvx2", "./");
  printf("Livox file conversion from .lvx2 to .lvx3 Demo End!\n");
  return 0;
}
