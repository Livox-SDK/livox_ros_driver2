//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#ifndef LIVOX_FILE_MANAGER_
#define LIVOX_FILE_MANAGER_

#include <string>
#include <vector>
#include <map>
#include <cstdint>

namespace livox {
namespace lidar {

uint64_t GetDirTotalSize(const std::string& dir_name);

bool GetFileNames(const std::string& dir_name, std::multimap<std::string, std::string>& files_name);
bool ChangeHiddenFiles(const std::string& dir_name);
bool ChangeCurrentFileName(const std::string& dir_name, std::string file_name);
bool StoreFileName(const char* filename, std::multimap<std::string, std::string>& files_name);
bool DeleteHidFiles(const std::string& dir_name);

bool MakeDirecotory(std::string dir);

bool IsDirectoryExits(std::string dir);

} // namespace lidar
}  // namespace livox

#endif  // LIVOX_FILE_MANAGER_
