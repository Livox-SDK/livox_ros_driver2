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

#ifndef LIVOX_PARAMS_CHECK_H_
#define LIVOX_PARAMS_CHECK_H_

#include "livox_lidar_def.h"

#include "comm/define.h"

#include <iostream>
#include <string>
#include <memory>
#include <vector>

namespace livox {
namespace lidar {

class ParamsCheck {
 public:
  ParamsCheck(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
      std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);
  bool Check();
 private:
  bool CheckLidarIp();
 private:
  std::shared_ptr<std::vector<LivoxLidarCfg>> lidars_cfg_ptr_;
  std::shared_ptr<std::vector<LivoxLidarCfg>> custom_lidars_cfg_ptr_;
};

} // namespace lidar
} // namespace livox

#endif // LIVOX_PARSE_CFG_FILE_H_

