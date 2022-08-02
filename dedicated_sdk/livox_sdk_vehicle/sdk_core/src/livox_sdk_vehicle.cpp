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

#include "livox_sdk_vehicle.h"
#include "preconfig_manager.h"
#include "base/logging.h"

using namespace livox::vehicle;

void GetLivoxVehicleSdkVersion(LivoxVehicleSdkVersion *version) {
  if (version != NULL) {
    version->major = LIVOX_SDK_VEHICLE_MAJOR_VERSION;
    version->minor = LIVOX_SDK_VEHICLE_MINOR_VERSION;
    version->patch = LIVOX_SDK_VEHICLE_PATCH_VERSION;
  }
}

bool LivoxPreConfigInit(const char* net_if, LidarPreConfigParam* param) {
  InitLogger();
  return preconfig_manager().Init(std::string(net_if), *param);
}

void LivoxPreConfigUninit() {
  UninitLogger();
  preconfig_manager().Uninit();
}

void SetLidarPreConfigParamCallback(OnPreConfigCallback cb, void* client_data) {
  preconfig_manager().SetLidarPreConfigParamCallback(cb, client_data);
}