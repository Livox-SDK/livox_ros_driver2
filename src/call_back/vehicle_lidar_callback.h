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
#ifndef LIVOX_ROS_DRIVER_VEHICLE_LIDAR_CALLBACK_H_
#define LIVOX_ROS_DRIVER_VEHICLE_LIDAR_CALLBACK_H_	

#include "../lds.h"
#include "../lds_lidar.h"
#include "../comm/comm.h"

#include "livox_def_vehicle.h"
#include "livox_sdk_vehicle.h"

#include "livox_def_common.h"

#include "livox_def.h"
#include "livox_sdk.h"
#include "livox_sdk_common.h"

namespace livox_ros {

class VehicleLidarCallback {
 public:
   static void LidarInfoChangeCallback(VehicleDeviceInfo* info, void* client_data);
   // Set the point cloud resolution
   static void EnableVehicleLidarHighResolutionPointType(livox_vehicle_status status, uint8_t slot, LidarSyncControlResponse *response, void *client_data);
   static void DisableVehicleLidarHighResolutionPointType(livox_vehicle_status status, uint8_t slot, LidarSyncControlResponse *response, void *client_data);
   // Setting scan Mode
   static void SetVehicleLidarScanPattern(livox_vehicle_status status, uint8_t slot, LidarSyncControlResponse *response, void *client_data);
   // Set the blind area
   static void SetVehicleLidarBlindSpot(livox_vehicle_status status, uint8_t slot, LidarSyncControlResponse *response, void *client_data);
   // Set dual emit
   static void EnableVehicleLidarDualEmit(livox_vehicle_status status, uint8_t slot, LidarSyncControlResponse *response, void *client_data);


	 static void LidarWorkModeControl();
   static void LidarWorkModeControlCallback(livox_vehicle_status status, uint8_t slot, LidarSyncControlResponse *rsp, void* client_data);
 private:
   static LidarDevice* GetLidarDevice(const uint8_t slot, void* client_data);
};

}

#endif
