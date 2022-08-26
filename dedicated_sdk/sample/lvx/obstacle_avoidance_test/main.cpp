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

#include "livox_sdk_common.h"
#include "livox_def_common.h"
#include <iostream>
#include <condition_variable>
#include <mutex>
#include <vector>


std::condition_variable cv;
std::mutex mtx;

void OnProgressCallback(uint8_t percent, void* client_data) {
  std::cout << "convert progress: " << (int)percent << std::endl;
  if (percent == 100) {
    cv.notify_one();
  }
}


int main(int argc, const char *argv[]) {
   std::vector<ObstacleZoneCubeParam> cube_array;
   ObstacleZoneCubeParam cube = {
    1500,
    0,
    0,
    3000,
    2000,
    2000,
    0,
    0,
    0,
    1,
    1
  };
  cube_array.push_back(cube);

  std::vector<ObstacleZoneCylinderParam> cylinder_array;
  ObstacleZoneCylinderParam cylinder = {
    0,
    0,
    500,
    1000,
    500,
    1000,
    180,
    0,
    0,
    0,
    1,
    1
  };
  cylinder_array.push_back(cylinder);


  //SetObstacleAvoidanceCubeParams(cube_array.data(), cube_array.size());
  SetObstacleAvoidanceCylinderParams(cylinder_array.data(), cylinder_array.size());
  AvoidanceObstacleZoneFileConvert("./area.txt", OnProgressCallback, nullptr);

 {
   std::unique_lock<std::mutex> lock(mtx);
   cv.wait(lock);
 }

  //result 1 pass
  std::cout <<  "Test Point 500, 0, 500 ,Result:" << AvoidanceObstacleTestPoint(500, 0, 500) << std::endl;
  //result 1 pass
  std::cout << "Test Point 1000, 0, 500 ,Result:" << AvoidanceObstacleTestPoint(1000, 0, 500) << std::endl;
  //result 1 pass
  std::cout << "Test Point 0, 800, 500 ,Result:" << AvoidanceObstacleTestPoint(0, 800, 500) << std::endl;
  //result 1 pass
  std::cout << "Test Point 0, -800, 500 ,Result:" << AvoidanceObstacleTestPoint(0, -800, 500) << std::endl;
  //result 0 pass
  std::cout << "Test Point 1500, 0, 500 ,Result:" << AvoidanceObstacleTestPoint(1500, 0, 500) << std::endl;
  //result 0 pass
  std::cout << "Test Point -500, 0, 500 ,Result:" << AvoidanceObstacleTestPoint(-500, 0, 500) << std::endl;
  //result 1 pass
  std::cout << "Test Point 800, 0, 1000 ,Result:" << AvoidanceObstacleTestPoint(800, 0, 1000) << std::endl;

  ResetAllObstacleAvoidanceParams();
  getchar();
  return 0;
}
