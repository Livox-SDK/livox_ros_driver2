#ifndef LIVOX_OBSTACLE_ZONE_PARAM_CONVERTER_H_
#define LIVOX_OBSTACLE_ZONE_PARAM_CONVERTER_H_

#include <functional>
#include <memory>
#include <cstring>
#include <map>
#include <vector>
#include <thread>

#include <livox_def_common.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace livox {

namespace common {

class ObstacleZoneParamConverter {
public:
  using ObstacleZoneConvertProgressCallback = std::function<void(uint8_t progress, void* client_data)>;
public:
  ObstacleZoneParamConverter();
  ~ObstacleZoneParamConverter();
public:
  void SetCubeArray(std::vector<ObstacleZoneCubeParam>& cube_array);
  void SetCylinderArray(std::vector<ObstacleZoneCylinderParam>& cylinder_array);
  void Convert(std::string file_name, ObstacleZoneConvertProgressCallback cb, void* client_data);
  void Clear();

  int TestPoint(int x, int y, int z);
private:
  void InitAtanList();
  float GetAtan(double a);
  void GetExtrinsic();

  int TransferList1();
  int TransferList2();
  int CompareMethode();
  bool SetTableValue(int index, int value, int work_mode);
  int GetTableValue(int index, int work_mode);

  float GetPointAngle(int x, int y);
  bool JudgeCube(int x, int y, int z, ObstacleZoneCubeParam cube, int index);
  bool JudgeCylinder(int x, int y, int z, ObstacleZoneCylinderParam cylinder, int index);

  int GetIndex(int length, int width, int height);
  int XyzToIndex(int x, int y, int z);

  int GetRawSpace(std::vector<std::vector<int>> &raw_space1,
    std::vector<std::vector<int>> &raw_space2,
    std::vector<std::vector<int>> &raw_space3);

private:
  const int kPointTableSize = 38764000;
  const int kBitEachElement = 8;
  const int kLevelBitsNum = 2;
  const uint64_t kLevelMask = 0x00000003;
  const int kPointTableElementNum = kPointTableSize * kLevelBitsNum / kBitEachElement;

  const int side_length = 50;
  const int half_side_length = side_length / 2;
  const int a3 = 150;

  const int length1 = 10000;
  const int length2 = -10000;
  const int width1 = 10000;
  const int width2 = -10000;
  const int height1 = 10000;
  const int height2 = -1500;
  const int length = length1 - length2;
  const int width = width1 - width2;
  const int height = height1 - height2;

  const int length_num = length / side_length;
  const int width_num = width / side_length;
  const int height_num = height / side_length;
  const int error = 100;
private:
  float list_angle_[1999];  // list of angle value of atan
  float list_tan_[1999];  // list of 1999 values from -1 to 1

  std::vector<Eigen::Matrix4d> vector_ext_cube_;
  std::vector<Eigen::Matrix4d> vector_ext_cylinder_;
  std::vector<std::vector<uint8_t> > point_table_;
  std::vector<ObstacleZoneCubeParam> array_cube_;
  std::vector<ObstacleZoneCylinderParam> array_cylinder_;

  void* client_data_ = nullptr;
  ObstacleZoneConvertProgressCallback cb_;
  std::shared_ptr<std::thread> converter_thread_;
};

ObstacleZoneParamConverter &obstacle_zone_converter();
}

}  // namespace livox

#endif  // LIVOX_OBSTACLE_ZONE_PARAM_CONVERTER_H_