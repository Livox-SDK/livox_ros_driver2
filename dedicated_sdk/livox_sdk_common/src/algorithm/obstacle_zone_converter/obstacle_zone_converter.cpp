#include "obstacle_zone_converter.h"
#include <algorithm>

namespace livox {

namespace common {

static const double kPi = 3.1415927;
Eigen::Matrix4d ParamToMat(float roll, float pitch, float yaw, int x, int y, int z) {
  Eigen::Matrix4d result;
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw * kPi / 180, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch * kPi / 180, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll * kPi / 180, Eigen::Vector3d::UnitX());
  result << rotation(0, 0), rotation(0, 1), rotation(0, 2), x,
    rotation(1, 0), rotation(1, 1), rotation(1, 2), y,
    rotation(2, 0), rotation(2, 1), rotation(2, 2), z,
    0, 0, 0, 1;
  return result.inverse();
}

ObstacleZoneParamConverter &obstacle_zone_converter() {
  static ObstacleZoneParamConverter converter;
  return converter;
}

ObstacleZoneParamConverter::ObstacleZoneParamConverter() {
  point_table_.resize(3);
  for (uint32_t i = 0; i < point_table_.size(); i++) {
    point_table_[i].resize(kPointTableElementNum);
  }
  for (auto &point_table : point_table_) {
    std::fill(point_table.begin(), point_table.end(), 0);
  }
  InitAtanList();
}

ObstacleZoneParamConverter::~ObstacleZoneParamConverter() {
  Clear();
}

void ObstacleZoneParamConverter::SetCubeArray(std::vector<ObstacleZoneCubeParam>& cube_array) {
  array_cube_ = std::move(cube_array);
}

void ObstacleZoneParamConverter::SetCylinderArray(std::vector<ObstacleZoneCylinderParam>& cylinder_array) {
  array_cylinder_ = std::move(cylinder_array);
}

void ObstacleZoneParamConverter::Convert(std::string file_name, ObstacleZoneConvertProgressCallback cb, void* client_data) {
  client_data_ = client_data;
  cb_ = cb;
  converter_thread_ = std::make_shared<std::thread>([this, file_name](){
    GetExtrinsic();
    if (1 == CompareMethode()) {
      TransferList1();
    } else {
      TransferList2();
    }
    auto fp = std::fopen(file_name.c_str(), "wb");
    for (const auto& point_table : point_table_) {
      for (const auto& point : point_table) {
        std::fwrite(&point, sizeof(point), 1, fp);
      }
    }
    std::fclose(fp);
    if (cb_) {
      cb_(100, client_data_);
    }
  });
}

void ObstacleZoneParamConverter::InitAtanList() {
  for (int i = 1; i < 2000; ++i) {
    list_tan_[i - 1] = -1 + i / 1000.0f;
    list_angle_[i - 1] = atan(list_tan_[i - 1]) * 180 / kPi;
  }
}

float ObstacleZoneParamConverter::GetAtan(double a) {
  int index;
  if (a < 1 && a > -1) {
    index = a * 1000 + 999;
    if (index < 0) {
      return list_angle_[0];
    }
    else if (index > 1998) {
      return list_angle_[1998];
    }
    else {
      return list_angle_[index];
    }
  }
  else if (a >= 1) {
    float b = (a - 1) / (a + 1); // to get the value of tan of (a - 45)
    index = b * 1000 + 999;
    if (index < 0) {
      return (list_angle_[0] + 45);
    }
    else if (index > 1998) {
      return (list_angle_[1998] + 45);
    }
    else {
      return (list_angle_[index] + 45);
    }
  }
  else {
    float b = -(-a - 1) / (-a + 1);  // to get the value of tan of (a + 45)
    index = b * 1000 + 999;
    if (index < 0) {
      return (list_angle_[0] - 45);
    }
    else if (index > 1998) {
      return (list_angle_[1998] - 45);
    }
    else {
      return (list_angle_[index] - 45);
    }
  }
}

void ObstacleZoneParamConverter::GetExtrinsic() {
  Eigen::Matrix4d mat;
  for (size_t i = 0; i < array_cube_.size(); ++i) {
    mat = ParamToMat(array_cube_[i].roll, array_cube_[i].pitch, array_cube_[i].yaw,
      array_cube_[i].x, array_cube_[i].y, array_cube_[i].z);
    vector_ext_cube_.push_back(mat);
  }
  for (size_t i = 0; i < array_cylinder_.size(); ++i) {
    mat = ParamToMat(array_cylinder_[i].roll, array_cylinder_[i].pitch, array_cylinder_[i].yaw,
      array_cylinder_[i].x, array_cylinder_[i].y, array_cylinder_[i].z);
    vector_ext_cylinder_.push_back(mat);
  }
}

void ObstacleZoneParamConverter::Clear() {
  for (auto &point_table : point_table_) {
    std::fill(point_table.begin(), point_table.end(), 0);
  }
  vector_ext_cube_.clear();
  vector_ext_cylinder_.clear();
  array_cube_.clear();
  array_cylinder_.clear();
  if (converter_thread_ && converter_thread_->joinable()) {
    converter_thread_->detach();
  }
}

bool ObstacleZoneParamConverter::SetTableValue(int index, int value, int work_mode) {
  if (index > kPointTableSize) {
    return false;
  }

  if (work_mode != 0 && work_mode != 1 && work_mode != 2) {
    return false;
  }

  int bit_num = index * kLevelBitsNum;
  int n = bit_num / kBitEachElement;
  int m = bit_num - n * kBitEachElement;

  point_table_[work_mode][n] &= ~(kLevelMask << m);
  point_table_[work_mode][n] |= (value & kLevelMask) << m;
  return true;
}

int ObstacleZoneParamConverter::GetTableValue(int index, int work_mode) {
  if (index > kPointTableSize) {
    return -1;
  }
  if (work_mode != 0 && work_mode != 1 && work_mode != 2) {
    return -1;
  }

  int bit_num = (index * kLevelBitsNum);
  int n = bit_num / (kBitEachElement);
  int m = bit_num - n * kBitEachElement;

  return (point_table_[work_mode][n] >> m) & kLevelMask;
}

float ObstacleZoneParamConverter::GetPointAngle(int x, int y) {
  float angle;
  if (x == 0) {
    if (y == 0) {
      angle = 0;
    }
    else if (y > 0) {
      angle = -90;  // right hand coordinate
    }
    else {
      angle = 90;
    }
  }
  else if (x > 0) {
    double temp = 1.0f * y / x;
    angle = -GetAtan(temp);  // angle is negative on the left and positive on the right
  }
  else {
    double temp = 1.0f * y / x;
    angle = GetAtan(temp);
    if (angle < 0) {
      angle = -(angle + 180);
    }
    else {
      angle = 180 - angle;
    }
  }
  return angle;
}

bool ObstacleZoneParamConverter::JudgeCube(int x, int y, int z,  ObstacleZoneCubeParam cube, int index) {
  Eigen::Vector4d point_raw, point_transfer;
  point_raw << x, y, z, 1;
  point_transfer = vector_ext_cube_[index] * point_raw;
  double x_ = point_transfer(0);
  double y_ = point_transfer(1);
  double z_ = point_transfer(2);
  if (x < length1 && x > length2 && y < width1
    && y > width2 && z < height1 && z > height2) {
    if (z_ > (-cube.h / 2.0f - error) && z_ < (cube.h / 2.0f + error)
      && x_ >(-cube.l / 2.0f - error) && x_ < (cube.l / 2.0f + error)
      && y_ >(-cube.w / 2.0f - error) && y_ < (cube.w / 2.0f + error)) {
      return true;
    }
  }
  return false;
}

bool ObstacleZoneParamConverter::JudgeCylinder(int x, int y, int z, ObstacleZoneCylinderParam cylinder, int index) {
  Eigen::Vector4d point_raw, point_transfer;
  point_raw << x, y, z, 1;
  point_transfer = vector_ext_cylinder_[index] * point_raw;
  double x_ = point_transfer(0);
  double y_ = point_transfer(1);
  double z_ = point_transfer(2);

  if (x < length1 && x > length2 && y < width1
    && y > width2 && z < height1 && z > height2) {
    if (z_ > (-cylinder.h / 2.0f - error) && z_ < (cylinder.h / 2.0f + error)) {
      double distance = sqrt(x_ * x_ + y_ * y_);
      if (distance > (cylinder.r1 - error) && distance < (cylinder.r2 + error)) {
        if (360 == cylinder.theta) {
          return true;
        }
        float angle = GetPointAngle(x_, y_);
        if (angle >= (-cylinder.theta / 2.0f) && angle <= (cylinder.theta / 2.0f)) {
          // here we add 5 degrees to avoid the difference between two directions
          return true;
        }
      }
    }
  }
  return false;
}

int ObstacleZoneParamConverter::GetIndex(int length, int width, int height) {
  return (length * width_num * height_num + width * height_num + height);
}

int ObstacleZoneParamConverter::TransferList1() {
  int l, w, h;
  size_t i;
  int count = 0;
  int all = (height_num + 1) * (length_num + 1) * (width_num + 1);
  int percent = 0;

  for (h = 0; h <= height_num; ++h) {
    for (l = 0; l <= length_num; ++l) {
      for (w = 0; w <= width_num; ++w) {
        ++count;
        int current = count * 100 / all;
        if (current != percent && percent > 1) {
          percent = current;
          if (cb_) {
            cb_(percent - 1, client_data_);
          }
        }
        int x, y, z;
        // x->z; y->x; z->y
        z = h * side_length + half_side_length + height2;
        x = l * side_length + half_side_length + length2;
        y = w * side_length + half_side_length + width2;

        if (z > a3) {
          if ((z - a3)*(z - a3) > (x * x + y * y)) {
            continue;
          }
        }
        else if (z < -a3) {
          if (25 * (z + a3)*(z + a3) > (x * x + y * y)) {
            continue;
          }
        }

        for (i = 0; i < array_cube_.size(); ++i) {
          if (array_cube_[i].priority == 1) {
            if (JudgeCube(x, y, z, array_cube_[i], i)) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, 1, array_cube_[i].mode)) {
                return -1;
              }
            }
          }
        }

        for (i = 0; i < array_cylinder_.size(); ++i) {
          if (array_cylinder_[i].priority == 1) {
            if (JudgeCylinder(x, y, z, array_cylinder_[i], i)) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, 1, array_cylinder_[i].mode)) {
                return -1;
              }
            }
          }
        }

        for (i = 0; i < array_cube_.size(); ++i) {
          if (array_cube_[i].priority == 2) {
            if (JudgeCube(x, y, z, array_cube_[i], i)) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, 2, array_cube_[i].mode)) {
                return -1;
              }
            }
          }
        }

        for (i = 0; i < array_cylinder_.size(); ++i) {
          if (array_cylinder_[i].priority == 2) {
            if (JudgeCylinder(x, y, z, array_cylinder_[i], i)) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, 2, array_cylinder_[i].mode)) {
                return -1;
              }
            }
          }
        }

        for (i = 0; i < array_cube_.size(); ++i) {
          if (array_cube_[i].priority == 3) {
            if (JudgeCube(x, y, z, array_cube_[i], i)) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, 0, array_cube_[i].mode)) {
                return -1;
              }
            }
          }
        }

        for (i = 0; i < array_cylinder_.size(); ++i) {
          if (array_cylinder_[i].priority == 3) {
            if (JudgeCylinder(x, y, z, array_cylinder_[i], i)) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, 0, array_cylinder_[i].mode)) {
                return -1;
              }
            }
          }
        }
      }
    }
  }
  return 0;
}

int ObstacleZoneParamConverter::TransferList2() {

  std::vector<std::vector<int>> vec_temp1;
  std::vector<std::vector<int>> vec_temp2;
  std::vector<std::vector<int>> vec_temp3;
  if (GetRawSpace(vec_temp1, vec_temp2, vec_temp3) != 0) {
    //cout << "something wrong" << endl;
    //exit(1);
    return -1;
  }

  for (auto one : vec_temp1) {
    for (int l = one[0]; l <= one[3]; ++l) {
      for (int w = one[1]; w <= one[4]; ++w) {
        for (int h = one[2]; h <= one[5]; ++h) {
          int x, y, z;
          z = h * side_length + half_side_length + height2;
          x = l * side_length + half_side_length + length2;
          y = w * side_length + half_side_length + width2;

          if (one[7] == 0) {
            if (JudgeCube(x, y, z, array_cube_[one[6]], one[6]) == 1) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, one[8], array_cube_[one[6]].mode)) {
                return -1;
              }
            }
          } else {
            if (JudgeCylinder(x, y, z, array_cylinder_[one[6]], one[6]) == 1) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, one[8], array_cylinder_[one[6]].mode)) {
                return -1;
              }
            }
          }
        }
      }
    }
  }

  //Todo jerry.lin process callback hardcode
  if (cb_) {
    cb_(33, client_data_);
  }

  for (auto one : vec_temp2) {
    for (int l = one[0]; l <= one[3]; ++l) {
      for (int w = one[1]; w <= one[4]; ++w) {
        for (int h = one[2]; h <= one[5]; ++h) {
          int x, y, z;
          z = h * side_length + half_side_length + height2;
          x = l * side_length + half_side_length + length2;
          y = w * side_length + half_side_length + width2;

          if (one[7] == 0) {
            if (JudgeCube(x, y, z, array_cube_[one[6]], one[6]) == 1) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, one[8], array_cube_[one[6]].mode)) {
                return -1;
              }
            }
          } else {
            if (JudgeCylinder(x, y, z, array_cylinder_[one[6]], one[6]) == 1) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, one[8], array_cylinder_[one[6]].mode)) {
                return -1;
              }
            }
          }
        }
      }
    }
  }

  if (cb_) {
    cb_(66, client_data_);
  }

  for (auto one : vec_temp3) {
    for (int l = one[0]; l <= one[3]; ++l) {
      for (int w = one[1]; w <= one[4]; ++w) {
        for (int h = one[2]; h <= one[5]; ++h) {
          int x, y, z;
          z = h * side_length + half_side_length + height2;
          x = l * side_length + half_side_length + length2;
          y = w * side_length + half_side_length + width2;

          if (one[7] == 0) {
            if (JudgeCube(x, y, z, array_cube_[one[6]], one[6]) == 1) {
              int index = GetIndex(l, w, h);
              if (!SetTableValue(index, one[8], array_cube_[one[6]].mode)) {
                return -1;
              }
            }
          } else {
            if (JudgeCylinder(x, y, z, array_cylinder_[one[6]], one[6]) == 1) {
              int index = GetIndex(l, w, h);
              if(!SetTableValue(index, one[8], array_cylinder_[one[6]].mode)) {
                return -1;
              }
            }
          }
        }
      }
    }
  }

  if (cb_) {
    cb_(99, client_data_);
  }

  return 0;
}

int ObstacleZoneParamConverter::GetRawSpace(std::vector<std::vector<int>> &raw_space1,
  std::vector<std::vector<int>> &raw_space2,
  std::vector<std::vector<int>> &raw_space3) {
  raw_space1.clear();
  raw_space2.clear();
  raw_space3.clear();
  //cout << "Begin to generate the raw space" << endl;
  for (size_t i = 0; i < array_cube_.size(); ++i) {
    if (array_cube_[i].priority == 1) {
      std::vector<int> space(9);
      float ll = sqrt(array_cube_[i].l * array_cube_[i].l +
        array_cube_[i].w * array_cube_[i].w +
        array_cube_[i].h * array_cube_[i].h);
      int L = ceil(ll / 2) + 3 * error;
      //cout << "L is : " << L << endl;
      space[0] = (array_cube_[i].x - L > length2) ? (array_cube_[i].x - L - length2) / side_length : 0;
      space[1] = (array_cube_[i].y - L > width2) ? (array_cube_[i].y - L - width2) / side_length : 0;
      space[2] = (array_cube_[i].z - L > height2) ? (array_cube_[i].z - L - height2) / side_length : 0;
      space[3] = (array_cube_[i].x + L < length1) ? (array_cube_[i].x + L - length2) / side_length : length_num;
      space[4] = (array_cube_[i].y + L < width1) ? (array_cube_[i].y + L - width2) / side_length : width_num;
      space[5] = (array_cube_[i].z + L < height1) ? (array_cube_[i].z + L - height2) / side_length : height_num;
      space[6] = i;  // to note this space is cube or cylinder
      space[7] = 0;
      space[8] = 1;
      raw_space1.push_back(space);
    }
  }

  for (size_t i = 0; i < array_cylinder_.size(); ++i) {
    if (array_cylinder_[i].priority == 1) {
      std::vector<int> space(9);
      float ll = sqrt(4 * array_cylinder_[i].r2 * array_cylinder_[i].r2 +
        4 * array_cylinder_[i].r2 * array_cylinder_[i].r2 +
        array_cylinder_[i].h  * array_cylinder_[i].h);
      int L = ceil(ll / 2) + 3 * error;
      //cout << "L is : " << L << endl;
      space[0] = (array_cylinder_[i].x - L > length2) ? (array_cylinder_[i].x - L - length2) / side_length : 0;
      space[1] = (array_cylinder_[i].y - L > width2) ? (array_cylinder_[i].y - L - width2) / side_length : 0;
      space[2] = (array_cylinder_[i].z - L > height2) ? (array_cylinder_[i].z - L - height2) / side_length : 0;
      space[3] = (array_cylinder_[i].x + L < length1) ? (array_cylinder_[i].x + L - length2) / side_length : length_num;
      space[4] = (array_cylinder_[i].y + L < width1) ? (array_cylinder_[i].y + L - width2) / side_length : width_num;
      space[5] = (array_cylinder_[i].z + L < height1) ? (array_cylinder_[i].z + L - height2) / side_length : height_num;
      space[6] = i;
      space[7] = 1;
      space[8] = 1;
      raw_space1.push_back(space);
    }
  }

  for (size_t i = 0; i < array_cube_.size(); ++i) {
    if (array_cube_[i].priority == 2) {
      std::vector<int> space(9);
      float ll = sqrt(array_cube_[i].l * array_cube_[i].l +
        array_cube_[i].w * array_cube_[i].w +
        array_cube_[i].h * array_cube_[i].h);
      int L = ceil(ll / 2) + 3 * error;
      space[0] = (array_cube_[i].x - L > length2) ? (array_cube_[i].x - L - length2) / side_length : 0;
      space[1] = (array_cube_[i].y - L > width2) ? (array_cube_[i].y - L - width2) / side_length : 0;
      space[2] = (array_cube_[i].z - L > height2) ? (array_cube_[i].z - L - height2) / side_length : 0;
      space[3] = (array_cube_[i].x + L < length1) ? (array_cube_[i].x + L - length2) / side_length : length_num;
      space[4] = (array_cube_[i].y + L < width1) ? (array_cube_[i].y + L - width2) / side_length : width_num;
      space[5] = (array_cube_[i].z + L < height1) ? (array_cube_[i].z + L - height2) / side_length : height_num;
      space[6] = i;
      space[7] = 0;
      space[8] = 2;
      raw_space2.push_back(space);
    }
  }

  for (size_t i = 0; i < array_cylinder_.size(); ++i) {
    if (array_cylinder_[i].priority == 2) {
      std::vector<int> space(9);
      float ll = sqrt(4 * array_cylinder_[i].r2*array_cylinder_[i].r2 +
        4 * array_cylinder_[i].r2 * array_cylinder_[i].r2 +
        array_cylinder_[i].h * array_cylinder_[i].h);
      int L = ceil(ll / 2) + 3 * error;
      space[0] = (array_cylinder_[i].x - L > length2) ? (array_cylinder_[i].x - L - length2) / side_length : 0;
      space[1] = (array_cylinder_[i].y - L > width2) ? (array_cylinder_[i].y - L - width2) / side_length : 0;
      space[2] = (array_cylinder_[i].z - L > height2) ? (array_cylinder_[i].z - L - height2) / side_length : 0;
      space[3] = (array_cylinder_[i].x + L < length1) ? (array_cylinder_[i].x + L - length2) / side_length : length_num;
      space[4] = (array_cylinder_[i].y + L < width1) ? (array_cylinder_[i].y + L - width2) / side_length : width_num;
      space[5] = (array_cylinder_[i].z + L < height1) ? (array_cylinder_[i].z + L - height2) / side_length : height_num;
      space[6] = i;
      space[7] = 1;
      space[8] = 2;
      raw_space2.push_back(space);
    }
  }

  for (size_t i = 0; i < array_cube_.size(); ++i) {
    if (array_cube_[i].priority == 3) {
      std::vector<int> space(9);
      float ll = sqrt(array_cube_[i].l*array_cube_[i].l + array_cube_[i].w*array_cube_[i].w + array_cube_[i].h*array_cube_[i].h);
      int L = ceil(ll / 2) + 3 * error;
      space[0] = (array_cube_[i].x - L > length2) ? (array_cube_[i].x - L - length2) / side_length : 0;
      space[1] = (array_cube_[i].y - L > width2) ? (array_cube_[i].y - L - width2) / side_length : 0;
      space[2] = (array_cube_[i].z - L > height2) ? (array_cube_[i].z - L - height2) / side_length : 0;
      space[3] = (array_cube_[i].x + L < length1) ? (array_cube_[i].x + L - length2) / side_length : length_num;
      space[4] = (array_cube_[i].y + L < width1) ? (array_cube_[i].y + L - width2) / side_length : width_num;
      space[5] = (array_cube_[i].z + L < height1) ? (array_cube_[i].z + L - height2) / side_length : height_num;
      space[6] = i;
      space[7] = 0;
      space[8] = 0;
      raw_space3.push_back(space);
    }
  }

  for (size_t i = 0; i < array_cylinder_.size(); ++i) {
    if (array_cylinder_[i].priority == 3) {
      std::vector<int> space(9);
      float ll = sqrt(4 * array_cylinder_[i].r2*array_cylinder_[i].r2 + 4 * array_cylinder_[i].r2*array_cylinder_[i].r2 + array_cylinder_[i].h*array_cylinder_[i].h);
      int L = ceil(ll / 2) + 3 * error;
      space[0] = (array_cylinder_[i].x - L > length2) ? (array_cylinder_[i].x - L - length2) / side_length : 0;
      space[1] = (array_cylinder_[i].y - L > width2) ? (array_cylinder_[i].y - L - width2) / side_length : 0;
      space[2] = (array_cylinder_[i].z - L > height2) ? (array_cylinder_[i].z - L - height2) / side_length : 0;
      space[3] = (array_cylinder_[i].x + L < length1) ? (array_cylinder_[i].x + L - length2) / side_length : length_num;
      space[4] = (array_cylinder_[i].y + L < width1) ? (array_cylinder_[i].y + L - width2) / side_length : width_num;
      space[5] = (array_cylinder_[i].z + L < height1) ? (array_cylinder_[i].z + L - height2) / side_length : height_num;
      space[6] = i;
      space[7] = 1;
      space[8] = 0;
      raw_space3.push_back(space);
    }
  }
  return 0;
}


int ObstacleZoneParamConverter::CompareMethode() {
  float overall_space = (length1 / 100 - length2 / 100) *
    (width1 / 100 - width2 / 100) *
    (height1 / 100 - height2 / 100);
  float select_space = 0;
  for (size_t i = 0; i < array_cube_.size(); ++i) {
    select_space += pow((array_cube_[i].l * array_cube_[i].l / 10000 +
      array_cube_[i].w * array_cube_[i].w / 10000 +
      array_cube_[i].h * array_cube_[i].h / 10000), 1.5);
  }
  for (size_t i = 0; i < array_cylinder_.size(); ++i) {
    select_space += pow((array_cylinder_[i].r2 * array_cylinder_[i].r2 / 10000 +
      array_cylinder_[i].r2 * array_cylinder_[i].r2 / 10000 +
      array_cylinder_[i].h * array_cylinder_[i].h / 10000), 1.5);
  }

  if (select_space > overall_space) {
    return 1;
  }
  return 2;
}

int ObstacleZoneParamConverter::XyzToIndex(int x, int y, int z) {
  int length_index = length_num * (x - length2) / length;
  int width_index = width_num * (y - width2) / width;
  int height_index = height_num * (z - height2) / height;
  int index = GetIndex(length_index, width_index, height_index);
  return index;
}

int ObstacleZoneParamConverter::TestPoint(int x, int y, int z) {
  int index = XyzToIndex(x, y, z);
  return GetTableValue(index, 1);
}

}
}
