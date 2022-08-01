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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;

void RosShutdown();

inline void SignalHandler(int signum) {
  printf("livox ros driver will exit\r\n");
  RosShutdown();
  exit(signum);
}

#ifdef BUILDING_ROS1
void RosShutdown()
{
  ros::shutdown();
}

int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
  signal(SIGINT, SignalHandler);
  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    DRIVER_INFO(livox_node, "The SDK version[%d.%d.%d] is too low", _sdkversion.major,
             _sdkversion.minor, _sdkversion.patch);
    return 0;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  /** Lidar data distribute control and lidar data source set */
  Lddc *lddc = new Lddc(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  lddc->SetRosNode(&livox_node);

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.GetNode().getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.GetNode().getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);
    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path, bd_code_list))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else if (data_src == kSourceRawHub) {
    DRIVER_INFO(livox_node, "Data Source is hub.");

    std::string user_config_path;
    livox_node.GetNode().getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.GetNode().getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_hub));

    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      DRIVER_INFO(livox_node, "Init lds hub success!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds hub fail!");
    }
  } else {
    DRIVER_INFO(livox_node, "Data Source is lvx file.");

    std::string cmdline_file_path;
    livox_node.GetNode().getParam("cmdline_file_path", cmdline_file_path);

    DRIVER_INFO(livox_node, "cmdline file path:%s.\n", cmdline_file_path.c_str());
    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        DRIVER_ERROR(livox_node, "File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(publish_freq);
      int ret = read_lvx->Init(cmdline_file_path.c_str());
      if (!ret) {
        DRIVER_INFO(livox_node, "Init Ros1 lds lvx file success!");
      } else {
        DRIVER_ERROR(livox_node, "Init Ros2 lds lvx file fail!");
      }

      lddc->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc->CreateBagFile(rosbag_file_path);

      read_lvx->ReadLvxFile();
    } while (0);
  }

  ros::Time::init();
  while (ros::ok()) {
    lddc->DistributeLidarData();
  }

  return 0;
}

#elif defined BUILDING_ROS2
void RosShutdown()
{
  rclcpp::shutdown();
}
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);

  signal(SIGINT, SignalHandler);

  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    DRIVER_INFO(*this,
      "The SDK version[%d.%d.%d] is too low", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);
    rclcpp::shutdown();
    return;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path, bd_code_list))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
  } else if (data_src == kSourceRawHub) {
    DRIVER_INFO(*this, "Data Source is hub.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_hub));

    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      DRIVER_INFO(*this, "Init lds hub success!");
    } else {
      DRIVER_ERROR(*this, "Init lds hub fail!");
    }
  } else {
    DRIVER_ERROR(*this, "Data Source is lvx file.");
    std::string cmdline_file_path;
    this->get_parameter("cmdline_file_path", cmdline_file_path);

    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        DRIVER_INFO(*this, "File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(publish_freq);
      int ret = read_lvx->Init(cmdline_file_path.c_str());
      if (!ret) {
        DRIVER_INFO(*this, "Init lds lvx file success!");
      } else {
        DRIVER_ERROR(*this, "Init lds lvx file fail!");
      }

      lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc_ptr_->CreateBagFile(rosbag_file_path);
      read_lvx->ReadLvxFile();
    } while (0);
  }

  poll_thread_ = std::make_shared<std::thread>(&DriverNode::pollThread, this);
}

void DriverNode::pollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeLidarData();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}
}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2






















