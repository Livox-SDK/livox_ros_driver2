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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <pluginlib/class_list_macros.h>

#include "livox_ros_driver2/nodelet.h"
#include "livox_ros_driver2/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

namespace livox_ros
{

livox_ros::Nodelet::Nodelet()
{

}


void
livox_ros::Nodelet::onInit()
{
  const ros::NodeHandle nh = getPrivateNodeHandle();
  livox_node_ = std::make_shared<livox_ros::DriverNode>(nh);
  //livox_ros::DriverNode livox_node;//(nh);
  //livox_node_ = livox_ros::DriverNode{nh};
  DRIVER_INFO(livox_node_, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);
  DRIVER_INFO(livox_node_, "Livox Node NameSpace: %s ", nh.getNamespace().c_str()); 

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;
  bool dust_filter = false;
  nh.getParam("xfer_format", xfer_format);
  nh.getParam("multi_topic", multi_topic);
  nh.getParam("data_src", data_src);
  nh.getParam("publish_freq", publish_freq);
  nh.getParam("output_data_type", output_type);
  nh.getParam("frame_id", frame_id);
  DRIVER_INFO(livox_node_, "frame_id: %s", frame_id.c_str());
  DRIVER_INFO(nh, "NameSpace: %s ", nh.getNamespace().c_str()); 
  nh.getParam("enable_lidar_bag", lidar_bag);
  nh.getParam("enable_imu_bag", imu_bag);
  nh.getParam("enable_dust_filter", dust_filter);
  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node_->future_ = livox_node_->exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node_->lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag, dust_filter);
  livox_node_->lddc_ptr_->SetRosNode(livox_node_.get());

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node_, "Data Source is raw lidar.");

    std::string user_config_path;
    nh.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node_, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);

    Lds* lds = static_cast<Lds *>(read_lidar);
    livox_node_->lddc_ptr_->RegisterLds(lds);
    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node_, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node_, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node_, "Invalid data src (%d), please check the launch file", data_src);
  }

  std::vector<int8_t> indices = livox_node_->lddc_ptr_->lds_->cache_index_.GetIndices();
  DRIVER_INFO(livox_node_, "Number of sensors %d", static_cast<int>(indices.size()));
  for (auto index : indices)
  {
    livox_node_->pointclouddata_poll_threads_.emplace_back(std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, livox_node_.get(), static_cast<unsigned int>(index)));
  }

  livox_node_->imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, livox_node_.get());
  //while (ros::ok()) { usleep(10000); }

}

void DriverNode::PointCloudDataPollThread(unsigned int index)
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData(index);
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}
  
}
PLUGINLIB_EXPORT_CLASS(livox_ros::Nodelet, nodelet::Nodelet)
