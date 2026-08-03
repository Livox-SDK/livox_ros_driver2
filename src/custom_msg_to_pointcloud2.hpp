#pragma once

#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <memory>

namespace livox_ros {

class CustomMsgToPointCloud2Node : public rclcpp::Node {
 public:
  explicit CustomMsgToPointCloud2Node(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~CustomMsgToPointCloud2Node();

 private:
  void CustomMsgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  
  sensor_msgs::msg::PointCloud2 ConvertCustomMsgToPointCloud2(
      const livox_ros_driver2::msg::CustomMsg& custom_msg);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_msg_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
};

}  // namespace livox_ros
