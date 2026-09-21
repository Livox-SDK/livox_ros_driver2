#include "custom_msg_to_pointcloud2.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace livox_ros {

CustomMsgToPointCloud2Node::CustomMsgToPointCloud2Node(const rclcpp::NodeOptions& node_options)
    : rclcpp::Node("custom_msg_to_pointcloud2", node_options),
      input_topic_("/livox/lidar"),
      output_topic_("/livox/cloud"),
      frame_id_("livox_frame") {
  
  // Get topic names from parameters
  this->declare_parameter<std::string>("input_topic", input_topic_);
  this->declare_parameter<std::string>("output_topic", output_topic_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();

  // Create subscriber and publisher
  custom_msg_subscriber_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_,
      10,
      std::bind(&CustomMsgToPointCloud2Node::CustomMsgCallback, this, std::placeholders::_1));

  pointcloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      10);

  RCLCPP_INFO(this->get_logger(), "CustomMsg to PointCloud2 converter node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
}

CustomMsgToPointCloud2Node::~CustomMsgToPointCloud2Node() {}

void CustomMsgToPointCloud2Node::CustomMsgCallback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  auto pointcloud2 = ConvertCustomMsgToPointCloud2(*msg);
  pointcloud2_publisher_->publish(pointcloud2);
}

sensor_msgs::msg::PointCloud2 CustomMsgToPointCloud2Node::ConvertCustomMsgToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg& custom_msg) {
  
  sensor_msgs::msg::PointCloud2 cloud;
  
  // Set header
  cloud.header.stamp = custom_msg.header.stamp;
  cloud.header.frame_id = frame_id_;
  
  // Get point count
  uint32_t point_count = custom_msg.point_num;
  if (point_count == 0 || custom_msg.points.empty()) {
    cloud.width = 0;
    cloud.height = 1;
    cloud.point_step = 0;
    cloud.row_step = 0;
    cloud.data.clear();
    return cloud;
  }

  // Define PointCloud2 fields: x, y, z (float32), intensity (uint8)
  cloud.fields.clear();
  cloud.fields.resize(5);
  
  // x field
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  
  // y field
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  
  // z field
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  
  // intensity field (reflectivity)
  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[3].count = 1;
  
  // tag field (livox tag)
  cloud.fields[4].name = "tag";
  cloud.fields[4].offset = 13;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[4].count = 1;
  
  // Set point info
  cloud.point_step = 16;  // 4 (x) + 4 (y) + 4 (z) + 1 (intensity) + 1 (tag) + 2 (padding for alignment)
  cloud.width = point_count;
  cloud.height = 1;  // Unorganized point cloud
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_bigendian = false;
  cloud.is_dense = false;  // May contain NaN or Inf values
  
  // Allocate data buffer
  cloud.data.resize(cloud.row_step);
  
  // Copy points
  for (uint32_t i = 0; i < point_count && i < static_cast<uint32_t>(custom_msg.points.size()); ++i) {
    const auto& point = custom_msg.points[i];
    uint8_t* point_data = cloud.data.data() + i * cloud.point_step;
    
    // Copy x, y, z
    float* pf = reinterpret_cast<float*>(point_data);
    pf[0] = point.x;
    pf[1] = point.y;
    pf[2] = point.z;
    
    // Copy intensity (reflectivity)
    uint8_t* pu8 = reinterpret_cast<uint8_t*>(point_data + 12);
    pu8[0] = point.reflectivity;
    pu8[1] = point.tag;
  }
  
  return cloud;
}

}  // namespace livox_ros

// Register as component
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::CustomMsgToPointCloud2Node)
