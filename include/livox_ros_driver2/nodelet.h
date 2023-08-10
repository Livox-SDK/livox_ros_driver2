/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#pragma once

#include <nodelet/nodelet.h>
#include "driver_node.h"

namespace livox_ros
{

class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet();
  ~Nodelet() final = default;
  void onInit() final;

private:
  std::shared_ptr<livox_ros::DriverNode> livox_node_;
  /*void cmdAckermannCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& command);
  void commandUpdateLoop(const double control_rate);

  ros::Subscriber sub_command_;
  ros::Publisher pub_command_;

  std::unique_ptr<AckermannToSimpleDrive> ackermann_to_simple_drive_;

  std::thread command_update_thread_;
  std::atomic_bool shutting_down_ {false};*/
};

}  // namespace livox_ros

