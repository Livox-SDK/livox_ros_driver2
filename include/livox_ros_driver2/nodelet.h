/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#pragma once

#include <memory>

#include <nodelet/nodelet.h>
#include "driver_node.h"

namespace livox_ros
{

class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet() = default;
  ~Nodelet() final = default;
  void onInit() final;

private:
  std::shared_ptr<livox_ros::DriverNode> livox_node_;
};

}  // namespace livox_ros
