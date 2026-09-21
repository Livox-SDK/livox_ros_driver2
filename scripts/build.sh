#!/bin/bash


colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE --packages-select livox_ros_driver2

