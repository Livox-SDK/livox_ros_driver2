How to build?

1. Source your ROS installation.

Examples:
  `source /opt/ros/noetic/setup.bash` for ROS1, if you installed Noetic, or
  `source /opt/ros/foxy/setup.bash` for ROS2, if you installed Foxy.


2. Execute the 'build.sh' located in the same directory with this text file:
  `./build.sh [ROS_VERSION]`
  - [ROS_VERSION] parameters:
    - ROS1: build the ROS1 Driver;  
    - ROS2: build the ROS2 Driver.

Examples:
  `./build.sh ROS1` to build ROS1 Driver.

