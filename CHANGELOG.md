# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2022-12-12

### Added

- support Mid-360 Lidar.
- support for Ubuntu 22.04 ROS2 humble.
- support multi-topic fuction, the suffix of the topic name corresponds to the ip address of each Lidar. 

### Changed

- remove the embedded SDK.
- Constraint: Livox ROS Driver 2 for ROS2 does not support message passing with PCL native data types.

### Fixed

- Fix IMU packet loss.
- Fix some conflicts with livox ros driver.
- Fixed HAP Lidar publishing PointCloud2 and CustomMsg format point clouds with no line number.
