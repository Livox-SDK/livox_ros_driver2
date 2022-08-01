# Livox ROS Driver 2

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox.

#### **Notes**
Livox ROS Driver 2 of current version support only ROS1 (noetic recommended), and it can be run with ROS2 in the near future.

## 1. Prepare to install Livox ROS Driver 2
### 1.1 Prerequisites
* OS: Ubuntu 18.04/20.04
* ROS: Noetic Ninjemys (ROS2 would be supported in the near future)
* SDK: Livox SDK 2

### 1.2 Install ROS
For ROS Noetic installation, please refer to:  
[ROS Noetic installation instructions](https://wiki.ros.org/noetic/Installation)  

Desktop-Full installation is recommend.


### 1.3 Install Livox SDK 2
For Livox SDK 2 installation, please refer to:  
[Intallation of Livox SDK 2](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md#21-installation)


## 2. Build & run Livox ROS Driver 2
1. Clone source code from GitHub:  
```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src
```
2. Follow these steps to build the Livox ROS Driver 2 (take Noetic as example):  
```shell
cd  ws_livox/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```
3. In the same terminal (and in the same directory) with the previous step, use the following commands to run Livox ROS Driver 2:  
```shell
source ../../devel/setup.sh
roslaunch livox_ros_driver2 [launch file]
```
in which,
* livox_ros_driver2: *the ROS package name of Livox ROS Driver 2;*
* [launch file]: *the ROS launch file in the 'launch_ROS1' folder, containing the launch info and config about the target LiDAR(s);*

A rviz launch example for HAP LiDAR would be:  
```shell
roslaunch livox_ros_driver2 livox_lidar_rviz_HAP.launch
```

## 3. LiDAR config
LiDAR Configurations (such as ip, port, data type... etc.) can be set via a json-style config file. The parameter naming *'user_config_path'* in launch files indicates such json file path.

Follow is a configuration example for HAP LiDAR(located in config_ROS1/HAP_config.json):
```json
{
	"lidar_summary_info" : {
		"lidar_type": 8   # protocol type index, please don't revise this value
	},
	"HAP": {
		"device_type" : "HAP",
		"lidar_ipaddr": "",
		"lidar_net_info" : {
			"cmd_data_port": 56000,     # command port
			"push_msg_port": 0,
			"point_data_port": 57000,
			"imu_data_port": 58000,
			"log_data_port": 59000
		},
		"host_net_info" : {
			"cmd_data_ip" : "192.168.1.5",   # host ip
			"cmd_data_port": 56000,
			"push_msg_ip": "",
			"push_msg_port": 0,
			"point_data_ip": "192.168.1.5",  # host ip
			"point_data_port": 57000,
			"imu_data_ip" : "192.168.1.5",   # host ip
			"imu_data_port": 58000,
			"log_data_ip" : "",
			"log_data_port": 59000
		}
	},
	"lidar_configs" : [
		{
			"ip" : "192.168.1.100",  # ip of the LiDAR you want to config
			"data_type" : 1,         # pcl_data_type
			"pattern_mode" : 0,      # pattern_mode
			"blind_spot_set" : 50,   # blind_spot_set
			"extrinsic_parameter" : {
				"roll": 0.0,
				"pitch": 0.0,
				"yaw": 0.0,
				"x": 0,
				"y": 0,
				"z": 0
			}
		}
	]
}

```

For more infomation about the HAP config, please refer to:  
[HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)


## 4. Supported LiDAR list
* HAP
* (more types are comming soon...)  


## 5. FAQ
### 5.1 launch with "livox_lidar_rviz_HAP.launch" but no point cloud display on the grid?
Please check the "Global Options - Fixed Frame" field in the RViz "Display" pannel. Set the field value to "livox_frame" and check the "PointCloud2" option in the pannel.




