## Launch que permite o passo de parâmetros além de executar o nó rviz2

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path_ad = os.path.join(cur_config_path, 'final_config.json')


def generate_launch_description():
    xfer_format = LaunchConfiguration('xfer_format')
    multi_topic = LaunchConfiguration('multi_topic')
    data_src= LaunchConfiguration('data_src')            
    publish_freq= LaunchConfiguration('publish_freq')
    output_type= LaunchConfiguration('output_type')
    frame_id= LaunchConfiguration('frame_id')
    lvx_file_path=LaunchConfiguration('lvx_file_path')
    user_config_path= LaunchConfiguration('user_config_path')
    cmdline_bd_code= LaunchConfiguration('cmdline_bd_code')
    
    xfer_format_launch_arg = DeclareLaunchArgument('xfer_format', default_value= '0', description='Transfer format (0-Pointcloud2, 1-customized)')
    multi_topic_launch_arg = DeclareLaunchArgument('multi_topic', default_value= '1', description='Multi-topic configuration (0-All LiDARs share same topic, 1-One LiDAR one topic)')
    data_src_launch_arg = DeclareLaunchArgument('data_src', default_value= '0', description='Data source configuration (0-lidar, others-invalid)')
    publish_freq_launch_arg = DeclareLaunchArgument('publish_freq', default_value='10.0', description='Publish frequency')
    output_data_type_launch_arg = DeclareLaunchArgument('output_type', default_value='0', description='Output type')
    frame_id_launch_arg = DeclareLaunchArgument('frame_id', default_value='livox_frame', description='Frame ID')
    lvx_file_path_launch_arg = DeclareLaunchArgument('lvx_file_path', default_value='/home/livox/livox_test.lvx', description='Path to LVX file')
    cmdline_input_bd_code_launch_arg = DeclareLaunchArgument('cmdline_bd_code', default_value='livox0000000001', description='Command line board code')
    user_config_path_launch_arg = DeclareLaunchArgument('user_config_path', default_value=user_config_path_ad, description='Path to the user config file')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters= [
            {"xfer_format": xfer_format},
            {"multi_topic": multi_topic},
            {"data_src": data_src},
            {"publish_freq": publish_freq},
            {"output_data_type": output_type},
            {"frame_id": frame_id},
            {"lvx_file_path": lvx_file_path},
            {"user_config_path": user_config_path},
            {"cmdline_input_bd_code": cmdline_bd_code}    
        ]
        )
    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )
    return LaunchDescription([
        xfer_format_launch_arg,
        multi_topic_launch_arg,
        data_src_launch_arg,
        publish_freq_launch_arg,
        output_data_type_launch_arg,
        frame_id_launch_arg,
        lvx_file_path_launch_arg,
        user_config_path_launch_arg,
        cmdline_input_bd_code_launch_arg,
        livox_driver,
        livox_rviz,
])