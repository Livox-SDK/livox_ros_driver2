import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    livox_ros_driver2_share_dir = get_package_share_directory('livox_ros_driver2')

    container = ComposableNodeContainer(
        name='livox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='livox_ros_driver2',
                plugin='livox_ros::DriverNode',
                name='livox_lidar_publisher',
                parameters=[
                    os.path.join(livox_ros_driver2_share_dir, 'config', 'livox_lidar_config.json'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='livox_ros_driver2',
                plugin='livox_ros::CustomMsgToPointCloud2Node',
                name='custom_msg_to_pointcloud2',
                parameters=[
                    {'input_topic': '/livox/lidar'},
                    {'output_topic': '/livox/cloud'},
                    {'frame_id': 'livox_frame'},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
