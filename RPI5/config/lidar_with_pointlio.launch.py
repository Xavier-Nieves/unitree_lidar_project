#!/usr/bin/env python3
"""
Launch file for Unitree LiDAR L1 with Point-LIO integration
This launch file starts both the LiDAR driver and Point-LIO SLAM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    unitree_lidar_dir = get_package_share_directory('unitree_lidar_sdk')

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'unitree_lidar_project/RPI5/config/lidar_params.yaml'
        ),
        description='Path to LiDAR configuration file'
    )

    pointlio_config_arg = DeclareLaunchArgument(
        'pointlio_config',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'unitree_lidar_project/RPI5/config/pointlio_params.yaml'
        ),
        description='Path to Point-LIO configuration file'
    )

    # Unitree LiDAR node (customize based on actual SDK)
    lidar_node = Node(
        package='unitree_lidar_sdk',
        executable='unitree_lidar_node',  # Adjust to actual executable name
        name='unitree_lidar_l1',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/cloud', '/cloud_registered'),
        ]
    )

    # Point-LIO node (if package exists)
    # Note: Adjust based on actual Point-LIO ROS2 implementation
    pointlio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',  # Adjust to actual executable name
        name='point_lio',
        output='screen',
        parameters=[LaunchConfiguration('pointlio_config')],
    )

    # RViz node (optional)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('unitree_lidar_sdk'),
        'rviz',
        'lidar_pointlio.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('use_rviz')
    )

    # Static transform from base_link to lidar_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link']
    )

    return LaunchDescription([
        use_rviz_arg,
        config_file_arg,
        pointlio_config_arg,
        lidar_node,
        pointlio_node,
        static_tf_node,
        rviz_node,
    ])
