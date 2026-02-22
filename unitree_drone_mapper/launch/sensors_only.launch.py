#!/usr/bin/env python3
"""Sensors-only launch file — just drivers for testing hardware.

No SLAM, no flight control. Use to verify LiDAR is working.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0")

    lidar_node = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_lidar_ros2_node",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "rotate_yaw_bias": 0.0,
            "range_scale": 0.001,
            "range_max": 50.0,
            "range_min": 0.0,
            "cloud_frame": "unilidar_lidar",
            "cloud_topic": "unilidar/cloud",
            "cloud_scan_num": 18,
            "imu_frame": "unilidar_imu",
            "imu_topic": "unilidar/imu",
        }],
    )

    return LaunchDescription([
        port_arg,
        lidar_node,
    ])
