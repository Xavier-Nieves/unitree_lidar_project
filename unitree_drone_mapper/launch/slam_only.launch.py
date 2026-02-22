#!/usr/bin/env python3
"""SLAM-only launch file — LiDAR + Point-LIO without flight control.

Use for handheld scanning or testing SLAM independently.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true")

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

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("point_lio"),
                "config", "unilidar_l1.yaml",
            ]),
            {
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "point_filter_num": 1,
                "filter_size_surf": 0.1,
                "filter_size_map": 0.1,
                "pcd_save.pcd_save_en": True,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("point_lio"),
            "rviz_cfg", "loam_livox.rviz",
        ])],
        condition=IfCondition(LaunchConfiguration("rviz")),
        prefix="nice",
    )

    return LaunchDescription([
        port_arg,
        rviz_arg,
        lidar_node,
        point_lio_node,
        rviz_node,
    ])
