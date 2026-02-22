#!/usr/bin/env python3
"""
Launch file for photogrammetry image capture

This launches:
1. USB camera driver (Insta360 X3)
2. Photogrammetry capture node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='Video device for Insta360 X3'
    )

    capture_interval_arg = DeclareLaunchArgument(
        'capture_interval', default_value='2.0',
        description='Time between captures (seconds)'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='./photogrammetry_captures',
        description='Output directory for images'
    )

    auto_capture_arg = DeclareLaunchArgument(
        'auto_capture', default_value='true',
        description='Enable automatic capture'
    )

    # USB Camera Node (Insta360 X3)
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='insta360_camera',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'camera_name': 'insta360_x3',
            'frame_id': 'camera_link',
            'image_width': 1920,
            'image_height': 1080,
            'pixel_format': 'mjpeg2rgb',  # Decode MJPEG to RGB
            'framerate': 30.0,
            'io_method': 'mmap',
        }],
        output='screen'
    )

    # Photogrammetry Capture Node
    capture_node = Node(
        package='insta360_photogrammetry',
        executable='photogrammetry_capture.py',
        name='photogrammetry_capture',
        parameters=[{
            'capture_interval': LaunchConfiguration('capture_interval'),
            'output_dir': LaunchConfiguration('output_dir'),
            'auto_capture': LaunchConfiguration('auto_capture'),
            'image_topic': '/insta360_x3/image_raw',
            'save_format': 'jpg',
            'jpeg_quality': 95,
        }],
        output='screen'
    )

    return LaunchDescription([
        video_device_arg,
        capture_interval_arg,
        output_dir_arg,
        auto_capture_arg,
        camera_node,
        capture_node,
    ])
