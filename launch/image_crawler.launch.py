#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

USERNAME = os.getlogin()

def generate_launch_description():
    # configurations
    is_debug = LaunchConfiguration('is_debug', default=True)
    color_topic = LaunchConfiguration('color_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/aligned_depth_to_color/image_raw')
    cycle = LaunchConfiguration('cycle', default='1.0')
    output_dir = LaunchConfiguration('output_dir', default='/home/' + USERNAME + '/clawed_image')

    return LaunchDescription([
        # declare arguments
        DeclareLaunchArgument('is_debug', default_value=is_debug),
        DeclareLaunchArgument('color_topic', default_value=color_topic),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic),
        DeclareLaunchArgument('cycle', default_value=cycle),
        DeclareLaunchArgument('output_dir', default_value=output_dir),
        # run node
        Node(
            package='rosbag_image_crawler',
            executable='image_crawler',
            name='image_crawler',
            output='screen',
            parameters=[
                {'is_debug': is_debug},
                {'color_topic': color_topic},
                {'depth_topic': depth_topic},
                {'cycle': cycle},
                {'output_dir': output_dir},
            ],
        ),
    ])
