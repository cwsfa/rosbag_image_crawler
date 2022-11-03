#!/usr/bin/env python3

import os
import getpass as gt

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

USERNAME = gt.getuser()

def generate_launch_description():

    color_topic = LaunchConfiguration('color_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/aligned_depth_to_color/image_raw')
    cycle = LaunchConfiguration('cycle', default='1.0')
    output_dir = LaunchConfiguration('output_dir', default='/home/' + USERNAME + '/clawed_image')

    return LaunchDescription([

        Node(
            package='rosbag_image_crawler',
            executable='image_crawler',
            name='image_crawler',
            output='screen',
            parameters=[
                {'color_topic': color_topic},
                {'depth_topic': depth_topic},
                {'cycle': cycle},
                {'output_dir': output_dir},
            ],
        ),
    ])
