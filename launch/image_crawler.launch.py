#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

USERNAME = os.getlogin()

def generate_launch_description():
    # configurations
    is_debug = LaunchConfiguration('is_debug', default=False)
    is_label = LaunchConfiguration('is_label', default=False)
    color_topic = LaunchConfiguration('color_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/aligned_depth_to_color/image_raw')
    rate = LaunchConfiguration('rate', default='1.0')
    label_rate = LaunchConfiguration('label_rate', default='0.1')
    output_dir = LaunchConfiguration('output_dir', default='/home/' + USERNAME + '/clawed_image')

    return LaunchDescription([
        # declare arguments
        DeclareLaunchArgument('is_debug', default_value=is_debug),
        DeclareLaunchArgument('is_label', default_value=is_label),
        DeclareLaunchArgument('color_topic', default_value=color_topic),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic),
        DeclareLaunchArgument('rate', default_value=rate),
        DeclareLaunchArgument('output_dir', default_value=output_dir),
        DeclareLaunchArgument('label_rate', default_value=label_rate),
        # crawling node
        Node(
            package='rosbag_image_crawler',
            executable='image_crawler',
            name='image_crawler',
            output='screen',
            parameters=[
                {'is_debug': is_debug},
                {'is_label': is_label},
                {'color_topic': color_topic},
                {'depth_topic': depth_topic},
                {'rate': rate},
                {'output_dir': output_dir},
            ],
        ),
        # label node
        Node(
            condition=IfCondition(
                PythonExpression(
                    [LaunchConfiguration('is_label'), " == True"]
                )
            ),
            package='rosbag_image_crawler',
            executable='get_label',
            name='get_label',
            output='screen',
            parameters=[
                {'label_rate': label_rate},
            ],
        ),
    ])
