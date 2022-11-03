import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # get the launch directory
    bagimg_dir  = os.path.join(get_package_share_directory('rosimgset'))
    img_dir = LaunchConfiguration('img_dir',default=os.path.join(bagimg_dir, 'config'))
    launch_dir  = os.path.join(get_package_share_directory('growth_meter_application'), 'launch')

    return LaunchDescription([
        Node(
            package='rosimgset',
            namespace='rosimgset',
            executable='rosbag_to_imgset.py',
            name='rosimgset',
            output="screen",
            emulate_tty=True,
            parameters=[
                {'num1': 1},
                {'num2': 1},
                {'num3': 1}]
        ),
        launch.actions.ExecuteProcess(
            cmd=[ bagimg_dir + 'ros2', 'bag', 'play', '20200923_sosori_camera_only/'],
            output='screen'
        )
    ])
