# Rosbag Image Crawler (for ROS2)

## Package Information

- **Package Name**: rosbag_image_crawler
- **Node Name**: image_crawler.py
- **Subscriber Topics**: /camera/color/image_raw & /camera/depth/image_rect_raw
- **Note:** This node Run your Ros2 record file first
## Parameters

- 'color_topic': color_topic
- 'depth_topic': depth_topic
- 'cycle': cycle
- 'output_dir': output_dir

## Quick Start

```bash
$ cd ~/robot_ws/src
$ git clone https://github.com/cwsfa/rosbag_image_crawler.git
$ cd ..
$ colcon build --symlink-install
$ cd ~/robot_ws/src/rosbag_image_crawler/
$ ros2 bag play 'your ros2 bag record File'
$ ros2 launch rosbag_image_crawler image_crawler.launch.py
```
