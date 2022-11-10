# Rosbag Image Crawler (for ROS2)

## Package Information

- **Package Name**: `rosbag_image_crawler`
- **Node Name**: `image_crawler`
- **Subscriber Topics**: `/camera/color/image_raw` & `/camera/depth/image_rect_raw`
- **Note:** This node Run your Ros2 record file first

## Parameters

- **is_debug**: for checking visual (defualt: `False`)
- **is_label**: for labeling while crawling (defualt: `False`)
- **color_topic**: RGB image topic (defualt: `/camera/color/image_raw`)
- **depth_topic**: Depth image topic (defualt: `/camera/aligned_depth_to_color/image_raw`)
- **rate**: image crawling rate (default: `1` sec)
- **label_rate**: label publishing rate (default: `0.1` sec)
- **output_dir**: output_dir (default: `/home/USER/clawed_image`)

## Quick Start

```bash
$ cd ~/robot_ws/src
$ git clone https://github.com/cwsfa/rosbag_image_crawler.git
$ cd ~/robot_ws
$ colcon build --symlink-install --packages-select rosbag_image_crawler
$ ros2 bag play ${BAG_FOLDER}
$ ros2 launch rosbag_image_crawler image_crawler.launch.py
```
