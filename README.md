# Rosbag Image Crawler (for ROS2)

The image crawler that extracts and stores image topics every set period from rosbag file.

## Package Information

- **Package Name**: rosbag_image_crawler
- **Node Name**: image_crawler.py
- **Subscriber Topics**: /camera/color/image_raw & /camera/depth/image_rect_raw
- **Note:** this node run your ros2 record file first

## Parameters

- 'color_topic': ```color_topic```
- 'depth_topic': ```depth_topic``` 
- 'cycle': ```cycle```
- 'output_dir': ```output_dir```

| color_topic & depth_topic is From the camera msg.

| cycle is Time in Hertz

| output_dir is Storage Path

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
