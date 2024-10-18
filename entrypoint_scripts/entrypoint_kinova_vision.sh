#!/bin/bash

# Source the ROS2 installation
source /opt/ros/iron/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
ros2 launch kinova_vision kinova_vision.launch.py \
    depth_registration:=true \
    launch_depth:=true \
    max_color_pub_rate:=5.0 \
    max_depth_pub_rate:=5.0 
    # color_camera_info_url:=file:///home/user/custom_color_calib_1280x720.ini \
    # depth_camera_info_url:=file:///home/user/custom_depth_calib_480x270.ini 