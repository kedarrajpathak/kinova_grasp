#!/bin/bash

# Source the ROS2 installation
source /opt/ros/iron/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
ros2 launch kinova_vision kinova_vision.launch.py \
    device:=10.20.0.100 \
    depth_registration:=true \
    launch_depth:=false 
    # color_camera_info_url:=file:///home/user/custom_color_calib_1280x720.ini \
    # depth_camera_info_url:=file:///home/user/custom_depth_calib_480x270.ini 