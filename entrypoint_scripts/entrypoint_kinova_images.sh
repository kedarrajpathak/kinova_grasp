#!/bin/bash

# Source the ROS2 installation
source /opt/ros/iron/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
ros2 launch kinova_images camera_subscriber.launch.py