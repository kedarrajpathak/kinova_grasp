#!/bin/bash

# Source the ROS2 installation
source /opt/ros/iron/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
ros2 launch moveit_task_constructor_kinova kinova_mtc.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true --debug