#!/bin/bash

# Source the ROS2 installation
source /opt/ros/iron/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Allow non-zero velocity at trajectory end
ros2 param set /joint_trajectory_controller allow_nonzero_velocity_at_trajectory_end True