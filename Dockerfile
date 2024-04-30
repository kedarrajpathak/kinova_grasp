FROM osrf/ros:humble-desktop

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install dependencies with apt
RUN apt-get update && apt-get upgrade -y
RUN apt-get install ament-cmake -y
RUN apt-get install python3-pip -y
RUN apt-get install ros-humble-kortex-bringup -y
RUN apt-get install ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config -y

RUN apt-get update --fix-missing

# Fix rviz2 black screen
RUN apt-get install -y software-properties-common 
RUN add-apt-repository ppa:kisak/kisak-mesa 
RUN apt-get -y update 
RUN apt-get -y upgrade

# RMW cyclonedds
RUN apt-get install ros-humble-rmw-cyclonedds-cpp -y
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Copy colcon_ws to docker image
COPY colcon_ws/src/ /colcon_ws/src/

# # Install module dependencies of colcon_ws
# WORKDIR /colcon_ws/
# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y

# # Source and build colcon_ws
# RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Copy overlay_ws to docker image
COPY overlay_ws/src/ /overlay_ws/src/

# # Install module dependencies of overlay_ws
# WORKDIR /overlay_ws/
# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y 

# # Source and build overlay_ws
# RUN source /colcon_ws/install/setup.bash && colcon build --symlink-install

# Copy cyclonedds config
COPY cyclonedds/config.xml /config.xml
COPY cyclonedds/10-cyclone-max.conf /etc/sysctl.d/10-cyclone-max.conf

# Copy entrypoint scripts and make executable
COPY entrypoint_scripts/ /entrypoint_scripts
RUN chmod +x /entrypoint_scripts/*.sh