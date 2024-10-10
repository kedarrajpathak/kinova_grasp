ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV ROS_DISTRO=${ROS_DISTRO}

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install dependencies with apt
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        ament-cmake \
        python3-pip \
        ros-${ROS_DISTRO}-xacro \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-kortex-bringup \
        ros-${ROS_DISTRO}-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
        wget && \
    apt-get update --fix-missing && \
    apt-get dist-upgrade -y

# Copy colcon_ws to docker image
COPY colcon_ws/src/ /colcon_ws/src/

# Install module dependencies of colcon_ws
WORKDIR /colcon_ws/
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Source and build colcon_ws
RUN source /opt/ros/humble/setup.bash && colcon build 
#--symlink-install

# Copy overlay_ws to docker image
COPY overlay_ws/src/ /overlay_ws/src/

# Install module dependencies of overlay_ws
WORKDIR /overlay_ws/
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y 

# Source and build overlay_ws
RUN source /colcon_ws/install/setup.bash && colcon build 
#--symlink-install