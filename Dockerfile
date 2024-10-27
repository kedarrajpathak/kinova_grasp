ARG ROS_DISTRO=iron
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
        wget && \
    apt-get update --fix-missing && \
    apt-get dist-upgrade && \
    rosdep update -y

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt-get remove ros-$ROS_DISTRO-moveit*

# Copy colcon_ws to docker image
COPY colcon_ws/src/ /colcon_ws/src/
WORKDIR /colcon_ws/src/
# RUN git clone https://github.com/moveit/moveit2.git -b ${ROS_DISTRO}

# Install module dependencies of colcon_ws
RUN apt-get update && \
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Source and build colcon_ws
WORKDIR /colcon_ws/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --parallel-workers 2 --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /colcon_ws/install/setup.bash

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        ros-${ROS_DISTRO}-kortex-bringup \
        ros-${ROS_DISTRO}-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
        ros-${ROS_DISTRO}-tf-transformations

RUN apt-get install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base -y
RUN apt-get install ros-${ROS_DISTRO}-depth-image-proc -y

# Copy overlay_ws to docker image
COPY overlay_ws/src/ /overlay_ws/src/

# Install module dependencies of overlay_ws
WORKDIR /overlay_ws/
RUN apt-get update && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Source and build overlay_ws
RUN source /colcon_ws/install/setup.bash && colcon build 
#--symlink-install