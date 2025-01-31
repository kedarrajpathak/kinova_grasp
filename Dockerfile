ARG ROS_DISTRO=humble
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV ROS_DISTRO=${ROS_DISTRO}

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install dependencies with apt
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        ament-cmake \
        python3-pip \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp \
        wget \
        gstreamer1.0-tools \
        gstreamer1.0-libav \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-base && \
    apt-get update --fix-missing && \
    apt-get dist-upgrade && \
    rosdep update -y

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-moveit \
        ros-$ROS_DISTRO-ros2-control \
        ros-$ROS_DISTRO-ros2-controllers \
        ros-$ROS_DISTRO-topic-based-ros2-control \
        ros-$ROS_DISTRO-tf-transformations \
        # ros-$ROS_DISTRO-robotiq-controllers \
        # ros-$ROS_DISTRO-robotiq-description \
        # ros-$ROS_DISTRO-robotiq-driver \
        # ros-$ROS_DISTRO-robotiq-hardware-tests \
        # ros-$ROS_DISTRO-kortex-api \
        # ros-$ROS_DISTRO-kortex-bringup \
        # ros-$ROS_DISTRO-kortex-description \
        # ros-$ROS_DISTRO-kortex-driver \
        # ros-$ROS_DISTRO-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
        ros-$ROS_DISTRO-depth-image-proc 

COPY colcon_ws/src/ /colcon_ws/src/
WORKDIR /colcon_ws/
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build
RUN source /colcon_ws/install/setup.bash


COPY overlay_ws/src/ /overlay_ws/src/
WORKDIR /overlay_ws/
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
RUN source /colcon_ws/install/setup.bash && colcon build 
RUN source /overlay_ws/install/setup.bash