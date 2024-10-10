#!/bin/bash

SCRIPT_DIR="$(dirname $(readlink -f $0))"
REPO_DIR="$(realpath "${SCRIPT_DIR}/..")"


xhost +
docker run \
		-it \
		--rm \
		--net=host \
		--pid=host \
		--ipc=host \
		--privileged \
        --gpus all \
        --runtime=nvidia \
		-v /dev:/dev \
		-v $HOME/.ros/log:/.ros/log \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		--env RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
		--env DISPLAY=$DISPLAY \
        --name ros2-kortex \
        -v "$REPO_DIR:/docker-base-image-ros:rw" \
        -w /docker-base-image-ros \
        kinova_gen3_7dof:main \
        bash