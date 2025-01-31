#!/bin/bash

SCRIPT_DIR="$(dirname $(readlink -f $0))"
REPO_DIR="$(realpath "${SCRIPT_DIR}/..")"
PARENT_DIR="$(realpath "${SCRIPT_DIR}/../..")"

# Get common params
: "${HOST_NAME:=${USER}}"
echo "HOST_NAME is set to: $HOST_NAME"

: "${HOST_IP:=localhost}"
echo "HOST_IP is set to: $HOST_IP"

: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
echo "RMW_IMPLEMENTATION is set to: $RMW_IMPLEMENTATION"

: "${USE_SIM_TIME:=False}"
echo "USE_SIM_TIME is set to: $USE_SIM_TIME"

CONTAINER_NAME_VISION="vision"
CONTAINER_NAME_OPS="ops"
CONTAINER_NAME_GRASP="grasp"
CONTAINER_NAME_PY_PUBSUB="py_pubsub"

# Docker kill commands
DOCKER_KILL_COMMAND_VISION="docker ps -q --filter name=${CONTAINER_NAME_VISION} | grep -q . && docker rm -fv ${CONTAINER_NAME_VISION}"
DOCKER_KILL_COMMAND_OPS="docker ps -q --filter name=${CONTAINER_NAME_OPS} | grep -q . && docker rm -fv ${CONTAINER_NAME_OPS}"
DOCKER_KILL_COMMAND_GRASP="docker ps -q --filter name=${CONTAINER_NAME_GRASP} | grep -q . && docker rm -fv ${CONTAINER_NAME_GRASP}"
DOCKER_KILL_COMMAND_PY_PUBSUB="docker ps -q --filter name=${CONTAINER_NAME_PY_PUBSUB} | grep -q . && docker rm -fv ${CONTAINER_NAME_PY_PUBSUB}"

# Tmux session
TMUX_SESSION='tmux-session_'
RANDOM_NUMBER=$RANDOM
TMUX_SESSION_NAME="$TMUX_SESSION$RANDOM_NUMBER"

# Function to test the connection
function ping_test() {
	if ping -c 1 $1 &>/dev/null; then
		echo "Connection to $1 successful!"
	else
		echo "Can't establish a connection to $1!"
	fi
}

# Function to kill all docker containers with q
function kill_all() {
	bash -c "ssh ${HOST_NAME}@${HOST_IP} '${DOCKER_KILL_COMMAND_LOCALIZATION}'"
	bash -c "ssh ${HOST_NAME}@${HOST_IP} '${DOCKER_KILL_COMMAND_INITIAL_POSE_SETTER}'"

	if [ "${USE_RVIZ}" == "True" ]; then
        bash -c "ssh ${HOST_NAME}@${HOST_IP} '${DOCKER_KILL_COMMAND_RVIZ}'"
    fi

	# Kill tmux
	sleep 5
    TMUX_SESSION_NAME_ATTACHED=$(tmux display-message -p '#S')
    tmux kill-session -t ${TMUX_SESSION_NAME_ATTACHED}
    tmux kill-session -t ${TMUX_SESSION_NAME}
}

# Function that waits for user input and then kills all running docker containers and tmux session
function wait_for_user_and_kill() {
	echo "Press 'q' to kill every process"
	while :; do
		read -n 1 k <&1

		if [[ $k = q ]]; then
			printf "\nQuitting all processes\n"
			kill_all
		fi
	done
}
"$@"

# Grant access to the X server 
xhost +

# Create a new session named $TMUX_SESSION_NAME, split panes and change directory in each
tmux new-session -d -s $TMUX_SESSION_NAME

# Config tmux
tmux set -g mouse on

# Start Kinova vision module
ping_test $HOST_IP
tmux send-keys -t $TMUX_SESSION_NAME "ssh -Y -t ${HOST_NAME}@${HOST_IP} \
	'${DOCKER_KILL_COMMAND_VISION}; \
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
		    --env USE_SIM_TIME=${USE_SIM_TIME} \
            --name ${CONTAINER_NAME_VISION} \
            -v "$REPO_DIR:/kinova-humble:rw" \
            -v $PARENT_DIR:/root/ws/kinova_repos:rw \
            -w /kinova-humble \
            kinova_gen3_7dof_humble:main \
            /kinova-humble/entrypoint_scripts/entrypoint_kinova_vision.sh'" Enter

sleep 2

# Run Kinova Grasp
ping_test $HOST_IP
tmux split-window -hf -t $TMUX_SESSION_NAME
tmux send-keys -t $TMUX_SESSION_NAME "ssh -t ${HOST_NAME}@${HOST_IP} \
	'${DOCKER_KILL_COMMAND_GRASP}; \
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
		    --env USE_SIM_TIME=${USE_SIM_TIME} \
            --name ${CONTAINER_NAME_GRASP} \
            -v "$REPO_DIR:/kinova-humble:rw" \
            -v $PARENT_DIR:/root/ws/kinova_repos:rw \
            -w /kinova-humble \
            kinova_gen3_7dof_humble:main \
            /kinova-humble/entrypoint_scripts/entrypoint_kinova_grasp.sh'" Enter

sleep 2

# Run Kinova operations
# for debugging add "-v ${REPO_DIR}/overlay_ws/src/initial_pose_setter:/overlay_ws/src/initial_pose_setter \"
ping_test $HOST_IP
tmux split-window -hf -t $TMUX_SESSION_NAME
tmux send-keys -t $TMUX_SESSION_NAME "ssh -Y -t ${HOST_NAME}@${HOST_IP} \
	'${DOCKER_KILL_COMMAND_OPS}; \
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
		    --env USE_SIM_TIME=${USE_SIM_TIME} \
            --name ${CONTAINER_NAME_OPS} \
            -v "$REPO_DIR:/kinova-humble:rw" \
            -v $PARENT_DIR:/root/ws/kinova_repos:rw \
            -w /kinova-humble \
            kinova_gen3_7dof_humble:main \
            /kinova-humble/entrypoint_scripts/entrypoint_kinova_ops.sh'" Enter

sleep 2

# # Run Grasp Pose Publisher
# ping_test $HOST_IP
# tmux split-window -hf -t $TMUX_SESSION_NAME
# tmux send-keys -t $TMUX_SESSION_NAME "ssh -t ${HOST_NAME}@${HOST_IP} \
# 	'${DOCKER_KILL_COMMAND_PY_PUBSUB}; \
#     docker run \
#             -it \
#             --rm \
#             --net=host \
#             --pid=host \
#             --ipc=host \
#             --privileged \
#             --gpus all \
#             --runtime=nvidia \
#             -v /dev:/dev \
#             -v $HOME/.ros/log:/.ros/log \
#             -v /tmp/.X11-unix:/tmp/.X11-unix \
#             --env RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
#             --env DISPLAY=$DISPLAY \
# 		    --env USE_SIM_TIME=${USE_SIM_TIME} \
#             --name ${CONTAINER_NAME_PY_PUBSUB} \
#             -v "$REPO_DIR:/kinova-humble:rw" \
#             -v $PARENT_DIR:/root/ws/kinova_repos:rw \
#             -w /kinova-humble \
#             kinova_gen3_7dof_humble:main \
#             /kinova-humble/entrypoint_scripts/entrypoint_py_pubsub.sh'" Enter

# sleep 2


# Kill process terminal
tmux split-window -t $TMUX_SESSION_NAME
tmux send-keys -t $TMUX_SESSION_NAME "bash $0 wait_for_user_and_kill" Enter

# Attach to session named $TMUX_SESSION_NAME
tmux attach -t $TMUX_SESSION_NAME
