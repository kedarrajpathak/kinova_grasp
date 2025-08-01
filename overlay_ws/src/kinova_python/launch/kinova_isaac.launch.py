# Copyright (c) 2023 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    vision = LaunchConfiguration("vision")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    sim_isaac = LaunchConfiguration("sim_isaac")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")

    launch_arguments = {
        "robot_ip": robot_ip,
        "vision": vision,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
        "sim_isaac": sim_isaac,
    }
# Example:
#     moveit_configs = MoveItConfigsBuilder("robot_name").to_moveit_configs()
#     ...
#     moveit_configs.package_path
#     moveit_configs.robot_description
#     moveit_configs.robot_description_semantic
#     moveit_configs.robot_description_kinematics
#     moveit_configs.planning_pipelines
#     moveit_configs.trajectory_execution
#     moveit_configs.planning_scene_monitor
#     moveit_configs.sensors_3d
#     moveit_configs.move_group_capabilities
#     moveit_configs.joint_limits
#     moveit_configs.moveit_cpp
#     moveit_configs.pilz_cartesian_limits
#     # Or to get all the parameters as a dictionary
#     moveit_configs.to_dict()

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_python")
        .robot_description(mappings=launch_arguments)
        .robot_description_semantic(mappings=launch_arguments)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        #.moveit_cpp(file_path="config/kinova_python.yaml")
        .pilz_cartesian_limits()
        .to_moveit_configs()
    )

    octomap_config = {'octomap_frame': 'camera_rgb_optical_frame', 
                      'octomap_resolution': 0.01,
                      'max_range': 5.0}
    
    # Enable the execution of tasks in the MoveIt Task Constructor (MTC).
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
     
    # example_file = DeclareLaunchArgument(
    #     "example_file",
    #     default_value="kinova_grasp.py",
    #     description="Python API tutorial file name",
    # )

    # moveit_py_node = Node(
    #     name="moveit_py",
    #     package="kinova_python",
    #     executable=LaunchConfiguration("example_file"),
    #     output="both",
    #     parameters=[moveit_config.to_dict()],
    # )

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            octomap_config,
            move_group_capabilities,
        ],
    )

    # MTC Demo node
    pick_place = Node(
        package="kinova_mtc",
        executable="kinova_mtc",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Timer action to delay the start of pick_place
    delayed_pick_place = TimerAction(
        period=7.0,  # Delay time in seconds
        actions=[pick_place]
    )

    # Register the event handler to trigger delayed_pick_place when move_group_node starts
    on_move_group_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[delayed_pick_place]
        )
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # # Gripper Center TF
    # gripper_center_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="gripper_center_tf",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.145", "0.0", "0.0", "0.0", "end_effector_link", "TCP"],
    # )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    )

    # fault_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["fault_controller", "-c", "/controller_manager"],
    #     condition=UnlessCondition(use_fake_hardware),
    # )

    # rviz with moveit configuration
    rviz_config_file = (
        get_package_share_directory("kinova_python")
        + "/config/moveit_mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        # fault_controller_spawner,
        move_group_node,
        static_tf,
        # gripper_center_tf,
        # example_file,
        # moveit_py_node,
        on_move_group_start_handler,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            default_value="true",
            description="Enable vision?",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Start robot with topic based control plugin for controlling robot in isaac sim.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use arm's internal gripper connection",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_external_cable",
            default_value="false",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])