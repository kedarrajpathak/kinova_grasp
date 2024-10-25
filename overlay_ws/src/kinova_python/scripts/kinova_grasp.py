#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import numpy as np

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
import message_filters
from sensor_msgs.msg import Image


def image_callback(color_msg, depth_msg):
    # Convert both messages to NumPy arrays
    color_image = image_msg_to_numpy(color_msg)  # RGB image: (height, width, 3)
    depth_image = image_msg_to_numpy(depth_msg)  # Depth image: (height, width)

    # Ensure depth image is reshaped to match the height and width of the color image
    if color_image.shape[:2] != depth_image.shape[:2]:
        get_logger().error('Color and depth images do not have matching sizes.')
        return

    # Normalize depth image to 0-255 and convert it to uint8 (optional)
    # depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Stack the color and depth images along the last axis to create a (height, width, 4) array
    rgbd_image = np.dstack((color_image, depth_image))  # Shape: (height, width, 4)

    # Save the combined RGB-D image as a NumPy array
    np.save('/kinova-ros2/image_transfer/rgbd_image.npy', rgbd_image)

    get_logger().info('Saved RGB-D image (4 channels).')

def image_msg_to_numpy(msg):
    """Convert ROS Image message to NumPy array."""
    height = msg.height
    width = msg.width
    encoding = msg.encoding

    if encoding == 'rgb8':
        np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
    elif encoding == 'mono16' or encoding == '16UC1':  # Handle 16-bit single-channel images
        np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
    else:
        np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

    return np_array

# Create subscribers for color and depth_registered image topics
color_image_sub = message_filters.Subscriber( Image, '/camera/color/image_raw')
depth_registered_image_sub = message_filters.Subscriber( Image, '/camera/depth_registered/image_rect')

# Synchronize the color and depth_registered image topics
ts = message_filters.ApproximateTimeSynchronizer([color_image_sub, depth_registered_image_sub], 10, 0.1)
ts.registerCallback(image_callback)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    logger.info("# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #")

    time.sleep(sleep_time)


def grasp_loop(gen3, gen3_gripper, gen3_manipulator, logger):
    ###################################################################
    # Move to named pose "Observe"
    ###################################################################
    # set plan start state using predefined state
    gen3_manipulator.set_start_state_to_current_state()

    # set pose goal using predefined state
    gen3_manipulator.set_goal_state(configuration_name="observe")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)    
    ###################################################################
    # Open the gripper
    ###################################################################    
    # set plan start state using predefined state
    gen3_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    gen3_gripper.set_goal_state(configuration_name="Open")

    # plan to goal
    plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)
    ###################################################################
    # Call the function image_callback to save the RGB-D image
    ###################################################################
    ###################################################################
    # Load the numpy array containing grasp poses
    ###################################################################
    ###################################################################
    # Choose a grasp pose; if no grasp pose is available, exit the loop
    # by returning finish_grasp_loop = True
    ###################################################################
    ###################################################################
    # Calculate the first goalpose as 10 cm in z-direction from grasp pose
    # Multiply the 4x4 matrix of the grasp pose by the 4x1 vector [0, 0, 0.1, 0]
    ###################################################################
    ###################################################################
    # Plan and Execute to the first goalpose
    ###################################################################
    # set plan start state to current state
    gen3_manipulator.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "camera_link"
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.0
    gen3_manipulator.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)
    ###################################################################
    # The second goalpose is the grasp pose
    # Plan and Execute to the second goalpose
    ###################################################################
    # set plan start state to current state
    gen3_manipulator.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "camera_link"
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.0
    gen3_manipulator.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)
    ###################################################################
    # Close the gripper
    ###################################################################    
    # set plan start state using predefined state
    gen3_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    gen3_gripper.set_goal_state(configuration_name="Close")

    # plan to goal
    plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)
    ###################################################################
    # Move up 30 cm in z-direction
    ###################################################################
    ###################################################################
    # Move to the named pose "Drop"
    ###################################################################
    ###################################################################
    # Open the gripper
    ###################################################################    
    # set plan start state using predefined state
    gen3_gripper.set_start_state_to_current_state()
    # gen3_gripper.set_start_state(configuration_name="Open")

    # set pose goal using predefined state
    gen3_gripper.set_goal_state(configuration_name="Open")

    # plan to goal
    plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)
    return False

def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    gen3 = MoveItPy(node_name="moveit_py")
    gen3_manipulator = gen3.get_planning_component("manipulator")
    gen3_gripper = gen3.get_planning_component("gripper")
    logger.info("MoveItPy instance created")

    finish_grasp_loop = False
    while not finish_grasp_loop:
        finish_grasp_loop = grasp_loop(gen3, gen3_gripper, gen3_manipulator, logger)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()