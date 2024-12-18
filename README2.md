<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#about-the-repo">About the Repo</a></li>
    <li><a href="#network-setup">Network Setup</a></li>
    <li><a href="#gripper-selection">Gripper Selection</a></li>
    <li><a href="#installation">Installation</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#configuration">Configuration</a></li>
    <li><a href="#setting-up-protection-zones">Setting Up Protection Zones</a></li>
    <li><a href="#known-issues-and-workarounds">Known Issues and Workarounds</a></li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About the Repo

This repo is used to generate simple trajectories using ROS move_it python interface for Kinova Gen3 7 DOF manipulator with Robotiq 2F 85 gripper. Trajectories can also be saved, and replayed. It is based on the [ros_kortex](https://github.com/Kinovarobotics/ros_kortex.git) repository (specifically [movieit example](https://github.com/Kinovarobotics/ros_kortex/tree/noetic-devel/kortex_examples/src/move_it)). 

### Network Setup

1. Use Ethernet cable to connect to the robot
2. Open wired connection settings
3. Click on + to add a new connection
4. In Identity tab, give it a name like "Kinova"
5. In IPv4 tab set method to "Manual"
6. Set Address to "192.168.1.11" and Netmask to "255.255.255.0"
7. Save the connection and now the kinova web application should be accessible at "192.168.1.10"
8. Use "admin" as both username and password

### Gripper Selection

In Web Application - Configurations - Robot - Arm - Product - End Effector Type - select "Robotiq 2F-85 gripper, 2 fingers".  
The robot might need to restart after this step.

### Installation

1. Clone the repo
   ```sh
   git clone --recurse-submodules -j8 https://git-ce.rwth-aachen.de/wzl-mq-ms/docker-ros/ros/kinova-ros.git
   ```
    **All the following commands should be executed from the root of the repository `~/path-to-kinova-ros$`**

2. Execute the setup.sh script
   ```sh
   bash setup.sh
   ```
3. Build the Docker Image
   ```sh
   bash docker_build.sh
   ```


### Usage

1. Start a Docker Container from docker_run file
   ```sh
   bash docker_run/docker_run.sh
   ```
2. In a new terminal window access the same container with docker_exec
   ```sh
   bash docker_run/docker_exec.sh
   ```
3. In one of the terminals start either kortex_driver or kortex_gazebo
   ```sh
   roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_85 # physical robot 
   ```
   OR
   ```sh
   roslaunch kortex_gazebo spawn_kortex_robot.launch gripper:=robotiq_2f_85 # simulated robot
   ```
4. In the other terminal window start the example move_it script
   ```sh
   roslaunch kortex_examples moveit_example.launch mode:=2 pickup_height:=0.22
   ```
   ```sh
    # 0: Plan -> execute
    # 1: Plan -> execute -> save
    # 2: Load -> execute
   ```
    - The default mode is 2 which assumes that the trajectories were planned, tested and saved for later use prior to execution of the above command. Also in this mode the robot will keep repeating the motion in loop for 99 iterations.  
    - Mode 0 can be used with simulated robot to test the modified trajectories before testing them on the physical robot and saving for later use.  
    - Mode 1 is used after mode 0 when you want to save the tested trajectories.
    - Pickup height can be set according to the payload geometry. The default height is 0.3. For each new payload the trajectories first need to be planned and saved using modes 0 and 1 before using mode 2. Alway use the height of the lowermost point on the cylinder that will be grasped.

### Making Changes

Always make changes to the python script and launch file that are located inside the `./moveit/` folder. After making changes kill all active containers, run `bash setup.sh` and start the container again by following the steps in the <a href="#usage">Usage</a> section. There is no need to rebuild the container.

### Configuration

1. The trajectories are saved by default at `./moveit/`; while generating or executing new trajectories, appropriate names should be set in the python script `example_move_it_trajectories.py`.
2. To modify the velocity and acceleration scaling, the values need to be changed in the move_it config files at `catkin_ws/src/ros_kortex/kortex_move_it_config/gen3_robotiq_2f_85_move_it_config/config/7dof/default_joint_limits.yaml`.
3. To be able to modify the above mentioned files inside the container, appropriate volumes are mounted in `docker_run.sh`

### Setting Up Protection Zones

In the "Kinova Web Application - Operations - Protection Zones" basic geometrical shapes can be added to set up virtual walls which the robot will not cross. Three walls have already been set up for left, right and front at a distance of +/- 60 cm in y direction and 90 cm in x direction.

### Known Issues and Workarounds

If the moveit_example fails to execute then check the error messages on both the terminal windows one for the driver/gazebo and the other for the moveit. Those could be similar to one of the following:

1. Deviation between actual robot start pose and start pose from trajectory being executed
    - move the end effector a few centimeters using xbox controller and execute the script again
2. Velocity/Acceleration values out of bound
    - modify the scaling factors for velocity and acceleration in the moveit config files
3. Load too heavy / max power
    - try reducing the load first or move it closer to base (need to generate trajectories again for new pickup drop poses)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

