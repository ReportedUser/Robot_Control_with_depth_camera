# Universal_Robots_Depth_camera_control

This RoS package is used to control an Universal Robot with a depth camera.

## Components

In order to do so, the following components where used:

- UR3 simulator
- D435i camera
- RoS Noetic

## Project

This project objective is to follow the hand position with a robot.
In order to do so this package makes use of mediapipe to identify the hand and moveit for the communication with the robot.

## Contents
This repository contains hand_tracking and some helper scripts:
- Hand_tracking: Actual script used to track the hand and move the robot.
- moveit_test: Helper script to check if the moveit and connection to the robot section of the program actually works.
- hand_to_coordinates_mediapipe: Helper script to check if the mediapipe section of the program actually works.

## Requirements

This package is used on ROS Noetic and makes use of Universal Robot drivers, you will also need to install some python libraries to make everything work.

Download [Ubuntu 20.04.6 LTS (Focal Fossa)](https://releases.ubuntu.com/focal/) to be able to install RoS Noetic.

Install [ROS Noteic](http://wiki.ros.org/noetic/Installation/Ubuntu) and then follow [Universal Robot drivers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master) installation guide.

Create a RoS workspace by:
```
$ source /opt/ros/<your_ros_version>/setup.bash

#creation and building of the workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

$ catkin_make

#source your working environment so you can work with it.
$ source devel/setup.bash
```

Now check if your workspace was correctly made by:
```
$ echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/noetic/share
```


Check that your ROS connection with the [UR simulator](https://www.universal-robots.com/download/?filters[]=98759&query=) is correct by using the tests presented in [UR Usage Example](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md). There you will also see an example of [MoveIt](https://moveit.ros.org/), some parts of it will be used in the setup, but don't worry right now!

Now that your workspace is created, is time to start the package installation.

## Package installation

To install this package, simply clone it to your workspace and build it.

```
# cd ~/catkin_ws/

$ git clone https://github.com/ReportedUser/opencv_to_universal_robot.git src/Universal_Robots_Depth_camera_control

$ catkin_make

```

Once this is done, you will have to give your robot some joint angle limits depending on your robot workspace so it doesn't bump on to the surroundings.
To do so, just go to:
```
$ rosed ur3_moveit_config joint_limits.yaml
```
This file contains limits for your robot, add your corresponding limits by typing at the end of each desired joint. An example would be:
```
  ... 
shoulder_lift_joint:
  has_velocity_limit: true
  ...
  max_acceleration: 0
  has_position_limits: true
  max_position: (max joint angle on radiants)
  min_position: (min joint angle on radiants)
shoulder_pan_joint:
  ...
```

If you don't know your joint limits, you can figure them out once the program is running.

To finish it up, some libraries are needed:
mediapipe      0.10.9 
pyrealsense2   2.54.2.5684 
opencv-python  4.9.0.80 

## Setting it up

Now that the requirements are complete, we can start playing with it!

First, on the simulator external control (can be found inside Installation -> URCaps) add your Host IP with custom port 50002 and your host name, then start the UR robot and load the [urcap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases).

On ROS side you will need to start two launch files on different terminal windows.
```
$ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=your.robot.ip.adress
```
```
$ roslaunch ur3_moveit_config moveit_planning_execution.launch
```
And then the script.
```
rosrun Universal_Robots_Depth_camera_control hand_tracking
```

If everything was done correctly, the robot should be following your every movement!

*In case you are setting up your joint limits, you'll need to stop and relaunch ur3_moveit_config for the changes to be loaded.
