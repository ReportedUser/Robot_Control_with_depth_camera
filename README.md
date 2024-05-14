# Universal Robots Depth camera control

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

You also need to install the [UR simulator](https://www.universal-robots.com/download/?filters[]=98759&query=), make sure it's on the same network as your ROS machine. Follow the setuup tutorial on [UR Usage Example](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md) to install it and run the test_move script as said inside the control the robot section.

There you will also see an example of [MoveIt](https://moveit.ros.org/), some parts of it will be used in the setup, but don't worry right now!

Now that your workspace is created and the simulator works correctly, it's time to start the package installation.

## Package installation

To install this package, simply clone it to your workspace and build it.

```
# cd ~/catkin_ws/

$ git clone https://github.com/ReportedUser/Robot_Control_with_depth_camera.git src/Robot_Control_with_depth_camera

$ catkin_make

```

Once this is done, you will have to give your robot some joint angle limits depending on your robot workspace so it doesn't bump on to the surroundings.
To do so, just go to:
```
$ rosed ur3_moveit_config joint_limits.yaml
```
This file contains limits for your robot joints, add your corresponding limits by typing at the end of each desired joint. An example would be:
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


## Setting it up

Now that the requirements are complete, we can start playing with it!

First, start the UR robot and load the [urcap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases).

First of all, you should calibrate the robot. Follow the [Universal Robot Calibration Guide](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_calibration/README.md) in order to calibrate.
Once it has been calibrated, either follow then next instrucction adding the calibration file or launch it via your new launch file with the calibration.

Next step will be to start two launch files on different terminal windows.
```
$ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=your.robot.ip.adress
```
```
$ roslaunch ur3_moveit_config moveit_planning_execution.launch
```
First one is the launch file that connects to the robot, second one adds ros nodes for moveit controls.

Now that the launch files are runing, you can start playing with this package scripts, For example, the next script gets the cobot imitating the hand movement.

```
rosrun Universal_Robots_Depth_camera_control hand_tracking
```

If everything was done correctly, the robot should be following your every movement!

*In case you are setting up your joint limits, you'll need to stop ur3_moveit_config before every change for them to be processed. Also, has_position_limits: false doesn't work and seems to bug the urcap on the simulator, so if you want to deactivate something just comment it out.


## Next goals

Next steps I'll be adding to this project:

- Pick and Place using the depth camera.
- Improve code qualty of life.
