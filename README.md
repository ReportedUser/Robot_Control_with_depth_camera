#  Robots Depth camera control

This RoS package is used to control a  Robot with a depth camera.

## Components

In order to do so, the following components where used:

- UR3 simulator
- D435i camera
- RoS Noetic

## Project

This project's objective is to follow the position of a hand with a robot.
In order to do so, this package makes use of mediapipe to identify the hand and moveit for the communication with the robot..

## Contents
This repository contains the main scripts hand_tracking and tracking_orientation.py with some helper scripts:
- [Hand_tracking](https://youtu.be/usjSbZUHQrM): Actual script used to track the hand and move the robot.
- [tracking_orientation](https://youtu.be/mS1w9SVrdP4): Script to track movement and orientation, still needs a bit of work.
- moveit_test: Helper script to check if moveit and connection to the robot section of the program actually works.
- hand_to_coordinates_mediapipe: Helper script to check if the mediapipe section of the program actually works.

## Requirements

This package is used on ROS Noetic and makes use of Universal Robots drivers. You will also need to install some Python libraries to enable proper functioning.

Download [Ubuntu 20.04.6 LTS (Focal Fossa)](https://releases.ubuntu.com/focal/) to be able to install RoS Noetic.

Install [ROS Noteic](http://wiki.ros.org/noetic/Installation/Ubuntu) and then follow [ Robot drivers](https://github.com/Robots/_Robots_ROS_Driver/tree/master) installation guide.

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

You also need to install the [UR simulator](https://www.-robots.com/download/?filters[]=98759&query=). Ensure it is on the same network as your ROS machine. Follow the set-up tutorial on [UR Usage Example](https://github.com/Robots/_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md) to install it and run the test_move script as outlined at control the robot section.

There you will also see an example of [MoveIt](https://moveit.ros.org/), some parts of which will be used in the set-up, but don't worry right now!

Now that your workspace is created and the simulator works correctly, it's time to start the package installation.

## Package installation

To install this package, simply clone it to your workspace and build it.

```
# cd ~/catkin_ws/

$ git clone https://github.com/ReportedUser/Robot_Control_with_depth_camera.git src/Robot_Control_with_depth_camera

$ catkin_make

```

Once this is done, you will have to give your robot some joint angle limits depending on your robot workspace so it doesn't bump into the surroundings.
To do so, just go to:
```
$ rosed ur3_moveit_config joint_limits.yaml
```
This file contains limits for your robot joints. Add your corresponding limits by typing at the end of each desired joint. An example would be:
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

If you don't know your joint limits yet, you can figure them out once the program is running.

Inside the directory joint_angle_files you can find some limits I used for my specific case, you can try them out if you consider it can be useful for you.

## Setting it up

Now that the requirements are complete, we can start playing with it!

First, start the UR robot and load the [urcap](https://github.com/Robots/_Robots_ExternalControl_URCap/releases).

Next, you should calibrate the robot. Follow the [ Robot Calibration Guide](https://github.com/Robots/_Robots_ROS_Driver/blob/master/ur_calibration/README.md) in order to calibrate.
Once it has been calibrated, you can either follow the next instruction by adding the calibration file or launch it via your new launch file with the calibration.

The next step is to start two launch files on different terminal windows.
```
$ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=your.robot.ip.adress
```
```
$ roslaunch ur3_moveit_config moveit_planning_execution.launch
```
The first one is the launch file that connects to the robot, and the second one adds ros nodes for moveit controls.

Now that the launch files are running, you can start playing with this package's scripts. For example, the following gets the robot to imitate the hand movement:

```
rosrun _Robots_Depth_camera_control hand_tracking
```

If everything was done correctly, the robot should be following your every movement!

If you want to change the workspace limits, it is done with arguments when defining the robot class inside the scripts.

*In case you are setting up your joint limits, you'll need to stop ur3_moveit_config before every change for them to be processed. Also, has_position_limits: false doesn't work and seems to bug the urcap on the simulator, so if you want to deactivate something just comment it out.


## Next goals

Next steps I'll be adding to this project:

- Pick and Place using the depth camera. (Unfortunately, I no longer have access to the camera, so I won't be able to do this.)
- Improve code quality of life.
- Config file so there is no need to go inside the scripts to change information.

## Recomendation
For the joint angles configuration, make sure to first close the moveit_planning_execution.launch file so whenever it is relaunched it does so with the new values. Also, the has_position_limits: false input option inside joint_limits.yaml won't work, so I recommend commenting out that line along it's respective joint's limits.


If there is any doubt, just ask, I can't promise to answer fast but I'll do my best to help.
