# Universal_Robots_Depth_camera_control

This RoS package is used to control Universal Robots with a depth camera.

# Components

To test this, the following components where used:

- UR3
- D435i camera
- RoS Noetic

# Project

The objective of this project was to follow the hand position with a robot.
In order to do so this package makes use of mediapipe to identify the hand and moveit for the communication with the robot.

# Contents
This repository contains hand_tracking and some helper scripts:
- Hand_tracking: Actual script used to track the hand and move the robot.
- moveit_test: Helper script to check if the moveit and connection to the robot section of the program actually works.
- hand_to_coordinates_mediapipe: Helper script to check if the mediapipe section of the program actually works.

# Requirements

This package requires ROS Noetic and [Universal Robot drivers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master).
