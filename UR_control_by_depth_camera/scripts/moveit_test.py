#!/usr/bin/env python3

from UR_control_by_depth_camera.robot_classes import RobotClass


def main():
    try:
        ur3 = RobotClass()
        while True:
            print("choose x:")
            choose_x = float(input())
            print("choose y:")
            choose_y = float(input())
            print("choose z:")
            choose_z = float(input())
            ur3.move_to_position(choose_x, choose_y, choose_z)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()