#!/usr/bin/env python3

from UR_control_by_depth_camera.robot_classes import RobotClass

box_constrains = (-0.2, 0.2, 0.4, -0.35, -0.2, 0.25)


def main():
    try:
        ur3 = RobotClass("manipulator", box_constrains)

        x_ant = -0.25
        y_ant = 0
        z_ant = 0.3

        ur3.move_to_position(x_ant, y_ant, z_ant)

        while True:
            print("choose x:")
            choose_x = float(input())
            print("choose y:")
            choose_y = float(input())
            print("choose z:")
            choose_z = float(input())
            ur3.euler2quaternion(180, 0, 0)
            ur3.move_to_position(choose_x, choose_y, choose_z)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()