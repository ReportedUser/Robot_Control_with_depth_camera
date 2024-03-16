#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint
import geometry_msgs.msg


class RobotClass:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        # display_trajectory_publisher = rospy.Publisher("/scaled_pos_joint_traj_controller/follow_joint_trajectory", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.upright_constraints = Constraints()
        self.upright_constraints.name = "upright"

        self.robot_information()
        self.robot_constrains()

    def robot_information(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this self.group:
        eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the self.groups in the robot:
        print("============ Robot Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def robot_constrains(self):
        joint_constraints = JointConstraint()
        joint_constraints.position = 0
        joint_constraints.tolerance_above = 0
        joint_constraints.tolerance_below = 3.14/4
        joint_constraints.weight = 1

        joint_constraints.joint_name = "wrist_1_joint"
        self.upright_constraints.joint_constraints.append(joint_constraints)



    def box_limits(self):
        pass
        max_x = "0.3"
        max_y = "0.4"
        max_z = "0.4"

        min_x = "-0.3"
        min_y = "0.15"
        min_z = "0.1"

    def move_to_position(self):

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = 0.3
        pose_target.position.y = 0.4
        pose_target.position.z = 0.1
        self.group.set_pose_target(pose_target)

        plan = self.group.go(wait=True)
        self.finish_movement(plan)

    def finish_movement(self, finish_plan):
        self.group.stop()
        self.group.clear_pose_targets()
        # self.group.execute(finish_plan, wait=True)


def main():
    ur3 = RobotClass()
    ur3.move_to_position()


if __name__ == '__main__':
    main()