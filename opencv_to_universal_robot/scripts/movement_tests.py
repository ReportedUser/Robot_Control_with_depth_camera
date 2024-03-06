#!/usr/bin/env python3
import sys

import actionlib
import geometry_msgs.msg
import rospy
from controller_manager_msgs.srv import (
    SwitchControllerRequest, SwitchController,
    LoadController, LoadControllerRequest,
    ListControllers, ListControllersRequest,
)
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
    )

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]


class TrajectoryClient:

    def __init__(self):
        rospy.init_node("UR_movement")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controller", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg {}".format(err))
            sys.exit(-1)

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

    def send_cartesian_trajectory(self):
        self.load_srv(self.cartesian_trajectory_controller)

        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        pose_list = [
            geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(0.4, -0.1, 0.4), geometry_msgs.msg.Quaternion(0,0,0,1)),
            geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(0.4, -0.1, 0.6), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(0.4, 0.3, 0.6), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(0.4, 0.3, 0.4), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(0.4, -0.1, 0.4), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
        ]
        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.ask_confirmation(pose_list)
        rospy.loginfo("Executing trajectory using the {}".format(self.cartesian_trajectory_controller))
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def ask_confirmation(self, waypoint_list):
        rospy.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input("Please confirm. Fill whenever It's not a simulation.")
            valid = input_str in ["y", "n"]
            if not valid:
                rospy.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                confirmed = input_str == 'y'

        if not confirmed:
            rospy.loginfo("Existing as requested by user.")
            sys.exit(0)

    def switch_controller(self, target_controller):
        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)


if __name__ == "__main__":
    client = TrajectoryClient()
    try:
        client.send_cartesian_trajectory()
    except:
        rospy.logerr("Could not execute cartesian service. Check what's wrong.")