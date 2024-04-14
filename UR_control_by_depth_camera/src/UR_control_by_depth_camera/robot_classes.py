import sys
import rospy
import moveit_commander
from moveit_msgs.msg import Constraints, OrientationConstraint
import geometry_msgs.msg

import mediapipe as mp
import numpy as np
import cv2


class RobotClass:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_planning_time(5)
        # display_trajectory_publisher = rospy.Publisher("/scaled_pos_joint_traj_controller/follow_joint_trajectory", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.upright_constraints = Constraints()
        self.upright_constraints.name = "upright"


        self.robot_information()
        # self.robot_constraints()

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

    def robot_constraints(self):
        constraint_pose = self.group.get_current_pose()

        orientation_constrains = OrientationConstraint()
        orientation_constrains.header.frame_id = constraint_pose.header.frame_id
        orientation_constrains.link_name = "wrist_3_link"

        orientation_constrains.orientation.w = 0

        orientation_constrains.orientation.x = -1
        orientation_constrains.orientation.y = 0
        orientation_constrains.orientation.z = 0


        orientation_constrains.absolute_x_axis_tolerance = 0.3
        orientation_constrains.absolute_y_axis_tolerance = 0.3
        orientation_constrains.absolute_z_axis_tolerance = 0.1

        orientation_constrains.weight = 1

        self.upright_constraints.orientation_constraints = [orientation_constrains]
        self.group.set_path_constraints(self.upright_constraints)



    @staticmethod
    def box_limits(check_x, check_y, check_z):
        max_x= 0.3
        max_y = 0.4
        max_z = 0.4

        min_x = -0.3
        min_y = 0.15
        min_z = 0.1

        if check_x > max_x:
            check_x = max_x
        if check_x < min_x:
            check_x = min_x
        if check_y > max_y:
            check_y = max_y
        if check_y < min_y:
            check_y = min_y
        if check_z > max_z:
            check_z = max_z
        if check_z < min_z:
            check_z = min_z

        return check_x, check_y, check_z

    def move_to_position(self, x, y, z):

        x, y, z = self.box_limits(x, y, z)

        print("x:",x)
        print("y:",y)
        print("z:", z)

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = -1.0
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        self.group.set_pose_target(pose_target)

        plan = self.group.go(wait=True)
        self.finish_movement(plan)

    def finish_movement(self, finish_plan):
        self.group.stop()
        self.group.clear_pose_targets()
        # self.group.execute(finish_plan, wait=True)


class HandDetection:
    def __init__(self, depth_scale):

        #mediapipe hand recognition variables
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands()
        self.mpDraw = mp.solutions.drawing_utils

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.org = (10, 10)
        self.fontScale = .5
        self.color = (0, 50, 255)
        self.thickness = 1
        self.background_removed_color = 153

        self.depth_scale = depth_scale
        self.clipping_distance_in_meters = 1
        self.clipping_distance = self.clipping_distance_in_meters / depth_scale


    def frame_processing(self, depth_frame, color_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image_flipped = cv2.flip(depth_image, 1)
        color_image = np.asanyarray(color_frame.get_data())

        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image))  # Depth image is 1 channel, while color image is 3
        background_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0),
                                      self.background_removed_color, color_image)

        processing_images = cv2.flip(background_removed, 1)
        color_image = cv2.flip(color_image, 1)
        color_images_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        self.hand_processing(depth_image_flipped, processing_images, color_images_rgb)

    def hand_processing(self, hand_depth_image_flipped, hand_images, hand_color_images_rgb):
        results = self.hands.process(hand_color_images_rgb)
        if results.multi_hand_landmarks:
            number_of_hands = len(results.multi_hand_landmarks)
            i = 0
            for handLms in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(hand_images, handLms, self.mpHands.HAND_CONNECTIONS)
                org2 = (20, self.org[1] + (20 * (i + 1)))
                hand_side_classification_list = results.multi_handedness[i]
                hand_side = hand_side_classification_list.classification[0].label
                middle_finger_knuckle = results.multi_hand_landmarks[i].landmark[9]
                x = int(middle_finger_knuckle.x * len(hand_depth_image_flipped[0]))
                y = int(middle_finger_knuckle.y * len(hand_depth_image_flipped))
                if x >= len(hand_depth_image_flipped[0]):
                    x = len(hand_depth_image_flipped[0]) - 1
                if y >= len(hand_depth_image_flipped):
                    y = len(hand_depth_image_flipped) - 1
                mfk_distance = hand_depth_image_flipped[y, x] * self.depth_scale  # meters
                hand_images = cv2.putText(hand_images,
                                     f"{hand_side} Hand Distance: {mfk_distance:0.3} meters away on position x:{x} and y:{y}",
                                     org2, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

                self.x = x
                self.y = y
                self.z = mfk_distance

                i += 1
            hand_images = cv2.putText(hand_images, f"Hands: {number_of_hands}", self.org, self.font, self.fontScale, self.color, self.thickness,
                                 cv2.LINE_AA)
        else:
            hand_images = cv2.putText(hand_images, "No Hands", self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

        self.hand_images = hand_images

