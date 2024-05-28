import sys
import rospy
import moveit_commander
from moveit_msgs.msg import Constraints
import geometry_msgs.msg
from scipy.spatial.transform import Rotation
from typing import Tuple
import mediapipe as mp
import numpy as np
import cv2

ArgsType = Tuple[int, int, int, int, int, int]


def transformation_to_ur_coordinates(trans_x: float, trans_y: float, trans_z: float) -> Tuple[float, float, float]:
    """
    I changed the output of x and y to simplify my specific real life case as the camera long side is x
    while the robot long side is the y.

    Transforms coordinates taken from the camera to the robot workspace.

    You should make your own transformation_to_ur_coordinates.
    :param trans_x: x value taken from camera.
    :param trans_y: y value taken from camera.
    :param trans_z: z value taken from camera.
    :return:
    """

    decimal_number = 2

    if 0.8 >= trans_z >= 0.65:
        trans_z = 0.25 + (trans_z - 0.65)
    elif trans_z > 0.8:
        trans_z = 0.4
        print("Z outside of limits.")
    elif trans_z < 0.65:
        trans_z = 0.25
        print("Z outside of limits.")

    if trans_y <= -0.2:
        trans_y = -0.2
        print("Y outside of limits.")
    elif trans_y >= 0.05:
        trans_y = -0.35
        print("Y outside of limits.")
    else:
        trans_y = ((trans_y-0.05)/(-0.2-0.05))*(-0.2--0.35)-0.35

    if trans_x > 0.2:
        trans_x = 0.2
        print("X outside of limits.")
    elif trans_x < -0.2:
        trans_x = -0.2
        print("X outside of limits.")

    return round(trans_y, decimal_number), round(trans_x, decimal_number), round(trans_z, decimal_number)


class RobotClass:

    def __init__(self, robot_name: str, *args: ArgsType):

        try:
            self.box_constrains = self.unpack_args(args)
        except ValueError as e:
            print(f"Error: {e}")

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(robot_name)
        self.group.set_planning_time(5)
        # display_trajectory_publisher = rospy.Publisher("/scaled_pos_joint_traj_controller/follow_joint_trajectory",
        # moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.upright_constraints = Constraints()
        self.upright_constraints.name = "upright"

        self.pose_target = geometry_msgs.msg.Pose()

        self.euler2quaternion(180, 0, 0)
        self.robot_information()

    @staticmethod
    def unpack_args(arguments):
        limits = [list(arguments)[0][i] for i in range(6)]
        print(limits)
        if len(limits) == 6:
            return limits
        else:
            raise ValueError("Only accepts 6 arguments on the following order; max x, y, z and min x, y, z")

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

    def box_limits(self, check_x, check_y, check_z):
        """
        Defines the limits of the robot workspace.

        :param check_x: x given to move at.
        :param check_y: y given to move at.
        :param check_z: z given to move at.
        :return: returns the same value if inside the limits or the limit.
        """

        max_x = self.box_constrains[0]
        max_y = self.box_constrains[1]
        max_z = self.box_constrains[2]

        min_x = self.box_constrains[3]
        min_y = self.box_constrains[4]
        min_z = self.box_constrains[5]

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

    def define_quaternions(self, quaternions_list):
        """
        Used to define robot orientation via quaternions. accepts a list of 4 quaternions on the following order:
        :param quaternions_list: [w, x, y, z]
        :return:
        """

        if len(quaternions_list) != 4:
            print(f"Only accepts 4 values, but {len(quaternions_list)} where given.")
        else:
            self.pose_target.orientation.x = quaternions_list[1]
            self.pose_target.orientation.y = quaternions_list[2]
            self.pose_target.orientation.z = quaternions_list[3]
            self.pose_target.orientation.w = quaternions_list[0]

    def euler2quaternion(self, i, j, k):
        """
        takes roll, pitch and yaw and transforms to quaternion
        if no values are given, uses 180, 0 and 0 by default
        """
        rot = Rotation.from_euler('xzy', [i, j, k], degrees=True)
        quaternions = rot.as_quat()
        self.pose_target.orientation.x = quaternions[0]
        self.pose_target.orientation.y = quaternions[1]
        self.pose_target.orientation.z = quaternions[2]
        self.pose_target.orientation.w = quaternions[3]

    def move_to_position(self, x, y, z):
        """
        Moves to the given location.

        :param x:
        :param y:
        :param z:
        :return:
        """

        x, y, z = self.box_limits(x, y, z)

        print("============ Moving to position:")
        print("x:",x)
        print("y:",y)
        print("z:", z)
        print("============ Current working space")
        print(f"Max x: {self.box_constrains[0]}, max y: {self.box_constrains[1]}, max z: {self.box_constrains[2]} \n"
              f"min x: {self.box_constrains[3]}, min y: {self.box_constrains[4]}, min z: {self.box_constrains[5]}")

        self.pose_target.position.x = x
        self.pose_target.position.y = y
        self.pose_target.position.z = z
        self.group.set_pose_target(self.pose_target)

        plan = self.group.go(wait=True)
        self.finish_movement(plan)

    def finish_movement(self, finish_plan):
        """
        NOT USED. If a trajectory was wanted, here would be where it would be executed.

        :param finish_plan:
        :return:
        """
        self.group.stop()
        self.group.clear_pose_targets()


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

        self.x = 320.0
        self.y = 180.0
        self.z = 0.8

        self.orientation_dictionary = {5: [320, 180, 0.80], 8: [325, 180, 0.80]}

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

        return depth_image_flipped, processing_images, color_images_rgb

    def return_finger_positon(self, hand_to_inspect, flipped_hand_position_image, position_landmark):

        hand_landmark_extraction = hand_to_inspect.landmark[position_landmark]
        x = int(hand_landmark_extraction.x * len(flipped_hand_position_image[0]))
        y = int(hand_landmark_extraction.y * len(flipped_hand_position_image))
        if x >= len(flipped_hand_position_image[0]):
            x = len(flipped_hand_position_image[0]) - 1
        if y >= len(flipped_hand_position_image):
            y = len(flipped_hand_position_image) - 1
        mfk_distance = flipped_hand_position_image[y, x] * self.depth_scale  # meters

        return x, y, mfk_distance

    def hand_processing(self, depth_frame, color_frame):
        hand_depth_image_flipped, hand_images, hand_color_images_rgb = self.frame_processing(depth_frame, color_frame)
        results = self.hands.process(hand_color_images_rgb)
        if results.multi_hand_landmarks:
            number_of_hands = len(results.multi_hand_landmarks)
            i = 0
            for handLms in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(hand_images, handLms, self.mpHands.HAND_CONNECTIONS)
                hand_side_classification_list = results.multi_handedness[i]
                hand_side = hand_side_classification_list.classification[0].label
                hand_inspection = results.multi_hand_landmarks[i]
                self.x, self.y, self.z = self.return_finger_positon(hand_inspection, hand_depth_image_flipped, 9)

                self.draw_hand_and_comments(hand_side, hand_images, number_of_hands, i)
                i += 1
        else:
            self.hand_images = cv2.putText(hand_images, "No Hands", self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

    def give_robot_orientation(self, depth_frame, color_frame):
        hand_depth_image_flipped, hand_images, hand_color_images_rgb = self.frame_processing(depth_frame, color_frame)
        results = self.hands.process(hand_color_images_rgb)
        if results.multi_hand_landmarks:
            number_of_hands = len(results.multi_hand_landmarks)
            i = 0
            for handLms in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(hand_images, handLms, self.mpHands.HAND_CONNECTIONS)
                hand_side_classification_list = results.multi_handedness[i]
                hand_side = hand_side_classification_list.classification[0].label
                hand_inspection = results.multi_hand_landmarks[i]
                for position_landmark_finger in self.orientation_dictionary.keys():
                    self.orientation_dictionary[position_landmark_finger] = (
                        list(self.return_finger_positon(hand_inspection, hand_depth_image_flipped,
                                                        position_landmark_finger)))
                self.x, self.y, self.z = self.orientation_dictionary[8]
                self.draw_hand_and_comments(hand_side, hand_images, number_of_hands, i)

        else:
            self.hand_images = cv2.putText(hand_images, "No Hands", self.org, self.font, self.fontScale, self.color,
                                           self.thickness, cv2.LINE_AA)

    def draw_hand_and_comments(self, hand_type, draw_image, hand_count, i):
        org2 = (20, self.org[1] + (20 * (i + 1)))
        draw_image= cv2.putText(draw_image,
                                  f"{hand_type} Hand Distance: {self.z:0.3} meters away on position x:{self.x} and y:{self.y}",
                                  org2, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
        draw_image = cv2.putText(draw_image, f"Hands: {hand_count}", self.org, self.font, self.fontScale,
                                  self.color, self.thickness,
                                  cv2.LINE_AA)
        cv2.rectangle(draw_image, (145, 115), (502, 262), (0, 255, 0), 2)
        cv2.putText(draw_image, f"65 cm from camera, highest point on the robot.", (20, 135), self.font, self.fontScale, self.color, self.thickness,
                    cv2.LINE_AA)
        cv2.rectangle(draw_image, (198, 153), (446, 253), (255, 0, 0), 2)

        self.hand_images = draw_image
