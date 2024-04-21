import sys
import rospy
import moveit_commander
from moveit_msgs.msg import Constraints
import geometry_msgs.msg
from scipy.spatial.transform import Rotation


import mediapipe as mp
import numpy as np
import cv2

def transformation_to_ur_coordinates(trans_x, trans_y, trans_z):
    decimal_number = 2

    if 0.95 >= trans_z >= 0.65:
        trans_z = 0.1 + (trans_z - 0.65)
    elif trans_z > 0.95:
        trans_z = 0.4
    elif trans_z < 0.65:
        trans_z = 0.1

    if trans_y <= -0.2:
        trans_y = 0.4
    elif trans_y >= 0.05:
        trans_y = 0.15
    else:
        trans_y = ((trans_y-0.05)/(-0.2-0.05))*(0.4-0.15)+0.15

    if trans_x > 0.3:
        trans_x = 0.3
    elif trans_x < -0.3:
        trans_x = -0.3

    return round(trans_x, decimal_number),round(trans_y, decimal_number), round(trans_z, decimal_number)


class RobotClass:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_planning_time(5)
        # display_trajectory_publisher = rospy.Publisher("/scaled_pos_joint_traj_controller/follow_joint_trajectory",
        # moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.upright_constraints = Constraints()
        self.upright_constraints.name = "upright"

        self.pose_target = geometry_msgs.msg.Pose()

        self.euler2quaternion(180, 0, 0)
        self.robot_information()

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

    def euler2quaternion(self, i, j, k):
        """
        takes roll, pitch and yaw and transforms to quaternion
        if no values are given, uses 180, 0 and 0 by default
        """
        rot = Rotation.from_euler('xyz', [i, j, k], degrees=True)
        quaternions = rot.as_quat()
        self.pose_target.orientation.x = quaternions[0]
        self.pose_target.orientation.y = quaternions[1]
        self.pose_target.orientation.z = quaternions[2]
        self.pose_target.orientation.w = quaternions[3]

    def move_to_position(self, x, y, z):

        x, y, z = self.box_limits(x, y, z)

        print("x:",x)
        print("y:",y)
        print("z:", z)

        self.pose_target.position.x = x
        self.pose_target.position.y = y
        self.pose_target.position.z = z
        self.group.set_pose_target(self.pose_target)

        plan = self.group.go(wait=True)
        self.finish_movement(plan)

    def finish_movement(self, finish_plan):
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

        self.x = 11.0
        self.y = 11.0
        self.z = 21.0

        self.orientation_dictionary = {}

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
        pass
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
                for position_landmark_finger in [5, 8]:
                    self.orientation_dictionary[f"position {position_landmark_finger}"] = (
                        list(self.return_finger_positon(hand_inspection, hand_depth_image_flipped, 9)))
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
