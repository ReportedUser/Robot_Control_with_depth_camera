import cv2
import pyrealsense2 as rs
import datetime as dt
import mediapipe as mp
import numpy as np

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint
import geometry_msgs.msg

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
        pose_target.orientation.w = 1.0
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
    def __init__(self):

        #mediapipe hand recognition variables
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands()
        self.mpDraw = mp.solutions.drawing_utils


    def frame_processing(self, depth_frame, color_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image_flipped = cv2.flip(depth_image, 1)
        color_image = np.asanyarray(color_frame.get_data())

        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image))  # Depth image is 1 channel, while color image is 3
        background_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0),
                                      background_removed_color, color_image)

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
                org2 = (20, org[1] + (20 * (i + 1)))
                hand_side_classification_list = results.multi_handedness[i]
                hand_side = hand_side_classification_list.classification[0].label
                middle_finger_knuckle = results.multi_hand_landmarks[i].landmark[9]
                x = int(middle_finger_knuckle.x * len(hand_depth_image_flipped[0]))
                y = int(middle_finger_knuckle.y * len(hand_depth_image_flipped))
                if x >= len(hand_depth_image_flipped[0]):
                    x = len(hand_depth_image_flipped[0]) - 1
                if y >= len(hand_depth_image_flipped):
                    y = len(hand_depth_image_flipped) - 1
                mfk_distance = hand_depth_image_flipped[y, x] * depth_scale  # meters
                hand_images = cv2.putText(hand_images,
                                     f"{hand_side} Hand Distance: {mfk_distance:0.3} meters away on position x:{x} and y:{y}",
                                     org2, font, fontScale, color, thickness, cv2.LINE_AA)

                self.x = x
                self.y = y
                self.z = mfk_distance

                i += 1
            hand_images = cv2.putText(hand_images, f"Hands: {number_of_hands}", org, font, fontScale, color, thickness,
                                 cv2.LINE_AA)
        else:
            hand_images = cv2.putText(hand_images, "No Hands", org, font, fontScale, color, thickness, cv2.LINE_AA)

        self.hand_images = hand_images



font = cv2.FONT_HERSHEY_SIMPLEX
org = (10, 10)
fontScale = .5
color = (0, 50, 255)
thickness = 1

# ====== Realsense ======
realsense_ctx = rs.context()
device = realsense_ctx.devices[0].get_info(rs.camera_info.serial_number)
pipeline = rs.pipeline()
config = rs.config()
background_removed_color = 153  # Grey


# ====== Enable Streams ======
config.enable_device(device)

# # For worse FPS, but better resolution:
# stream_res_x = 1280
# stream_res_y = 720
# # For better FPS. but worse resolution:
stream_res_x = 640
stream_res_y = 480

stream_fps = 30

config.enable_stream(rs.stream.depth, stream_res_x, stream_res_y, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_res_x, stream_res_y, rs.format.bgr8, stream_fps)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# ====== Get depth Scale ======
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"\tDepth Scale for Camera SN {device} is: {depth_scale}")

# ====== Set clipping distance ======
clipping_distance_in_meters = 1
clipping_distance = clipping_distance_in_meters / depth_scale
print(f"\tConfiguration Successful for SN {device}")

# ====== Get and process images ======
print(f"Starting to capture images on SN: {device}")

hand = HandDetection()
ur3 = RobotClass()
z=0

while True:
    start_time = dt.datetime.today().timestamp()

    # Get and align frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    input_color_image = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not input_color_image:
        continue

    hand.frame_processing(aligned_depth_frame, input_color_image)
    images = hand.hand_images
    if 0.95 >= hand.z >= 0.65:
        z = 0.1 + (hand.z - 0.65)
    elif hand.z > 0.95:
        z = 0.4
    elif hand.z < 0.65:
        z = 0.1

    x_screen = ((z-0.1)/(0.4-0.1))*(560-468)+468
    y_screen = ((z-0.1)/(0.4-0.1))*(206-129)+129

    x = (hand.x-320)*(0.6/x_screen)
    y = hand.y*(0.25/y_screen)

    ur3.move_to_position(x, y, z)

    time_diff = dt.datetime.today().timestamp() - start_time
    fps = int(1 / time_diff)
    org3 = (20, org[1] + 60)
    images = cv2.putText(images, f"FPS: {fps}", org3, font, fontScale, color, thickness, cv2.LINE_AA)

    name_of_window = 'SN: ' + str(device)

    # Display images
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.rectangle(images, (40, 137), (600, 343), (0, 255, 0), 2)
    cv2.putText(images, f"65 cm from camera, highest point on the robot.", (20, 135), font, fontScale, color, thickness,
                cv2.LINE_AA)
    cv2.rectangle(images, (86, 175), (554, 305), (255, 0, 0), 2)
    cv2.imshow(name_of_window, images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")
