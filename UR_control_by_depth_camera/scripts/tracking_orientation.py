from typing import List
from statistics import mean
import cv2
import numpy as np
import pyrealsense2 as rs
import datetime as dt
from scipy.spatial.transform import Rotation
from UR_control_by_depth_camera.robot_classes import RobotClass, HandDetection, transformation_to_ur_coordinates


box_limits = (-0.2, 0.2, 0.4, -0.35, -0.2, 0.25)
original_x = -0.25
original_y = 0
original_z = 0.3
original_roll = original_yawn = 0
original_pitch = 180

realsense_ctx = rs.context()
intrinsics = rs.intrinsics()
device = realsense_ctx.devices[0].get_info(rs.camera_info.serial_number)
pipeline = rs.pipeline()
config = rs.config()

stream_res_x = 640
stream_res_y = 480
stream_fps = 30

config.enable_device(device)
config.enable_stream(rs.stream.depth, stream_res_x, stream_res_y, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_res_x, stream_res_y, rs.format.bgr8, stream_fps)

profile = pipeline.start(config)
profile_stream = profile.get_stream(rs.stream.depth)
print(profile_stream.as_video_stream_profile().get_intrinsics())

align_to = rs.stream.color
align = rs.align(align_to)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

intrinsics.width = 640
intrinsics.height = 480
intrinsics.ppx = 321.454
intrinsics.ppy = 232.919
intrinsics.fx = 388.656
intrinsics.fy = 388.656
intrinsics.model = rs.distortion.brown_conrady
intrinsics.coeffs = [0, 0, 0, 0, 0]

print(f"\tDepth Scale for Camera SN {device} is: {depth_scale}")
print(f"\tConfiguration Successful for SN {device}")
print(f"Starting to capture images on SN: {device}")

hand = HandDetection(depth_scale)
ur3 = RobotClass("manipulator", box_limits)

previous_direction = None
ur3.move_to_position(original_x, original_y, original_z)
ur3.euler2quaternion(original_roll, original_pitch, original_yawn)


def distorted_to_position(input_position: List) -> List:
    """
    Input: positions from the rgbd camera.
    Output: positions in a 3D space.
    """
    pos_x, pos_y, pos_z = rs.rs2_deproject_pixel_to_point(
        intrinsics, [input_position[0], input_position[1]], input_position[2])

    return [pos_x, pos_y, pos_z]


def orientation_tracking(orientation_dictionary, last_direction=None):
    """
    Getting rotations pitch, roll and yawn from vector made with fingers points.
    """

    mcp_index_finger_position = orientation_dictionary["base"]
    tip_index_finger_position = orientation_dictionary["tip"]

    vector = (np.array(tip_index_finger_position)+1) - (np.array(mcp_index_finger_position)+1)
    direction_norm = np.linalg.norm(vector)

    if direction_norm != 0:
        vector = vector / direction_norm
    reference_x = np.array([0, 0, 1])

    # reference_x = last_direction if last_direction is not None else np.array([0,0, 1])

    cross = np.cross(reference_x, vector)
    dot = np.dot(reference_x, vector)
    skew_symmetric_cross = np.array([[0, -cross[2], cross[1]], [cross[2], 0, -cross[0]], [-cross[1], cross[0], 0]])
    rotation_matrix = np.eye(3) + skew_symmetric_cross + np.dot(skew_symmetric_cross
                                                                , skew_symmetric_cross) * (1 / (1 + dot))

    # Extract Euler angles
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    pitch = np.arcsin(-rotation_matrix[2, 0])
    roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    yaw_deg = np.degrees(yaw)
    pitch_deg = np.degrees(pitch)
    roll_deg = np.degrees(roll)
    return roll_deg, pitch_deg, yaw_deg, vector


filter_position_list = {"x": [original_x], "y": [original_y], "z": [original_z]}
filter_angle_list = {"roll": [original_roll], "pitch": [original_pitch], "yawn": [original_yawn]}

"""
def position_and_angle_filters(position: List, angles: List):

    current_x = position[0]
    current_y = position[1]
    current_z = position[2]

    current_roll = angles[0]
    current_pitch = angles[1]
    current_yawn = angles[2]

    if all(abs(current_yawn - roll) < 5 for roll in filter_angle_list["yawn"]):
            # all(abs(yawn < 5) for yawn in filter_angle_list["yawn"])):
        ur3.euler2quaternion(0, 0, current_yawn)
        print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n changed angle \n XXXXXXXXXXXXXXXXXXXXXXXXXXXX")

    if (all(abs(current_x - filter_x) < 0.02 for filter_x in filter_position_list["x"])
            or all(abs(current_y - filter_y) < 0.02 for filter_y in filter_position_list["y"])
            or all(abs(current_z - filter_z) < 0.02 for filter_z in filter_position_list["z"])):
        ur3.move_to_position(current_x, current_y, current_z)

    filter_angle_list["roll"].append(current_roll)
    filter_angle_list["pitch"].append(current_pitch)
    filter_angle_list["yawn"].append(current_yawn)

    if len(filter_angle_list["roll"]) >= 5:
        del filter_angle_list["roll"][0]
        del filter_angle_list["pitch"][0]
        del filter_angle_list["yawn"][0]

    filter_position_list["x"].append(current_x)
    filter_position_list["y"].append(current_y)
    filter_position_list["z"].append(current_z)

    if len(filter_position_list["x"]) >= 5:
        del filter_position_list["x"][0]
        del filter_position_list["y"][0]
        del filter_position_list["z"][0]
"""


def create_vector(p1, p2):
    return np.array(p2) - np.array(p1)


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        raise ValueError("Nope")
    return v / norm


def vector_to_quaternion(current_vector, new_vector):
    current_vector = normalize_vector(current_vector)
    new_vector = normalize_vector(new_vector)

    axis = np.cross(current_vector, new_vector)
    axis = normalize_vector(axis)

    dot_product = np.dot(current_vector, new_vector)
    angle = np.arccos(dot_product)

    q_w = np.cos(angle / 2)
    q_x, q_y, q_z = axis * np.sin(angle / 2)

    return np.array([q_w, q_x, q_y, q_z])


quaternion_list = []


class QuaternionFilters:
    def __init__(self, threshold):
        self.threshold = threshold
        self.quaternions = []

    def add_quaternion(self, new_quaternion):
        if len(self.quaternions) < 10:
            self.quaternions.append(new_quaternion)
            return self.mean_quaternions()
        else:
            # differences = np.array([np.linalg.norm(new_quaternion -q) for q in self.quaternions])
            # if np.all(differences < self.threshold):
            self.quaternions.pop(0)
            self.quaternions.append(new_quaternion)
            return self.mean_quaternions()

    def mean_quaternions(self):
        if len(self.quaternions) == 0:
            return None
        elif len(self.quaternions) == 1:
            return self.quaternions[0]
        else:
            rotations = Rotation.from_quat(self.quaternions)
            mean_rotations = rotations.mean()
            return mean_rotations.as_quat()


name_of_window = 'Camera being used: ' + str(device)
original_dict = 0

q_filter = QuaternionFilters(threshold=0.1)

while True:
    start_time = dt.datetime.today().timestamp()

    # Get and align frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    input_color_image = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not input_color_image:
        continue

    hand.give_robot_orientation(aligned_depth_frame, input_color_image)
    images = hand.hand_images
    # print(hand.orientation_dictionary[8], hand.orientation_dictionary[5])

    orientation_dict = {"tip": distorted_to_position(hand.orientation_dictionary[8]),
                        "base": distorted_to_position(hand.orientation_dictionary[5])}
    x, y, z = transformation_to_ur_coordinates(orientation_dict["base"][0], orientation_dict["base"][1],
                                               orientation_dict["base"][2])
    roll_euler, pitch_euler, yaw_euler, previous_direction = orientation_tracking(
        orientation_dict, previous_direction)

    print(orientation_dict)
    # print(f"x:{x}, y:{y}, z: {z}")
    print(f"Roll: {roll_euler}, Pitch: {pitch_euler}, Yaw: {yaw_euler}")

    # Display images
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)

    if type(original_dict) is int:
        original_dict = orientation_dict
        continue

    if all(value == 0 for value in orientation_dict["base"]) or all(value == 0 for value in orientation_dict["tip"]):
        continue

    new_orientation = {"base": [orientation_dict["base"][1], orientation_dict["base"][0], orientation_dict["base"][2]],
                       "tip": [orientation_dict["tip"][1], orientation_dict["tip"][0], orientation_dict["tip"][2]]
                       }

    if abs(x - original_x) > 0.02 or abs(y - original_y) > 0.02 or abs(z - original_z) > 0.02:
        ur3.move_to_position(x, y, z)
        original_x = x
        original_y = y
        original_z = z

    # create_vector(original_dict["base"], original_dict["tip"]),
    quaternion_pls = vector_to_quaternion(
        np.array([0, 0, 1]),
        create_vector(new_orientation["base"], new_orientation["tip"])
        )

    quaternion_to_move = q_filter.add_quaternion(quaternion_pls)
    print("#########@########")
    with open("/home/david/Documents/quaternions_info/quaternions_log_lastvector_to_quaternionrotation.txt", "a") as file:
        file.write(str(quaternion_to_move))
    print(quaternion_pls)
    print("#########@########")

    ur3.define_quaternions(quaternion_to_move)

    original_dict = orientation_dict

    # position_and_angle_filters([x, y, z], [roll_euler, pitch_euler, yaw_euler])

    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")
