from typing import Tuple, List

import cv2
import numpy as np
import pyrealsense2 as rs
import datetime as dt
from UR_control_by_depth_camera.robot_classes import RobotClass, HandDetection, transformation_to_ur_coordinates

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
"""
stream_res_x = 640
stream_res_y = 480
stream_fps = 30
intel_camera = RGBDCamera(stream_res_x, stream_res_y, stream_fps)
"""

print(f"\tDepth Scale for Camera SN {device} is: {depth_scale}")
print(f"\tConfiguration Successful for SN {device}")
print(f"Starting to capture images on SN: {device}")


hand = HandDetection(depth_scale)
ur3 = RobotClass("manipulator")


def distorted_to_position(input_position: List) -> Tuple[float, float, float]:
    """
    Input: positions from the rgbd camera.
    Output: positions in a 3D space.
    """
    pos_x, pos_y, pos_z = rs.rs2_deproject_pixel_to_point(
        intrinsics, [input_position[0], input_position[1]], input_position[2])

    return pos_x, pos_y, pos_z


def orientation_tracking(orientation_dictionary, last_direction=None):
    """
    Getting rotations pitch, roll and yawn from vector made with fingers points.
    """

    mcp_index_finger_position = orientation_dictionary[5]
    tip_index_finger_position = [orientation_dictionary[8][1], orientation_dictionary[8][0], orientation_dictionary[8][2]]

    vector = np.array(tip_index_finger_position) - np.array(mcp_index_finger_position)
    direction_norm = np.linalg.norm(vector)

    if direction_norm != 0:
        vector = vector / direction_norm
    reference_x = np.array([0, 0, 1])

    # reference_x = last_direction if last_direction is not None else vector

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


name_of_window = 'Camera being used: ' + str(device)
x_ant = 0.2
y_ant = 0
z_ant = 0.3
roll_ant = yawn_ant = 0
pitch_ant = 180
previous_direction = None
ur3.move_to_position(x_ant, y_ant, z_ant)
ur3.euler2quaternion(pitch_ant, yawn_ant, roll_ant)

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
    print(hand.orientation_dictionary[8], hand.orientation_dictionary[5])

    x, y, z = distorted_to_position(hand.orientation_dictionary[8])
    x, y, z = transformation_to_ur_coordinates(x, y, z)
    roll_euler, pitch_euler, yaw_euler, previous_direction = orientation_tracking(
        hand.orientation_dictionary, previous_direction)

    print(f"x:{x}, y:{y}, z: {z}")
    print(f"Pitch: {pitch_euler}, Yaw: {yaw_euler}, Roll: {roll_euler}")

    # Display images
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)

    if abs(pitch_euler - pitch_ant) > 3 or abs(yaw_euler - yawn_ant) > 3 or abs(roll_euler - roll_ant) > 3:
        ur3.euler2quaternion(roll_euler, pitch_euler, yaw_euler)
        pitch_ant = pitch_euler
        yawn_ant = yaw_euler
        roll_ant = roll_euler

    if abs(x - x_ant) > 0.02 or abs(y - y_ant) > 0.02 or abs(z - z_ant) > 0.02 or abs(z - z_ant) < 0.3:
        ur3.move_to_position(x, y, z)
        x_ant = x
        y_ant = y
        z_ant = z

    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")
