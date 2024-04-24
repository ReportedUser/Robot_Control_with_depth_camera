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
ur3 = RobotClass()


def position_to_coordinates(position_dictionary):
    """
    position_dictionary: gets a dictionary where there are 2 hand landmarks.
    Keys are hand landmark and values are a list of i, j and k positions.
    Edits the hand.orientation_dictionary to get the deproject values for both points.
    Returns the landmark of 8 (fingertip), where the TCP will move to.
    """

    for position_landmarks in position_dictionary.keys():
        position_dictionary[position_landmarks] = rs.rs2_deproject_pixel_to_point(
            intrinsics, [position_dictionary[position_landmarks][0],
                         position_dictionary[position_landmarks][1]],
            position_dictionary[position_landmarks][2]
        )
        print(f"Current values for {position_landmarks} are; \n "
              f"x: {position_dictionary[position_landmarks][0]}, "
              f"y: {position_dictionary[position_landmarks][1]}, "
              f"z: {position_dictionary[position_landmarks][2]}."
              )

    coordinates_x, coordinates_y, coordinates_z = transformation_to_ur_coordinates(
        position_dictionary[8][0],
        position_dictionary[8][1],
        position_dictionary[8][2]
    )

    return coordinates_x, coordinates_y, coordinates_z


def orientation_tracking(orientation_dictionary):
    """
    3rd point has 5 x and 8 y
    """

    mcp_index_finger_position = orientation_dictionary[5]
    tip_index_finger_position = orientation_dictionary[8]
    """
    reference_point = [mcp_index_finger_position[0], tip_index_finger_position[1]]

    try:
        to_tan = ((reference_point[1] - mcp_index_finger_position[1]) /
                  (tip_index_finger_position[0] - reference_point[0]))
        orientation_quaternion_x = round(math.degrees(math.tan(to_tan)), 0)
        return orientation_quaternion_x
    except:
        print("Can't reach that position.")
    """
    vector = np.array(tip_index_finger_position) - np.array(mcp_index_finger_position)
    vector_norm = vector / np.linalg.norm(vector)
    reference_x = np.array([1, 0, 0])

    v = np.cross(reference_x, vector_norm)
    s = np.linalg.norm(v)
    c = np.dot(reference_x, vector_norm)
    skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    r = np.eye(3) + skew + np.dot(skew, skew) * ((1 - c)/(s**2))

    # Extract Euler angles
    yaw = np.arctan2(r[1, 0], r[0, 0])
    pitch = np.arcsin(-r[2, 0])
    roll = np.arctan2(r[2, 1], r[2, 2])

    yaw_deg = np.degrees(yaw)
    pitch_deg = np.degrees(pitch)
    roll_deg = np.degrees(roll)
    return pitch_deg, yaw_deg, roll_deg


x_ant = y_ant = z_ant = 100
roll_ant = yawn_ant = 0
pitch_ant = 180
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
    x, y, z = position_to_coordinates(hand.orientation_dictionary)
    pitch_euler, yaw_euler, roll_euler = orientation_tracking(hand.orientation_dictionary)

    print(f"Pitch: {pitch_euler}, Yaw: {yaw_euler}, Roll: {roll_euler}")

    name_of_window = 'Camera being used: ' + str(device)

    # Display images
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)

    if abs(pitch_euler - pitch_ant) > 3 or abs(yaw_euler - yawn_ant) > 3 or abs(roll_euler - roll_ant) > 3:
        ur3.euler2quaternion(pitch_euler, yaw_euler, roll_euler)
        pitch_ant = pitch_euler
        yawn_ant = yaw_euler
        roll_ant = roll_euler

    if abs(x - x_ant) > 0.02 or abs(y - y_ant) > 0.02 or abs(z - z_ant) > 0.02:
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