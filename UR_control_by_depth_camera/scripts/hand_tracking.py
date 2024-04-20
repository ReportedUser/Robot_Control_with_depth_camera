#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
import datetime as dt

from UR_control_by_depth_camera.robot_classes import RobotClass, HandDetection

"""
class RGBDCamera:
    def __init__(self, res_x, res_y, camera_fps):
        realsense_ctx = rs.context()
        self.intrinsics = rs.intrinsics()
        self.device = realsense_ctx.devices[0].get_info(rs.camera_info.serial_number)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.device)
        self.config.enable_stream(rs.stream.depth, res_x, res_y, rs.format.z16, camera_fps)
        self.profile = self.pipeline.start(self.config)

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        profile_stream = self.profile.get_stream(rs.stream.depth)
        print(profile_stream.as_video_stream_profile().get_intrinsics())
        self.intrinsics.width = 640
        self.intrinsics.height = 480
        self.intrinsics.ppx = 321.454
        self.intrinsics.ppy = 232.919
        self.intrinsics.fx = 388.656
        self.intrinsics.fy = 388.656
        self.intrinsics.model = rs.distortion.brown_conrady
        self.intrinsics.coeffs = [0, 0, 0, 0, 0]
"""
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

    result = rs.rs2_deproject_pixel_to_point(intrinsics, [hand.x, hand.y], hand.z)

    print(f"Current values are; \n x: {result[0]}, y: {result[1]}, z: {result[2]}")
    x, y, z = transformation_to_ur_coordinates(result[0], result[1], result[2])

    name_of_window = 'Camera being used: ' + str(device)

    # Display images
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)

    # Move to position
    ur3.move_to_position(x, y, z)

    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")
