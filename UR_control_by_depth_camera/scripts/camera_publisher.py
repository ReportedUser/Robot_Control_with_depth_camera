#!/usr/bin/env python3
import numpy as np
import rospy
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs

model_path="/home/david/Documents/mediapipe/handlandmarker/hand_landmarker.task"

publisherNodeName='camera_sensor_publisher'



topicName='video_topic'
rospy.init_node(publisherNodeName, anonymous=True)
publisher=rospy.Publisher(topicName, Image, queue_size=60)
rate=rospy.Rate(15)

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


def print_result(result: vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    print('hand landmarker result: {}'.format(result))


# video_capture_object=cv2.VideoCapture(2)

bridge_object=CvBridge()
base_options = python.BaseOptions(model_asset_path= model_path)
options = vision.HandLandmarkerOptions(base_options = base_options, running_mode=vision.RunningMode.LIVE_STREAM, result_callback=print_result)


pipe.start(cfg)

with vision.HandLandmarker.create_from_options(options) as landmarker:

    time_stamp = 0
    while not rospy.is_shutdown():
        time_stamp += 1

        frame = pipe.wait_for_frames()
        color_frame = frame.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        transformed_color_image = mp.Image(image_format=mp.ImageFormat.SRGB, data = color_image)

        landmarker.detect_async(transformed_color_image, time_stamp)

        cv2.imshow('rgb', color_image)

        if cv2.waitKey(1) == ord('q'):
            break


        """
        boolean_captured_frame, capturedFrame = video_capture_object.read()

        capturedFrame = mp.Image(image_format=mp.ImageFormat.SRGB, data=capturedFrame)

        landmarker.detect_async(capturedFrame, time_stamp)
        
        
        if boolean_captured_frame == True:
            rospy.loginfo('Video frame captured and published')
            publisher.publish()
        """

        rate.sleep()

pipe.stop()