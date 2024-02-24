#!/usr/bin/env python3

import rospy
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

model_path="/home/david/Documents/mediapipe/handlandmarker/hand_landmarker.task"

publisherNodeName='camera_sensor_publisher'

topicName='video_topic'
rospy.init_node(publisherNodeName, anonymous=True)
publisher=rospy.Publisher(topicName, Image, queue_size=60)
rate=rospy.Rate(15)


def print_result(result: vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    print('hand landmarker result: {}'.format(result))


video_capture_object=cv2.VideoCapture(2)
bridge_object=CvBridge()
base_options = python.BaseOptions(model_asset_path= model_path)
options = vision.HandLandmarkerOptions(base_options = base_options, running_mode=vision.RunningMode.LIVE_STREAM, result_callback=print_result)


with vision.HandLandmarker.create_from_options(options) as landmarker:

    time_stamp = 0
    while not rospy.is_shutdown():
        time_stamp += 1

        boolean_captured_frame, capturedFrame = video_capture_object.read()

        capturedFrame = mp.Image(image_format=mp.ImageFormat.SRGB, data=capturedFrame)

        landmarker.detect_async(capturedFrame, time_stamp)
        if boolean_captured_frame == True:
            rospy.loginfo('Video frame captured and published')
            publisher.publish()
        rate.sleep()