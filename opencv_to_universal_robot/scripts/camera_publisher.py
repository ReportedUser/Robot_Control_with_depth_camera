#!/usr/bin/env python3

import rospy
import mediapipe as mp

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

publisherNodeName='camera_sensor_publisher'

topicName='video_topic'

rospy.init_node(publisherNodeName, anonymous=True)
publisher=rospy.Publisher(topicName, Image, queue_size=60)
rate=rospy.Rate(15)


hands = mp.solutions.hands
drawing = mp.solutions.drawing_utils

video_capture_object=cv2.VideoCapture(0)
bridge_object=CvBridge()

while not rospy.is_shutdown():
    returnValue, capturedFrame = video_capture_object.read()
    capturedFrame = cv2.cvtColor(capturedFrame, cv2.COLOR_BGR2RGB)
    captured_frame_with_hand = hands.Hands().process(capturedFrame)
    capturedFrame = cv2.cvtColor(capturedFrame, cv2.COLOR_RGB2BGR)

    if captured_frame_with_hand.multi_hand_landmarks:
        for hand_recognized in captured_frame_with_hand.multi_hand_landmarks:
            drawing.draw_landmarks(capturedFrame, hand_recognized, connections=hands.HAND_CONNECTIONS)

    if returnValue == True:
        rospy.loginfo('Video frame captured and published')
        image_to_transmit = bridge_object.cv2_to_imgmsg(capturedFrame)
        publisher.publish(image_to_transmit)

    rate.sleep()
