#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2

subscriber_node_name='camera_sensor_subscriber'

topic_name='video_topic'


def callbackfunction(message):
    bridge_object=CvBridge()
    rospy.loginfo('received a video message/frame')
    converted_frame_back_to_cv=bridge_object.imgmsg_to_cv2(message)
    cv2.imshow('camera', converted_frame_back_to_cv)
    cv2.waitKey(1)


rospy.init_node(subscriber_node_name, anonymous=True)
rospy.Subscriber(topic_name, Image, callbackfunction)

rospy.spin()
cv2.destroyAllWindows()
