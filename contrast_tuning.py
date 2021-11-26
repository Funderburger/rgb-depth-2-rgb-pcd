#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

net = None
args = None
pub = None


def process_image(msg):
    global net
    global args
    global pub
    global output
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    try:
        pass
    except Exception as err:
        print(err)

    # equ = cv2.equalizeHist()
    alpha = 0.10 # Contrast control (1.0-3.0)
    beta = 0.5 # Brightness control (0-100)
    frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
    frame_msg = bridge.cv2_to_imgmsg(frame,"mono8")
    frame_msg.header = msg.header
    pub.publish(frame_msg)


def start_node():
    global pub
    rospy.init_node('contrast')
    rospy.loginfo('contrast started')
    pub = rospy.Publisher('/aditof_roscpp/aditof_contrast_ir', Image, queue_size=1)
    rospy.Subscriber("/aditof_roscpp/aditof_ir", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
