#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import message_filters
import std_msgs.msg

# ir_pub = None
# depth_pub = None

bridge = CvBridge()

def callback_sync(rgb_msg, depth_msg):
    # global ir_pub
    # global depth_pub
    folder = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb_pcd/"

    depth = bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")
    rgb = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough")
    cv2.imwrite(folder+"depth_pico_5.png", depth)
    cv2.imwrite(folder+"rgb_pico_5.png", rgb)

    cv2.waitKey(1)
    # height = frame.shape[0]
    # width = frame.shape[1]
    # rgba_img = np.zeros((height,width,4), np.uint8)
    # rgba_img[:,:,:] = [0,0,0,255]
    
    # valid = frame != 0                                                                                                                  
    # thousands_digits = frame/1000
    # thousands = thousands_digits * 1000
    # tens_digits = (frame - thousands) / 10
    # units_digits = frame - thousands - tens_digits*10 

    # rgba_img[:,:,0] = np.where(valid,thousands_digits,0)
    # rgba_img[:,:,1] = np.where(valid,tens_digits,0)
    # rgba_img[:,:,2] = np.where(valid,units_digits,0)

    # scaled_image = bridge.cv2_to_imgmsg(frame)
    # scaled_image.header = camera_info.header

    # depth_pub.publish(bridge.cv2_to_imgmsg(rgba_img,"rgba8"))
    # ir_pub.publish(ir_msg)


def start_node():
    global depth_pub
    global ir_pub

    rospy.init_node('depth_rgb_saver')
    rospy.loginfo('Save 2 frames: rgb and IR')

    # ir_pub = rospy.Publisher('/pico_IR_sync', Image, queue_size=1)
    # depth_pub = rospy.Publisher('/pico_rgba8_depth_sync', Image, queue_size=1)
    
    rgb_sub = message_filters.Subscriber("/pico_zense/colour/image_raw", Image)
    depth_sub = message_filters.Subscriber("/pico_zense/depth/image_raw", Image)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.2, allow_headerless=False)
    ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass