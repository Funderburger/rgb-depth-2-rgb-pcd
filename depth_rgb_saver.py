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

def special_scale_RGB(image):
    rgb_image = np.float64(np.copy(image))
    minVal = rgb_image.min()
    maxVal = rgb_image.max()
    minAvg = 0
    maxAvg = 512
    minAvg = minAvg * 0.9 + minVal * 0.1
    maxAvg = maxAvg * 0.9 + maxVal * 0.1
    rgb_image -= minAvg / 2
    rgb_image *= 65535 / (maxAvg - minAvg / 2)
    avg = cv2.mean(rgb_image)
    rgb_image *= 32768.0 / avg[0]
    return np.uint16(rgb_image)
    
    

def callback_sync(rgb_msg, depth_msg):
    # global ir_pub
    # global depth_pub
    global nr

    folder = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0018_v3/capturi/"

    depth = bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")


    rgb_not = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough") #BAYER_BGGR16    

    # rgb = special_scale_RGB(rgb_not)

    rgb = cv2.cvtColor(rgb_not, cv2.COLOR_BayerBG2RGB)
    # rgb = np.frombuffer(rgb_msg.data,np.uint16).reshape(rgb_msg.height,rgb_msg.width,1)
    # cv2.imwrite(folder+"ir/" +"ir_p0018_"+ str(nr) + ".png", depth)
    cv2.imwrite(folder+"test/" +"depth_p0018_"+ str(nr) + ".png", depth)
    cv2.imwrite(folder+"test/" +"rgb_p0018_"+ str(nr) + ".png", rgb)
    # cv2.imwrite(folder+"rgb/" + "rgb_p0018_"+ str(nr) + ".png", rgb)

    cv2.waitKey(1)
    nr += 1
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
    global nr

    rospy.init_node('ir_rgb_saver')
    rospy.loginfo('Save 2 frames: rgb and IR')
    nr =0
    # ir_pub = rospy.Publisher('/pico_IR_sync', Image, queue_size=1)
    # depth_pub = rospy.Publisher('/pico_rgba8_depth_sync', Image, queue_size=1)
    
    rgb_sub = message_filters.Subscriber("/aditof_roscpp/aditof_rgb", Image)
    
    ir_sub = message_filters.Subscriber("/aditof_roscpp/aditof_depth", Image)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, ir_sub], 1, 0.1, allow_headerless=False)
    ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass