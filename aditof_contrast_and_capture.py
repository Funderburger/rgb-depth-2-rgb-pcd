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

ir_pub = None
rgb_pub = None 

bridge = CvBridge()
    

def callback_sync(rgb_msg, ir_msg, depth_msg):
    global ir_pub
    global rgb_pub
    global nr

    folder = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0003/capturi/"

    ir = bridge.imgmsg_to_cv2(ir_msg,desired_encoding="passthrough")


    rgb_not = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough") #BAYER_BGGR16    
    # rgb = np.frombuffer(rgb_msg.data,np.uint16).reshape(rgb_msg.height,rgb_msg.width,1)
    rgb = cv2.cvtColor(rgb_not, cv2.COLOR_BayerBG2RGB)

    depth = bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")

    # cv2.imwrite(folder+"ir/" +"ir_p0018_"+ str(nr) + ".png", ir)
    # # cv2.imwrite(folder+"test/" +"depth_p0018_"+ str(nr) + ".png", depth)
    # cv2.imwrite(folder+"test/" +"rgb_p0018_"+ str(nr) + ".png", rgb)
    # cv2.imwrite(folder+"rgb/" + "rgb_p0018_"+ str(nr) + ".png", rgb)
    # nr += 1

    # alpha = 0.005 # Contrast control (1.0-3.0)
    # beta = 1 # Brightness control (0-100)
    # ir_cont = cv2.convertScaleAbs(ir, alpha=alpha, beta=beta)
    # ir_cont_msg = bridge.cv2_to_imgmsg(ir_cont,"mono8")
    # ir_cont_msg.header = ir_msg.header

    alpha = 0.01 # Simple contrast control
    beta = 1    # Simple brightness control
    rgb_cont = cv2.convertScaleAbs(rgb, alpha=alpha, beta=beta)
    # rgb_cont = cv2.resize(rgb_cont, (640,480))
    rgb_cont_msg = bridge.cv2_to_imgmsg(rgb_cont,encoding="bgr8")
    rgb_cont_msg.header = rgb_msg.header

    # cv2.imwrite(folder +"ir_p0018_"+ str(nr) + ".png", ir)
    # cv2.imwrite(folder +"depth_p0018_"+ str(nr) + ".png", depth)
    # cv2.imwrite(folder +"rgb_p0018_"+ str(nr) + ".png", rgb_cont)

    cv2.imwrite(folder +"ir/ir_p0003_"+ str(nr) + ".png", ir)
    # cv2.imwrite(folder +"depth/depth_p0018_"+ str(nr) + ".png", depth)
    cv2.imwrite(folder +"rgb/rgb_p0003_"+ str(nr) + ".png", rgb_cont)

    nr+=1
    # rgb_pub.publish(rgb_cont_msg)
    # ir_pub.publish(ir_cont_msg)

    # cv2.waitKey(1000)


def start_node():
    global rgb_pub
    global ir_pub
    global nr

    rospy.init_node('ir_rgb_brightner')
    rospy.loginfo('Brightening 2 frames: rgb and IR')
    nr =0

    ir_pub = rospy.Publisher('/aditof_roscpp/aditof_ir_contrast', Image, queue_size=1)
    rgb_pub = rospy.Publisher('/aditof_roscpp/aditof_rgb_contrast', Image, queue_size=1)
    
    
    ir_sub = message_filters.Subscriber("/aditof_roscpp/aditof_ir", Image)
    rgb_sub = message_filters.Subscriber("/aditof_roscpp/aditof_rgb", Image)
    depth_sub = message_filters.Subscriber("/aditof_roscpp/aditof_depth", Image)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, ir_sub, depth_sub], 1, 0.1, allow_headerless=False)
    ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass