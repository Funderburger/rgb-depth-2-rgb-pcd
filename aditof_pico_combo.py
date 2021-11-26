#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, BatteryState
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import message_filters
from std_msgs.msg import String 


ir_pub = None
rgb_pub = None 

bridge = CvBridge()

# from pynput import keyboard
# 
# def on_press(key):
#     try:
#         print('Alphanumeric key pressed: {0} '.format(
#             key.char))
#     except AttributeError:
#         print('special key pressed: {0}'.format(
#             key))
# with keyboard.Listener(on_press=on_press) as listener:
#         print("callback_sync")
#         listener.join()
# Collect events until released


def callback_sync(rgb_msg, ir_msg, pico_ir_msg, pico_rgb_msg):
    global ir_pub
    global rgb_pub
    global nr


    folder = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/p0018_and_pico/test/"

    ir = bridge.imgmsg_to_cv2(ir_msg,desired_encoding="passthrough")
    # cv2.imshow(ir,"ceva")

    rgb_not = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough") #BAYER_BGGR16    
    # rgb = np.frombuffer(rgb_msg.data,np.uint16).reshape(rgb_msg.height,rgb_msg.width,1)
    rgb = cv2.cvtColor(rgb_not, cv2.COLOR_BayerBG2RGB)

    pico_ir = bridge.imgmsg_to_cv2(pico_ir_msg,desired_encoding="passthrough")
    pico_rgb = bridge.imgmsg_to_cv2(pico_rgb_msg,desired_encoding="passthrough")



    # alpha = 0.005 # Contrast control (1.0-3.0)
    # beta = 1 # Brightness control (0-100)
    # ir_cont = cv2.convertScaleAbs(ir, alpha=alpha, beta=beta)
    
    alpha = 0.01 # Simple contrast control
    beta = 1    # Simple brightness control
    rgb_cont = cv2.convertScaleAbs(rgb, alpha=alpha, beta=beta)
    
    print("callback")
    # if save_image_msg.present == True:
    # cv2.imwrite(folder+"p0018/" +"ir_p0018_"+ str(nr) + ".png", ir_cont)
    # cv2.imwrite(folder+"p0018/" +"rgb_p0018_"+ str(nr) + ".png", rgb_cont)
    # cv2.imwrite(folder+"pico/" +"ir_pico_"+ str(nr) + ".png", pico_ir)
    # cv2.imwrite(folder+"pico/" +"rgb_pico_"+ str(nr) + ".png", pico_rgb)
    # nr += 1

    cv2.imwrite(folder+"p0018/" +"depth_p0018_"+ str(nr) + ".png", ir)
    cv2.imwrite(folder+"p0018/" +"rgb_p0018_"+ str(nr) + ".png", rgb_cont)
    cv2.imwrite(folder+"pico/" +"depth_pico_"+ str(nr) + ".png", pico_ir)
    cv2.imwrite(folder+"pico/" +"rgb_pico_"+ str(nr) + ".png", pico_rgb)
    nr += 1
        
    # elif key == 27:
    #     break
    # else:
    #     continue


    # cv2.waitKey(1000)


def start_node():
    global rgb_pub
    global ir_pub
    global nr

    rospy.init_node('ir_rgb_brightner')
    rospy.loginfo('Brightening 2 frames: rgb and IR')
    nr = 0

    # ir_pub = rospy.Publisher('/aditof_roscpp/aditof_ir_contrast', Image, queue_size=1)
    # rgb_pub = rospy.Publisher('/aditof_roscpp/aditof_rgb_contrast', Image, queue_size=1)
    
    
    ir_sub = message_filters.Subscriber("/aditof_roscpp/aditof_depth", Image)
    rgb_sub = message_filters.Subscriber("/aditof_roscpp/aditof_rgb", Image)
    pico_ir_sub = message_filters.Subscriber("/pico_zense/depth/image_raw", Image)
    pico_rgb_sub = message_filters.Subscriber("/pico_zense/colour/image_raw", Image) 
    save_image_sub = message_filters.Subscriber("/take_pics", BatteryState)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, ir_sub, pico_ir_sub, pico_rgb_sub], 1, 1, allow_headerless=False)
    
    ts.registerCallback(callback_sync)


    # print("end start node")
    

if __name__ == '__main__':
    try:
        start_node()
        # while(True):
        #     key = cv2.waitKey(0)
        #     print(key)
        #     if key == 112:
        #         print("ceva")
        #     elif key == 27:
        #         break
        # print("end")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass