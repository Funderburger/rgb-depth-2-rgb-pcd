#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import message_filters
import std_msgs.msg

from numpy import loadtxt
from struct import pack, unpack


ir_pub = None
rgb_pub = None 

bridge = CvBridge()
    

def callback_sync(): #rgb_msg, pcd_msg, depth_cam_info_msg):

    pcd = loadtxt("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb_pcd/p0025_cam_pcd.txt", dtype=np.float64, delimiter=" ", unpack=False)

    rgb_img = cv2.imread("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0025/test/res-rgb_p0025_0.png", -1)

    # rgb_not = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough") #BAYER_BGGR16    
    # # rgb = np.frombuffer(rgb_msg.data,np.uint16).reshape(rgb_msg.height,rgb_msg.width,1)
    # rgb = cv2.cvtColor(rgb_not, cv2.COLOR_BayerBG2RGB)


    Krgb = np.array([[314.100533382509,             0,            321.814357865278],
                     [0,                    419.882158000649,     236.903650998016],
                     [0,                            0,                   1        ]],np.float64)

    R = np.array([[ 0.999926329273159,   0.000776708049317,  -0.012113329472651],
                  [-0.000902746171809,   0.999945480696576,  -0.010402917083351],
                  [ 0.012104589032931,   0.010413085954697,   0.999872515156429]],np.float64)
    
    t = np.array([-36.560085894521158,  -0.888879973399496,  -3.214890777135737],np.float64)
    t = np.transpose(t)



    n_points= pcd.shape[0]

    scan3d = pcd
    # transform in mm
    scan3d = pcd*1000
    xyz = scan3d[:,:3]
    # xyz = scan3d(:,1:3);
    
    P = np.matmul(np.matmul(Krgb,R),np.concatenate((np.identity(3),np.array([t]).T),axis=1))
    abc = np.matmul(P, np.concatenate((xyz, np.ones((n_points,1))),axis=1).T)
    ab = np.zeros((3, n_points))
    ab = np.round(abc/abc[2,:])
    ab[0,:] = np.where(ab[1,:] <= 0, 100, ab[0,:])
    ab[0,:] = np.where(ab[0,:] <= 0, 100, ab[0,:])
    ab[1,:] = np.where(ab[0,:] <= 0, 100, ab[1,:])
    ab[1,:] = np.where(ab[1,:] <= 0, 100, ab[1,:])
    
    scan=np.fliplr(np.round(ab[:2,:].T))
    

    print()

    scanRGB = np.zeros((n_points,3))
    imrows = rgb_img.shape[0]
    imcols = rgb_img.shape[1]

    inliers = np.where((scan[:,0]>0) & (scan[:,0]<imrows) & (scan[:,1]>0) & (scan[:,1]<imcols))

    # ???????????????????????????????????????
    inliers_index = np.ravel_multi_index((np.array(scan[inliers,0].T,np.int), np.array(scan[inliers,1].T,np.int)), dims=(imrows, imcols), order='F')
    
    # flip the image BGR -> RGB
    rgb_img = rgb_img[:, :, ::-1]
    
    # reshape the image from imrows*imcols*3 to (imrows*imcols)*3
    rgb_img = np.reshape(rgb_img,(imrows*imcols,3),order="F").copy()
    
    scanRGB[inliers[0],:] = rgb_img[inliers_index,:].transpose((0,2,1)).squeeze()

    scanRGB = np.array(scanRGB,np.uint32)

    print("Writing pcd...")

    scanRGB_ok = (scanRGB[:,0] << 16) | (scanRGB[:,1] << 8) | scanRGB[:,2]

    buf = pack('%sI' % len(scanRGB_ok), *scanRGB_ok)
    scanRGB_float = unpack('%sf' % len(scanRGB_ok), buf)

    rgb_pcd = np.concatenate((scan3d,np.array([scanRGB_float]).T),axis=1)

    pcd_file = open("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb-depth-2-rgb-pcd/catkin_ws/python_rgb_pcd_p0025.pcd","w")
    np.savetxt(pcd_file, rgb_pcd, delimiter=" ")

    # put this at the top of your pcd file
    # .PCD v.7 - Point Cloud Data file format
    # VERSION .7
    # FIELDS x y z rgb
    # SIZE 4 4 4 4
    # TYPE F F F F
    # COUNT 1 1 1 1
    # WIDTH 307200
    # HEIGHT 1
    # VIEWPOINT 0 0 0 1 0 0 0
    # POINTS 307200
    # DATA ascii

    # alpha = 0.01 # Simple contrast control
    # beta = 1    # Simple brightness control
    # rgb_cont = cv2.convertScaleAbs(rgb, alpha=alpha, beta=beta)
    # rgb_cont = cv2.resize(rgb_cont, (640,480))
    # rgb_cont_msg = bridge.cv2_to_imgmsg(rgb_cont,encoding="bgr8")
    # rgb_cont_msg.header = rgb_msg.header


def start_node():

    callback_sync()
    # rospy.init_node('point_cloud_xyzrgb_genesis')
    # rospy.loginfo('Conceiving point_cloud_xyzrgb')
    
    # depth_camera_info = message_filters.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo)
    # rgb_sub = message_filters.Subscriber("/aditof_roscpp/aditof_rgb", Image)
    # pcd_sub = message_filters.Subscriber("/aditof_roscpp/aditof_pcloud", PointCloud)

    # ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, pcd_sub, depth_camera_info], 1, 0.1, allow_headerless=False)
    # ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass