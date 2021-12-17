#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2

import cv2

import numpy as np
import message_filters
from std_msgs.msg import Header

# from cv_bridge import CvBridge

import struct

import ros_numpy # sudo apt install ros-<distro>-ros-numpy

# bridge = CvBridge()

def callback_sync(rgb_msg, pcd_msg, rgb_cam_info_msg):
    
    # rgb_not = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough") #BAYER_BGGR16    
    # rgb = cv2.cvtColor(rgb_not, cv2.COLOR_BayerBG2RGB) 
    rgb_img = np.frombuffer(rgb_msg.data, np.uint16).reshape(rgb_msg.height, rgb_msg.width, 3) # alternative to bridge
    
    rgb_img_unscaled = rgb_img
    
    min_value = 0
    max_value = 255

    rgb_img = (max_value - min_value)*((rgb_img_unscaled-rgb_img_unscaled.min())/(rgb_img_unscaled.max()- rgb_img_unscaled.min())) + min_value

    rgb_img = cv2.resize(rgb_img, (640,480))

    points = ros_numpy.numpify(pcd_msg)
    # resize because it is an ordered point cloud
    height = points.shape[0]
    width = points.shape[1]
    pcd_points = np.zeros((height*width, 3), dtype=np.float32)
    pcd_points[:, 0] = np.resize(points['x'], height * width)
    pcd_points[:, 1] = np.resize(points['y'], height * width)
    pcd_points[:, 2] = np.resize(points['z'], height * width)

    Krgb = np.array([[314.100533382509,             0,            321.814357865278],
                     [0,                    419.882158000649,     236.903650998016],
                     [0,                            0,                   1        ]],np.float64)

    R = np.array([[ 0.999926329273159,   0.000776708049317,  -0.012113329472651],
                  [-0.000902746171809,   0.999945480696576,  -0.010402917083351],
                  [ 0.012104589032931,   0.010413085954697,   0.999872515156429]],np.float64)
    
    t = np.array([-36.560085894521158,  -0.888879973399496,  -3.214890777135737],np.float64)
    t = np.transpose(t) / 1000.0

    print("Computing rgb pcd...")

    n_points= pcd_points.shape[0]

    # transform in mm
    scan3d = pcd_points
    xyz = scan3d[:,:3]
    
    P = np.matmul(np.matmul(Krgb,R),np.concatenate((np.identity(3),np.array([t]).T),axis=1))
    abc = np.matmul(P, np.concatenate((xyz, np.ones((n_points,1))),axis=1).T)
    ab = np.zeros((3, n_points))
    ab = np.round(abc/abc[2,:])
    ab[0,:] = np.where(ab[1,:] <= 0, 100, ab[0,:])
    ab[0,:] = np.where(ab[0,:] <= 0, 100, ab[0,:])
    ab[1,:] = np.where(ab[0,:] <= 0, 100, ab[1,:])
    ab[1,:] = np.where(ab[1,:] <= 0, 100, ab[1,:])
    
    scan=np.fliplr(np.round(ab[:2,:].T))


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

    # a = 255
    # rgb = struct.unpack('I', struct.pack('BBBB', scanRGB[2], scanRGB[1], scanRGB[0], a))[0]

    scanRGB_ok = (scanRGB[:,0] << 16) | (scanRGB[:,1] << 8) | scanRGB[:,2]
    buf = struct.pack('%sI' % len(scanRGB_ok), *scanRGB_ok)
    scanRGB_float = struct.unpack('%sf' % len(scanRGB_ok), buf)
    # pcd_rgb = np.concatenate((scan3d,np.array([scanRGB_float]).T),axis=1)
    
    scanRGB_ok = tuple(scanRGB_ok)

    pcd_rgb_not_yet = np.array(([xyz[:,0], xyz[:,1], xyz[:,2], scanRGB_float])).T

    pcd_rgb = list(pcd_rgb_not_yet)
    header.stamp = rospy.Time.now()
    rgb_pcloud_msg = point_cloud2.create_cloud(header, fields, pcd_rgb)

    rgbpcd_pub.publish(rgb_pcloud_msg)

    

def start_node():
    global rgbpcd_pub
    global rgb_pub
    global fields
    global header


    rospy.init_node('point_cloud_xyzrgb_genesis')
    rospy.loginfo('Conceiving point_cloud_xyzrgb')
    
    rgb_sub = message_filters.Subscriber("/aditof_roscpp/image_rect_color", Image)
    pcd_sub = message_filters.Subscriber("/aditof_roscpp/aditof_pcloud", PointCloud2)

    rgbpcd_pub = rospy.Publisher("/aditof_roscpp/aditof_pcloud_rgb", PointCloud2,queue_size=5)

    rgb_pub = rospy.Publisher("aditof_roscpp/aditof_rgb_rect_res", Image, queue_size=5)

    # for rgb pcd publisher
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 16, PointField.FLOAT32, 1),]

    header = Header()
    header.frame_id = "base_link"

    rgb_camera_info = message_filters.Subscriber("/aditof_roscpp/aditof_rgb_camera_info", CameraInfo)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, pcd_sub, rgb_camera_info], 1, 0.1, allow_headerless=False)
    ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass