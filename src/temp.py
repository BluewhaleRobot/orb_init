#!/usr/bin/env python
#coding:utf-8
'''
This is used to scale the orb odometry. We will move xiaoqiang around and
calculate the distance it has moved by odometry. Scale the orb output by compairing
with it. Boardcast a tf from camera to base_link.Because in ros ,tf only means
rigid body transform ,so we also publish the scale from camera world to real world.
'''
#已知R 已知T 计算SCALE（多次路程比平均）
# /orb2base_scale
# odom_combined--->odom_combined_new
# ORB_SLAM/world_new--->ORB_SLAM/world
# base_link--->camera
# ORB_SLAM/camera--->base_footprint会等价于ORB_SLAM/world_new--->odom_combined_new
import rospy
from std_msgs.msg import String, UInt32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Pose
from sensor_msgs.msg import Image
from system_monitor.msg import *
from system_monitor.msg import Status
import math
import threading
import os
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as  np
import scipy.linalg

getTrackThread = None
scaleOrbThread = None
scaleOrbFlag = False
scaleStatusPub = ""
velPub = ""
camOdomPub = ""
transform = None
lisener=None
odom_combined_tf=None
scale = 0.
prviousCamPose = None
prviousstamp = None
tf_rot=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
tf_trans=np.array([0.0195,0.0227,0.])
# tf_rot=np.array([[-0.204581198792803,0.001599309043100,0.978848290242714],
#                 [-0.978741427909822,-0.015200270566771,-0.204534029119214],
#                 [0.014551645733118,-0.999883190170373,0.004674999591037]])
# tf_trans=np.array([0.0024,0.0025,0.])
tf_rda=np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
tf_tda=np.array([0.,0.,0.])
tf_rbc=np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
tf_tbc=np.array([0.,0.,0.])

# try to get track again when losing track
import threading

def publish_Tfs():
    global tf_rot,tf_trans,tf_tda,tf_rda,tf_tbc,tf_rbc,scale
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        rate.sleep()
        camOdom = Odometry()
        camOdom.header.stamp = rospy.Time.now()
        camOdom.header.frame_id = "/odom_combined"
        camOdom.pose.pose.position.x =0
        camOdom.pose.pose.position.y =0
        camOdom.pose.pose.position.z =0. #Tbc[2]
        M=np.identity(4)
        q=tf.transformations.quaternion_from_matrix(M)
        camOdom.pose.pose.orientation.x = q[0]
        camOdom.pose.pose.orientation.y = q[1]
        camOdom.pose.pose.orientation.z = q[2]
        camOdom.pose.pose.orientation.w = q[3]
        camOdom.child_frame_id = "/base_footprint"
        camOdom.twist.twist.linear.x = 0
        camOdom.twist.twist.linear.y = 0
        camOdom.twist.twist.angular.z = 0
        T=np.array([0.,0.,0.])
        M=np.identity(4)
        q=tf.transformations.quaternion_from_matrix(M)
        br.sendTransform(T,q,
            rospy.Time.now(),
            "/base_footprint",
            "/odom_combined")
        camOdomPub.publish(camOdom)



def init():
    global scaleStatusPub, camOdomPub,lisener,scaleOrbFlag,scale,odom_combined_tf
    global velPub
    rospy.init_node("orb_scale", anonymous=True)
    camOdomPub = rospy.Publisher("/ORB_SLAM/Odom", Odometry, queue_size=5)
    t=threading.Thread(target=publish_Tfs)
    t.start()

if __name__ == "__main__":
    init()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
