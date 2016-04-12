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

class getTrack(threading.Thread):

    def __init__(self):
        super(getTrack, self).__init__()
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global velPub, getTrackThread
        if velPub == "":
            # driver is not ready
            getTrackThread = None
            return
        print "start get track"
        CarTwist = Twist()
        # rotate small angels
        CarTwist.linear.x = 0.0
        CarTwist.linear.y = 0.0
        CarTwist.linear.z = 0.0
        CarTwist.angular.x = 0.0
        CarTwist.angular.y = 0.0
        CarTwist.angular.z = 0.3
        velPub.publish(CarTwist)
        self.wait(6)
        CarTwist.angular.z = -0.3
        velPub.publish(CarTwist)
        self.wait(6)
        CarTwist.angular.z = 0.
        velPub.publish(CarTwist)
        getTrackThread = None
        print "stop get track"

    def wait(self, waitTime):
        sleepTime = 0
        while sleepTime < waitTime and not self.stopped():
            time.sleep(0.05)
            sleepTime += 0.05

# scale orb odometry after get track

class scaleOrb(threading.Thread):

    def __init__(self):
        super(scaleOrb, self).__init__()
        self._stop = threading.Event()
        self.startPose = None
        self.startCamera = None
        self.endPose = None
        self.endCamera = None
        self.currentPose = None
        self.currentCamera = None
        self.Pose=None
        self.Camera=None
        self.currentPoseLock = threading.Lock()
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global velPub, scaleOrbThread, scaleOrbFlag, scale,tf_rot,tf_trans
        if velPub == "":
            # driver is not ready
            scaleOrbThread = None
            return

        def setCurrentPose(pose):
            self.currentPoseLock.acquire()
            self.currentPose = pose
            self.currentPoseLock.release()

        def setCurrentCamera(pose):
            self.currentPoseLock.acquire()
            self.currentCamera = pose
            self.currentPoseLock.release()

        poseSub = rospy.Subscriber("/xqserial_server/Pose2D", Pose2D, setCurrentPose)
        cameraSub = rospy.Subscriber("/ORB_SLAM/Camera", Pose, setCurrentCamera)
        print "start scale orb"
        vel_angle=math.pi*0.25*0.5
        vel_x=0.1
        step_angle=40.0*2.0/180.0*math.pi
        step_len=0.2*4
        t_forward=step_len/vel_x
        t_circle=step_angle/vel_angle
        t_forward1=step_len/4.0*math.sqrt(2)*0.5/vel_x
        t_circle1=math.pi*0.25/vel_angle

        #先小半径移动两圈，使orb_slam建立经过loop closed的local map，提高准确度
        CarTwist = Twist()
        # stop the car
        CarTwist.linear.x = 0.0
        CarTwist.linear.y = 0.0
        CarTwist.linear.z = 0.0
        CarTwist.angular.x = 0.0
        CarTwist.angular.y = 0.0
        CarTwist.angular.z = 0.0
        velPub.publish(CarTwist)
        self.wait(1)
        #进入准备位置
        CarTwist.linear.x = 0.0
        CarTwist.angular.z = vel_angle
        velPub.publish(CarTwist)
        self.wait(t_circle1)
        CarTwist.linear.x = vel_x
        CarTwist.angular.z = 0.0
        velPub.publish(CarTwist)
        self.wait(t_forward1)
        #开始转圈
        for i in range(9):
            CarTwist.linear.x = 0.0
            CarTwist.angular.z = vel_angle
            velPub.publish(CarTwist)
            self.wait(t_circle)
            CarTwist.linear.x = vel_x
            CarTwist.angular.z = 0.0
            velPub.publish(CarTwist)
            self.wait(t_forward/4.0)
            # CarTwist.linear.x = 0.0
            # CarTwist.angular.z = 0.0
            # velPub.publish(CarTwist)
            # self.wait(1)
        #开始记录pose对
        CarTwist.linear.x = 0.0
        CarTwist.angular.z = 0.0
        velPub.publish(CarTwist)
        self.wait(2)

        while not self.stopped():
            self.wait(0.5)
            self.currentPoseLock.acquire()
            if self.currentPose != None and self.currentCamera != None:
                self.startPose = self.currentPose
                self.startCamera = self.currentCamera
                self.currentPoseLock.release()
                break
            self.currentPoseLock.release()

        CarTwist.linear.x = vel_x
        velPub.publish(CarTwist)
        self.wait(t_forward)
        CarTwist.linear.x = 0.
        velPub.publish(CarTwist)
        # get current pose and camera pose
        self.wait(2)
        self.currentPoseLock.acquire()
        self.endPose = self.currentPose
        self.endCamera = self.currentCamera
        self.currentPoseLock.release()
        # calculate scale
        posDis = math.sqrt(math.pow(self.startPose.x - self.endPose.x, 2) +
            math.pow(self.startPose.y - self.endPose.y, 2))
        camDis = math.sqrt(math.pow(self.startCamera.position.x - self.endCamera.position.x, 2) +
            math.pow(self.startCamera.position.y - self.endCamera.position.y, 2)+
            math.pow(self.startCamera.position.z - self.endCamera.position.z, 2))
        scale = posDis / camDis
        print "scale: " + str(scale)
        rospy.set_param('/orb2base_scale',scale)
        #保存变量到文件
        file_path=os.path.split(os.path.realpath(__file__))[0]
        fp3=open("%s/scale.txt"%(file_path),'a+')
        fp3.write(str(scale))
        fp3.write('\n')
        fp3.close

        scaleOrbThread = None
        poseSub.unregister()
        cameraSub.unregister()
        scaleOrbFlag = True
        print "stop scale orb"

    def wait(self, waitTime):
        sleepTime = 0.
        while sleepTime < waitTime and not self.stopped():
            time.sleep(0.05)
            sleepTime += 0.05

def systemStatusHandler(system_status):
    global getTrackThread, scaleOrbFlag, scaleOrbThread

    # wait until motor odometry is ready
    if not system_status.odomStatus:
        return
    # start get track
    if system_status.orbStartStatus and not system_status.orbInitStatus and getTrackThread == None:
        if scaleOrbThread !=None:
            scaleOrbThread.stop()
        getTrackThread = getTrack()
        getTrackThread.start()
        #scaleOrbFlag = False
        return
    # already tracked, stop thread
    if system_status.orbInitStatus and getTrackThread != None:
        getTrackThread.stop()

    # start scale orb thread
    if system_status.orbInitStatus and not scaleOrbFlag and scaleOrbThread == None :
        scaleOrbThread = scaleOrb()
        scaleOrbThread.start()
        return
    # already scaled, stop thread
    if scaleOrbFlag and scaleOrbThread != None:
        scaleOrbThread.stop()

def cameraOdom(campose):
    global camOdomPub,lisener,scale, prviousCamPose,prviousstamp,tf_rot,tf_trans,odom_combined_tf
    if camOdomPub == "" or scale <= 0.000001 or not scaleOrbFlag:
        return

    Tad=np.array([campose.position.x,campose.position.y,campose.position.z])
    q=[campose.orientation.x,campose.orientation.y,campose.orientation.z,campose.orientation.w]
    M=tf.transformations.quaternion_matrix(q)
    Rad=M[:3,:3]
    #为了简化计算，下文的计算中base_link 和base_footprint被看成是相同的坐标系
    #转到odom_combined，得到camera在odom_combined中的pose
    Rbd=tf_rot.dot(Rad)
    Tbd=scale*(tf_rot.dot(Tad))+tf_trans
    #由camera 的 pose 得到 base_footprint 的pose，这也是下文要发布的pose
    Rdc=tf_rot.T
    Tdc=-1/scale*(Rdc.dot(tf_trans))
    Rbc=Rbd.dot(Rdc)
    Tbc=Rbd.dot(Tdc)+Tbd

    camOdom = Odometry()
    camOdom.header.stamp = rospy.Time.now()
    camOdom.header.frame_id = "/odom_combined"
    camOdom.pose.pose.position.x =Tbc[0]
    camOdom.pose.pose.position.y =Tbc[1]
    camOdom.pose.pose.position.z =0. #Tbc[2]
    M=np.identity(4)
    M[:3,:3]=Rbc
    q=tf.transformations.quaternion_from_matrix(M)
    camOdom.pose.pose.orientation.x = q[0]
    camOdom.pose.pose.orientation.y = q[1]
    camOdom.pose.pose.orientation.z = q[2]
    camOdom.pose.pose.orientation.w = q[3]
    var_len=math.pow(0.01,2)
    var_angle=math.pow(0.01*math.pi/180.0,2)
    camOdom.pose.covariance=[var_len,0.,0.,0.,0.,0.,
                             0.,var_len,0.,0.,0.,0.,
                             0.,0.,999.,0.,0.,0.,
                             0.,0.,0.,999.,0.,0.,
                             0.,0.,0.,0.,999.,0.,
                             0.,0.,0.,0.,0.,var_angle]
    # #publish tf:/odom_combined-->/base_footprint
    # if odom_combined_tf != None:
    #     T=Tbc
    #     odom_combined_tf.sendTransform(T,q,
    #         now,
    #         "/odom_combined",
    #         "/base_footprint")

    #publish /base_footprint-->/odom_combined
    if odom_combined_tf != None:
        Rcb=Rbc.T
        T=-Rcb*Tbc
        M[:3,:3]=Rcb
        q=tf.transformations.quaternion_from_matrix(M)
        odom_combined_tf.sendTransform(T,q,
            now,
            "/base_footprint",
            "/odom_combined")
    #计算速度
    camOdom.child_frame_id = "/base_footprint"

    if prviousCamPose == None:
        prviousCamPose=campose
    Tad_old=np.array([prviousCamPose.position.x,prviousCamPose.position.y,prviousCamPose.position.z])
    q=[prviousCamPose.orientation.x,prviousCamPose.orientation.y,prviousCamPose.orientation.z,prviousCamPose.orientation.w]
    M=tf.transformations.quaternion_matrix(q)
    Rad_old=M[:3,:3]
    #获取camera到base_footprint的转换关系
    if lisener == None:
        return
    try:
        (trans,rot) = lisener.lookupTransform("base_footprint", "camera", rospy.Time(0))
    except (tf.LookupException,tf.ConnectivityException):
        return
    T=np.array(trans)
    M=tf.transformations.quaternion_matrix(rot)
    R=M[:3,:3]
    #获取base_footprint在ORB_SLAM/world中的值
    Rac=Rad.dot(R.T)
    Tac=Rad.dot(-1.0/scale*(R.T.dot(T)))+Tad

    Rda_old=Rad_old.T
    Tda_old=-Rda_old.dot(Tad_old)

    Rca_old=R.dot(Rda_old)
    Tca_old=scale*R.dot(Tda_old)+T

    #base_footprint本体坐标系下的增量
    delta_T=scale*(Rca_old.dot(Tac))+Tca_old
    delta_R=Rca_old.dot(Rac)


    M=np.identity(4)
    M[:3,:3]=delta_R
    delta_q=tf.transformations.quaternion_from_matrix(M)
    delta_angle=euler_from_quaternion(delta_q)
    #转成速度
    if prviousstamp ==None:
        prviousstamp=camOdom.header.stamp
    dt = camOdom.header.stamp.to_sec() - prviousstamp.to_sec()
    prviousstamp=camOdom.header.stamp
    if dt>=0.0000001:
        camOdom.twist.twist.linear.x = delta_T[0]/dt
        camOdom.twist.twist.linear.y = delta_T[1]/dt
        camOdom.twist.twist.angular.z = delta_angle[2]/dt
    var_len=math.pow(0.006,2)
    var_angle=math.pow(0.1*math.pi/180.0,2)
    camOdom.twist.covariance=[var_len,0.,0.,0.,0.,0.,
                             0.,var_len,0.,0.,0.,0.,
                             0.,0.,999.,0.,0.,0.,
                             0.,0.,0.,999.,0.,0.,
                             0.,0.,0.,0.,999.,0.,
                             0.,0.,0.,0.,0.,var_angle]
    prviousCamPose = campose
    camOdomPub.publish(camOdom)




def publish_Tfs():
    global tf_rot,tf_trans,tf_tda,tf_rda,tf_tbc,tf_rbc,scale
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        rate.sleep()
        if not scaleOrbFlag or scale <=0.000001:
            continue
        now=rospy.Time.now();
        R=tf_rot
        T=tf_trans
        M=np.identity(4)
        M[:3,:3]=R
        q=tf.transformations.quaternion_from_matrix(M)
        br.sendTransform(T,q,
            now,
            "camera",
            "base_link")

def init():
    global scaleStatusPub, camOdomPub,lisener,scaleOrbFlag,scale,odom_combined_tf
    global velPub
    rospy.init_node("orb_scale", anonymous=True)
    scaleOrbFlag=rospy.get_param('/orb2base_scaleFlag',False)
    if scaleOrbFlag:
        #load scale value from scale.txt
        file_path=os.path.split(os.path.realpath(__file__))[0]
        fp3=open("%s/scale.txt"%(file_path),'r+')
        for line in fp3:
            value_list=line.split(" ")
        scale=float(value_list[0])
        if scale<=0.000001:
            scale=0.
            scaleOrbFlag=False
        else:
            rospy.set_param('/orb2base_scale',scale)
            fp3.close
        print "scale: "+str(scale)
    lisener=tf.TransformListener()
    odom_combined_tf=tf.TransformBroadcaster()
    rospy.Subscriber("/system_monitor/report", Status, systemStatusHandler)
    scaleStatusPub = rospy.Publisher('/orb_scale/scaleStatus', Bool , queue_size=5)
    camOdomPub = rospy.Publisher("/ORB_SLAM/Odom", Odometry, queue_size=5)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, cameraOdom)
    velPub = rospy.Publisher('/cmd_vel', Twist , queue_size=5)
    t=threading.Thread(target=publish_Tfs)
    t.start()

if __name__ == "__main__":
    init()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if scaleStatusPub != "":
            scaleStatusPub.publish(scaleOrbFlag)
        else:
            scaleStatusPub.publish(False)
        rate.sleep()
