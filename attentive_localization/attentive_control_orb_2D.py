#!/usr/bin/env python

import sys, os
attentive_dir = os.getcwd()
sys.path.append("/home/flo/ma-florian/attentive_localization/attentive_control_helper/")
#sys.path.append(attentive_dir+"/attentive_control_helper/")

import helper_FUNCTIONS as HELPER
import subscriber_ORB as ORB
from movement_helper import movement

import rospy
import numpy as np
import time
from collections import deque
import matplotlib.pyplot as plt
import yaml
import pprint
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped


xoffset = 0
yoffset = 0

def queueStable(q):
    if len(q) != qlen:
        return False
    for i in range(1,len(q)):
        if (q[0] != q[i]):
            return False
    return True

def getStableBins(xcurr, ycurr):
    xQueue.append(xcurr)
    yQueue.append(ycurr)

    if queueStable(xQueue) and queueStable(yQueue):
        xstable = xQueue[len(xQueue)-2]
        ystable = yQueue[len(xQueue)-2]
        return xstable, ystable, True
    else:
        return False, False, False

def adaptiveQueue(qlen, counter):
    if counter > cfg[modus]['ADAP_SPEED']:#args.ADAP_SPEED: #must be bigger then arg.modus(qlen)!!!
    #if qlen is de/inc: init new q(maxlen=qlen-1)
        qlen -=1
        print "Adaptive Stability decreased: ", qlen
        counter = 0
        if qlen <= 1:
            return 1, 10, False
        return qlen, counter, True
    else:
        return qlen, counter, False

def move_camera(yaw, pitch, xoffset, yoffset, dist):
    #dist = 1.5
    xlimit = cfg[modus]['MAX_YAW']
    ylimit = cfg[modus]['MAX_PITCH']

    if yaw > -xlimit and yaw < xlimit:
        if yaw > -120 and yaw < 120:
            #hardware limitations
            mov.move_robot_x(yaw, xoffset,dist)
            xoffset = yaw
    if pitch > -ylimit and pitch < ylimit:
        if pitch > -36 and pitch < 60:
            #hardware limitations
            #neg pitch means up for robot!
            mov.move_robot_y(pitch, yoffset,dist)
            yoffset = pitch
    return xoffset, yoffset

orb_features    = None
orb_cloud       = None
orb_pose        = None

def callback_pixel(data):
    global orb_features
    orb_features = data.points

def callback_cloud(data):
    global orb_cloud
    #cloud = data.points.position.x
    orb_cloud = data.points


def callback_pose(data):
    global orb_pose
    orb_pose = data#.pose.position

def feature_control():
    rospy.init_node('attentive_localization_feature', anonymous=True)

    print "ORBSLAM MODUS - loading params:"
    global cfg, modus
    #with open(attentive_dir+"/config.yaml", 'r') as ymlfile:
    with open("/home/flo/ma-florian/attentive_localization/config.yaml", 'r') as ymlfile:
       cfg = yaml.load(ymlfile)

    modus           = 'ORB'
    X_DIM           = cfg[modus]['X_DIM']
    Y_DIM           = cfg[modus]['Y_DIM']
    X_BINS          = cfg[modus]['X_BINS']
    Y_BINS          = cfg[modus]['Y_BINS']
    ADAP_QLEN       = cfg[modus]['ADAP_QLEN']
    ADAP_SPEED      = cfg[modus]['ADAP_SPEED']
    PLOT_WEIGHT_MAT = cfg[modus]['PLOT_WEIGHT_MAT']
    ADAP_CAM_SPEED  = cfg[modus]['ADAP_CAM_SPEED']
    FIX_CAM_SPEED   = cfg[modus]['FIX_CAM_SPEED']
    MAX_YAW         = cfg[modus]['MAX_YAW']
    MAX_PITCH       = cfg[modus]['MAX_PITCH']

    pprint.pprint(cfg[modus])

    degz = pitch_init
    degx = yaw_init
    xoffset = yaw_init
    yoffset = pitch_init
    mov = movement(degz, degx)
    rate = rospy.Rate(10)


    qcounter = 0
    global args
    global qlen
    qlen = ADAP_QLEN
    global xQueue, yQueue
    xQueue = deque( maxlen=qlen )
    yQueue = deque( maxlen=qlen )
    qlenDefault = qlen
    no_data_flag = 0
    #X_BINS = 8
    #Y_BINS = 6
    #X_DIM  = 640
    #Y_DIM  = 480
    c_loop = 0

    feature_sub = rospy.Subscriber(cfg['ROS']['ORB_FEATURES'], PointCloud, callback_pixel)
    cloud_sub   = rospy.Subscriber(cfg['ROS']['ORB_CLOUD'], PointCloud, callback_cloud)
    pose_sub    = rospy.Subscriber(cfg['ROS']['ORB_POSE'], PoseStamped, callback_pose)

    while not rospy.is_shutdown():
        no_data_flag = 0
        if orb_features is None: no_data_flag=1
        if no_data_flag:
            print "no ros data! ", cfg['ROS']['ORB_FEATURES']
            no_data_flag = 0
            continue


        orb_x,orb_y = HELPER.feature_to_array(orb_features)
        yedges = np.linspace(0,X_DIM,X_BINS+1)
        xedges = np.linspace(0,Y_DIM,Y_BINS+1)
        orb_matrix, orb_xedges, orb_yedges = np.histogram2d(orb_x,orb_y, bins=(xedges,yedges))
        ycurr, xcurr = np.unravel_index(np.argmax(orb_matrix, axis=None), orb_matrix.shape)
        if PLOT_WEIGHT_MAT: HELPER.plot_weight_mat(orb_matrix, 0)

        xstable, ystable, stable = getStableBins(xcurr, ycurr)
        if qlen <= 1:
            stable = True
            xstable = xcurr
            ystable = ycurr

        if not stable:
            qcounter += 1
            qlen, qcounter, qinit  = adaptiveQueue(qlen, qcounter)
            if qinit:
                xQueue = deque( maxlen=qlen )
                yQueue = deque( maxlen=qlen )

        else:
        #if stable:
            #reset variables for adative Queue
            qcounter = 0
            qlen = qlenDefault
            xQueue = deque( maxlen=qlenDefault )
            yQueue = deque( maxlen=qlenDefault )


            if ADAP_CAM_SPEED:
                depth = HELPER.get_depth(orb_cloud, orb_pose)
            else:
                depth = FIX_CAM_SPEED
            #depth = HELPER.get_depth()
            vel   = HELPER.lookup_vel(depth)
            xpixel, ypixel  = HELPER.bin2pix_v2(xstable, ystable, X_BINS, Y_BINS)
            degx, degy      = HELPER.pix2degree(xpixel, ypixel)
            degx, degy      = degx + xoffset , degy + yoffset
            print "------------------------------------------------------------------------------"
            print "MOVE: ",degx, degy,"degree // ", "target_pixel: ",xpixel,ypixel, " // bin: ",  xcurr, ycurr, " // depth: ", depth
            print "------------------------------------------------------------------------------"
            xoffset, yoffset = move_camera(degx, degy, xoffset, yoffset, vel)
            print "Waiting for stable bins (Stability Threshold: ", qlen, ")"

def run_feature():
    #os.system("cloud_depth_estimation.py")
    try:
        feature_control()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    global yaw_init, pitch_init
    yaw_init    = 0
    pitch_init  = 0
    mov = movement(pitch_init, yaw_init)
    #os.system("cloud_depth_estimation.py")
    #p = subprocess.Popen([sys.executable, 'attentive_control_helper/cloud_depth_estimation.py'],
    #                                stdout=subprocess.PIPE,
    #                                stderr=subprocess.STDOUT)
    try:
        feature_control()
    except rospy.ROSInterruptException:
        pass
