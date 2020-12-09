#!/usr/bin/env python

import sys, os
attentive_dir = os.getcwd()
sys.path.append("/home/flo/ma-florian/attentive_localization/attentive_control_helper/")
#sys.path.append(attentive_dir+"/attentive_control_helper/")

import helper_FUNCTIONS as HELPER
from movement_helper import movement
import subscriber_VOCUS as VOCUS

import rospy
from std_msgs.msg import Int8, Float64
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from math import pi
import numpy as np
import time
from collections import deque
import yaml
import pprint

sal_norm        = None
orb_cloud       = None
orb_pose        = None


def callback_cloud(data):
    global orb_cloud
    #cloud = data.points.position.x
    orb_cloud = data.points


def callback_pose(data):
    global orb_pose
    orb_pose = data#.pose.position

def callback_saliency(data):
    global sal_norm
    bridge = CvBridge()
    sal_norm = bridge.imgmsg_to_cv2(data)

xoffset = 0
yoffset = 0

def queueStable(q):
    if len(q) != qlen:
        return False
    for i in range(1,len(q)):
        if (q[0] != q[i]):
            return False
    return True

def getStableBins(max_x, max_y):
    xQueue.append(max_x)
    yQueue.append(max_y)

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



def move_camera(yaw, pitch, xoffset, yoffset, vel):
    xlimit = cfg[modus]['MAX_YAW']
    ylimit = cfg[modus]['MAX_PITCH']

    if yaw > -xlimit and yaw < xlimit:
        if yaw > -120 and yaw < 120:
            #hardware limit
            mov.move_robot_x(yaw, xoffset,vel)
            xoffset = yaw
    if pitch > -ylimit and pitch < ylimit:
        if pitch > -36 and pitch < 60:
            #hardware limit
        #neg pitch means up for robot!
            mov.move_robot_y(pitch, yoffset,vel)
            yoffset = pitch
    return xoffset, yoffset


def feature_control():
    rospy.init_node('attentive_localization_feature', anonymous=True)
    #rate = rospy.Rate(5)
    mov = movement(0, 0)
    print "SALIENCY MODUS"

    global X_BINS, Y_BINS
    global cfg, modus
    #with open(attentive_dir+"/config.yaml", 'r') as ymlfile:
    with open("/home/flo/ma-florian/attentive_localization/config.yaml", 'r') as ymlfile:
       cfg = yaml.load(ymlfile)

    modus               = 'SAL'
    ADAP_QLEN           = cfg[modus]['ADAP_QLEN']
    ADAP_SPEED          = cfg[modus]['ADAP_SPEED']
    SCALE               = cfg[modus]['SCALE']
    PLOT_WEIGHT_MAT     = cfg[modus]['PLOT_WEIGHT_MAT']
    ADAP_CAM_SPEED      = cfg[modus]['ADAP_CAM_SPEED']
    FIX_CAM_SPEED       = cfg[modus]['FIX_CAM_SPEED']
    MAX_YAW             = cfg[modus]['MAX_YAW']
    MAX_PITCH           = cfg[modus]['MAX_PITCH']
    pprint.pprint(cfg[modus])

    xoffset = 0
    yoffset = 0
    qcounter = 0

    global qlen
    global xQueue, yQueue

    qlen = ADAP_QLEN
    xQueue = deque( maxlen=qlen )
    yQueue = deque( maxlen=qlen )
    qlenDefault = qlen

    #downsample sal map
    sal_sizing = 1/float(SCALE)
    no_data_flag = 0
    saliency_sub    = rospy.Subscriber(cfg['ROS']['SALIENCY_NORM'], Image, callback_saliency)
    cloud_sub       = rospy.Subscriber(cfg['ROS']['ORB_CLOUD'], PointCloud, callback_cloud)
    pose_sub        = rospy.Subscriber(cfg['ROS']['ORB_POSE'], PoseStamped, callback_pose)

    print "Waiting for stable bins (Stability Threshold: ", qlen, ")"
    while not rospy.is_shutdown():
        no_data_flag = 0
        #xpixel, ypixel = VOCUS.extractPOI(cfg['ROS']['SALIENCY_PIXEL'])
        #sal_norm        = VOCUS.saliencymap_segment(cfg['ROS']['SALIENCY_NORM'])
        if sal_norm is None: no_data_flag=1
        if no_data_flag:
            print "no ros data! (/saliency/image_norm)"
            continue

        sailency_matrix = HELPER.resize_saliencymatrix(sal_norm, sal_sizing)
        max_y, max_x = np.unravel_index(np.argmax(sailency_matrix, axis=None), sailency_matrix.shape)
        xstable, ystable, stable = getStableBins(max_x, max_y)

        if PLOT_WEIGHT_MAT: HELPER.plot_weight_mat(sailency_matrix, 0)

        if qlen <= 1:
            stable = True

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
            prevx = xstable
            prevy = ystable

            if ADAP_CAM_SPEED:
                depth = HELPER.get_depth(orb_cloud, orb_pose)
            else:
                depth = FIX_CAM_SPEED

            vel   = HELPER.lookup_vel(depth)
            #xpixel, ypixel = HELPER.bin2pix(xstable, ystable, X_BINS, Y_BINS)
            degx, degy = HELPER.pix2degree(max_x/sal_sizing, max_y/sal_sizing)
            degx, degy = degx+xoffset, degy+yoffset
            print "------------------------------------------------------------------------------"
            print "MOVE: ",degx, degy,"degree // ", "target_pixel: ",max_x/sal_sizing, max_y/sal_sizing, " // bin: ",  max_x, max_y, " // depth: ", depth
            print "------------------------------------------------------------------------------"
            xoffset, yoffset = move_camera(degx, degy, xoffset, yoffset, vel)
            print "Waiting for stable bins (Stability Threshold: ", qlen, ")"


def run_feature():
    try:
        feature_control()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    mov = movement(0, 0)
    try:
        feature_control()
    except rospy.ROSInterruptException:
        pass
