import sys, os
attentive_dir = os.getcwd()
sys.path.append("/home/flo/ma-florian/attentive_localization/attentive_control_helper/")
#sys.path.append(attentive_dir+"/attentive_control_helper/")

import helper_FUNCTIONS as HELPER
import subscriber_ORB as ORB
import subscriber_VOCUS as VOCUS
from movement_helper import movement


import rospy
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from collections import deque
import yaml

import matplotlib.pyplot as plt

global mov
import time
import math
import pprint

from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

xoffset = 0
yoffset = 0

sal_norm        = None
orb_features    = None
orb_cloud       = None
orb_pose        = None

def callback_saliency(data):
    global sal_norm
    bridge = CvBridge()
    sal_norm = bridge.imgmsg_to_cv2(data)

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
    if counter > cfg[modus]['ADAP_SPEED']: #must be bigger then arg.modus(qlen)!!!
    #if qlen is de/inc: init new q(maxlen=qlen-1)
        qlen -=1
        print "Adaptive Stability decreased: ", qlen
        counter = 0
        if qlen <= 1:
            return 1, 10, True #test false
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


def norm_matrix(mat):
    xmax, xmin = mat.max(), mat.min()
    if np.sum(mat) == 0: #zero mat
        return mat
    else:
        return (mat - xmin)/(xmax - xmin)

def surround_weights(mat, idx):
    surround_weight    = np.zeros(mat.shape)
    mask = cfg[modus]['WEIGHT_MASK']
    for i in idx:
        submatrix = mat[i[0]-mask:i[0]+mask,i[1]-mask:i[1]+mask]
        count = np.sum(submatrix)
        surround_weight[i[0],i[1]] = count
    return surround_weight

def estimate_weightsV2(orb_matrix, orb_idx, sal_matrix, sal_idx):
    orb    = np.zeros(orb_matrix.shape)
    sal    = np.zeros(sal_matrix.shape)
    orb    = surround_weights(orb_matrix, sal_idx)
    sal    = surround_weights(sal_matrix, orb_idx)
    return orb, sal

def orbfusionV4(sailency_matrix, orb_matrix, dimx,dimy, threshold):
    #orb_matrix              = norm_matrix(orb_matrix)
    #sailency_matrix         = norm_matrix(sailency_matrix)
    inital_weight_matrix    = orb_matrix*threshold + sailency_matrix*(1-threshold)
    orb_dim, sailency_dim   = orb_matrix.shape, sailency_matrix.shape

    saliency_mean           = np.mean(sailency_matrix)
    saliency_max            = np.max(sailency_matrix)
    orb_mean                = np.mean(orb_matrix)
    orb_max                 = np.max(orb_matrix)

    #threshold
    #   1:      pure orb
    #   0:      pure saliency
    #   (0,1):  mix

    saliency_threshold  = (threshold)*saliency_max  + saliency_mean
    orb_threshold       = (1-threshold)*orb_max     + orb_mean

    #orb_matrix = norm_matrix(orb_matrix)
    #sailency_matrix = norm_matrix(sailency_matrix)

    orb_idx = np.argwhere(orb_matrix > orb_threshold)
    sal_idx = np.argwhere(sailency_matrix > saliency_threshold)

    #print "orb count: ", len(orb_idx), "sal count: ", len(sal_idx)
    orb_weights, sal_weights    = estimate_weightsV2(orb_matrix, orb_idx, sailency_matrix, sal_idx)
    final_weight_matrix = inital_weight_matrix + orb_weights*threshold + sal_weights*(1-threshold)

    #orb_weights_norm    = norm_matrix(orb_weights)
    #sal_weights_norm    = norm_matrix(sal_weights)
    return norm_matrix(final_weight_matrix)

def orbfusion_control():
    rospy.init_node('attentive_localization_orbfusion', anonymous=True)
    #rate = rospy.Rate(5)
    counter = qcounter = 0
    global qlen
    global xQueue, yQueue

    global cfg, modus
    #with open(attentive_dir+"/config.yaml", 'r') as ymlfile:
    with open("/home/flo/ma-florian/attentive_localization/config.yaml", 'r') as ymlfile:
       cfg = yaml.load(ymlfile)

    modus           = 'FUSION'
    X_DIM           = cfg[modus]['X_DIM']
    Y_DIM           = cfg[modus]['Y_DIM']
    ADAP_QLEN       = cfg[modus]['ADAP_QLEN']
    ADAP_SPEED      = cfg[modus]['ADAP_SPEED']
    SCALE           = cfg[modus]['SCALE']
    THRESHOLD       = cfg[modus]['THRESHOLD']
    WEIGHT_MASK     = cfg[modus]['WEIGHT_MASK']
    PLOT_WEIGHT_MAT = cfg[modus]['PLOT_WEIGHT_MAT']
    ADAP_CAM_SPEED  = cfg[modus]['ADAP_CAM_SPEED']
    FIX_CAM_SPEED   = cfg[modus]['FIX_CAM_SPEED']
    MAX_YAW         = cfg[modus]['MAX_YAW']
    MAX_PITCH       = cfg[modus]['MAX_PITCH']
    pprint.pprint(cfg[modus])

    qlen = ADAP_QLEN
    xQueue = deque( maxlen=qlen )
    yQueue = deque( maxlen=qlen )
    qlenDefault = qlen

    #deges for histogram2d
    xedges = np.linspace(0,X_DIM,X_DIM/SCALE)
    yedges = np.linspace(0,Y_DIM,Y_DIM/SCALE)

    #downsample sal map
    sal_sizing = 1/float(SCALE)
    SCALE_w = 1/(2*float(SCALE))

    saliency_sub    = rospy.Subscriber(cfg['ROS']['SALIENCY_NORM'], Image, callback_saliency)
    feature_sub     = rospy.Subscriber(cfg['ROS']['ORB_FEATURES'], PointCloud, callback_pixel)
    cloud_sub       = rospy.Subscriber(cfg['ROS']['ORB_CLOUD'], PointCloud, callback_cloud)
    pose_sub        = rospy.Subscriber(cfg['ROS']['ORB_POSE'], PoseStamped, callback_pose)


    print "Waiting for stable bins (Stability Threshold: ", qlen, ")"
    while not rospy.is_shutdown():

        no_data_flag_orb = 0
        no_data_flag_sal = 0
        global xoffset
        global yoffset

        #sal_norm        = VOCUS.saliencymap_segment(cfg['ROS']['SALIENCY_NORM'])
        if sal_norm is None: no_data_flag_sal=1
        if no_data_flag_sal:
            print "no ros data! ", cfg['ROS']['SALIENCY_NORM']
            continue
        #sal_pixel      = VOCUS.saliencymap_pixel()
        #orb_features    = ORB.orb_pixel_feature_Subscriber(cfg['ROS']['ORB_FEATURES'])
        if orb_features is 0: no_data_flag_orb=1
        if no_data_flag_orb:
            print "no ros data! ", cfg['ROS']['ORB_FEATURES']
            continue

        array_features = np.asarray(orb_features[:])
        orb_x,orb_y = HELPER.feature_to_array(array_features)
        orb_matrix, orb_xedges, orb_yedges = np.histogram2d(orb_x,orb_y, [Y_DIM/SCALE,X_DIM/SCALE])
        orb_mean = np.mean(orb_matrix)


        sal_temp = np.zeros([Y_DIM,X_DIM])
        sal_idx = np.argwhere(sal_norm>0)
        for i in sal_idx:
            sal_temp[i[0],i[1]] = 1*orb_mean
        sailency_matrix = HELPER.resize_saliencymatrix(sal_temp, sal_sizing)
        weight_matrix =  orbfusionV4(sailency_matrix, orb_matrix, X_DIM/SCALE, Y_DIM/SCALE, THRESHOLD)
        weight_matrix = HELPER.resize_saliencymatrix(weight_matrix, 0.5)
        max_y, max_x  = np.unravel_index(np.argmax(weight_matrix, axis=None), weight_matrix.shape)

        if PLOT_WEIGHT_MAT: HELPER.plot_weight_mat(weight_matrix, THRESHOLD)

        xstable, ystable, stable = getStableBins(max_x, max_y)

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
            vel     = HELPER.lookup_vel(depth)
            yaw, pitch = HELPER.pix2degree(max_x/SCALE_w, max_y/SCALE_w)
            degx, degy = yaw+xoffset, pitch+yoffset

            print "------------------------------------------------------------------------------"
            print "MOVE: ",degx, degy,"degree // ", "target_pixel: ",max_x/SCALE_w, max_y/SCALE_w, " // bin: ",  max_x, max_y, " // depth: ", depth
            print "------------------------------------------------------------------------------"
            xoffset, yoffset = move_camera(degx, degy, xoffset, yoffset, vel)
            print "Waiting for stable bins (Stability Threshold: ", qlen, ")"

def run_orbfusion():
    try:
        orbfusion_control()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pitch = 0
    yaw = 0
    mov = movement(pitch, yaw)
    try:
        orbfusion_control()
    except rospy.ROSInterruptException:
        pass
