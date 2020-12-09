#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Float64
from geometry_msgs.msg import Twist, PoseStamped, Vector3, Point, Pose
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import cv2



xcurr = 0
ycurr = 0

pixel = None
segment = None

def saliencymap_segment(SALIENCY_NORM):
    #cv_image = image_converter()
    segment = saliencymap_segment_Subscriber(SALIENCY_NORM)
    #print sal_mat
    return segment

def saliencymap_pixel(SALIENCY_PIXEL):
    #cv_image = image_converter()
    pixel = saliencymap_pixel_Subscriber(SALIENCY_PIXEL)
    #print sal_mat
    return pixel

def callback_x(data):
    global xcurr
    xcurr = data.x

def callback_y(data):
    global ycurr
    ycurr = data.y

def callback_segment(data):

    global segment
    bridge = CvBridge()
    segment = bridge.imgmsg_to_cv2(data)

def callback_pixel(data):

    global pixel
    bridge = CvBridge()
    pixel = bridge.imgmsg_to_cv2(data)




def saliencymap_segment_Subscriber(SALIENCY_NORM):
    rospy.Subscriber(SALIENCY_NORM, Image, callback_segment)#, queue_size = 100)
    rospy.sleep(0.1)
    return segment

def saliencymap_pixel_Subscriber(SALIENCY_PIXEL):
    rospy.Subscriber(SALIENCY_PIXEL, Image, callback_pixel, queue_size = 1)
    return pixel

def Subscriber():
    rospy.Subscriber("/saliency/salientpoint", Point, callback_x)
    rospy.Subscriber("/saliency/salientpoint", Point, callback_y)
    #rospy.Subscriber("/saliency/image", Image, callback_image)

    return xcurr, ycurr#, ca_x, ca_z



def extractPOI():
    x,y = Subscriber() #returns pixels coordis
    #x,y = estimateBin(x,y)
    return x,y

def estimateBin(xpixel,ypixel):
    dimx = 640
    dimy = 480
    nxbins =9-1
    nybins =7-1
    xbinvec = np.linspace(0,dimx,nxbins)
    ybinvec = np.linspace(0,dimy,nybins)
    for xidx in range(0,nxbins):
        if xpixel >= xbinvec[xidx] and xpixel <= xbinvec[xidx+1]:
            xbin = xidx
    for yidx in range(0,nybins):
        if ypixel >= ybinvec[yidx] and ypixel <= ybinvec[yidx+1]:
            ybin = yidx
    return xbin, ybin
