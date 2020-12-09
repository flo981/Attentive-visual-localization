#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Float64
from geometry_msgs.msg import Twist, PoseStamped, Vector3
import numpy as np
import time

xcurr = 0
ycurr = 0

def callback_x(data):
    global xcurr
    xcurr = data.data
def callback_y(data):
    global ycurr
    ycurr = data.data

def Subscriber():
    rospy.Subscriber("/xbin", Int8, callback_x)
    rospy.Subscriber("/ybin", Int8, callback_y)
    #rospy.Subscriber("/joint_pos", Twist, callback_angular_x)

    #rospy.Subscriber("/joint_pos/angular/z", Int8, callback_angular_z)
    #rospy.Subscriber("/orb_pose", PoseStamped, callback_pose)
    #print xcurr, ycurr, "   sub test"
    return xcurr, ycurr#, ca_x, ca_z
