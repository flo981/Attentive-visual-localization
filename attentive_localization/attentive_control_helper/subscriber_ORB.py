#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped


xcurr = 0
ycurr = 0
cloud = 0
pose = 0
pixel = 0

def callback_x(data):
    global xcurr
    xcurr = data.data
def callback_y(data):
    global ycurr
    ycurr = data.data

def callback_pixel(data):
    global pixel
    pixel = data.points

def callback_depth(data):
    global depth
    depth = data

def callback_cloud(data):
    global cloud
    #cloud = data.points.position.x
    cloud = data.points


def callback_pose(data):
    global pose
    pose = data#.pose.position

def Subscriber():
    rospy.Subscriber("/xbin", Int8, callback_x)
    rospy.Subscriber("/ybin", Int8, callback_y)
    #rospy.Subscriber("/joint_pos", Twist, callback_angular_x)

    #rospy.Subscriber("/joint_pos/angular/z", Int8, callback_angular_z)
    #rospy.Subscriber("/orb_pose", PoseStamped, callback_pose)
    #print xcurr, ycurr, "   sub test"W
    return xcurr, ycurr#, ca_x, ca_z

def orb_pixel_feature_Subscriber(ORB_FEATURES):
    rospy.Subscriber(ORB_FEATURES, PointCloud, callback_pixel)
    rospy.sleep(0.1)
    rospy.spin()
    #return pixel





def cloudSubscriber(ORB_CLOUD, ORB_POSE):
    #rospy.init_node('attentive_localization', anonymous=True)
    rospy.Subscriber(ORB_CLOUD, PointCloud, callback_cloud)
    rospy.sleep(0.2)
    rospy.Subscriber(ORB_POSE, PoseStamped, callback_pose)
    rospy.sleep(0.2)

    #rospy.spin()
    return cloud, pose
