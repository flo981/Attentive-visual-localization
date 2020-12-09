#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped

cloud = 0
pose = 0
def callback_cloud(data):
    global cloud
    #cloud = data.points.position.x
    cloud = data.points


def callback_pose(data):
    global pose
    pose = data#.pose.position

def cloudSubscriber():
    #rospy.init_node('attentive_localization', anonymous=True)
    rospy.Subscriber("/orb_cloud", PointCloud, callback_cloud)
    rospy.sleep(0.1)
    rospy.Subscriber("/stereo_pose", PoseStamped, callback_pose)
    rospy.sleep(0.1)
    #rospy.spin()
    return cloud, pose
