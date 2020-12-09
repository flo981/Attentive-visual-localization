#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped
from std_msgs.msg import Float32
from scipy import stats
import math
from tf.transformations import euler_from_quaternion

cloud = 0
pose = 0
pixel = 0
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

def cloud_to_array(cloud):
    x = []
    y = []
    z = []
    for element in cloud:
        z = np.append(z,element.x)
        x = np.append(x,-element.y)
        y = np.append(y,-element.z)
    return x,y,z

def depth_estimation():
    rospy.init_node('attentive_localization_depth', anonymous=True)
    pub = rospy.Publisher('cloud_depth', Float32, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cloud, pose    = cloudSubscriber()
        pose_roll,pose_pitch,pose_yaw = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        pose_pitch = -pose_pitch
        pose_yaw = -pose_yaw

        array_cloud = np.asarray(cloud[:])
        x,y,z =cloud_to_array(array_cloud)

        kde = stats.gaussian_kde([x,y,z])
        density = kde([x,y,z])
        maxIdx = np.argmax(density)

        pose_z =  pose.pose.position.x
        #pose_x = -pose.pose.position.y
        #pose_y = -pose.pose.position.z

        dist_z =  z[maxIdx] -pose_z
        #dist_x =  x[maxIdx] -pose_x
        #dist_y =  y[maxIdx] -pose_y

        #atan_yaw_densest = math.atan2(dist_x,dist_z)
        #yaw  = math.degrees(atan_yaw_densest)

        #depth = round(math.cos(pose_yaw+yaw) * dist_z,1)
        depth = abs(dist_z)
        if depth > 2:
            depth = 2
        if depth < 0.5:
            depth = 0.5

        pub.publish(depth)
        rate.sleep()

if __name__ == '__main__':
    try:
        depth_estimation()
    except rospy.ROSInterruptException:
        pass
