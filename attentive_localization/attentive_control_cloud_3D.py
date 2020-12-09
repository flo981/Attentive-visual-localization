#!/usr/bin/env python

import sys, os
attentive_dir = os.getcwd()
#sys.path.append("/home/flo/ma-florian/attentive_localization/attentive_control_helper/")
sys.path.append(attentive_dir+"/attentive_control_helper/")

import publish_density as PD
import subscriber_ORB as ORB
from movement_helper import movement
import helper_FUNCTIONS as HELPER


import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler, unit_vector, quaternion_about_axis, euler_from_matrix, quaternion_multiply,quaternion_inverse, quaternion_from_matrix
# from tf import transformations as t
# from tf import TransformerROS
import numpy as np
import time
import math
from scipy import stats
#import tf
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Pose, TransformStamped, Vector3, Transform,PoseStamped, Vector3Stamped, TransformStamped
# from geometry_msgs.msg import Quaternion
# from nav_msgs.msg import Odometry
import yaml
# import tf2_ros
# import tf2_geometry_msgs
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import pprint



xoffset = 0
yoffset = 0
global mov

orb_cloud       = None
orb_pose        = None


def callback_cloud(data):
    global orb_cloud
    #cloud = data.points.position.x
    orb_cloud = data.points


def callback_pose(data):
    global orb_pose
    orb_pose = data#.pose.position

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

def pubLine():
    #pointcloud_publisher = rospy.Publisher("/lineee",  marker.LINE_STRIP, queue_size=1)
    rospy.sleep(0.5)

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def cloud_control():
    rospy.init_node('attentive_localization_cloud', anonymous=True)
    rate = rospy.Rate(10)

    global cfg, modus
    #with open(attentive_dir+"/config.yaml", 'r') as ymlfile:
    with open("/home/flo/ma-florian/attentive_localization/config.yaml", 'r') as ymlfile:
       cfg = yaml.load(ymlfile)

    modus           = 'CLOUD'
    ADAP_CAM_SPEED  = cfg[modus]['ADAP_CAM_SPEED']
    FIX_CAM_SPEED   = cfg[modus]['FIX_CAM_SPEED']
    MAX_YAW         = cfg[modus]['MAX_YAW']
    MAX_PITCH       = cfg[modus]['MAX_PITCH']
    pprint.pprint(cfg[modus])

    threshold = 0.9
    cloud_threshold_points_previous = 0


    cloud_sub   = rospy.Subscriber(cfg['ROS']['ORB_CLOUD'], PointCloud, callback_cloud)
    pose_sub    = rospy.Subscriber(cfg['ROS']['ORB_POSE'], PoseStamped, callback_pose)
    no_data_flag = 0
    while not rospy.is_shutdown():
        global xoffset
        global yoffset
        #cloud, pose = ORB.cloudSubscriber(cfg['ROS']['ORB_CLOUD'], cfg['ROS']['ORB_POSE'])
        if orb_cloud is None: no_data_flag=1
        if orb_pose is None: no_data_flag=1
        if no_data_flag:
            print "no ros data! ", cfg['ROS']['ORB_FEATURES']
            no_data_flag = 0
            continue


        # m = TransformStamped()
        # m.header.frame_id = "camera_link"
        # m.parent_id = "map"
        # t.setTransform(m)

        x = np.array([])
        y = np.array([])
        z = np.array([])
        npts = len(orb_cloud)
        point_ros = np.zeros((npts,3))
        for i in range(0,npts):
            point_ros[i,:] = [orb_cloud[i].x, orb_cloud[i].y, orb_cloud[i].z]
            #point_ros[:,0] = z
            #point_ros[:,1] = -x
            #point_ros[:,2] = -y

        #Density Estimaton
        cloud_points = point_ros.T
        #val = np.vstack([cloud_points[0,:],cloud_points[1,:],cloud_points[2,:]])

        kde = stats.gaussian_kde(cloud_points)
        kde.set_bandwidth(bw_method='silverman')
        density = kde(cloud_points)

        cloud_max_idx           = np.argmax(density)
        cloud_max_density_point = cloud_points[:,cloud_max_idx]
        cloud_max_density_value = density[cloud_max_idx]

        cloud_threshold_idx     = np.argwhere(density>cloud_max_density_value*threshold)
        cloud_threshold_points = np.zeros([len(cloud_threshold_idx)-1,3])

        for i in range(0,len(cloud_threshold_idx)-1):
            cloud_threshold_points[i,:] = cloud_points[:,i]

        PD.pub_density(cloud_threshold_points)
        #print "max_coord: ", cloud_points[:,cloud_max_idx], "value: ", density[cloud_max_idx]

        ######################################################
        pose_roll,pose_pitch,pose_yaw = euler_from_quaternion([orb_pose.pose.orientation.x, orb_pose.pose.orientation.y, orb_pose.pose.orientation.z, orb_pose.pose.orientation.w])
        pose_pitch = -pose_pitch
        pose_yaw = -pose_yaw
        #yaw    = rot x (right - minus)
        #pitch = rot y (up - minus)
        pose_z =  orb_pose.pose.position.x
        pose_x = -orb_pose.pose.position.y
        pose_y = -orb_pose.pose.position.z
        ######################################################
        dist_z =  cloud_points[0,cloud_max_idx] -pose_z#np.linalg.norm(cloud_points[0,cloud_max_idx] -pose_z) #point_ros[idx_cloud_max_density_point,0] -pose_z
        dist_x =  -cloud_points[1,cloud_max_idx] -pose_x#np.linalg.norm(-cloud_points[1,cloud_max_idx] -pose_x) #point_ros[idx_cloud_max_density_point,1] -pose_x
        dist_y =  -cloud_points[2,cloud_max_idx] -pose_y#np.linalg.norm(-cloud_points[2,cloud_max_idx] -pose_y) #point_ros[idx_cloud_max_density_point,2] -pose_y

        target          = np.array([-cloud_points[1,cloud_max_idx],-cloud_points[2,cloud_max_idx],cloud_points[0,cloud_max_idx]])
        robot_position  = np.array([pose_x,pose_y,pose_z])

        dist = np.linalg.norm([target-robot_position])
        PD.pub_maxdensity(cloud_points[:,cloud_max_idx])
        # for i in range(0,cloud_points.shape[1]):
        #      PD.pub_kde(cloud_points[:,i],density[i])
        #PD.pub_kde(cloud_points[:,cloud_max_idx],density[cloud_max_idx])



        atan_yaw_densest = math.atan2(dist_x,dist_z)
        yaw  = math.degrees(atan_yaw_densest)

        atan_pitch_densest = math.atan2(dist_y,abs(dist_z))
        pitch = math.degrees(atan_pitch_densest)

        yaw = int(yaw-math.degrees(pose_yaw))
        pitch = int(pitch+math.degrees(pose_pitch))


        # test_pose.pose.position.x = -cloud_points[1,cloud_max_idx]
        # test_pose.pose.position.y = -cloud_points[2,cloud_max_idx]
        #test_pose.pose.position.z = cloud_points[0,cloud_max_idx]

        # pose_transformed = tf2_geometry_msgs.do_transform_pose(test_pose, transform)
        # print("pose_transformed",pose_transformed.pose)
        if ADAP_CAM_SPEED:
            depth = round(abs(dist_z),1)
        else:
            depth = FIX_CAM_SPEED

        #depth = round(abs(dist_z),1)
        vel   = HELPER.lookup_vel(depth)
        #depth = round(math.cos(pose_yaw) * dist_z,1)
        #speed threshold
        if depth > 2:
            depth = 2
        if depth < 0.5:
            depth = 0.5

        if yaw > 90:
            print "yaw > 90:", yaw
            yaw = yaw-360
        if yaw < -90:
            yaw = yaw+360

        print "------------------------------------------------------------------------------"
        print  "max_coord: ", cloud_points[:,cloud_max_idx], "value: ", density[cloud_max_idx]
        print "pose:             :" , math.degrees(pose_yaw), math.degrees(pose_pitch)
        print "yaw, pitch        : ", yaw, pitch
        xoffset, yoffset = move_camera(yaw, pitch, xoffset, yoffset, vel)
        print "------------------------------------------------------------------------------"

def run_cloud():
    pitch = 0
    yaw = 0
    mov = movement(pitch, yaw)
    try:
        cloud_control()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pitch = 0
    yaw = 0
    mov = movement(pitch, yaw)
    try:
        cloud_control()
    except rospy.ROSInterruptException:
        pass
