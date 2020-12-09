#!/usr/bin/env python

import orbsubscriber_helper
import orbslam_subsciber as orbsub
import vocus2_subscriber as vsub
import orbfusion
import helpers as h
from helpers import movement
import pub_density as pd

from tf.transformations import euler_from_quaternion
import rospy
from std_msgs.msg import Int8, Float64
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from collections import deque
from math import pi
import numpy as np
import time
import serial
import argparse
import subprocess
import matplotlib.pyplot as plt
from collections import OrderedDict
import scipy.spatial
from scipy.spatial import Voronoi, voronoi_plot_2d, SphericalVoronoi, Delaunay
import math

from matplotlib import colors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from scipy.spatial import SphericalVoronoi, ConvexHull


from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped

parser = argparse.ArgumentParser(description='description')
parser.add_argument("--modus", type=int, help="0: ORB Feature (default), 1: SALIENCY (VOCUS2), 2: ORB_FUSION",default=0)
parser.add_argument("--adap_qlen", type=int, help="the higher the stable the feature area is, but can be an overkill: at movement at all (default=15)", default=15)
parser.add_argument("--adap_speed", type=int, help="how fast qlen is reduced (default=20)", default=20)



def voronoi_volumes(points):
    v = Voronoi(points)
    vol = np.zeros(v.npoints)
    for i, reg_num in enumerate(v.point_region):
        indices = v.regions[reg_num]
        if -1 in indices: # some regions can be opened
            vol[i] = np.inf
        else:
            try:
                vol[i] = ConvexHull(v.vertices[indices]).volume
            except:
                vol[i] = np.inf
    return vol


#global xoffset, yoffset
xoffset = 0
yoffset = 0
global mov
trafo = [[0, 0, 1],[-1, 0, 0], [0, -1, 0]]
def run_pointmap():
    print "######################################################"
    global xoffset
    global yoffset
    cloud, pose = orbsubscriber_helper.cloudSubscriber()
    x = np.array([])
    y = np.array([])
    z = np.array([])

    npts = len(cloud)
    point_ros = np.zeros((npts,3))
    for i in range(0,npts):
        point_ros[i,:] = [cloud[i].x, cloud[i].y, cloud[i].z]
        #point_ros[:,0] = z
        #point_ros[:,1] = -x
        #point_ros[:,2] = -y

    mean_depth = abs(np.median(point_ros[:,0]))


    vol= voronoi_volumes(point_ros)
    idx = np.argwhere(vol < mean_depth/500.) #1/vol =density trashold
    idx_max_density = np.argmin(vol)
    print "n: ",len(idx)

    if len(idx) > 10:
        #dens_point_max = [point_ros[idx_max_density,0],point_ros[idx_max_density,1],point_ros[idx_max_density,2]]

        dens_point = np.zeros((len(idx),3))
        count = 0
        for i in idx:
            dens_point[count,:] = [point_ros[i,0],point_ros[i,1],point_ros[i,2]]
            count += 1

        print "n: ",len(idx)
        #print "n dense points: ",count, "factor: ", mean_depth/100., "mean depth: ", mean_depth
        ######################################################
        pose_roll,pose_pitch,pose_yaw = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        pose_pitch = -pose_pitch
        pose_yaw = -pose_yaw
        #yaw    = rot x (right - minus)
        #pitch = rot y (up - minus)
        ######################################################
        ##denest point:
        pose_z =  pose.pose.position.x
        pose_x = -pose.pose.position.y
        pose_y = -pose.pose.position.z

        dist_z =  np.linalg.norm(point_ros[idx_max_density,0] -pose_z) #point_ros[idx_max_density,0] -pose_z
        dist_x =  np.linalg.norm(-point_ros[idx_max_density,1] -pose_x) #point_ros[idx_max_density,1] -pose_x
        dist_y =  np.linalg.norm(-point_ros[idx_max_density,2] -pose_y) #point_ros[idx_max_density,2] -pose_y
        #yaw
        atan_yaw_densest = math.atan2(dist_x,dist_z)
        yaw_densest  = int(math.degrees(atan_yaw_densest)+math.degrees(pose_yaw))

        #pitch
        atan_pitch_densest = math.atan2(dist_y,dist_z)
        pitch_densest = int(math.degrees(atan_pitch_densest)+math.degrees(pose_pitch))
        ######################################################
        ## publisher
        pd.pub_maxdensity(point_ros[idx_max_density])
        pd.pub_density(dens_point)
        ######################################################
        ## avarage point
        atan2_f  = np.vectorize(math.atan2)
        degree_f = np.vectorize(math.degrees)
        distance_f = np.vectorize(np.linalg.norm)

        pose_z =  np.ones(len(idx))*pose.pose.position.x
        pose_x = -np.ones(len(idx))*pose.pose.position.y
        pose_y = -np.ones(len(idx))*pose.pose.position.z

        # dist_z =  distance_f(dens_point[:,0]  -pose_z)   #dens_point[:,0] -pose_z
        # dist_x =  distance_f(-dens_point[:,1] -pose_x)   #-dens_point[:,1] -pose_x
        # dist_y =  distance_f(-dens_point[:,2] -pose_y)   #-dens_point[:,2] -pose_y

        dist_z =  dens_point[:,0] -pose_z
        dist_x = -dens_point[:,1] -pose_x
        dist_y = -dens_point[:,2] -pose_y

        ### yaw:
        #for angle calculation, make distance_z always positivie
        atan_yaw = atan2_f(dist_x,dist_z)
        yaw = degree_f(atan_yaw)

        ### pitch:
        atan_pitch = atan2_f(dist_y,abs(dist_z)) #always project z in positve plane for atan2 (get rid of special cases +-180deg)
        pitch = degree_f(atan_pitch)

        yaw   = int(np.median(yaw)-math.degrees(pose_yaw))
        pitch = int(np.median(pitch)+math.degrees(pose_pitch))


        if yaw > 90:
            print "yaw > 90:", yaw
            yaw = yaw-360
        if yaw < -90:
            yaw = yaw+360

        print "yaw, pitch        : ", yaw, pitch
        #print "densest yaw, pitch: ",yaw_densest,pitch_densest
        ######################################################
        #raw_input("Press Enter to continue Y")
        #safty boarders
        if yaw > -45 and yaw < 45:
            mov.move_robot_x(yaw, xoffset)
            xoffset = yaw
        if pitch > -36 and pitch < 60:
            #neg pitch means up for robot!
            mov.move_robot_y(pitch, yoffset)
            yoffset = pitch







    #print pvol_pre, dens_point
    #pd.talker(dens_point)
    #fig = voronoi_plot_2d(vor)
    #plt.show()

    #print "total volume= "+str(vtot)
    #voronoi_volumes(point_ros)
    # print point_ros
    # vor = Voronoi(point_ros)
    # print vor.point_region
    # fig = voronoi_plot_2d(vor)
    # plt.show()
    # npts = len(x)
    # points = np.zeros((npts,3))
    # for i in range(1,npts):
    #      points[i,:] = [x[i], y[i], z[i]]
    #
    # tmp = OrderedDict()
    # for points2 in zip(x, y, z):
    #     tmp.setdefault(points2[:2], points2)
    # points2 = tmp.values()
    #

    # points = np.zeros((npts,3))
    # for i in range(1,npts):
    #     points[i,:] = points2[i]

    #points = unique_by_first(points)

    # idx = np.random.randint(npts, size=10)
    # points = points[idx,:]
    # print points
    # radius = 1
    # center = np.array([0, 0, 0])
    # sv = scipy.spatial.SphericalVoronoi(points, radius, center)
    # sv.sort_vertices_of_regions()
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # # plot the unit sphere for reference (optional)
    # u = np.linspace(0, 2 * np.pi, 100)
    # v = np.linspace(0, np.pi, 100)
    # x = np.outer(np.cos(u), np.sin(v))
    # y = np.outer(np.sin(u), np.sin(v))
    # z = np.outer(np.ones(np.size(u)), np.cos(v))
    # ax.plot_surface(x, y, z, color='y', alpha=0.1)
    # # plot generator points
    # ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b')
    # # plot Voronoi vertices
    # ax.scatter(sv.vertices[:, 0], sv.vertices[:, 1], sv.vertices[:, 2],
    #                    c='g')
    # # indicate Voronoi regions (as Euclidean polygons)
    # for region in sv.regions:
    #    random_color = colors.rgb2hex(np.random.rand(3))
    #    polygon = Poly3DCollection([sv.vertices[region]], alpha=1.0)
    #    polygon.set_color(random_color)
    #    ax.add_collection3d(polygon)
    # plt.show()



    # xpose, ypose, zpose       = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    # xorient, yorient, zorient, worient= pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w
    #
    # # euler = tf.transformations.euler_from_quaternion(pose.pose.orientation)
    # # roll = euler[0]
    # # pitch = euler[1]
    # # yaw = euler[2]
    #
    # xmin, xmax = min(x), max(x)
    # ymin, ymax = min(y), max(y)
    # xi      = np.linspace(xmin, xmax, nbins)
    # yi      = np.linspace(ymin, ymax, nbins)
    #
    # depth, bins, binloc, depthmap = h.griddata_self2(x, y, z)
    #
    # plt.imshow(depthmap)
    # plt.pause(0.01)
    #
    # pointset = h.getMaxMapPoint(depth, np.copy(bins), depthmap, xi,yi)
    # angleset = h.getAngle(pointset, pose)
    # degz = 0
    # degx = int(angleset[0])
    #raw_input("Press Enter to continue...")
    #mov = movement(degz, degx)
    #mov.move_robot_x(degx, xoffset)
    #xoffset = degx
    #print degx

    #degx = angleset[0]
    #mov.move_robot_x(degx, xoffset)


if __name__ == '__main__':
    rospy.init_node('attentive_localization', anonymous=True)
    args = parser.parse_args()
    qlen = args.adap_qlen
    #mov = h.movement(0, 0)
    if args.modus == 0:
        #not working yet
        print "----------------"
        print "ORBSLAM MODUS"
        print "----------------"
        xQueue = deque( maxlen=qlen )
        yQueue = deque( maxlen=qlen )
    if args.modus == 1:
        ## TODO: strart launch file from here
        print "----------------"
        print "SALIENCY MODUS"
        print "----------------"
        xQueue = deque( maxlen=qlen )
        yQueue = deque( maxlen=qlen )
    if args.modus == 2:
        print "----------------"
        print "ORB FUSION MODUS"
        print "----------------"
        xQueue = deque( maxlen=qlen )
        yQueue = deque( maxlen=qlen )
    if args.modus == 3:
        print "----------------"
        print "POINTCLOUD  MODUS"
        print "----------------"
        orbsubscriber_helper.cloudSubscriber()
        pitch = 0
        yaw = 0
        mov = movement(pitch, yaw)

    while not rospy.is_shutdown():

        if args.modus == 0:
            xcurr, ycurr = orbsub.Subscriber()
        if args.modus == 1:
            ## TODO: strart launch file from here
            xcurr, ycurr = vsub.extractPOI()
        if args.modus == 2:
            xcurr, ycurr = orbfusion.bins()
        if args.modus == 3:
            run_pointmap()
            #cloud, pose = orbsubscriber_helper.cloudSubscriber()
