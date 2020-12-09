#!/usr/bin/env python

import numpy as np
import math
import serial
import rospy
import time


from geometry_msgs.msg import Twist, PoseStamped, Vector3
global mov

class movement:
    # actuator resolution 0.29

    def __init__(self, degz, degx):
        #rospy.init_node('attentive_localization', anonymous=True)
        self.pub_move = rospy.Publisher("/joint_pos",Twist,queue_size=1)

        self.move = Twist()
        self.degz = degz
        self.degx = degx
        self.resolution = 0.5
        self.sleeptime = 0.05
        #make variable depending how many features are around/how fare they are away?

    def publish_vel(self):
        self.pub_move.publish(self.move)

    def move_robot_x(self, degree, xoffset):
        if degree < xoffset:
            inc = -self.resolution
        else:
            inc = self.resolution
        print xoffset, degree
        while xoffset != degree:
            xoffset = xoffset+inc
            self.move.angular.z=xoffset
            print "x: ", xoffset, " - goal: ",degree, "y: " ,self.degz
            self.publish_vel()
            time.sleep(self.sleeptime)

    def move_robot_y(self, degree, yoffset):
        if degree < yoffset:
            inc = -self.resolution
        else:
            inc = self.resolution
        while yoffset != degree:
            yoffset = yoffset+inc
            self.move.angular.x=yoffset
            print "y: ", yoffset, " - goal: ",degree, "x: ", self.degx
            self.publish_vel()
            time.sleep(self.sleeptime)



def move_robot(degx, degz, xoffset):
    mov = movement(degz, degx)
    print mov
    mov.move_robot_x(degx, xoffset)


class Point( object ):
    def __init__( self, x, y, z):
        self.x, self.y, self.z = x, y, z
        #self.data = data
    def distFrom( self, x, y, z ):
        return math.sqrt( (self.x-x)**2 + (self.y-y)**2 + (self.z-z)**2 )

class Angle( object ):
    def __init__( self, x, y):
        self.x, self.y = x, y
        #self.data = data


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

def bin2pix(x, y):
    dimx = 640
    dimy = 480
    nbinsx = 9-1#7-1
    nbinsy = 7-1#5-1
    xpixel = x * (dimx/nbinsx) + (dimx/(2*nbinsx))
    ypixel = y * (dimy/nbinsy) + (dimy/(2*nbinsy))
    return xpixel, ypixel

def pix2degree(xpixel, ypixel):
    angelofview = 90
    dimx = 640
    dimy = 480
    linx = np.linspace(0, dimx, angelofview).astype(int)
    liny = np.linspace(0, dimy, angelofview).astype(int)
    for i in range(0,len(linx)-1):
        if linx[i] > xpixel:
            xidx = i-1
            break
    for i in range(0,len(liny)-1):
        if liny[i] > ypixel:
            yidx = i-1
            break
    lin = np.linspace(-45,45,angelofview+1).astype(int)
    return lin[xidx]/2, lin[yidx]/2 #test

def adaptiveQueue(qlen, counter):
    if counter > args.adap_speed: #must be bigger then arg.modus(qlen)!!!
    #if qlen is de/inc: init new q(maxlen=qlen-1)
        qlen -=1
        counter = 0
        if qlen <= 1:
            return 1, 10, False
        return qlen, counter, True
    else:
        return qlen, counter, False

def griddata_self2(x, y, z):
    nbins = 50
    retbin=True
    retloc=True

    # get extrema values.
    xmin, xmax = min(x), max(x)
    ymin, ymax = min(y), max(y)

    # make coordinate arrays.
    xi      = np.linspace(xmin, xmax, nbins)
    yi      = np.linspace(ymin, ymax, nbins)
    xi, yi = np.meshgrid(xi,yi)
    # make the grid.
    depth           = np.zeros(xi.shape, dtype=x.dtype)
    depthdensity        = np.zeros(xi.shape, dtype=x.dtype)
    nrow, ncol = depth.shape
    #print("test",nrow, ncol)
    if retbin: bins = np.copy(depth)

    # create list in same shape as grid to store indices
    if retloc:
        wherebin = np.copy(depth)
        wherebin = wherebin.tolist()


    binsize = np.abs(xi[0,1]-xi[0,0])
    # fill in the grid.
    for row in range(nrow):
        for col in range(ncol):
            count = 0
            xc = xi[row, col]    # x coordinate.
            yc = yi[row, col]    # y coordinate.

            # find the position that xc and yc correspond to.
            posx = np.abs(x - xc) #vector with distance all points to xc(curren)
            posy = np.abs(y - yc)
            ibin = np.logical_and(posx < binsize/2., posy < binsize/2.)
            ind  = np.where(ibin == True)[0] #ind true if x and y are in current row/col ind: pos of input x/y

            if (posx < binsize/2., posy < binsize/2.):
                count = len(ind)
                depthdensity[row,col] = count


            # fill the bin.
            bin = z[ibin]
            if retloc: wherebin[row][col] = ind
            if retbin: bins[row, col] = bin.size
            if bin.size != 0:
                binval         = np.median(bin)
                depth[row, col] = binval
            else:
                depth[row, col] = np.nan   # fill empty bins with nans.

    # return the grid
    if retbin:
        if retloc:
            return depth, bins, wherebin, depthdensity
        else:
            return depth, bins
    else:
        if retloc:
            return depth, wherebin
        else:
            return depth


def deleteClosePoints(x,y,z):
    indexlist = np.array([])
    for idx,ele in enumerate(z):
        if abs(ele) < 0.01:
            indexlist = np.append(indexlist,idx)
            #x = np.delete(x,[idx])
            #y = np.delete(y,[idx])
            #z = np.delete(z,[idx])
            #print "delete: ", ele
    return np.delete(x,indexlist),np.delete(y,indexlist),np.delete(z,indexlist)
    #return x,y,z


def deleteOutliers(x,y,z):
    for arr in x,y,z:
        #print(len(arr))
        elements = np.array(arr)
        offset = 2
        mean = np.mean(elements, axis=0)
        sd = np.std(elements, axis=0)
        for idx,ele in enumerate(arr):
            if(ele < mean-offset*sd) or (ele > mean + offset * sd):
                #delete
                #print("delete",idx, ele)
                x = np.delete(x,[idx])
                y = np.delete(y,[idx])
                z = np.delete(z,[idx])
        return x,y,z

def easygrid(x,y):
    xmin, xmax = min(x), max(x)
    ymin, ymax = min(y), max(y)

    # make coordinate arrays.
    bins=3
    xi      = np.linspace(xmin, xmax, bins+1)
    yi      = np.linspace(ymin, ymax, bins+1)

    l = len(x)

    xbins, ybins = np.zeros(xi.shape), np.zeros(yi.shape)

    for i in range(0,l):
        for xidx in range(0,bins-1):
            if( x[i] >= xi[xidx] and x[i] <= xi[xidx+1]):
                xbins[xidx] +=1
        for yidx in range(0,bins-1):
            if(y[i] >= yi[yidx] and y[i] <= yi[yidx+1]):
                ybins[yidx] +=1
    #print(xbins, ybins, np.argmax(xbins), np.argmax(ybins))
    print(np.argmax(xbins), np.argmax(ybins))

def plotCloudPose(x,y,z, xpose, ypose):
    xmin, xmax = min(x), max(x)
    ymin, ymax = min(y), max(y)
    npts = len(x)
    # define grid.
    xi = np.linspace(xmin,xmax,10)
    yi = np.linspace(ymin,ymax,10)
    # grid the data.
    zi = griddata((x, y), z, (xi[None,:], yi[:,None]),method='nearest')

    #CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors='k')
    #CS = plt.contourf(xi,yi,zi,15,cmap=plt.cm.jet)
    #plt.colorbar() # draw colorbar
    plt.scatter(x,y,marker='o',c='g',s=5)
    plt.scatter(xpose,ypose,marker='*',c='b',s=50)
    plt.plot(xmin, ymin, 'ro-')
    plt.plot(xmin, ymax, 'ro-')
    plt.plot(xmax, ymin, 'ro-')
    plt.plot(xmax, ymax, 'ro-')
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.xlabel('X values')
    plt.ylabel('Y values')
    plt.title('griddata test (%d points)' % npts)
    plt.pause(0.001)
    plt.clf()


def getMaxMapPoint(depth, bins, depthdensity, xi,yi): #change! nimmt immer die gleichen Punkte auf
    pointdata = np.array([])
    maxZ = np.array([])
    patch = np.zeros((4,4))
    #point = Point
    #print "---"
    for i in range(0,3):
        maxindex = np.unravel_index(np.argmax(depthdensity, axis=None), bins.shape) #index of max element of "bins" (where bins is densest = most comment z)
        #z = grid[np.unravel_index(np.argmax(bins, axis=None), bins.shape)] #get depth
        #z = grid[maxindex]
        #print maxindex, depthdensity[maxindex]
        z = depth[maxindex]
        x = xi[maxindex[0]]
        y = yi[maxindex[1]]
        point = Point(x,y,z)
        #print "point: ", point.x, point.y, point.z
        pointdata = np.append(pointdata,point)
        try:
            depthdensity[maxindex[0]-patch.shape[0]/2 : maxindex[0]+patch.shape[0]/2 , maxindex[1]-patch.shape[1]/2 : maxindex[1]+patch.shape[1]/2] = patch
        except ValueError:
            print("Invalid boarder point")
        #depthdensity[maxindex] = 0 #set patch to zero. not only one point
    return pointdata

def getAngle(point, pose):
    angleset = np.array([])
    #only x current add for y
    #print "---", len(point)
    print "---"
    for i in range(0,len(point)):
        #print("map x/z: ", point[i].x, point[i].z, "pose x/z: ", pose.pose.position.x, pose.pose.position.z)
        #z = np.abs(point[i].z-pose.pose.position.z) #distcane (correct?)
        #x = np.abs(point[i].x-pose.pose.position.x)
        #deg = math.degrees(np.arctan(x/z))
        #angleset = np.append(angleset,deg)

        #myradians = math.atan2(point[i].z, point[i].x)
        #myradians = math.atan2(point[i].z-pose.pose.position.z, point[i].x-pose.pose.position.x)
        myradians = math.atan2(point[i].x-pose.pose.position.x, abs(point[i].z-pose.pose.position.z))
        mydegrees = math.degrees(myradians)
        angleset = np.append(angleset,mydegrees)
        print "point: ", point[i].x, point[i].y, point[i].z
        print "pose:  ", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        #print mydegrees, point[i].x, pose.pose.position.x, abs(point[i].z), pose.pose.position.z
        #vector1 = [point[i].x- pose.pose.position.x, point[i].y - pose.pose.position.y] # Vector of aiming of the gun at the target
        #vector2 = [1,0] #vector of X-axis
        #print(angle(vector1, vector2))

        #print mydegrees-90, point[i].x, point[i].z
        #print mydegrees, point[i].x, point[i].z


    return angleset
