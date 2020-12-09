import subscriber_ORB as ORB

import numpy as np
from collections import deque
from tf.transformations import euler_from_quaternion
from scipy import stats
from scipy.ndimage import interpolation
from scipy.spatial import Voronoi, SphericalVoronoi, Delaunay, ConvexHull
import matplotlib.pyplot as plt


def plot_weight_mat(weight_matrix, threshold):
    plt.imshow(weight_matrix)
    cbar = plt.colorbar()
    cbar.set_label(r'weight factor')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.title('weight_matrix, T=%.2f' %threshold)
    plt.pause(0.01)
    plt.clf()

def lookup_vel(depth):
    depth = float(depth)
    sleep = np.linspace(0.1,0.025,10)
    dist = np.linspace(1,4,10)
    if depth<=1:
        return sleep[0]
    for idx, ele in enumerate(dist):
        if idx==len(dist)-1:
            return sleep[len(sleep)-1]
        if (depth >= dist[idx] and depth < dist[idx+1]):
            return sleep[idx]


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


def resize_saliencymatrix(sal,sizing):
    sailency_matrix = np.zeros(sal.shape)
    try:
        for idx, row in enumerate(sal):
            sailency_matrix[idx,:] = row
        sailency_matrix = interpolation.zoom(sailency_matrix,sizing)
    except Exception as e:
        print e
    return sailency_matrix

def estimateBin(xpixel,ypixel,xbins,ybins):
    dimx = 640
    dimy = 480
    xbinvec = np.linspace(0,dimx,xbins+1)
    ybinvec = np.linspace(0,dimy,ybins+1)
    for xidx in range(0,xbins+1):
        if xpixel >= xbinvec[xidx] and xpixel <= xbinvec[xidx+1]:
            xbin = xidx

    for yidx in range(0,ybins+1):
        if ypixel >= ybinvec[yidx] and ypixel <= ybinvec[yidx+1]:
            ybin = yidx
    return xbin, ybin

def kde_based_depth():
    cloud, pose    = ORB.cloudSubscriber()
    #depth = 1.3
    npts = len(cloud)
    point_ros = np.zeros(npts)
    for i in range(0,npts):
        point_ros[i] = cloud[i].x
        #point_ros[:,0] = z
        #point_ros[:,1] = -x
        #point_ros[:,2] = -y

    #Density Estimaton
    values = point_ros.T
    #val = np.vstack([values[0,:],values[1,:],values[2,:]])

    kde = stats.gaussian_kde(values)
    kde.set_bandwidth(bw_method='silverman')
    density = kde(values)
    maxIdx = np.argmax(density)
    dist_z =  values[0,maxIdx] -pose.pose.position.x
    return round(abs(dist_z),1)




def get_depth(cloud, pose):#c
        #cloud, pose    = ORB.cloudSubscriber()
        #depth = 1.3
        npts = len(cloud)
        point_ros = np.zeros(npts)
        for i in range(0,npts):
            point_ros[i] = cloud[i].x
            #point_ros[:,0] = z
            #point_ros[:,1] = -x
            #point_ros[:,2] = -y

        #Density Estimaton
        z_map  = np.mean(point_ros)
        z_pose = pose.pose.position.x

        depth = np.linalg.norm([z_map-z_pose])
        if depth > 2:
            depth = 2
        if depth < 0.5:
            depth = 0.5
        #depth = (cloud_depth - pose.pose.position.x)
        #print "mean depth", np.mean(depth)
        return depth

def feature_to_array(pixel_features):
    x = []
    y = []
    for element in pixel_features:
        x = np.append(x,element.x)
        y = np.append(y,element.y)
    return y,x

def cloud_to_array(cloud):#c
    x = []
    y = []
    z = []
    for element in cloud:
        z = np.append(z,element.x)
        x = np.append(x,-element.y)
        y = np.append(y,-element.z)
    return x,y,z


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

def bin2pix(x, y, nbinsx, nbinsy):
    dimx = 640
    dimy = 480
    #nbinsx = 9-1#7-1
    #nbinsy = 7-1#5-1
    xpixel = x * (dimx/nbinsx+1) + (dimx/(2*nbinsx+1))
    ypixel = y * (dimy/nbinsy+1) + (dimy/(2*nbinsy+1))
    return xpixel, ypixel

def bin2pix_v2(x, y, nbinsx, nbinsy):
    dimx = 640
    dimy = 480
    #nbinsx = 9-1#7-1
    #nbinsy = 7-1#5-1
    xbin_width = dimx/(2*float(nbinsx))
    ybin_width = dimy/(2*float(nbinsy))

    # xlin = np.linspace(0,dimx,nbinsx+1)
    # ylin = np.linspace(0,dimy,nbinsy+1)
    # offset since orb corners are not populated thus never hits corner bins

    xlin = np.linspace(-xbin_width,dimx+xbin_width,nbinsx+1)
    ylin = np.linspace(-ybin_width,dimy+ybin_width,nbinsy+1)


    xpixel = xlin[x] + xbin_width
    ypixel = ylin[y] + ybin_width
    return xpixel, ypixel


def pix2degree(xpixel, ypixel):
    #camera aof: 83, stereo camera aof:83/2 (split fot stereo vision)
    angelofview = 82/2
    dimx = 640
    dimy = 480
    linx = np.linspace(0, dimx, angelofview).astype(int)

    liny = np.linspace(0, dimy, angelofview).astype(int)
    xidx = linx[-1]
    yidx = liny[-1]
    for i in range(0,len(linx)-1):
        if linx[i] > xpixel:
            xidx = i-1
            break
    for i in range(0,len(liny)-1):
        if liny[i] > ypixel:
            yidx = i-1
            break
    angle = np.linspace(-angelofview,angelofview,angelofview+1).astype(int)

    return angle[xidx], angle[yidx]

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

#### cuurently unused

def feature_to_matrix(pixel_features):
        orb_matrix = np.zeros([480,640])
        for element in pixel_features:
            orb_matrix[int(element.y),int(element.x)] += 1
        return orb_matrix

def smooth_matrix(weight_matrix):
    sigma_y = 0.2
    sigma_x = 0.2
    sigma = [sigma_y, sigma_x]
    y = sp.ndimage.filters.gaussian_filter(weight_matrix, sigma, mode='constant')
    return y


def densest(array, size):
    density = np.convolve(array, np.ones([size]), mode='valid')
    return np.argmax(density)
