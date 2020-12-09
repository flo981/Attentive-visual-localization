import rospy
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np

def pub_density(pt):
    '''
    Publishes example pointcloud
    '''
    #rospy.init_node('density_pub')
    pointcloud_publisher = rospy.Publisher("/density", PointCloud, queue_size=1)
    #rospy.loginfo("pcl_publish_example")
    #giving some time for the publisher to register
    rospy.sleep(0.01)
    #declaring pointcloud

    pt_cloud = PointCloud()
    tempPt= Point32()
    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pt_cloud.header = header
    #filling some points

    for i in range(0,len(pt)):
        tempPt = Point32(pt[i,0],pt[i,1],pt[i,2])
        #tempPt.x = pt[i,0]
        #tempPt.y = pt[i,1]
        #tempPt.z = pt[i,2]
        pt_cloud.points = np.append(pt_cloud.points, tempPt)
    #pt_cloud.points.append(Point32(2.0, 2.0, 0.0))
    #pt_cloud.points.append(Point32(3.0, 3.0, 0.0))
    #publish

    pointcloud_publisher.publish(pt_cloud)
    rospy.sleep(0.01)

def pub_maxdensity(pt):
    #rospy.init_node('density_pub')
    pointcloud_publisher = rospy.Publisher("/max_density", PointCloud, queue_size=1)
    #rospy.loginfo("pcl_publish_example")
    #giving some time for the publisher to register
    rospy.sleep(0.01)
    #declaring pointcloud

    pt_cloud = PointCloud()
    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pt_cloud.header = header
    #filling some points

    pt_cloud.points.append(Point32(pt[0],pt[1],pt[2]))
    #pt_cloud.points.append(Point32(3.0, 3.0, 0.0))
    #publish

    pointcloud_publisher.publish(pt_cloud)
    rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        pub_density()
    except rospy.ROSInterruptException:
        pass
#[ 0.08553366 -0.04185199 -0.00350745]
