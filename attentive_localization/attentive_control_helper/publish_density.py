import rospy
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
from visualization_msgs.msg import Marker
def pub_density(pt):
    '''
    Publishes example pointcloud
    '''
    #rospy.init_node('density_pub')
    pointcloud_publisher = rospy.Publisher("/threshold_density_points", PointCloud, queue_size=1)
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
    #pt_cloud.type = SPHERE
    #filling some points

    pt_cloud.points.append(Point32(pt[0],pt[1],pt[2]))
    #pt_cloud.points.append(Point32(3.0, 3.0, 0.0))
    #publish

    pointcloud_publisher.publish(pt_cloud)
    rospy.sleep(0.01)

def pub_kde(pt,density):
   print density
   pointcloud_publisher = rospy.Publisher("/kde", Marker, queue_size=1)
   marker = Marker()
   marker.header.frame_id = "/map"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = density/500
   marker.scale.y = density/500
   marker.scale.z = density/500
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = pt[0]
   marker.pose.position.y = pt[1]
   marker.pose.position.z = pt[2]
   pointcloud_publisher.publish(marker)

if __name__ == '__main__':
    try:
        pub_density()
    except rospy.ROSInterruptException:
        pass
#[ 0.08553366 -0.04185199 -0.00350745]
