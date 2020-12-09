#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

joint_data = Twist()

def callback_joint(data):
    global joint_data
    joint_data = data

def joint_Subsriber():
    rospy.Subscriber("/joint_pos", Twist, callback_joint)
    return joint_data

def traj_lin(length):
    pub = rospy.Publisher("groundtruth", PoseStamped, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    traj = PoseStamped()
    #while not rospy.is_shutdown():

    joint = Twist()

    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"

        joint = joint_Subsriber()
        x,y,z,w = quaternion_from_euler(0,math.radians(-joint.angular.y),math.radians(-joint.angular.z))
        traj.pose.position.x = 0.0
        traj.pose.position.y = -i/float(100)
        traj.pose.position.z = 0.0


        traj.pose.orientation.x = x#math.radians(x)
        traj.pose.orientation.y = y#math.radians(y)
        traj.pose.orientation.z = z#math.radians(z)
        traj.pose.orientation.w = w#math.radians(w)
        print i
        pub.publish(traj)
        rate.sleep()

def traj_rectangle_old(length):
    #goal = start
    pub = rospy.Publisher("groundtruth", PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    traj = PoseStamped()
    #move in x
    h = 0
    joint = Twist()
    start = time.time()
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"

        joint = joint_Subsriber()
        x,y,z,w = quaternion_from_euler(0,math.radians(-joint.angular.y),math.radians(-joint.angular.z))

        traj.pose.position.x = 0
        traj.pose.position.y = -i/float(100)
        traj.pose.position.z = h

        traj.pose.orientation.x = x#math.radians(x)
        traj.pose.orientation.y = y#math.radians(y)
        traj.pose.orientation.z = z#math.radians(z)
        traj.pose.orientation.w = w#math.radians(w)
        pub.publish(traj)
        print 1, i
        rate.sleep()

    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    #move in y keep x
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"
        joint = joint_Subsriber()
        x,y,z,w = quaternion_from_euler(0,math.radians(-joint.angular.y),math.radians(-joint.angular.z))

        traj.pose.position.x = -i/float(100)
        traj.pose.position.y = -length/float(100)
        traj.pose.position.z = h

        traj.pose.orientation.x = x#math.radians(x)
        traj.pose.orientation.y = y#math.radians(y)
        traj.pose.orientation.z = z#math.radians(z)
        traj.pose.orientation.w = w#math.radians(w)
        pub.publish(traj)
        print 2, i

        rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    #move back in x keep y
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"
        joint = joint_Subsriber()
        x,y,z,w = quaternion_from_euler(0,math.radians(-joint.angular.y),math.radians(-joint.angular.z))

        traj.pose.position.x = -length/float(100)
        traj.pose.position.y = -(length-i)/float(100)
        traj.pose.position.z = h

        traj.pose.orientation.x = x#math.radians(x)
        traj.pose.orientation.y = y#math.radians(y)
        traj.pose.orientation.z = z#math.radians(z)
        traj.pose.orientation.w = w#math.radians(w)
        pub.publish(traj)
        print 3, i
        rate.sleep()
    #move back in y keep x
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"
        joint = joint_Subsriber()
        x,y,z,w = quaternion_from_euler(0,math.radians(-joint.angular.y),math.radians(-joint.angular.z))

        traj.pose.position.x = -(length-i)/float(100)
        traj.pose.position.y = 0
        traj.pose.position.z = h

        traj.pose.orientation.x = x#math.radians(x)
        traj.pose.orientation.y = y#math.radians(y)
        traj.pose.orientation.z = z#math.radians(z)
        traj.pose.orientation.w = w#math.radians(w)
        pub.publish(traj)
        print 4, i
        rate.sleep()
    print (time.time()-start)*1000




def traj_rectangle_new(length):
    #goal = start
    pub = rospy.Publisher("groundtruth", PoseStamped, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(8) # 10hz
    traj = PoseStamped()
    #move in x
    h = 0#72/float(100)
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"

        traj.pose.position.x = 0.0
        traj.pose.position.y = i/float(100)
        traj.pose.position.z = h

        traj.pose.orientation.x = 0.0
        traj.pose.orientation.y = 0.0
        traj.pose.orientation.z = 0.0
        traj.pose.orientation.w = 0.0
        pub.publish(traj)
        print 1, i
        rate.sleep()

    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    #move in y keep x
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"

        traj.pose.position.x = i/float(100)
        traj.pose.position.y = length/float(100)
        traj.pose.position.z = h

        traj.pose.orientation.x = 0.0
        traj.pose.orientation.y = 0.0
        traj.pose.orientation.z = 0.0
        traj.pose.orientation.w = 0.0
        pub.publish(traj)
        print 2, i

        rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    #move back in x keep y
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"

        traj.pose.position.x = length/float(100)
        traj.pose.position.y = (length-i)/float(100)
        traj.pose.position.z = h

        traj.pose.orientation.x = 0.0
        traj.pose.orientation.y = 0.0
        traj.pose.orientation.z = 0.0
        traj.pose.orientation.w = 0.0
        pub.publish(traj)
        print 3, i
        rate.sleep()
    #move back in y keep x
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    for i in range(0,length):
        traj.header.seq = 1
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "map"

        traj.pose.position.x = (length-i)/float(100)
        traj.pose.position.y = 0
        traj.pose.position.z = h

        traj.pose.orientation.x = 0.0
        traj.pose.orientation.y = 0.0
        traj.pose.orientation.z = 0.0
        traj.pose.orientation.w = 0.0
        pub.publish(traj)
        print 4, i
        rate.sleep()

    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()



if __name__ == '__main__':
    try:
        #traj_lin(110)
        traj_rectangle_old(150)
    except rospy.ROSInterruptException:
        pass
