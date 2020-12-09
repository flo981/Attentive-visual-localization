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

    def move_robot_x(self, degree, xoffset, distance):
        #distance = 1.5
        if degree < xoffset:
            inc = -self.resolution
        else:
            inc = self.resolution
        #print xoffset, degree
        while xoffset != degree:
            xoffset = xoffset+inc
            self.move.angular.z=xoffset
            #print "x: ", xoffset, " - goal: ",degree, " sleep: ", distance,
            self.publish_vel()

            time.sleep(distance)

    def move_robot_y(self, degree, yoffset, distance):
        #distance = 1.5
        if degree < yoffset:
            inc = -self.resolution
        else:
            inc = self.resolution
        while yoffset != degree:
            yoffset = yoffset+inc
            self.move.angular.x=yoffset
            self.move.angular.y=-yoffset
            #print "y: ", yoffset, " - goal: ",degree, " sleep: ", distance
            self.publish_vel()
            time.sleep(distance)



def move_robot(degx, degz, xoffset):
    mov = movement(degz, degx)
    print mov
    mov.move_robot_x(degx, xoffset)
