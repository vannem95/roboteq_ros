#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
import math as m
import numpy as np
from tf.transformations import *


#########################################################################################
#########################################################################################
############################### Callback function #######################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# takes the camera_pose flips the axes to ground and does the translation from camera to base
# 0.55 inches to the right and 15.675 inches to the bottom
#===========================================================================================

def callback(data):
    global pub
    global pos0
    global pos1
    global first

    in_to_m = 0.0254

    msg = PointStamped()

    msg.header.stamp = rospy.Time.now()

    quat = [0,0,0,0]

    quat[0] = data.pose.orientation.x
    quat[1] = data.pose.orientation.y
    quat[2] = data.pose.orientation.z
    quat[3] = data.pose.orientation.w

    x = data.pose.position.x
    y = data.pose.position.y

    euler = euler_from_quaternion(quat)
    yaw = euler[2]

    if   (-m.pi < yaw)      and (yaw < -0.5* m.pi):
        yaw = - (yaw + 0.5*m.pi)
    elif (-0.5* m.pi < yaw) and (yaw < 0):
        yaw = - (yaw + 0.5*m.pi)
    elif (0 < yaw)          and (yaw < 0.5* m.pi):
        yaw = - (yaw + 0.5*m.pi)
    elif (0.5* m.pi < yaw)  and (yaw < m.pi):
        yaw = - (yaw - 1.5*m.pi)
    else:
        yaw = yaw

    x_real = (-y) + (0.55 * in_to_m)
    y_real = (-x) - (15.675 * in_to_m)

    msg.point.x = x_real
    msg.point.y = y_real
    msg.point.z = yaw
    
    pub.publish(msg)

#########################################################################################



#########################################################################################
#########################################################################################
############################### Initialize function #####################################
#########################################################################################
#########################################################################################

def initialize():
    global pub
    rospy.init_node('base_pose_pub', anonymous=True)
    rospy.Subscriber("camera_pose", PoseStamped, callback)
    pub = rospy.Publisher('base_pose', PointStamped, queue_size = 50)
    rospy.spin()

#########################################################################################

if __name__ == '__main__':
    initialize()
