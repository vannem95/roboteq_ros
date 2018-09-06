#!/usr/bin/env python  

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Twist, Vector3
from tf.transformations import *
import math as m
import time



#########################################################################################
#########################################################################################
############################### pose/yaw updater ########################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# updates yaw when new pose is received                                                    |
#===========================================================================================

yaw = 0
pose = [0,0]
initial_pose = [0,0]
initial_yaw = 0
first = 0

def pose_update(data):

    global yaw
    global pose
    global initial_pose
    global initial_yaw
    global first

    quat = [0,0,0,0]

    if first == 1:
        initial_pose = pose
        initial_yaw = yaw
        first = 0

    if (pose[0] == 0) and (pose[1] == 0):
        first = 1


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

    pose[0] = -y
    pose[1] = -x

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Debug mode #######################################
#########################################################################################
#########################################################################################

debug = 1

def debug_printer(debug,variable,variable_value):

    if debug == 1:

        print '======',variable,':',variable_value,'======'

    return 

debug_printer(debug,'DEBUG MODE', 'ACTIVATED')

#########################################################################################



#########################################################################################
#########################################################################################
############################### curve ###################################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# for a given way point, it publishes twist to make the robot go on a curved path to the wp|
#===========================================================================================

distance_tolerance = 0.5 # in meters
velocity = 0.16
waypoint1 = [0.0 , 1.0 , 1.57]
waypoint2 = [0.0 , 1.5 , 1.57]

def curve():

    global pose
    global vel_pub
    global initial_pose
    global distance_tolerance
    global velocity
    global debug
    global rate
    global waypoint1
    global waypoint2

    distance = m.hypot(waypoint1[0] - initial_pose[0],waypoint1[1] - initial_pose[1])

    while (m.hypot(waypoint2[0] - pose[0],waypoint2[1] - pose[1]) > distance_tolerance) and (not rospy.is_shutdown()):

        y_new = pose[1] + m.sqrt((distance^2) - (pose[0]^2))
        wp_new = [0.0 , y_new , 1.57]
        curve_pub(wp_new,pose)
        rate.sleep()

#########################################################################################



#########################################################################################
#########################################################################################
############################### curve pub ###############################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# for a given way point, it publishes twist to make the robot go on a curved path to the wp|
#===========================================================================================

def curve_pub(waypoint,pose):

    global velocity
    global vel_pub
    global debug

    distance = m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1])
    target_theta = m.atan2(waypoint[1] - pose[1],waypoint[0] - pose[0])

    radius = (distance) / (2 * m.sin(target_theta - waypoint[2]))
    angular_vel = velocity / radius

    debug_printer(debug,'distance',distance)
    debug_printer(debug,'target_theta',target_theta)
    debug_printer(debug,'radius',radius)
    debug_printer(debug,'angular_vel',angular_vel)


    msg = Twist()
    msg.linear.x = velocity
    msg.angular.z = -angular_vel
    vel_pub.publish(msg)

#########################################################################################


if __name__ == '__main__':
    rospy.init_node('curve_pp')
    rate = rospy.Rate(25) # 25hz
    rospy.Subscriber('camera_pose',PoseStamped,pose_update)
    rospy.sleep(0.2)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 50)

    curve()

    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

