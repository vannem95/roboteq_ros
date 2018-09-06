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

    if first == 1:
        initial_pose = pose
        initial_yaw = yaw
        first = 0

    if (pose[0] == 0) and (pose[1] == 0):
        first = 1


    yaw = data.point.z

    pose[0] = data.point.x
    pose[1] = data.point.y

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
############################### curve1 ###################################################
#########################################################################################
#########################################################################################

theta_tolerance1 = 0.1
distance_tolerance1 = 0.2 # in meters
velocity1 = 0.16

#=========================================================================================== 
# for a given way point, turn and move forward and turn to goal yaw                         |
#===========================================================================================

def curve1(waypoint):

    global yaw
    global pose
    global vel_pub
    global initial_pose
    global initial_yaw
    global distance_tolerance1
    global velocity1
    global debug
    global rate

    distance = m.hypot(waypoint[0] - initial_pose[0],waypoint[1] - initial_pose[1])
    target_theta = m.atan2(waypoint[1] - initial_pose[1],waypoint[0] - initial_pose[0])
    msg = Twist()
    while (abs(yaw - target_theta) > theta_tolerance1):
        msg.angular.z = -velocity1
        vel_pub.publish(msg)
        rate.sleep()

    msg.angular.z = 0
    vel_pub.publish(msg)
    
    while (m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1]) > distance_tolerance1):
        msg.linear.x = velocity1
        vel_pub.publish(msg)
        rate.sleep()
    msg.linear.x = 0
    msg.angular.z = 0
    vel_pub.publish(msg)

    rospy.sleep(1)

#########################################################################################


#########################################################################################
#########################################################################################
############################### curve ###################################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# for a given way point, it publishes twist to make the robot go on a curved path to the wp|
#===========================================================================================

distance_tolerance = 0.2 # in meters
velocity = 0.16
update_time = 2.0 # new way point every "update_time" seconds

def curve(waypoint):

    global yaw
    global pose
    global vel_pub
    global initial_pose
    global initial_yaw
    global distance_tolerance
    global velocity
    global debug
    global rate
    global update_time

    distance = m.hypot(waypoint[0] - initial_pose[0],waypoint[1] - initial_pose[1])
    target_theta = m.atan2(waypoint[1] - initial_pose[1],waypoint[0] - initial_pose[0])

    radius = (distance) / (2 * m.sin(target_theta - waypoint[2]))
    angular_vel = velocity / radius

    debug_printer(debug,'distance',distance)
    debug_printer(debug,'target_theta',target_theta)
    debug_printer(debug,'radius',radius)
    debug_printer(debug,'angular_vel',angular_vel)
    debug_printer(debug,'pose,yaw',[pose,yaw])
    debug_printer(debug,'ini - pose,yaw',[initial_pose,initial_yaw])


    msg = Twist()
    msg.linear.x = velocity

    t_initial = rospy.get_time()

    while ( rospy.get_time() - t_initial > update_time) and (not rospy.is_shutdown()):

        debug_printer(debug,'angular_vel',angular_vel)
        debug_printer(debug,'distance',[m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1])])
        debug_printer(debug,'pose,yaw',[pose,yaw])

        msg.angular.z = -angular_vel
        vel_pub.publish(msg)
        rate.sleep()

    initial_pose = pose

#########################################################################################



if __name__ == '__main__':
    rospy.init_node('curve_multiple_wp')
    rate = rospy.Rate(25) # 25hz
    rospy.Subscriber('base_pose',PointStamped,pose_update)
    rospy.sleep(0.2)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 50)
    waypoints = [[0.0 , 1.0 , 1.57],[0.0 , 1.1 , 1.57],[0.0 , 1.2 , 1.57],[0.0 , 1.3 , 1.57],[0.0 , 1.4 , 1.57]]
    curve1(waypoints[0])
    curve(waypoints[1])
    curve(waypoints[2])
    curve(waypoints[3])
    curve(waypoints[4])

    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)
