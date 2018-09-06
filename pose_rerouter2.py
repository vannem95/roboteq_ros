#!/usr/bin/env python
import rospy
from aruco_mapping.msg import ArucoMarker
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import math as m
import numpy as np
from numpy import linalg as la


pos0  = [0,0]
pos1  = [0,0]
first =  1
base_radius = 25 * 0.0254 # 25 inches

#########################################################################################
#########################################################################################
############################### distance function #######################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# Takes 2 cartesian points as lists and returns the distance between them                  |
#===========================================================================================

def dist(a,b):
    a1 = np.array(a)
    b1 = np.array(b)
    x = la.norm(a1-b1)
    return x

#########################################################################################



#########################################################################################
#########################################################################################
############################### Callback function #######################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# Takes the pose from aruco_poses, rotates the quat(yaw by -pi/2) to match the rviz quat,  |
# checks if the new position is less than base radius away (if not, uses the last pose) and|
# publisher the pose to cam_pose_rerouter                                                  |
#===========================================================================================

def callback(data):
    global pub
    global pos0
    global pos1
    global first

    msg = PoseStamped()

    msg.header.frame_id = 'world'

    msg.header.stamp = rospy.Time.now()

    q_original = [0,0,0,0]

    q_original[0] = data.global_camera_pose.orientation.x
    q_original[1] = data.global_camera_pose.orientation.y
    q_original[2] = data.global_camera_pose.orientation.z
    q_original[3] = data.global_camera_pose.orientation.w

    q_rot = quaternion_from_euler(0, 0, -m.pi/2)
    q_new = quaternion_multiply(q_rot, q_original)

    pos1[0] = data.global_camera_pose.position.x
    pos1[1] = data.global_camera_pose.position.y

    if first == 1:
        pos0 = pos1
        first = 0

    if dist(pos0,pos1) > base_radius:
        pos1 = pos0

    [msg.pose.position.x, msg.pose.position.y] = pos1

    msg.pose.position.z = 0.0


    pos0 = pos1

    msg.pose.orientation.x = q_new[0]
    msg.pose.orientation.y = q_new[1]
    msg.pose.orientation.z = q_new[2]
    msg.pose.orientation.w = q_new[3]

    pub.publish(msg)

#########################################################################################



#########################################################################################
#########################################################################################
############################### Initialize function #####################################
#########################################################################################
#########################################################################################

def initialize():
    global pub
    rospy.init_node('cam_pose_rerouter', anonymous=True)
    rospy.Subscriber("aruco_poses", ArucoMarker, callback)
    pub = rospy.Publisher('camera_pose', PoseStamped, queue_size = 50)
    rospy.spin()

#########################################################################################

if __name__ == '__main__':
    initialize()
