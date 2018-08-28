#!/usr/bin/env python
import rospy
from aruco_mapping.msg import ArucoMarker
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import math as m



#########################################################################################
#########################################################################################
############################### Callback function #######################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# Takes the pose from aruco_poses, rotates the quat(yaw by -pi/2) to match the rviz quat   |
# and publisher the pose to cam_pose_rerouter                                              |
#===========================================================================================

def callback(data):
    global pub
    msg = PoseStamped()

    msg.header.frame_id = 'world'

    msg.header.stamp = rospy.Time.now()

    q_original = [1,2,3,4]

    q_original[0] = data.global_camera_pose.orientation.x
    q_original[1] = data.global_camera_pose.orientation.y
    q_original[2] = data.global_camera_pose.orientation.z
    q_original[3] = data.global_camera_pose.orientation.w

    q_rot = quaternion_from_euler(0, 0, -m.pi/2)
    q_new = quaternion_multiply(q_rot, q_original)

    msg.pose.position.x = data.global_camera_pose.position.x
    msg.pose.position.y = data.global_camera_pose.position.y
    msg.pose.position.z = data.global_camera_pose.position.z

    msg.pose.orientation.x = q_new[0]
    msg.pose.orientation.y = q_new[1]
    msg.pose.orientation.z = q_new[2]
    msg.pose.orientation.w = q_new[3]

    pub.publish(msg)

#########################################################################################



#########################################################################################
#########################################################################################
############################### Initialize ##############################################
#########################################################################################
#########################################################################################

def initialize():
    global pub
    rospy.init_node('cam_pose_rerouter', anonymous=True)
    rospy.Subscriber("aruco_poses", ArucoMarker, callback)
    pub = rospy.Publisher('cam_pose_rerouter', PoseStamped, queue_size = 50)
    rospy.spin()

#########################################################################################


if __name__ == '__main__':
    initialize()
