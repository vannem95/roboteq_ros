#!/usr/bin/env python  

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Twist, Vector3

t = tf.Transformer()

def tf_broadcaster(msg):

    global vel_pub
    global t

    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     msg.header.frame_id)

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0,0,0),
                     (0,0,0,1),
                     rospy.Time.now(),
                     "map",
                     msg.header.frame_id)

    try:

        m = TransformStamped()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.child_frame_id = "base_link"
        m.transform.translation.x = msg.pose.position.x
        m.transform.translation.y = msg.pose.position.y
        m.transform.translation.z = msg.pose.position.z

        m.transform.rotation.x = msg.pose.orientation.x
        m.transform.rotation.y = msg.pose.orientation.y
        m.transform.rotation.z = msg.pose.orientation.z
        m.transform.rotation.w = msg.pose.orientation.w

        t.setTransform(m)

        [linear,angular] = t.lookupTwist("base_link", "map", rospy.Time(0.0), rospy.Duration(1.001))

        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.twist = Twist(Vector3(linear[0], linear[1], linear[2]), Vector3(angular[0], angular[1], angular[2]))

        vel_pub.publish(twist)

    except Exception as e:
        print 'vel pub error'
     # catch *all* exceptions

if __name__ == '__main__':
    rospy.init_node('tf')
    vel_pub = rospy.Publisher('camera_vel', TwistStamped, queue_size=50)
    rospy.Subscriber('camera_pose',PoseStamped,tf_broadcaster)
    rospy.spin()
