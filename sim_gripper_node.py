#!/usr/bin/env python
import rospy
import tf
from visualization_msgs.msg import InteractiveMarkerUpdate
from std_msgs.msg import Bool

#########################################################################################
#########################################################################################
############################### gripper_tf_callback function ############################
#########################################################################################
#########################################################################################


def gripper_tf_callback(data):

    a = data.poses
    if len(a) == 1:
        br = tf.TransformBroadcaster()
        br.sendTransform((a[0].pose.position.x, a[0].pose.position.y, a[0].pose.position.z),
                         (a[0].pose.orientation.x, a[0].pose.orientation.y, a[0].pose.orientation.z,a[0].pose.orientation.w),
                         rospy.Time.now(),
                         "marker_ee_link",
                         "world")

#########################################################################################



#########################################################################################
#########################################################################################
############################### grasp_callback function #################################
#########################################################################################
#########################################################################################

gripper_pose = 0


def grasp():

    global reg
    global rev


    global regfp
    global regfm
    global revf

    global base

    global l1l
    global l2l
    global l3l

    global r1l
    global r2l
    global r3l

    global l1c
    global l2c
    global l3c

    global r1c
    global r2c
    global r3c

    global li1
    global li2
    global li3


    global lf1
    global lf2
    global lf3

    global ri1
    global ri2
    global ri3

    global rf1
    global rf2
    global rf3

    global gripper_pose


    if gripper_pose == 1:
        # for i in range(100):
        br = tf.TransformBroadcaster()









# <!- <node name="temporary_publisher3" pkg="tf" type="static_transform_publisher" args="0.061, 0.013, 0.0, 0.967, -0.256, 0.0, 0.0 robotiq_85_base_link robotiq_85_left_inner_knuckle_link 100"/>
#     <node name="temporary_publisher4" pkg="tf" type="static_transform_publisher" args="0.055, 0.031, 0.000, 0.967, -0.256, 0.0, 0.0 robotiq_85_base_link robotiq_85_left_knuckle_link 100"/>
#     <node name="temporary_publisher5" pkg="tf" type="static_transform_publisher" args="0.061, -0.013, 0.0, 0.0, 0.0, 0.256, 0.967 robotiq_85_base_link robotiq_85_right_inner_knuckle_link 100"/>
#     <node name="temporary_publisher6" pkg="tf" type="static_transform_publisher" args="0.055, -0.031, 0.0, 0.0, 0.0, 0.256, 0.967 robotiq_85_base_link robotiq_85_right_knuckle_link 100"/>
#  -->












        br.sendTransform((0.061, 0.013, 0.0),
                         (0.967, -0.256,0.0,0.0),
                         rospy.Time.now(),
                         'robotiq_85_left_inner_knuckle_link',
                         'robotiq_85_base_link')



        br.sendTransform((0.055, 0.031, 0.0),
                         (0.967, -0.256,0.0,0.0),
                         rospy.Time.now(),
                         'robotiq_85_left_knuckle_link',
                         'robotiq_85_base_link')



        br.sendTransform((0.061, -0.013, 0.0),
                         (0.0,0.0,0.256, 0.967),
                         rospy.Time.now(),
                         'robotiq_85_right_inner_knuckle_link',
                         'robotiq_85_base_link')



        br.sendTransform((0.055, -0.031, 0.0),
                         (0.0,0.0,0.256, 0.967),
                         rospy.Time.now(),
                         'robotiq_85_right_knuckle_link',
                         'robotiq_85_base_link')





# <!--    <node name="temporary_publisher7" pkg="tf" type="static_transform_publisher" args="0.043 -0.038 0.0 0.0 0.0 -0.256, 0.967 robotiq_85_left_inner_knuckle_link robotiq_85_left_finger_tip_link 100"/> -->

# <!--    <node name="temporary_publisher8" pkg="tf" type="static_transform_publisher" args="-0.004 -0.031 0.0 0.0 0.0 0.0 1.0 robotiq_85_left_knuckle_link robotiq_85_left_finger_link 100"/>
#  -->
# <!--    <node name="temporary_publisher9" pkg="tf" type="static_transform_publisher" args="0.043 -0.038 0.0 0.0 0.0 -0.256, 0.967 robotiq_85_right_inner_knuckle_link robotiq_85_right_finger_tip_link 100"/>
#  -->
#    <!-- <node name="temporary_publisher10" pkg="tf" type="static_transform_publisher" args="-0.004 -0.031 0.0 0 0 0 1 robotiq_85_right_knuckle_link robotiq_85_right_finger_link 100"/> -->









        br.sendTransform((0.043, -0.038, 0.0),
                         (0,0,-0.256, 0.967),
                         rospy.Time.now(),
                         'robotiq_85_left_finger_tip_link',
                         'robotiq_85_left_inner_knuckle_link')



        br.sendTransform((-0.004, -0.031, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_left_finger_link',
                         'robotiq_85_left_knuckle_link')




        br.sendTransform((0.043, -0.038, 0.0),
                         (0,0,-0.256, 0.967),
                         rospy.Time.now(),
                         'robotiq_85_right_finger_tip_link',
                         'robotiq_85_right_inner_knuckle_link')



        br.sendTransform((-0.004, -0.031, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_right_finger_link',
                         'robotiq_85_right_knuckle_link')




    elif gripper_pose == 0:
        # for i in range(100):
        br = tf.TransformBroadcaster()




 #    <node name="temporary_publisher3" pkg="tf" type="static_transform_publisher" args="0.061, 0.013, 0.0 1 0 0 0 robotiq_85_base_link robotiq_85_left_inner_knuckle_link 100"/>
 #    <node name="temporary_publisher4" pkg="tf" type="static_transform_publisher" args="0.055, 0.031, 0.0 1 0 0 0 robotiq_85_base_link robotiq_85_left_knuckle_link 100"/>
 #    <node name="temporary_publisher5" pkg="tf" type="static_transform_publisher" args="0.061, -0.013, 0.0 0.0 0 0 robotiq_85_base_link robotiq_85_right_inner_knuckle_link 100"/>
 #    <node name="temporary_publisher6" pkg="tf" type="static_transform_publisher" args="0.055, -0.031, 0.0 0.0 0 0 robotiq_85_base_link robotiq_85_right_knuckle_link 100"/>



 #    <node name="temporary_publisher7" pkg="tf" type="static_transform_publisher" args="0.043, -0.038, 0.0 0.0 0 0 robotiq_85_left_inner_knuckle_link robotiq_85_left_finger_tip_link 100"/>

 #    <node name="temporary_publisher8" pkg="tf" type="static_transform_publisher" args="-0.004, -0.031, 0.0 0.0 0 0 robotiq_85_left_knuckle_link robotiq_85_left_finger_link 100"/>

 #    <node name="temporary_publisher9" pkg="tf" type="static_transform_publisher" args="0.043, -0.038, 0.0 0.0 0 0 robotiq_85_right_inner_knuckle_link robotiq_85_right_finger_tip_link 100"/>

 #   <node name="temporary_publisher10" pkg="tf" type="static_transform_publisher" args="-0.004, -0.031, 0.0 0.0 0 0 robotiq_85_right_knuckle_link robotiq_85_right_finger_link 100"/>
 # -->




        br.sendTransform((0.061, 0.013, 0.0),
                         (1,0,0,0),
                         rospy.Time.now(),
                         'robotiq_85_left_inner_knuckle_link',
                         'robotiq_85_base_link')



        br.sendTransform((0.055, 0.031, 0.0),
                         (1,0,0,0),
                         rospy.Time.now(),
                         'robotiq_85_left_knuckle_link',
                         'robotiq_85_base_link')



        br.sendTransform((0.061, -0.013, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_right_inner_knuckle_link',
                         'robotiq_85_base_link')



        br.sendTransform((0.055, -0.031, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_right_knuckle_link',
                         'robotiq_85_base_link')


        br.sendTransform((0.043, -0.038, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_left_finger_tip_link',
                         'robotiq_85_left_inner_knuckle_link')



        br.sendTransform((-0.004, -0.031, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_left_finger_link',
                         'robotiq_85_left_knuckle_link')




        br.sendTransform((0.043, -0.038, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_right_finger_tip_link',
                         'robotiq_85_right_inner_knuckle_link')



        br.sendTransform((-0.004, -0.031, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         'robotiq_85_right_finger_link',
                         'robotiq_85_right_knuckle_link')




#########################################################################################

reg = [0,0,0,1]
rev = [1,0,0,0]


regfp = [0,0,0.256,0.967]
regfm = [0,0,-0.256,0.967]
revf = [0.967,-0.256,0,0]

base = 'robotiq_85_base_link'

l1l = 'robotiq_85_left_inner_knuckle_link'
l2l = 'robotiq_85_left_knuckle_link'
l3l = 'robotiq_85_left_finger_tip_link'

r1l = 'robotiq_85_right_inner_knuckle_link'
r2l = 'robotiq_85_right_knuckle_link'
r3l = 'robotiq_85_right_finger_tip_link'

l1c = [0.061 , 0.013 , 0.0]
l2c = [0.055 , 0.031 , 0.0]
l3c = [0.043 ,-0.038 , 0.0]

r1c = [0.061 ,-0.013 , 0.0]
r2c = [0.055 ,-0.031 , 0.0]
r3c = [0.043 ,-0.038 , 0.0]

li1 = [ l1c , rev , [base , l1l] ] 
li2 = [ l2c , rev , [base , l2l] ]
li3 = [ l3c , reg , [l1l , l3l] ]


lf1 = [ l1c , revf , [base , l1l] ]
lf2 = [ l2c , revf , [base , l2l] ]
lf3 = [ l3c , regfm , [l1l , l3l] ]

ri1 = [ r1c , reg , [base , r1l] ]
ri2 = [ r2c , reg , [base , r2l] ]
ri3 = [ r3c , reg , [r1l , r3l] ]

rf1 = [ r1c , regfp , [base , r1l] ]
rf2 = [ r2c , regfp , [base , r2l] ]
rf3 = [ r3c , regfm , [r1l , r3l] ]



    # args="0.061 0.013 0.0 1 0 0.0 0.0 robotiq_85_base_link robotiq_85_left_inner_knuckle_link 100"/>

    # args="0.055, 0.031, 0.000 1 0 0.0 0.0 robotiq_85_base_link robotiq_85_left_knuckle_link 100"/>

    # args="0.061 -0.013 0.0 0.0 0.0 0 1 robotiq_85_base_link robotiq_85_right_inner_knuckle_link 100"/>

    # args="0.055 -0.031 0.0 0.0 0.0 0.0 1 robotiq_85_base_link robotiq_85_right_knuckle_link 100"/>

    # args="0.043 -0.038 0.0 0.0 0.0 0.0 1 robotiq_85_left_inner_knuckle_link robotiq_85_left_finger_tip_link 100"/>

    # args="0.043 -0.038 0.0 0.0 0.0 0.0 1 robotiq_85_right_inner_knuckle_link robotiq_85_right_finger_tip_link 100"/>






   #  <node name="temporary_publisher8" pkg="tf" type="static_transform_publisher" args="-0.004 -0.031 0.0 0.0 0.0 0.0 1.0 robotiq_85_left_knuckle_link robotiq_85_left_finger_link 100"/>

   # <node name="temporary_publisher10" pkg="tf" type="static_transform_publisher" args="-0.004 -0.031 0.0 0 0 0 1 robotiq_85_right_knuckle_link robotiq_85_right_finger_link 100"/>





   #      br.sendTransform((-0.004, -0.031, 0.0),
   #                       (0.0,0.0,0.0,1.0),
   #                       rospy.Time.now(),
   #                       'robotiq_85_left_finger_link',
   #                       'robotiq_85_left_knuckle_link')

   #      br.sendTransform((-0.004, -0.031, 0.0),
   #                       (0.0,0.0,0.0,1.0),
   #                       rospy.Time.now(),
   #                       'robotiq_85_right_finger_link',
   #                       'robotiq_85_right_knuckle_link')




    # args="0.061 0.013 0.0 0.967 -0.256 0.0 0.0 robotiq_85_base_link robotiq_85_left_inner_knuckle_link 100"/>

    # args="0.055, 0.031, 0.000 0.967 -0.256 0.0 0.0 robotiq_85_base_link robotiq_85_left_knuckle_link 100"/>

    # args="0.061 -0.013 0.0 0.0 0.0 0.256 0.967 robotiq_85_base_link robotiq_85_right_inner_knuckle_link 100"/>

    # args="0.055 -0.031 0.0 0.0 0.0 0.256 0.967 robotiq_85_base_link robotiq_85_right_knuckle_link 100"/>

    # args="0.043 -0.038 0.0 0.0 0.0 -0.256 0.967 robotiq_85_left_inner_knuckle_link robotiq_85_left_finger_tip_link 100"/>

    # args="0.043 -0.038 0.0 0.0 0.0 -0.256 0.967 robotiq_85_right_inner_knuckle_link robotiq_85_right_finger_tip_link 100"/>





def pose_setter(data):

    global gripper_pose

    if data.data == True:
        gripper_pose = 1
    elif data.data == False:
        gripper_pose = 0
    else:
        print "error"






#########################################################################################
#########################################################################################
############################### Initialize function #####################################
#########################################################################################
#########################################################################################

def initialize():
    rospy.init_node('marker_pose_rerouter_node', anonymous=True)
    rospy.Subscriber("rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", InteractiveMarkerUpdate, gripper_tf_callback)
    rospy.Subscriber("sim_gripper", Bool, pose_setter)

#########################################################################################

if __name__ == '__main__':
    initialize()
    while (not rospy.is_shutdown()):
        grasp()

