#!/usr/bin/env python

import serial
import time
import io
import rospy
import tf
import math as m
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#########################################################################################
#########################################################################################
######################### Roboteq Controller Serial Connection ##########################
#########################################################################################
#########################################################################################

# begin the connection to the roboteq controller
try:

    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200, #8N1
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
    )

except:

    raise IOError

# reset the connection if need be
if (ser.isOpen()):

    ser.close()

ser.open()

# serial readline based on \r 
slash_r = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),  
                               newline = '\r',
                               line_buffering = True)

slash_r.write(unicode("ID\r"))

#########################################################################################



#########################################################################################
#########################################################################################
##################################### Initialize ########################################
#########################################################################################
#########################################################################################

def initialize():

    global odom_pub
    global odom_broadcaster
    global r
    global current_time
    global last_time
    global enc_last
    global pub_time
    global test_wheels

    rospy.init_node('odometry_publisher3')
    rospy.loginfo('status: odometry_publisher crearted')
    rospy.Subscriber('/cmd_vel', Twist, move_callback)

    # odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    # odom_broadcaster = tf.TransformBroadcaster()

    rospy.loginfo('status: odom_pub initiated')
    r = rospy.Rate(25.0)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    pub_time = rospy.get_time()

    getRCnENC()
    enc_last = test_wheels

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Debug mode #######################################
#########################################################################################
#########################################################################################

debug = 0
encoder_debug = 0
rc_debug = 0
mode_debug = 0
speed_debug = 0
reader_debug = 0
getdata_debug = 0
test_debug = 0
rc_enc_debug = 0
motor_command_debug = 1
callback_debug = 1

def debug_printer(debug,variable,variable_value):

    if debug == 1:

        print '======',variable,':',variable_value,'======'

    return 

debug_printer(debug,'DEBUG MODE', 'ACTIVATED')

#########################################################################################



#########################################################################################
#########################################################################################
############################### Get Data and Clean message ##############################
#########################################################################################
#########################################################################################

def getdata3(number_of_commands):

    global debug
    global getdata_debug

    line = slash_r.readline()
    output = []

    while (line == '\r') or (line == '') or (line == '+\r'):

        line = slash_r.readline()

    start = 1

    for i in range(number_of_commands):

        if start == 0:

            line = slash_r.readline()

        if line[0] == '@':

            output.append(str(line))

        start = 0

    debug_printer(debug,'getdata3 - funtion output',output)
    debug_printer(getdata_debug,'getdata3 - funtion output',output)

    return output

def clean_messages(message):

# Encoder output looks like this - "01 C=123456\r"
# This function reads output like that and returns "123456"

    clean_message = ''
    start = 0

    try:

        for i in range(0,len(message)):

            if (message[i]== '='):

                start = 1

            if (message[i]== '\r'):

                clean_message = int(clean_message)
                return clean_message

            if (start == 1):

                clean_message += message[i+1]

    except:

        return clean_message

#########################################################################################



#########################################################################################
#########################################################################################
####################################### reader ##########################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# --> Arguments - 1. query_id                                                              |                                                               |
#                                                                                          |
#   1. query_id    - Roboteq has about 20 or so values that can be queried but the current |
#                    version of function can get pulse input(pulse-PI), absolute encoder   | 
#                    value(enc_abs - C), relative encoder rpm(enc_rel_rpm - RS) and battery|
#                    voltage                                                               |
#                                                                                          |
# --> Output - The function returns what is read from the serial port for the command sent |
#===========================================================================================

command_merger = '_'

def reader(query_id):

    global debug
    global command_merger
    global reader_debug

    broadcast = ''
    number_of_nodes = 0
    number_of_channels = 0
    can_node_id = 1
    channel = 1
    command = ''

    if query_id == 'pulse':

        broadcast = '@0'
        q_id = 'PI'
        error = 0
        number_of_nodes = 1
        number_of_channels = 2
        can_node_id = 1

    elif query_id == 'battery':

        q_id = 'V'
        broadcast = '@0'
        channel = 2
        error = 0
        number_of_nodes = 2
        number_of_channels = 1

    elif query_id == 'enc_abs':

        q_id = 'C'
        broadcast = '@0'
        error = 0
        number_of_nodes = 2
        number_of_channels = 2

    elif query_id == 'enc_rel_rpm':

        q_id = 'SR'
        broadcast = '@0'
        error = 0
        number_of_nodes = 2
        number_of_channels = 2

    else:

        error = 1
        print 'error: query_id unknown'

    if error == 0:

        for i in range(number_of_nodes):

            c1 = broadcast + str(can_node_id + i) + '?' + q_id
            command += c1

            if number_of_channels != 1:

                for j in range(number_of_channels):

                    c2 = str(' ' + str(channel+j)) + command_merger
                    command += (j*c1) + c2 

            else:

                command += command_merger

        command += '\r'

        debug_printer(debug,'reader - command',command)
        debug_printer(reader_debug,'reader - command',command)

        time.sleep(0.0015)
        ser.write(command)  
        time.sleep(0.0015)
        command_count = (2*number_of_nodes*number_of_channels) -1
        output = getdata3(command_count)

    else:

        output = 'error: incorrect arguments for reader function'

    return output

#########################################################################################



#########################################################################################
#########################################################################################
###################################### RC Input #########################################
#########################################################################################
#########################################################################################

pulse1 = 1490
pulse2 = 1490

min_forward_pulse1 = 1510 
min_backward_pulse1 = 1480
min_forward_pulse2 = 1514 
min_backward_pulse2 = 1484

max_pulse = 1917 
min_pulse = 1070

communication_error = 0

def getRCInput():

    global pulse1
    global pulse2
    global max_pulse
    global min_pulse
    global debug
    global rc_debug
    global test_debug
    global communication_error

    try:

        pulses = reader('pulse')    
        pulse1 = clean_messages(pulses[0])
        pulse2 = clean_messages(pulses[1])

        if (pulse1 > max_pulse) or (pulse2 > max_pulse) or (pulse1 < min_pulse) or (pulse2 < min_pulse):

            communication_error = 1
            pulse1 = 1495
            pulse2 = 1492

    except Exception as e: # catch *all* exceptions

        print e
        print( "error: getRCInput" )

    debug_printer(debug,'getRCInput - pulses',pulses)
    debug_printer(rc_debug,'getRCInput - pulses',pulses)
    debug_printer(test_debug,'getRCInput - pulses',pulses)

    return pulse1, pulse2

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Pulse 2 Speed ####################################
#########################################################################################
#########################################################################################

def pulse2vel(pulses):

    global min_forward_pulse1 
    global min_backward_pulse1
    global min_forward_pulse2 
    global min_backward_pulse2

    signs = [0,0]

    if (pulses[0] > min_forward_pulse1):

        speed = pulses[0] - min_forward_pulse1
        signs[0] = 1
        signs[1] = -1
        left  = speed * signs[0]
        right = speed * signs[1] 

    elif (pulses[0] < min_backward_pulse1):

        speed =  -(pulses[0] - min_backward_pulse1)
        signs[0] = -1
        signs[1] = 1
        left  = speed * signs[0]
        right = speed * signs[1]

    elif (pulses[1] > min_forward_pulse2):

        speed = pulses[1] - min_forward_pulse2
        signs[0] = 1
        signs[1] = 1
        left  = speed * signs[0]
        right = speed * signs[1]

    elif (pulses[1] < min_backward_pulse1):

        speed =  -(pulses[1] - min_backward_pulse2)
        signs[0] = -1
        signs[1] = -1
        left  = speed * signs[0]
        right = speed * signs[1]

    else:

        speed = 0
        signs[0] = 1
        signs[1] = 1
        left  = speed * signs[0]
        right = speed * signs[1]

    return left,right

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Get Encoders #####################################
#########################################################################################
#########################################################################################

def getEncoders():

    global debug
    global encoder_debug
    global test_debug

    leftWheel = [0,0]
    rightWheel = [0,0]

    encoder_absolute = reader('enc_abs')

    encoder_values = [clean_messages(encoder_absolute[0]),clean_messages(encoder_absolute[1]),clean_messages(encoder_absolute[2]),clean_messages(encoder_absolute[3])]
    
    leftWheel = [encoder_values[0],encoder_values[2]]
    rightWheel = [encoder_values[1],encoder_values[3]]

    debug_printer(debug,'getEncoders - encoder_values',encoder_values)
    debug_printer(encoder_debug,'getEncoders - encoder_values',encoder_values)

    return leftWheel, rightWheel

#########################################################################################



#########################################################################################
#########################################################################################
###################################### RC + Encoders ####################################
#########################################################################################
#########################################################################################

def getRCnENC():

    global debug
    global encoder_debug
    global rc_enc_debug
    global test_pulsevals
    global test_wheels

#  command : @01?PI 1_@01?PI 2_@01?C 1_@01?C 2_@02?C 1_@02?C 2_

    leftWheel = [0,0]
    rightWheel = [0,0]
    pulse1 = 1495
    pulse2 = 1492

    command = "@01?PI 1_@01?PI 2_@01?C 1_@01?C 2_@02?C 1_@02?C 2_\r"

    time.sleep(0.0015)
    ser.write(command)  
    time.sleep(0.0015)

    command_count = (2*6) - 1
    output = getdata3(command_count)

    test_pulsevals = [clean_messages(output[0]),clean_messages(output[1])]
    test_encodervals = [clean_messages(output[2]),clean_messages(output[3]),clean_messages(output[4]),clean_messages(output[5])]

    leftWheel = [test_encodervals[0],test_encodervals[2]]
    rightWheel = [test_encodervals[1],test_encodervals[3]]

    test_wheels = [leftWheel,rightWheel]

    debug_printer(debug,'rc_enc output',output)
    debug_printer(rc_enc_debug,'rc_enc output',[test_pulsevals,test_wheels])

    return

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Move Command #####################################
#########################################################################################
#########################################################################################

def move_command(left,right):

    global command_merger
    global motor_command_debug

    debug_printer(motor_command_debug,"motor_command - left,right",[left,right])

    try:

        cmd = '@01!G 1 ' + str(left) + '_' + '@01!G 2 ' + str(right) + '_' + '@02!G 1 ' + str(left) + '_' + '@02!G 2 ' + str(right) + '\r'
        ser.write(cmd)

    except Exception as e: # catch *all* exceptions

        print e
        print( "error: move_command" )

    return 

#########################################################################################



#########################################################################################
#########################################################################################
###################################### data2vel #########################################
#########################################################################################
#########################################################################################

def data2vel(data):

    global mps_to_movecmd
    global rps_to_movecmd

    left  =   (data.linear.x * mps_to_movecmd) - (data.angular.z * rps_to_movecmd)
    right = - (data.linear.x * mps_to_movecmd) - (data.angular.z * rps_to_movecmd)
    
    return left,right

#########################################################################################



#########################################################################################
#########################################################################################
######################################## TEST ###########################################
#########################################################################################
#########################################################################################

communication_error = 0

def TEST():

    global communication_error

    for i in range(30):

        try:

            pulses = getRCInput()
            time.sleep(0.01)
            encabs = getEncoders()
            error = 0
            time.sleep(0.01)
            move_command(0,0)
            time.sleep(0.01)

        except Exception as e: # catch *all* exceptions

            print e
            print( "error: TEST" )
            error = 1

    if error == 1:

        communication_error = 1

    return 

#########################################################################################



#########################################################################################
#########################################################################################
#################################### odom publisher #####################################
#########################################################################################
#########################################################################################

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

encoder_ppr = 2048                                                      # encoder is connected to the shaft. Gear ratio between shaft and wheel is ~81
wheel_diameter = 16                                                     # in inches
in_to_m = 0.0254
 
enc_countperrev_left_front  = 165881.1                                  # Gear ratio between shaft and wheel is ~81 (2048 x 81 = 165888)
enc_countperrev_left_rear   = 165884.3                                 
enc_countperrev_right_front = 165883.9                                 
enc_countperrev_right_rear  = 165885.5                                 

DistancePerCount_left_front  = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_left_front               # (PI*D) / ppr
DistancePerCount_left_rear   = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_left_rear            
DistancePerCount_right_front = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_right_front            
DistancePerCount_right_front = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_right_rear              

lengthBetweenTwoWheels = 26.0 * in_to_m

def odom_publisher():

    global odom_pub
    global current_time
    global last_time
    global enc_last

    global test_wheels

    global DistancePerCount_left_front
    global DistancePerCount_left_rear
    global DistancePerCount_right_front
    global DistancePerCount_right_front

    global lengthBetweenTwoWheels

    global x
    global y
    global th

    global odom

    current_time = rospy.Time.now()

    getRCnENC()
    enc_now = test_wheels


    deltaLeft1  = enc_now[0][0]  - enc_last[0][0]
    deltaLeft2  = enc_now[0][1]  - enc_last[0][1]
    deltaRight1 = enc_now[1][0]  - enc_last[1][0]
    deltaRight2 = enc_now[1][1]  - enc_last[1][1]

    time_elapsed = current_time.to_sec() - last_time.to_sec()

    v_left1  = (deltaLeft1  * DistancePerCount_left_front)  / time_elapsed
    v_left2  = (deltaLeft2  * DistancePerCount_left_rear)   / time_elapsed
    v_right1 = (deltaRight1 * DistancePerCount_right_front) / time_elapsed
    v_right2 = (deltaRight2 * DistancePerCount_right_front) / time_elapsed

    v_left  = (v_left1  + v_left2)  / 2 
    v_right = -(v_right1 + v_right2) / 2 

    vx  = ((v_right + v_left) / 2);
    vy  = 0;
    vth = ((v_right - v_left) / lengthBetweenTwoWheels);

    # compute odometry in a typical way given the velocities of the robot
    dt       = (current_time - last_time).to_sec()
    delta_x  = (vx * cos(th) - vy * sin(th)) * dt
    delta_y  = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x  += delta_x
    y  += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom                 = Odometry()
    odom.header.stamp    = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist    = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # set the covariance
    odom.pose.covariance  = [1e-3,0,0,0,0,0, 0,1e-3,0,0,0,0, 0,0,1e6,0,0,0, 0,0,0,1e6,0,0, 0,0,0,0,1e6,0, 0,0,0,0,0,1e3] ;                                                    
    odom.twist.covariance = [1e-3,0,0,0,0,0, 0,1e-3,0,0,0,0, 0,0,1e6,0,0,0, 0,0,0,1e6,0,0, 0,0,0,0,1e6,0, 0,0,0,0,0,1e3] ;

    # publish the message
    odom_pub.publish(odom)

#########################################################################################



#########################################################################################
#########################################################################################
#################################### Move Callback ######################################
#########################################################################################
#########################################################################################

EM_STOP = 0
RC_MODE = 0

# new speed to movecommand
mps_to_movecmd = 20.0/0.1033401

# new ang_vel to movecommand
rps_to_movecmd = 20.0/0.20862

# max linear velocity
v_max = 2.0

# max angular velocity
w_max = 2.0


def sign(a):
    return (a/abs(a))

def move_callback(data):

    global debug
    global mode_debug
    global speed_debug
    global EM_STOP
    global RC_MODE
    global pub_time
    global test_pulsevals
    global v_max
    global w_max
    global callback_debug


    pulses = test_pulsevals
    signs = [1,-1]

    debug_printer(debug,"RC_MODE",RC_MODE)
    debug_printer(debug,"EM_STOP",EM_STOP)
    debug_printer(mode_debug,"RC_MODE",RC_MODE)
    debug_printer(mode_debug,"EM_STOP",EM_STOP)

    debug_printer(callback_debug,"cmdvel data received - [ vx , wz ]",[data.linear.x,data.angular.z])

    pub_time = rospy.get_time()

    if (pulses[0] > 1500) and (RC_MODE != 1):  #estop activated

        ser.write('@00!EX\r')
        EM_STOP = 1

    elif (EM_STOP) and (pulses[1] > 1500):

        ser.write('@00!MG\r')
        EM_STOP = 0
        RC_MODE = 1

    elif (RC_MODE):

        [left,right] = pulse2vel(pulses)
        move_command(left,right)

    else:

        if (abs(data.linear.x) < v_max) and (abs(data.angular.z) < w_max):

            [left,right] = data2vel(data)

        else:

            left  = 0
            right = 0

        debug_printer(debug,"left",left)
        debug_printer(debug,"right",right)
        debug_printer(speed_debug,"left",left)
        debug_printer(speed_debug,"right",right)


        move_command(left,right)

#########################################################################################



#########################################################################################
#########################################################################################
######################################### Main ##########################################
#########################################################################################
#########################################################################################

if __name__ == "__main__":

    TEST()
    initialize()

    while (not rospy.is_shutdown()) and (communication_error == 0):

        try:

            getRCnENC()

            enc_last = test_wheels
            last_time = current_time
            r.sleep()
            time_now = rospy.get_time()
            print "time diff:",time_now - pub_time

            if (time_now - pub_time > 0.1):

                RC_MODE = 1
                ser.write('@00!MG\r')

            else:

                RC_MODE = 0

            if (RC_MODE):

                [left,right] = pulse2vel(test_pulsevals)
                move_command(left,right)

        except KeyboardInterrupt:

            left = 0
            right = 0
            move_command(left,right)
            ser.close()

    left = 0
    right = 0
    move_command(left,right)
    ser.close()

#########################################################################################