#!/usr/bin/python
import serial
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


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
		bytesize=serial.EIGHTBITS
	)
except:
	raise IOError

# reset the connection if need be
if (ser.isOpen()):
	ser.close()
ser.open()

#########################################################################################



#########################################################################################
#########################################################################################
##################################### Initialize ########################################
#########################################################################################
#########################################################################################

def initialize():

	global encoder_pub

	rospy.init_node('roboteq_ros_node')
	rospy.loginfo('status: roboteq_ros_node crearted')

	rospy.Subscriber('/cmd_vel', Twist, move_callback)
	encoder_pub = rospy.Publisher('roboteq_ros/encoders', String, queue_size=1000)
	rospy.loginfo('status: cmd_vel subscribed, encoder_pub initiated')

#########################################################################################



#########################################################################################
#########################################################################################
############################### Get Data and Clean messages #############################
#########################################################################################
#########################################################################################

def getdata():

	info = ''
	while ser.inWaiting() > 0:                             # While data is in the buffer
		info += str(ser.read())
	return info



def clean_encoder_messages(message):

#=========================================================================================== 
# NOTE: can also be used to clean battery voltage messages 								   |
#===========================================================================================

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



def clean_pulse_messages(message):

# Pulse output looks like this - "PI=1234\r"
# This function reads output like that and returns "1234"

	clean_message = ''
	start = 0
	try:    
		for i in range(3,len(message)):
			if (message[i]== '\r'):
				clean_message = int(clean_message)
				return clean_message
			clean_message += message[i]
	except:
		return clean_message

#########################################################################################



#########################################################################################
#########################################################################################
####################################### reader ##########################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# --> Arguments - 1. query_id 															   |
#			      2. can_node_id 														   |
#			      3. channel 															   |
# 															                               |
# 	1. query_id    - Roboteq has about 20 or so values that can be queried but the current |
#				     version of function can get pulse input(pulse-PI), absolute encoder   | 
#				     value(enc_abs - C), relative encoder rpm(enc_rel_rpm - RS) and battery|
#				     voltage(battery - V)												   |
# 															                               |
# 	2. can_node_id - refers to the node id of the node in the RoboCAN network you wish     |
# 					 to contact 														   |
# 															                               |
# 	3. channel     - refers to the motor channel of the node you wish to contact           |
# 															                               |
# 															                               |
# 															                               |
# --> Returns - The function returns what is read from the serial port for the command     |
#===========================================================================================

def reader(query_id,can_node_id,channel):

	broadcast = ''
	if query_id == 'pulse':
		q_id = 'PI'
		error = 0
	elif query_id == 'battery':
		q_id = 'V'
		broadcast = '@0'+ str(can_node_id)
		channel = 2
		error = 0
	elif query_id == 'enc_abs':
		q_id = 'C'
		broadcast = '@0'+ str(can_node_id)
		error = 0
	elif query_id == 'enc_rel_rpm':
		q_id = 'SR'
		broadcast = '@0'+ str(can_node_id)
		error = 0
	else:
		error = 1
		print 'error: query_id unknown'

	if error == 0:
		command = broadcast + '?' + q_id + ' ' + str(channel) + '\r'
		ser.write(command)  #enc right wheel 1
		time.sleep(.005)
		output = getdata()
	else:
		output = 'error: incorrect arguments for reader function'

	return output

#########################################################################################


#########################################################################################
#########################################################################################
####################################### RC Input ########################################
#########################################################################################
#########################################################################################

pulse1 = 1490
pulse2 = 1490

def getRCInput():

	global pulse1
	global pulse2
	try:
		time.sleep(.01)
		getdata()   #clear buffer     
		pulse1 = clean_pulse_messages(reader('pulse',1,1))
		pulse2 = clean_pulse_messages(reader('pulse',1,2))
	except Exception as e: # catch *all* exceptions
		print e
		print( "error: getRCInput" )
	return pulse1, pulse2

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Encoders test ####################################
#########################################################################################
#########################################################################################

def getEncoders():

	global encoder_pub

	leftWheel = [0,0,0,0]
	rightWheel = [0,0,0,0]

	time.sleep(.01)
	getdata()   #clear buffer

	leftWheel[0] = clean_encoder_messages(reader('enc_abs',1,1))
	leftWheel[1] = clean_encoder_messages(reader('enc_abs',1,2))
	leftWheel[2] = clean_encoder_messages(reader('enc_rel_rpm',1,1))		
	leftWheel[3] = clean_encoder_messages(reader('enc_rel_rpm',1,2))

	rightWheel[0] = clean_encoder_messages(reader('enc_abs',2,1))		
	rightWheel[1] = clean_encoder_messages(reader('enc_abs',2,2))
	rightWheel[2] = clean_encoder_messages(reader('enc_rel_rpm',2,1))		
	rightWheel[3] = clean_encoder_messages(reader('enc_rel_rpm',2,2))		
	print leftWheel , rightWheel
	encoder_pub.publish(str(leftWheel)+str(rightWheel))

	return leftWheel, rightWheel

#########################################################################################



#########################################################################################
#########################################################################################
################################## Read Battery Voltage #################################
#########################################################################################
#########################################################################################

def getVoltage():

	front_controller_voltage = 0
	rear_controller_voltage = 0

	time.sleep(.01)
	getdata()   #clear buffer

	front_controller_voltage = clean_encoder_messages(reader('battery',1,2))
	rear_controller_voltage = clean_encoder_messages(reader('battery',2,2))		
	

	return front_controller_voltage, rear_controller_voltage

#########################################################################################



#########################################################################################
#########################################################################################
##################################### move callback #####################################
#########################################################################################
#########################################################################################

def move_callback(data):
	cmd = '@01!G 1 ' + str(data.linear.x) + '\r'                                                                                            
	ser.write(cmd)                                                                                            
	cmd = '@01!G 2 ' + str(-data.linear.x) + '\r'                                                                                            
	ser.write(cmd)                                                                                            
	cmd = '@02!G 1 ' + str(data.linear.x) + '\r'                                                                                            
	ser.write(cmd)                                                                                            
	cmd = '@02!G 2 ' + str(-data.linear.x) + '\r'                                                                                            
	ser.write(cmd)

#########################################################################################



#===========================================================================================| 
#                                                                                           | 
# exit = 0                                                                                  |          
#                                                                                           | 
# while exit != 1:                                                                          |                  
# 	try:                                                                                    |        
# 		speed = 25                                                                          |                  
# 		cmd = '@01!G 1 ' + str(speed) + '\r'                                                |                                            
# 		ser.write(cmd)                                                                      |                      
# 		cmd = '@01!G 2 ' + str(speed) + '\r'                                                |                                            
# 		ser.write(cmd)                                                                      |                      
# 		cmd = '@02!G 1 ' + str(speed) + '\r'                                                |                                            
# 		ser.write(cmd)                                                                      |                      
# 		cmd = '@02!G 2 ' + str(speed) + '\r'                                                |                                            
# 		ser.write(cmd)                                                                      |                      
#                                                                                           |
# 		time.sleep(0.01)                                                                    |                       
# 		getEncoders()                                                                       |                     
#                                                                                           | 
# 	except KeyboardInterrupt:                                                               |                             
# 		cmd = '@01!G 1 ' + str(0) + '\r'                                                    |                                        
# 		ser.write(cmd)                                                                      |                      
# 		cmd = '@01!G 2 ' + str(0) + '\r'                                                    |                                        
# 		ser.write(cmd)                                                                      |                      
# 		cmd = '@02!G 1 ' + str(0) + '\r'                                                    |                                        
# 		ser.write(cmd)                                                                      |                      
# 		cmd = '@02!G 2 ' + str(0) + '\r'                                                    |                                        
# 		ser.write(cmd)                                                                      |                      
#                                                                                           | 
# 		getEncoders()                                                                       |                     
# 		getVoltage()                                                                        |                    
# 		ser.close()                                                                         |                   
# 		exit = 1                                                                            |                
#                                                                                           | 
#===========================================================================================|



#########################################################################################
#########################################################################################
######################################### Main ##########################################
#########################################################################################
#########################################################################################

if __name__ == "__main__":
	initialize()
	try:
		# rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			getEncoders()
	except KeyboardInterrupt:
		ser.close()

#########################################################################################


