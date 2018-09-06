Instructions:


Operation:

1. Turn on the rc transmitter before turning on the base. Otherwise, things might turn bad.

2. If whenever the base is not doing what it is supposed to do, it just means it's running out of power so go ahead and charge it for 3-4 hours.

3. There are 3 main files to run for tests.

	test.launch           - initiates base serial + rc communication - listens to "/cmd_vel" topic
	curve.launch          - starts up the camera and publishes the base pose at "/base_pose" topic 
	curve_multiple_wp.py  - the current version doesn't listen to waypoints. It just tries to reach 5 points on x = 0 [[0,1,1.57],[0,1.1,1.57],[0,1.2,1.57],[0,1.3,1.57],[0,1.4,1.57]] - [x,y,yaw_goal]

4. First off, run curve.launch and make sure that the arrow is vertical on the popped rviz window to make sure the right marker is detected as origin [you can also check this by scrolling up the output to see what aruco marker id. It should say 401. If it says 357 or something else, stop the file(ctrl + c, avoid using ctrl + z,x as they might not stop the publishers properly)] and rc-move the base to a location where the camera is directly under the marker covering the tubelight.

5. After getting the vertical arrow on rviz (401 detected as origin), run test.launch. Make sure you are not getting errors. If, by chance, you are getting errors or robot is misbehaving, turn off the base and inform me what the error says. This has never happened before in my last 200-ish runs.

6. To test the cirve files, what i was doing is i. had the cursor on the curve file. 
											   ii. removed the monitor cable and safely curled up in the base frame
											  iii. placed the keyboard on top of the cpu
											   iv. slowly rc-moved the base to the initial spot
											    v. pressed enter and had my fingers on the throttle(right joystick) ready to move it up 1-2 cm (Emergency brake)
											   vi. after pressing emergency, if the robot is not close to the monitor, press ctrl+c, then ctrl+tab, repeat these 2 commands 3 times so all 3 codes are stopped, and turn the base off and then turn it back on. The robot should go into semi-rc mode[upper-right means straight and lower-left is back and so on - Shantanu knows. Make sure to not the stick too fast]
											  vii. rc-move back to the monitor and debug your curve code

Nodes and Topics:

1.test.launch:

roboteq_ros_odom3 - node file

->listens to rc transmitter(for EM STOP) and cmd_vel

2.curve.launch

multiple files: camera + aruco_mapping

->publishes base_pose

3.curve_multiple_wp.py

->hardcoded 5 points
->publishes cmd_vel based to the waypoint for 2 seconds and goes to the next waypoint
