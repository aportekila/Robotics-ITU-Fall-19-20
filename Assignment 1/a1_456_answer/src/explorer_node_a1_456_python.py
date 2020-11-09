#!/usr/bin/env python
## AK
## explorer_node_py.py
##
## BLG456E Assignment 1 skeleton
##
## Instructions: Change the laser_callback function to make the robot explore more
## intelligently, using its sensory data (the laser range array).
##
## Advanced: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## 

## Abdullah AKGUL
## 150150133
## In this assigment, in order to implement wall following, I get inspiration and 
## some pieces of code from "https://www.theconstructsim.com/wall-follower-algorithm/"
## I delete some parts of the given reference and I also add other features. Therefore;
## my code is not complately takeb from this source, I just got help from there.
## I will not write any report or documentation because all the implemented functions 
## will be explained in comments

## Common ROS headers.
import rospy
## Required for some printing options
import sys
## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid

## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##

from datetime import datetime, timedelta
import random
import numpy as np


## I added some random events.
## In order to be same in every run, I give seed.
## 70 is my  hometown's plate number
random.seed(70)

## I divided the laser scan data in to 5 regions.
## These regions are right, left, front, front left and front right.
## I will use these regions to determine where to go
regions = {
	'right': 0,
	'left': 0,
	'front': 0,
	'front_right': 0,
	'front_left': 0
}

## In the case of nan, I will do interpolation for guessing the nan regions
regions_interpolated = {
	'right': 0,
	'left': 0,
	'front': 0,
	'front_right': 0,
	'front_left': 0
}

## I need a global state variable to determine which behaviour will be acted.
## Also I need this state variable to look what was the old state
state = 0

## In this state dictionary you can see all the stage' names
state_dictionary = {
	0: 'find wall', ## this calls the find wall behaviour
	1: 'turn left', ## this calls the turn left behaviour
	2: 'follow the wall', ## this calls the follow the wall behaviour
	3: 'pass a passage', ## this calls the pass a passage behaviour
	4: 'dark', ## this calls the dark behaviour
	5: 'random_turn_left', ## this calls the random_turn_left behaviour
	6: 'turn right', ## this calls the find wall behaviour
	7: 'go', ## this calls the turn right behaviour
	8: 'turn right and go', ## this calls the turn right and go behaviour
    9: 'turn left and go', ## this calls the turn left and go behaviour
    10: 'turn big left', ## this calls the turn big left behaviour
    11: 'turn big right', ## this calls the turn big right behaviour
    12: 'back' ## this calls the back behaviour

}

## In order to not to stuck in turn right and turn left events consecutively,
## I made a list of turns
state_with_turn_left = [1, 4, 5, 9, 10]
state_with_turn_right = [0, 6, 8, 11]

## this treshold is accepted space between an obstacle with the laser scanner
treshold = 0.775

## this function is changes the state.
## if the state is changed, it will print the change
def state_change(new_state):
	global state, state_dictionary
	if new_state is not state:
		print "State changing '", state_dictionary[state], "' to '", state_dictionary[new_state], "'"
		state = new_state

## find_wall behaviour just arranges the motor command to
## go little forward and turn right
## to find wall
def find_wall(motor_command):
	motor_command.linear.x = 0.2
	motor_command.angular.z = - 0.6

## turn_left behaviour just arranges the motor command to
## turn left
def turn_left(motor_command):
	motor_command.linear.x = 0
	motor_command.angular.z = 0.95

## follow_the_wall behaviour just arranges the motor command to
## go forward
## to follow the wall
def follow_the_wall(motor_command):
	motor_command.linear.x = 0.5
	motor_command.angular.z = 0

## pass_passage behaviour just arranges the motor command to
## go too little forward
## this is really slow speed in order to prevent any possible collusion
def pass_passage(motor_command):
	motor_command.linear.x = 0.1
	motor_command.angular.z = 0

## pass_dark behaviour just arranges the motor command to
## go too little forward and turn left
## in this behaviour, every range is nan. 
## i try to get some range data except nan by moving robot too little
def pass_dark(motor_command):
	motor_command.linear.x = 0.1
	motor_command.angular.z = 0.45

## random_turn_left behaviour just arranges the motor command to
## turn left according to generated random number
## this behaviour turns the robot randomly
def random_turn_left(motor_command, coeff):
	motor_command.linear.x = 0
	motor_command.angular.z = coeff / 15.

## turn_right behaviour just arranges the motor command to
## turn right
def turn_right(motor_command):
	motor_command.linear.x = 0
	motor_command.angular.z = - 0.95

## go behaviour just arranges the motor command to
## go forward
def go(motor_command):
	motor_command.linear.x = 0.7
	motor_command.angular.z = 0

## turn_right_and_go behaviour just arranges the motor command to
## go forward and turn right
## I write it in the case of usage but i dont use it
def turn_right_and_go(motor_command):
	motor_command.linear.x = 0.2
	motor_command.angular.z = - 0.3

## turn_left_and_go behaviour just arranges the motor command to
## go forward and turn left
## I write it in the case of usage but i dont use it
def turn_left_and_go(motor_command):
	motor_command.linear.x = 0.2
	motor_command.angular.z = 0.3

## turn_big_right behaviour just arranges the motor command to
## turn big right
## I write it in the case of usage but i dont use it
def turn_big_right(motor_command):
	motor_command.linear.x = 0
	motor_command.angular.z = - 1.8

## turn_big_left behaviour just arranges the motor command to
## and turn big left
## I write it in the case of usage but i dont use it
def turn_big_left(motor_command):
	motor_command.linear.x = 0
	motor_command.angular.z = 1.8

## back behaviour just arranges the motor command to
## go back
## I write it in the case of usage but i dont use it
def back(motor_command):
	motor_command.linear.x = - 0.2
	motor_command.angular.z = 0

## this function interpolates the nan regions
def interpolate_and_find_nan():
    global regions, regions_interpolated
    nan_count = 0
    nan = []
    not_nan = []

    if np.isnan(regions['right']):
    	nan_count += 1
    	nan.append('right')
    else:
    	not_nan.append('right')

    if np.isnan(regions['front_right']):
    	nan_count += 1
    	nan.append('front_right')
    else:
    	not_nan.append('front_right')

    if np.isnan(regions['front']):
    	nan_count += 1
    	nan.append('front')
    else:
    	not_nan.append('front')

    if np.isnan(regions['front_left']):
    	nan_count += 1
    	nan.append('front_left')
    else:
    	not_nan.append('front_left')

    if np.isnan(regions['left']):
    	nan_count += 1
    	nan.append('left')
    else:
    	not_nan.append('left')

    temp = nan_count
    ## copies the original regions to regions_interpolated because 
    ## i dont want to change original one
    regions_interpolated = {
    'right': regions['right'],
    'left': regions['left'],
    'front': regions['front'],
    'front_right': regions['front_right'],
    'front_left': regions['front_left']
    }

    if temp == 5:
    	## if nan_count is 5 everywhere is nan so it is the case of
    	## dark behaviour
    	return nan_count, nan, not_nan
    else:
    	while temp > 0:

    		if np.isnan(regions_interpolated['right']):

    			if not np.isnan(regions_interpolated['front_right']):
    				regions_interpolated['right'] = regions_interpolated['front_right'] * 0.75
    				## I multiplied the front_right region's value with 0.75 in order to prevent collison
    				temp -= 1

    		if np.isnan(regions_interpolated['front_right']):

    			if not np.isnan(regions_interpolated['right']) and not np.isnan(regions_interpolated['front']):
    				regions_interpolated['front_right'] = (regions_interpolated['right'] + regions_interpolated['front']) / 2.
    				## I take the average of the closest neightboors
    				temp -= 1
    			elif not np.isnan(regions_interpolated['right']):
    				regions_interpolated['front_right'] = regions_interpolated['right'] * 0.75
    				## I multiplied the right region's value with 0.75 in order to prevent collison
    				temp -= 1
    			elif not np.isnan(regions_interpolated['front']):
    				regions_interpolated['front_right'] = regions_interpolated['front'] * 0.75
    				## I multiplied the front region's value with 0.75 in order to prevent collison
    				temp -= 1

    		if np.isnan(regions_interpolated['front']):

    			if not np.isnan(regions_interpolated['front_right']) and not np.isnan(regions_interpolated['front_left']):
    				regions_interpolated['front'] = (regions_interpolated['front_right'] + regions_interpolated['front_left']) / 2.
    				## I take the average of the closest neightboors
    				temp -= 1
    			elif not np.isnan(regions_interpolated['front_right']):
    				regions_interpolated['front'] = regions_interpolated['front_right'] * 0.75
    				## I multiplied the front_right region's value with 0.75 in order to prevent collison
    				temp -= 1
    			elif not np.isnan(regions_interpolated['front_left']):
    				regions_interpolated['front'] = regions_interpolated['front_left'] * 0.75
    				## I multiplied the front_left region's value with 0.75 in order to prevent collison
    				temp -= 1

    		if np.isnan(regions_interpolated['front_left']):

    			if not np.isnan(regions_interpolated['front']) and not np.isnan(regions_interpolated['left']):
    				regions_interpolated['front_left'] = (regions_interpolated['front'] + regions_interpolated['left']) / 2.
    				## I take the average of the closest neightboors
    				temp -= 1
    			elif not np.isnan(regions_interpolated['front']):
    				regions_interpolated['front_left'] = regions_interpolated['front'] * 0.75
    				## I multiplied the front region's value with 0.75 in order to prevent collison
    				temp -= 1
    			elif not np.isnan(regions_interpolated['left']):
    				regions_interpolated['front_left'] = regions_interpolated['left'] * 0.75
    				## I multiplied the left region's value with 0.75 in order to prevent collison
    				temp -= 1

    		if np.isnan(regions_interpolated['left']):

    			if not np.isnan(regions_interpolated['front_left']):
    				regions_interpolated['left'] = regions_interpolated['front_left'] * 0.75
    				## I multiplied the front_left region's value with 0.75 in order to prevent collison
    				temp -= 1

    	return nan_count, nan, not_nan

## In this method, I check the forward is appropriate to go
def check_forward(ranges, length):
	flag = True
	forward = ranges[ int(length)/2 -30 : int(length)/2 + 30]
	nan = 0
	sum = 0
	len_forward = len(forward)

	for _ in forward:
		if not np.isnan(_):
			sum += _
		else:
			nan += 1

	avg = sum / ((len_forward - nan) * 1.)
	global treshold, regions_interpolated

	if nan != len:
		if avg >= 1:
			if regions_interpolated['front_right'] > treshold and regions_interpolated['front_left'] > treshold:
				return True
			else:
				return False
		else:
			return False
	else:

		if regions_interpolated['forward'] >= 1.2:
			if regions_interpolated['front_right'] > treshold and regions_interpolated['front_left'] > treshold:
				return True
			else:
				return False
		else:
			return False


def laser_callback(data):
    ## Lets fill a twist message for motor command
    motor_command = Twist()
    ## For now let us just set it up to drive forward ...
    #motor_command.linear.x = 0.3
    ## And left a little bit
    #motor_command.angular.z = 0.1
    ## Lets publish that command so that the robot follows it
    global motor_command_publisher
    ## getting the global variables
    global regions, regions_interpolated, treshold
    ## length is the len of the data.ranges
    length = len(data.ranges)
    ## inc will be used for dividing the data.ranges
    inc = int(length / 5)
    ## regions are created with the minumum values of the given part of data.ranges
    ## in order to be safe
    ## data.ranges[right, front_right, front, front_left, left]
    regions = {
    'right': min(min(data.ranges[0: inc - 1]), 10),
    'left': min(min(data.ranges[inc * 4: ]), 10),
    'front': min(min(data.ranges[inc * 2: inc * 3 - 1]), 10),
    'front_right': min(min(data.ranges[inc: inc * 2 - 1]), 10),
    'front_left': min(min(data.ranges[inc * 3: inc * 4 - 1]), 10)
    }

    ## interpolated regions, nan regions list and not_nan regions list 
    ## are taken from interpolate_and_find_nan
    nan_count, nan, not_nan = interpolate_and_find_nan()


    print "#########"
    print "regions['right']", regions['right']
    print "regions['left']", regions['left']
    print "regions['front']", regions['front']
    print "regions['front_right']", regions['front_right']
    print "regions['front_left']", regions['front_left']
    print "#########"

    print "nan_count", nan_count
    print "nan", nan

    print "#########"
    print "regions_interpolated['right']", regions_interpolated['right']
    print "regions_interpolated['left']", regions_interpolated['left']
    print "regions_interpolated['front']", regions_interpolated['front']
    print "regions_interpolated['front_right']", regions_interpolated['front_right']
    print "regions_interpolated['front_left']", regions_interpolated['front_left']
    print "#########"

    ## random int between 1 and 100
    ## according to this random int's value robot may turn left
    rand = random.randint(1, 100)

    #if rand <= 15 and state not in [2, 7, 6, 8, 0]:
    if 0:
    	pass
    	## if robot is not turned right before and not following the wall also <= 15 robot will turn left
    	# random turn left
    	state_change(5)
    else:

    	if nan_count == 5:
    		## if all the regions are nan it is dark stage
    		#print everywhere is dark for me 
    		#im afraid and don't know what to do
    		#dear God please help me
    		state_change(4)
    	elif nan_count == 4:
    		## if onlt 1 region is not nan so turn that side
    		if not_nan[0] == 'forward' and regions['forward'] > treshold:
    			#go
    			state_change(7)
    		elif not_nan[0] == 'front_left' or not_nan[0] == 'left':
    			if state not in state_with_turn_right:
    				#turn left
    				state_change(1)
    			else:
    				#turn right
    				state_change(6)
    		else:
    			if state not in state_with_turn_left:
    				#turn right
    				state_change(6)
    			else:
    				#turn left
    				state_change(1)
    	else:

    		if regions_interpolated['left'] < treshold and regions_interpolated['right'] < treshold:
    			## if left and right regions are lesser than the treshold
    			if regions_interpolated['front'] >= treshold:

    				if regions_interpolated['front_left'] < treshold and regions_interpolated['front_right'] < treshold:
    					## robot may be in a passage 
    					#pass a passage
    					state_change(3)
    				else:
    					## robot is not in passage so turn
    					if regions_interpolated['right'] < treshold:
    						if state not in state_with_turn_right:
    							#turn left
    							state_change(1)
    						else:
    							#turn right
    							state_change(6)
    					else:
    						if state not in state_with_turn_left:
    							#turn right
    							state_change(6)
    						else:
    							#turn left
    							state_change(1)

    			else:
    				## forward is also lesser than treshold so turn
    				if regions_interpolated['right'] < treshold:
    					if state not in state_with_turn_right:
    						#turn left
    						state_change(1)
    					else:
    						#turn right
    						state_change(6)
    				else:
    					if state not in state_with_turn_left:
    						#turn right
    						state_change(6)
    					else:
    						#turn left
    						state_change(1)


    		elif regions_interpolated['left'] < treshold:
    			## robot is close to left wall or obstacle
    			if regions_interpolated['front'] >= treshold:
    				if state not in state_with_turn_left:
    					# turn right
    					state_change(6)
    				else:
    					#turn left
    					state_change(1)
    				
    			else:
    				if state not in state_with_turn_left:
    					#turn right
    					state_change(6)
    				else:
    					#turn left
    					state_change(1)

    		elif regions_interpolated['right'] < treshold:
    			## robot is close to right wall or obstacle
    			if regions_interpolated['front'] >= treshold:
    				if state not in state_with_turn_right:
    					# turn left
    					state_change(1)
    				else:
    					#turn right
    					state_change(6)

    			else:
    				if state not in state_with_turn_right:
    					#turn left
    					state_change(1)
    				else:
    					#turn right
    					state_change(6)
    		else:
    			## robot is not close to wall or obstacle with both left and right
    			if regions_interpolated['front'] >= treshold:
    				## front is open then go
    				if regions_interpolated['front_left'] >= treshold:
    					if regions_interpolated['front_right'] >= treshold:
    						#go
    						state_change(7)
    					else:
    						#follow the wall
    						state_change(2)
    				else:
    					if regions_interpolated['front_right'] >= treshold:
    						#find wall
    						state_change(0)
    					else:
    						#find wall
    						state_change(0)
    			else:
    				## front is closed so turn
    				if regions_interpolated['front_left'] >= treshold:
    					if regions_interpolated['front_right'] >= treshold:
    						if state not in state_with_turn_right:
    							#turn left
    							state_change(1)
    						else:
    							#turn right
    							state_change(6)
    					else:
    						if state not in state_with_turn_right:
    							#turn left
    							state_change(1)
    						else:
    							#turn right
    							state_change(6)
    				else:
    					if regions_interpolated['front_right'] >= treshold:
    						if state not in state_with_turn_right:
    							#turn left
    							state_change(1)
    						else:
    							#turn right
    							state_change(6)
    					else:
    						if state not in state_with_turn_right:
    							#turn left
    							state_change(1)
    						else:
    							#turn right
    							state_change(6)

    ## the state decision has been made so give the command according to state
    if state == 0:
    	print "find_wall"
    	find_wall(motor_command)
    elif state == 1:
    	print "turn_left"
    	turn_left(motor_command)
    elif state == 2:
    	print "follow_the_wall"
    	follow_the_wall(motor_command)
    elif state == 3:
    	print "pass_passage"
    	pass_passage(motor_command)
    elif state == 4:
    	print "pass_dark"
    	pass_dark(motor_command)
    elif state == 5:
    	print "random_turn_left"
    	random_turn_left(motor_command, rand)
    elif state == 6:
    	print "turn_right"
    	turn_right(motor_command)
    elif state == 7:
    	print "go"
    	go(motor_command)
    elif state == 8:
    	print "turn_right_and_go"
    	turn_right_and_go(motor_command)
    elif state == 9:
    	print "turn_right_and_go"
    	turn_right_and_go(motor_command)
    elif state == 10:
    	print "turn_big_left"
    	turn_big_left(motor_command)
    elif state == 11:
    	print "turn_big_right"
    	turn_big_right(motor_command)
    else:
    	print("ops something went wrong about states")

    ## my part is done

    global motor_command_publisher
    motor_command_publisher.publish(motor_command)
    
    ## Alternatively we could have looked at the laser scan BEFORE we made this decision
    ## Well Lets see how we might use a laser scan
    ## Laser scan is an array of distances
    print 'Number of points in laser scan is: ', len(data.ranges)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print 'The distance to the leftmost scanned point is: ', data.ranges[-1]
    print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
    ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
    print 'The minimum angle scanned by the laser is: ', data.angle_min
    print 'The maximum angle scanned by the laser is: ', data.angle_max
    print 'The increment in the angles scanned by the laser is: ', data.angle_increment
    print 'The minimum range (distance) the laser can perceive is: ', data.range_min
    print 'The maximum range (distance) the laser can perceive is: ', data.range_max
    
## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
	"""print "################"
	print "data.info.width", data.info.width
	print "################"
	print "data.info.height", data.info.height
	print "################"
	print "data", data
	print "################"
	"""

	chatty_map = False
	if chatty_map:
		print "-------MAP---------"
		## Here x and y has been incremented with five to make it fit in the terminal
		## Note that we have lost some map information by shrinking the data
		for x in range(0,data.info.width-1,5):
			for y in range(0,data.info.height-1,5):
				index = x+y*data.info.width
				if data.data[index] > 50:
					## This square is occupied
					sys.stdout.write('X')
				elif data.data[index] >= 0:
					## This square is unoccupied
					sys.stdout.write(' ')
				else:
					sys.stdout.write('?')
			sys.stdout.write('\n')
		sys.stdout.flush()
		print "-------------------"
    
## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    rospy.init_node('amble')
    
    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    
    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)
    
    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
    
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    explorer_node()
