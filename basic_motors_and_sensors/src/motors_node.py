#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motors Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-16

motors_node.py
ROS Node to accept commands of "wheel_command_left" and "wheel_command_right" and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

import rospy
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Float32 
# import the "motors" object and MAX_SPEED setting from the Pololu library for the motor driver. 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motors_node',anonymous=False)

def listener(): 

    # Subscribe to the "wheel_command_left" topic
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
    sub = rospy.Subscriber('/wheel_command_left', Float32, set_wheel_command_left) 
 
    #### CODE HERE ####
    # Add a Subscriber for the Right wheel
    # Subscribe to the "wheel_speed_desired_right" topic
    subR = rospy.Subscriber('/wheel_command_right', Float32, set_wheel_command_right) 
    #### END CODE ####
   
    rospy.spin()    # keep the node from exiting


# Callback for setting the command of the left wheel    
def set_wheel_command_left(msg_in): 
    wheel_command_left = int(msg_in.data)
    motors.motor1.setSpeed(wheel_command_left)
    #    rospy.loginfo("left wheel set to {0}".format(wheel_command_left))   # could issue a Log message to ROS about it

# Callback for setting the command of the right wheel 
def set_wheel_speed_right(msg_in): 
    wheel_speed_desired_right = int(msg_in.data)
    motors.motor2.setSpeed(wheel_command_right)
#    rospy.loginfo("right wheel set to {0}".format(wheel_speed_desired_right))    # could issue a Log message to ROS about it


# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        listener()
    except : 
        traceback.print_exc()   # Print any error to the screen. 
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass
    
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    pass