#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motor Command Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-09

motor_command_node.py
ROS Node to get User Input of commands for wheel command (e.g. "wheel_command_left" - left wheel only at first) and publish them to a topic to set wheel commands in another node.  
"""

import rospy
from std_msgs.msg import Float32

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motor_command_node',anonymous=False)

def talker_for_wheel_commands():
    
    # Set up a Publisher
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
    pub_wheel_command_left = rospy.Publisher('wheel_command_left',Float32,queue_size=1)
    
    # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    wheel_command_left_msg = Float32()
    
    #### CODE HERE ####
    # Add a Publisher for the Right wheel
    #### END CODE ####
    
    
    # Code for the specific functions of this Node: 
    # Here a while loop that gets user input of desired wheel command and publishes it. 
    # The condition on the "while" (rospy.is_shutdown()) evaluates whether ROS is in the process of shutting down, or has shut down. See https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown 
    while not rospy.is_shutdown(): 
    
        # use the Python "input" command to get a value. Use the int() function to turn the string into an integer. 
        wheel_command_left = int(input('Enter wheel command left (-480 to +480) \n'))
            
        # Check for good inputs and fix them if bad: 
        if wheel_command_left < -480: 
            wheel_command_left = -480
        elif wheel_command_left > 480: 
            wheel_command_left = 480
        
        # Pack the message object with the current data.
        wheel_command_left_msg.data = wheel_command_left
        
        # Publish the message. 
        pub_wheel_command_left.publish(wheel_command_left_msg)
        
        
        #### CODE HERE ####
        # Add a input and publishing for the Right wheel
        #### END CODE ####





# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        talker_for_wheel_commands()
    except rospy.ROSInterruptException: 
        pass
