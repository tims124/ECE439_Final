#!/usr/bin/env python3

# ROS node to command a set of 6 servos using Sliders. 
# Peter Adamczyk, University of Wisconsin - Madison
# Updated 2021-01-26
 

import tkinter as tk
import traceback 
import rospy
import numpy as np
# IMPORT the messages: 
from sensor_msgs.msg import JointState


# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================
pub_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
servo_commands_msg = JointState()
servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05']

map_cmd_us_01 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_01'))
map_cmd_us_12 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_12'))
map_cmd_us_23 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_23'))
map_cmd_us_34 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_34'))
map_cmd_us_45 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_45'))
map_cmd_us_56 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_56'))


cmd_all = [0]*6  # List of 6 values of 1500 each
    
# Specific functions for the specific Servos/Sliders       
def move_servo_0(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[0] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)

def move_servo_1(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[1] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_2(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[2] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_3(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[3] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_4(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[4] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
def move_servo_5(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[5] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    

def shutdown_servos():
    cmd_all = [0,0,0,0,0,0]
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)    


#%% Section to set up a nice Tkinter GUI with sliders. 
def main(): 
    # Initialize ROS node
    rospy.init_node('manual_servo_us_sliders', anonymous=False)    
    
    # set up GUI
    root = tk.Tk()
    root.title("Manual Robot Arm Servo Control (Pulses in Microseconds)")
    
    # draw a big slider for servo 0 position
    min_us = min(map_cmd_us_01)
    max_us = max(map_cmd_us_01)
    mid_us = np.int(np.mean([min_us, max_us]))
    scale0 = tk.Scale(root,
        from_ = 300,
        to = 2700,
        command = move_servo_0,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_0 us: Current calibration valid from {0} to {1}'.format(min_us,max_us))
    scale0.set(mid_us)
    scale0.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 1 position
    min_us = min(map_cmd_us_12)
    max_us = max(map_cmd_us_12)
    mid_us = np.int(np.mean([min_us, max_us]))
    scale1 = tk.Scale(root,
        from_ = 300,
        to = 2700,
        command = move_servo_1,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_1 us: Current calibration valid from {0} to {1}'.format(min_us,max_us))
    scale1.set(mid_us)
    scale1.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 2 position
    min_us = min(map_cmd_us_23)
    max_us = max(map_cmd_us_23)
    mid_us = np.int(np.mean([min_us, max_us]))
    scale2 = tk.Scale(root,
        from_ = 300,
        to = 2700,
        command = move_servo_2,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_2 us: Current calibration valid from {0} to {1}'.format(min_us,max_us))
    scale2.set(mid_us)
    scale2.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 3 position
    min_us = min(map_cmd_us_34)
    max_us = max(map_cmd_us_34)
    mid_us = np.int(np.mean([min_us, max_us]))
    scale3 = tk.Scale(root,
        from_ = 300,
        to = 2700,
        command = move_servo_3,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_3 us: Current calibration valid from {0} to {1}'.format(min_us,max_us))
    scale3.set(mid_us)
    scale3.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 4 position
    min_us = min(map_cmd_us_45)
    max_us = max(map_cmd_us_45)
    mid_us = np.int(np.mean([min_us, max_us]))
    scale4 = tk.Scale(root,
        from_ = 300,
        to = 2700,
        command = move_servo_4,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_4 us: Current calibration valid from {0} to {1}'.format(min_us,max_us))
    scale4.set(mid_us)
    scale4.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 5 position
    min_us = min(map_cmd_us_56)
    max_us = max(map_cmd_us_56)
    mid_us = np.int(np.mean([min_us, max_us]))
    scale5 = tk.Scale(root,
        from_ = 300,
        to = 2700,
        command = move_servo_5,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_5 us: Current calibration valid from {0} to {1}'.format(min_us,max_us))
    scale5.set(mid_us)
    scale5.pack(anchor = tk.CENTER)
    
    # run Tk event loop
    root.mainloop()
    
    # Shut the servos down if the window closes 
    shutdown_servos()


if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        shutdown_servos()
        pass