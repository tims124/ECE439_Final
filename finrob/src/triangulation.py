#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# Import "serial" to get data from the AlaMode
import serial   
# "traceback" is a library that lets you track down errors. 
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from mobrob_util.msg import ME439WaypointXY
#from mobrob_util.msg import ME439LCR
import numpy as np
import time
#==============================================================================
# # Get parameters from rosparam
#==============================================================================
sens_sep = rospy.get_param('/sens_sep')
sens_dist = rospy.get_param('/sens_dist')

msg_waypoint = ME439WaypointXY()

l_flag = False
c_flag = False
r_flag = False
thresh = 500

# This is the central code: set up a Node, set up a Publisher(s)
def sensors_reader(): 
    global l_flag, c_flag, r_flag
    # Launch a node called "sensors_node"
    rospy.init_node('triangulation', anonymous=False)

    # Create the publishers. Name each topic "sensors_##", with message type "Int32" 
    # (because that's what is received over the serial port)
    # Note the queue_size=1 statement: don't let it develop a backlog! 
    

    pub_OBS_pt = rospy.Publisher('/obs_pt',  ME439WaypointXY, queue_size=1)

    msg_waypoint = ME439WaypointXY();

    # Declare the message that will go on the topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We put data in it using the .data field of the message.

    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyUSB0')  #serial port to alamode is /dev/ttyS0. # port to Arduino Nano is /dev/ttyUSB0 
    ser.baudrate = 57600 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. #

    ser.flush()  # Flush any data currently on the port
    ser.readline()
    tim1 = time.time()
    stall = True
    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    while not rospy.is_shutdown():

        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command

            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
            #print(line)
            line = line.split(":")
            # Even elements are the sensor, odd elements are the sensor value
            # Read serial data into variables L(eft sensor), R(ight sensor), C(enter sensor)
        
            #if(line[0] == 'Left'):
            #    l = float(line[1])
            #    l_flag = True
            #elif(line[0] == 'Center'):
            #    c = float(line[1])
            #    c_flag = True
            #elif(line[0] == 'Right'):
            #    r = line[1]
            #    r_flag = True
            
            #400 is min - dist 
            
            l = int(line[0])
            c = int(line[1])
            r = int(line[2])
            #print("sensor input")

            #print(line)
            if(time.time() - tim1 > 10):
                print(line)
                if (l > thresh):
                    msg_waypoint.x = 0.15
                    msg_waypoint.y = 2
                    tim1 = time.time()
                    print("l")
                elif(c > thresh):
                    msg_waypoint.x = 0.15
                    msg_waypoint.y = 2.
                    tim1 = time.time()
                    print("c")
                elif(r > thresh):    
                    msg_waypoint.x = 0.15
                    msg_waypoint.y = 1
                    tim1 = time.time()
                    print("r")
                else: 
                    msg_waypoint.x = 0
                    msg_waypoint.y = 0
                
            pub_OBS_pt.publish(msg_waypoint)


            # When all three sensor values have been read, start triagulation
            if(False):
                # reset flags
                #l_flag = False
                #c_flag = False
                #r_flag = False            
                l = int(line[0])
                c = int(line[1])
                r = int(line[2])
                #print("sensor input")
                print(line)
                #TODO: Convert from sensor value to distances

                # Detect obstacle within arm workspace
                # Obstacle position position in world coordinates
                # Obstacle Distace from sensor -> sensor xy-coordinates
                # sensor xy-coordinates + uniform transform -> world coordinates

                # Calculating triangle values 
                # sensor values: L, R, C
                # triangle sides: L, R, sens_sep
                # trangle angles: L, R, P 
                thetaL = np.arccos((sens_sep**2 + r**2 - l**2)/(2*r*sens_sep))
                thetaR = np.arccos((sens_sep**2 + l**2 - r**2)/(2*l*sens_sep))

                # x,y in sensor coordinates
                
                # Obstacle is on the Left side 
                if( l < r): 
                    x = l*np.sin(thetaR)
                    y = -1*(2.5 - l*np.cos(thetaR))
                
                # Obstacle is on the right side
                elif(l > r):    
                    x = r*np.sin(thetaL)
                    y = (2.5 - r*np.cos(thetaL))
                
                # Obstacle is directly in front
                else: 
                    x = c
                    y = 0

                # convert sensor coordinates to world coordinates
                msg_waypoint.x = x + sens_dist
                msg_waypoint.y = y
                #print("obs distance")
                #print(msg_waypoint)
                # Publish obstacle coordiantes
                pub_OBS_pt.publish(msg_waypoint)

            continue
            
        
        except Exception:
            traceback.print_exc()
            pass
            



if __name__ == '__main__':
    try: 
        sensors_reader()
    except rospy.ROSInterruptException: 
        pass
