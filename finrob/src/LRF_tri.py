#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# library imports
import rospy
import serial 

import time
import numpy as np
import traceback 

# message imports
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from armrob_util.msg import ObsPos
from mobrob_util.msg import ME439WaypointXY

#==============================================================================
# # Load parameters
#==============================================================================

thresh = rospy.get_param('/sens_thresh')

# This is the central code: set up a Node, set up a Publisher(s)
def sensors_reader(): 
    global l_flag, c_flag, r_flag
    # Launch a node called "sensors_node"
    rospy.init_node('LRF_triangulation', anonymous=False)

    # '/obs_pt' Publisher and message setup
    pub_obs_pos = rospy.Publisher('/obs_pos',  ObsPos, queue_size=1)
    msg_obs_pos = ObsPos()

    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyUSB0')  #serial port to alamode is /dev/ttyS0. # port to Arduino Nano is /dev/ttyUSB0 
    ser.baudrate = 57600 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. #

    ser.flush()  # Flush any data currently on the port
    ser.readline()

    # start of non-blocking timer variable
    # the tim1_dur allows for the timer to be ignored on start up
    tim1_dur = rospy.get_param('/tim1_dur')
    tim1 = time.time() - tim1_dur

    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    while not rospy.is_shutdown():

        try: 
           
            #serial input decoding
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
            
            line = line.split(":")
           
            #strips whitespace
            l = int(line[0])
            c = int(line[1])
            r = int(line[2])
   

            # Checks if there's an obstacle within the threshold
            # publishes to "/obs_pt" regardless
            # uses a non-blocking timer to prevent repeated triggers from arm
            if(time.time() - tim1 > tim1_dur):
                #print(line)

                # Picks which direction to move the arm
                # Typically the oposite side of the obstacle 

                # Preset 'True' to reduce the number of lines of code
                msg_obs_pos.det = True
                if(c > thresh):
                    # center sensor picks which direction to move the arm based on 
                    # which side is closer to the obstacle
                    if (l > r):
                        msg_obs_pos.lcr = True 
                        tim1 = time.time()
                    else: 
                        msg_obs_pos.lcr = False 
                        tim1 = time.time()
                    
                elif (l > thresh):
                    msg_obs_pos.lcr = True 
                    tim1 = time.time()
                    
                elif(r > thresh):    
                    msg_obs_pos.lcr = False 
                    tim1 = time.time()
                    
                else: 
                    msg_obs_pos.lcr = False
                    msg_obs_pos.det = False
            print(msg_obs_pos)                
            pub_obs_pos.publish(msg_obs_pos)


            continue
            
        
        except Exception:
            traceback.print_exc()
            pass
            

if __name__ == '__main__':
    try: 
        sensors_reader()
    except rospy.ROSInterruptException: 
        pass
