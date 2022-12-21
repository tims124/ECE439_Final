#!/usr/bin/env python3

import uuid, random
import numpy as np
from scipy import interpolate
import rospy


# Create a messed-up model of the arm using a unique ID leading to unique offsets
random.seed(a=uuid.UUID(int=uuid.getnode()))

us = np.ndarray((6,2))
deg = np.ndarray((6,2))

#for ii in np.arange(0,6):
#    us[ii] = np.array([random.randint(350,650), random.randint(2350,2650)])
#    deg[ii] = np.array([random.randint(-95,-75), random.randint(75,95)])
#rad = np.radians(deg)

deg[0] = np.array([random.randint(-105,-75), random.randint(75,105)])
us[0] = np.array([random.randint(350,650), random.randint(2350,2650)])
deg[1] = np.array([random.randint(-40,-30), random.randint(-170,-155)])
us[1] = np.array([random.randint(900,1100), random.randint(2350,2650)])
deg[2] = np.array([random.randint(-15,0), random.randint(145,165)])
us[2] = np.array([random.randint(2350,2650), random.randint(900,1100)])
deg[3] = np.array([random.randint(-105,-75), random.randint(75,105)])
us[3] = np.array([random.randint(350,650), random.randint(2350,2650)])
deg[4] = np.array([random.randint(-105,-75), random.randint(75,105)])
us[4] = np.array([random.randint(2350,2650), random.randint(350,650)])
deg[5] = np.array([random.randint(-105,-75), random.randint(75,105)])
us[5] = np.array([random.randint(350,650), random.randint(2350,2650)])

rad = np.radians(deg)

#print(us, deg)

## Interpolating functions using the model
f_interp_us_to_rad_01_model = interpolate.interp1d(us[0], rad[0])
f_interp_us_to_rad_12_model = interpolate.interp1d(us[1], rad[1])
f_interp_us_to_rad_23_model = interpolate.interp1d(us[2], rad[2])
f_interp_us_to_rad_34_model = interpolate.interp1d(us[3], rad[3])
f_interp_us_to_rad_45_model = interpolate.interp1d(us[4], rad[4])
f_interp_us_to_rad_56_model = interpolate.interp1d(us[5], rad[5])
        
    
# Load parameters from rosparam to keep handy for the functions below: 
# Matched lists of angles and microsecond commands
map_ang_rad_01 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_01')))
map_cmd_us_01 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_01'))
map_ang_rad_12 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_12')))
map_cmd_us_12 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_12'))
map_ang_rad_23 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_23')))
map_cmd_us_23 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_23'))
map_ang_rad_34 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_34')))
map_cmd_us_34 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_34'))
map_ang_rad_45 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_45')))
map_cmd_us_45 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_45'))
map_ang_rad_56 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_56')))
map_cmd_us_56 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_56'))

## Interpolating functions using the arm parameters
f_interp_us_to_rad_01_params = interpolate.interp1d(map_cmd_us_01, map_ang_rad_01)
f_interp_us_to_rad_12_params = interpolate.interp1d(map_cmd_us_12, map_ang_rad_12)
f_interp_us_to_rad_23_params = interpolate.interp1d(map_cmd_us_23, map_ang_rad_23)
f_interp_us_to_rad_34_params = interpolate.interp1d(map_cmd_us_34, map_ang_rad_34)
f_interp_us_to_rad_45_params = interpolate.interp1d(map_cmd_us_45, map_ang_rad_45)
f_interp_us_to_rad_56_params = interpolate.interp1d(map_cmd_us_56, map_ang_rad_56)


def convert_servo_commands_to_joint_state(cmd_all, arm_is_present):
    cmd00 = cmd_all[0]
    cmd01 = cmd_all[1]
    cmd02 = cmd_all[2]
    cmd03 = cmd_all[3]
    cmd04 = cmd_all[4]
    cmd05 = cmd_all[5]
    
    # If there is an arm present, just invert the interpolation using the ROS parameters to get back the intended angle.
    if arm_is_present:
        # Interpolate to find joint_state 
        jt00 = f_interp_us_to_rad_01_params(cmd00)
        jt01 = f_interp_us_to_rad_12_params(cmd01)
        jt02 = f_interp_us_to_rad_23_params(cmd02)
        jt03 = f_interp_us_to_rad_34_params(cmd03)
        jt04 = f_interp_us_to_rad_45_params(cmd04)
        jt05 = f_interp_us_to_rad_56_params(cmd05)
    else: 
        # Interpolate with the custom arm model: 
        jt00 = f_interp_us_to_rad_01_model(cmd00)
        jt01 = f_interp_us_to_rad_12_model(cmd01)
        jt02 = f_interp_us_to_rad_23_model(cmd02)
        jt03 = f_interp_us_to_rad_34_model(cmd03)
        jt04 = f_interp_us_to_rad_45_model(cmd04)
        jt05 = f_interp_us_to_rad_56_model(cmd05)

    
    jt_all = [jt00, jt01, jt02, jt03, jt04, jt05]
    
    return jt_all