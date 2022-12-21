#!/usr/bin/env python3
 

# Used with LRF_tri.py
# Works similar to a bubble rebound algorithm, but instead of bouncing \
# when an obstacle is within the bubble, this will actively move the   \
# obstacle out of the way


import rospy
import traceback 
import numpy as np
import time
# IMPORT the custom messages: 
from armrob_util.msg import ME439WaypointXYZ 
from armrob_util.msg import ObsPos
from sensor_msgs.msg import JointState
from pololu_drv8835_rpi import motors
from std_msgs.msg import Bool
# global variable to hold the waypoint currently being tracked
waypoint = ME439WaypointXYZ()
waypoint.xyz = np.array([0.,0.,0.])


# =============================================================================
# # Load transform parameters
# =============================================================================

# Sign of 'positive' rotations w.r.t. the y axis
y_rotation_sign = np.sign(rospy.get_param('/y_rotation_sign'))
# Vectors from each frame origin to the next frame origin, in the proximal
r_01 = np.matrix(rospy.get_param('/frame_offset_01')).transpose()
r_12 = np.matrix(rospy.get_param('/frame_offset_12')).transpose()
r_23 = np.matrix(rospy.get_param('/frame_offset_23')).transpose()
r_34 = np.matrix(rospy.get_param('/frame_offset_34')).transpose()        
r_45 = np.matrix(rospy.get_param('/frame_offset_45')).transpose()
r_56 = np.matrix(rospy.get_param('/frame_offset_56')).transpose()
r_6end = np.matrix(rospy.get_param('/endpoint_offset_in_frame_6')).transpose()

# limits for each of the joints
rotlim_01 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_01')))
rotlim_12 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_12')))
rotlim_23 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_23')))
rotlim_34 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_34')))
rotlim_45 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_45')))
rotlim_56 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_56')))

# =============================================================================

# '/joint_angles_desired' Publisher and message setup
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint'];
joint_angles_desired_msg.position = [0., -np.pi/2., np.pi/2., 0., 0., 0.]      # upright neutral position



# '/servo_commands' Publisher, message and intermediate var setup
pub_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
servo_commands_msg = JointState()
servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05']
cmd_all = [0]*6 

# Pre-generated positions for sweaping
L_start = ME439WaypointXYZ()
L_start.xyz[0] = 0
L_start.xyz[1] = 0.15
L_start.xyz[2] = 0.15

R_start = ME439WaypointXYZ()
R_start.xyz[0] = 0
R_start.xyz[1] = -0.15
R_start.xyz[2] = 0.15

# param for servo commands
base_us = rospy.get_param('/servo_cmd_us_for_mapping_joint_01')


# start of non-blocking timer variable
# the tim1_dur allows for the timer to be ignored on start up
tim1_dur = rospy.get_param('/tim1_dur')
tim1 = time.time() - tim1_dur

mtr_spd_l = rospy.get_param('/mtr_spd_l')
mtr_spd_r = rospy.get_param('/mtr_spd_r')
def main(): 
    rospy.init_node('arm_main', anonymous=False)

    sub_obs_pos = rospy.Subscriber('/obs_pos', ObsPos, check_obs)
      
    # Spin() to keep the node from exiting
    rospy.spin()


# =============================================================================
# # Function to update the path to the waypoint based on the robot's current position
# =============================================================================
def compute_joint_angles(endpoint_msg_in):  
#    print(endpoint_msg_in.xyz)
    # First compute the inverse kinematics for perfect endpoint positioning
    joint_angles_IK = compute_IK(endpoint_msg_in)
#    print(joint_angles_IK)
    
    # Then Limit the angle at each joint to its achievable range
    angles_limited = limit_joint_angles(joint_angles_IK)
    
    # Pack the joint angles and publish the message
    joint_angles_desired_msg.position = angles_limited
    joint_angles_desired_msg.header.stamp = rospy.Time.now()
    pub_joint_angles_desired.publish(joint_angles_desired_msg)
    
    
def compute_IK(endpoint_msg_in):
    xyz = endpoint_msg_in.xyz
    
    # Compute base rotation plus 2-link IK... 
    # Assuming that the "forearm" and "fingers" do not rotate 
    gamma3 = 0
    gamma5 = 0
    # ... and assume the wrist is controlled so the "fingers" point directly in the radial direction
    # (in order to hold the "marker" directly vertical)
    # THIS WOULD ALL CHANGE if the wrist is controlled in some other way. 

    # ... and assume the wrist is controlled so the "fingers" point directly in the radial direction
    # (in order to hold the "marker" directly vertical)
    # THIS WOULD ALL CHANGE if the wrist is controlled in some other way. 
    
    # First the out-of-plane rotation
    alpha0 = np.arctan2(xyz[1], xyz[0])
    
    # Now compute the radial and vertical distances spanned by the two links of the arm
    R = np.linalg.norm(xyz[0:2])   # Remember that this means "start at 0, stop BEFORE 2"
    dR = R - (r_12[0] + r_6end[0])        # subtract off the x of all the links that are not part of the 2-link kinematic solution. NOTE this only works because the X offsets are known to be positioned in the R direction. 
    dz = xyz[2] - (r_01[2] + r_12[2] + r_6end[2])   # subtract off the Z of all the links that are not part of the 2-link kinematic solution
    
    # Now compute the "overall elevation" angle from the "shoulder" to the "wrist" 
    # NOTE this assumes rotations about the +y axis (positive rotations push the wrist down)
    psi = -np.arctan2(dz, dR)  # use negative because of the positive-rotations-down convention. 
    # Now the difference between the actual shoulder angle and the overall elevation angle
    # ... being aware that there are two solutions and we want the "elbow up" configuration. 
    L1 = np.linalg.norm(r_23)  # vector magnitude of the link that spans from shoulder to elbow ("upper arm")
    L2 = np.linalg.norm(r_34)  # vector magnitude of the link that spans from elbow to wrist ("lower arm")
    H = np.linalg.norm(np.array((dz,dR))) # vector magnitude of the vector from shoulder to wrist. (H = hypotenuse)
    phi = np.arccos( (L2**2 - L1**2 - H**2)/(-2*L1*H) )  # arccos will always return a positive value. 
    
    # Compute the "elbow up" solution for beta1
    beta1 = psi - phi   #  phi is always positive (from arccos function) so "-phi" is the elbow pose in a more negative position (elbow up for the +y axis rotations) 
    
    # Compute the corresponding solution for beta2VL (VL = "virtual link" direct from joint 2 to joint 3 (elbow to wrist)
    # Use the ArcTangent (two quadrant)
    beta2VL = np.arctan2(H*np.sin(phi), H*np.cos(phi)-L1)
#    print(beta2VL)
    
    # Compute the offset in angle between  the VL (virtual link straight from joint 3 to joint 4) and the true link axis. 
    # True link should be more positive by this amount. 
    beta2_offset_from_VL = np.arctan2(r_34[2], r_34[0])  
    
    # Real-world beta2, assuming +y axis rotations
    beta2 = beta2VL + beta2_offset_from_VL 
       
    # Depending on the sign of positive rotations, give back the rotations. 
    beta1 = beta1 * y_rotation_sign
    beta2 = beta2 * y_rotation_sign
    
    # Compute beta4 to cancel out beta1 and beta2 (works regardless of the sign) 
    beta4 = -(beta1+beta2)

    
    return np.array([alpha0, beta1, beta2, gamma3, beta4, gamma5])

    
def limit_joint_angles(angles):
    angles_limited = angles
    
    # Clip (saturate) the angles at the achievable limits. 
    angles_limited[0] = np.clip(angles_limited[0], np.min(rotlim_01), np.max(rotlim_01))
    angles_limited[1] = np.clip(angles_limited[1], np.min(rotlim_12), np.max(rotlim_12))
    # special case for angle 2 below, skip it here:
    # Reinstated 2019 with modified arms
    angles_limited[2] = np.clip(angles_limited[2], np.min(rotlim_23), np.max(rotlim_23))
    angles_limited[3] = np.clip(angles_limited[3], np.min(rotlim_34), np.max(rotlim_34))
    angles_limited[4] = np.clip(angles_limited[4], np.min(rotlim_45), np.max(rotlim_45))
    angles_limited[5] = np.clip(angles_limited[5], np.min(rotlim_56), np.max(rotlim_56))
    
    return angles_limited


# =============================================================================
# # Function that returns arm to a present position
# =============================================================================
def rst_arm():
    pos = ME439WaypointXYZ()
    pos.xyz[0] = 0.05
    pos.xyz[1] = 0 
    pos.xyz[2] = 0.1
    compute_joint_angles(pos)


# =============================================================================
# # Base/Yaw servo controller
# # Used to sweap obstacles
# =============================================================================
def move_servo_0(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[0] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)


# =============================================================================
# # Function that checks if a detected obstacle is within workspace 
# =============================================================================
def move_obs(msg_in):
    global L_start, R_start
    pos = ME439WaypointXYZ()
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)

    if(msg_in.lcr == 1): #Left
        # Moves arm to starting position
        pos = L_start
        compute_joint_angles(pos)
        time.sleep(0.5)

         # Lowers arm 
        pos.xyz[2] = 0.0
        compute_joint_angles(pos)
        time.sleep(1)

        # Sweap
        move_servo_0(base_us[0])

        
    else: #Right
        # Moves arm to starting position
        pos = R_start
        compute_joint_angles(pos)
        time.sleep(0.5)

        # Lowers arm 
        pos.xyz[2] = 0.
        compute_joint_angles(pos)
        time.sleep(1)

        # Sweap
        move_servo_0(base_us[2])

    time.sleep(1)
    rst_arm()


# =============================================================================
# # Intermediate function between triangulation and move_obs
# =============================================================================
def check_obs(msg_in):
    global tim1, mtr_spd_l, mtr_spd_r
    # Uses a timer to prevent motors from turning on before LRFs have turned back on
    if(time.time() - tim1 > tim1_dur):
        if(msg_in.det == 1): 
            move_obs(msg_in)
            tim1 = time.time()
        else:
            rst_arm()
            motors.motor1.setSpeed(mtr_spd_l)
            motors.motor2.setSpeed(mtr_spd_r)   



if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
