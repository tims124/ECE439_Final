#!/usr/bin/env python3
import rospy
import traceback 
import numpy as np
import time
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439WaypointXY")

#mobrob imports
from mobrob_util.msg import ME439WaypointXY
from std_msgs.msg import Bool
import encoders_and_motors as encmot
from pololu_drv8835_rpi import motors  	# MAX_SPEED is 480 (hard-coded)
from mobrob_util.msg import ME439WheelSpeeds
from geometry_msgs.msg import Pose2D

from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

#armrob imports
from armrob_util.msg import ME439WaypointXYZ 
from mobrob_util.msg import ME439WaypointXY
from sensor_msgs.msg import JointState

#load parameters
y_rotation_sign = np.sign(rospy.get_param('/y_rotation_sign'))
wksp_r = rospy.get_param('/wksp_r')
base_us = rospy.get_param('/servo_cmd_us_for_mapping_joint_01')

encoder_update_rate = rospy.get_param('/encoder_update_rate_hz')
tim1 = time.time() + 8
mtr_spd_r = 165
mtr_spd_l = 145
# =============================================================================
# # Set waypoints to hit along the path 
# =============================================================================
# Get parameters from rosparam
## none needed for waypoint setting ##

# Waypoints to hit: a "numpy.array" of [x,y] coordinates. 
# Example: Square
waypoints = np.array([[0.5, 0.],[0.5,0.5],[0.,0.5],[0.,0.]])

# =============================================================================
# # END of section on waypoint setting
# =============================================================================

##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you could skip segments if you wanted to)
waypoint_number = 0  # for waypoint seeking. 
path_complete = Bool()
path_complete.data = False


#disable motors
pub_mtr_disable = rospy.Publisher('/mtr_disable', Bool, queue_size=1)

# Publish desired waypoints at the appropriate time. 
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_path_complete, mtr_spd
    # Launch this node with the name "set_waypoints"
    rospy.init_node('main', anonymous=False)
    
    # Declare the message to publish. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    msg_waypoint = ME439WaypointXY()

    # Create the publisher for the topic "waypoint_xy", with message type "ME439WaypointXY"
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)

    # Create the publisher for the topic "path_complete", with message type "Bool"
    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    # Create a subscriber that listens for messages on the "waypoint_complete" topic
    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)

    # Obstacle xy point subscriber
    sub_obs_pt = rospy.Subscriber('/obs_pt', ME439WaypointXY, check_dist)

    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz

    motors.motor1.setSpeed(mtr_spd_l)
    motors.motor2.setSpeed(mtr_spd_r)
    try: 
        # start a loop 
        while not rospy.is_shutdown():
            pub_path_complete.publish(path_complete)
            if path_complete.data:
                break
            else:
                msg_waypoint.x = waypoints[waypoint_number,0]
                msg_waypoint.y = waypoints[waypoint_number,1]
                
                # Actually publish the message
                pub_waypoint_xy.publish(msg_waypoint)
                # Log the info (optional)
                #rospy.loginfo(msg_waypoint)    
            
            r.sleep()

    except Exception:
        traceback.print_exc()
        pass

def rst_arm():
    pos = ME439WaypointXYZ()
    pos.xyz[0] = 0.05
    pos.xyz[1] = 0 
    pos.xyz[2] = 0.1
    compute_joint_angles(pos)



    
# =============================================================================
# # Function to publish waypoints in sequence: 
# # A Callback for whenever the '/waypoint_complete' topic comes in. 
# # This function increments the waypoint_number whenever one waypoint is satisfied. 
# # NOTE it does Not actually publish the new waypoint, 
# # because that's happening in the loop above. 
# # This function also checks if the whole path is done (if there are not more waypoints)
# # If so it publishes a "path_complete" message with value True.
# =============================================================================
def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_path_complete
    
####    CODE HERE: 
    # # If the message (stored in variable 'msg_in') tells us that '/waypoint_complete' is True, 
    # # Then increment the waypoint number (variable 'waypoint_number'.  
    # # Edit these lines to make that happen. 
    if msg_in.data:  # The data type is Boolean (True/False), so this condition is satisfied if the value of msg_in.data is "True"
        waypoint_number = waypoint_number + 1
####    CODE END
    
    # # Handle the special case of the last waypoint: 
    # # If the last waypoint was reached, set "path_complete" and publish it
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
        # waypoint_number = waypoint_number - 1  # This line prevents an array out of bounds error to make sure the node stayed alive. By commenting, allow it to increment past the end, which will throw an exception (array out of bounds) the next time it publishes a waypoint and cause the node to die. 
    else:
        path_complete.data = False
    
    pub_path_complete.publish(path_complete)






# global variable to hold the waypoint currently being tracked
waypoint = ME439WaypointXYZ()

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

# Create the message
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint'];
joint_angles_desired_msg.position = [0., -np.pi/2., np.pi/2., 0., 0., 0.]      # upright neutral position

# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================

# servo ms control
pub_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
# servo angle control 
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)

servo_commands_msg = JointState()
servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05']
cmd_all = [0]*6  # List of 6 values of 1500 each
    

# =============================================================================
# # Function that checks if a detected obstacle is within workspace 
# =============================================================================
def move_obs(msg_in):
        pos = ME439WaypointXYZ()
        rst = ME439WaypointXY()
        rst.x = 0
        rst.y = 0
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pub_mtr_disable.publish(Bool(True))

        if(msg_in.y == 1): #Left
            pos.xyz[0] = 0
            pos.xyz[1] = 0.15
            pos.xyz[2] = 0.15
            compute_joint_angles(pos)
            print("start right")

            time.sleep(0.5)
            pos.xyz[2] = 0.0
            compute_joint_angles(pos)

            time.sleep(1)
            move_servo_0(base_us[0])
            print("clear")

            
        else: #Right
            pos.xyz[0] = 0
            pos.xyz[1] = -0.15
            pos.xyz[2] = 0.15
            compute_joint_angles(pos)
            print("start left")

            time.sleep(0.5)
            pos.xyz[2] = 0.
            compute_joint_angles(pos)

            time.sleep(1)
            move_servo_0(base_us[2])
            print("clear")

        time.sleep(1)
        rst_arm()
        #check_dist(rst)
        pub_mtr_disable.publish(Bool(False))
        



def move_servo_0(pulse_width_us):
    global cmd_all, servo_commands_msg
    cmd_all[0] = int(pulse_width_us)
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)

# =============================================================================
# # Function that checks if a detected obstacle is within workspace 
# =============================================================================
def check_dist(msg_in):
    global tim1
    if(time.time() - tim1 > 8):
        if(msg_in.x == 0):
            rst_arm()
            motors.motor1.setSpeed(mtr_spd_l)
            motors.motor2.setSpeed(mtr_spd_r)
        else:
            move_obs(msg_in)
            tim1 = time.time()

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
    
#    rospy.loginfo(joint_angles_desired_msg)


    
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

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass