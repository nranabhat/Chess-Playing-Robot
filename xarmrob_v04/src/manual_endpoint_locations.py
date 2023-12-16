#!/usr/bin/env python3

import numpy as np
import rospy
import traceback
from sensor_msgs.msg import JointState

import FwdKinArmRob_serial as FK
import InvKinArmRob_serial as IK
import nico_InvKin as NIK
import smooth_interpolation as smooth
from chessboard_utils import make_move_path, load_config

# Load parameters from rosparam to keep handy for the functions below: 
# Matched lists of angles and microsecond commands
map_ang_rad_01 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_01')))
map_ang_rad_12 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_12')))
map_ang_rad_23 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_23')))
map_ang_rad_34 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_34')))
map_ang_rad_45 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_45')))
map_ang_rad_56 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_56')))
map_ang_rad_gripper = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_gripper')))

# limits for each of the joints
rotlim_01 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_01')))
rotlim_12 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_12')))
rotlim_23 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_23')))
rotlim_34 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_34')))
rotlim_45 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_45')))
rotlim_56 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_56')))
rotlim_gripper = np.radians(np.array(rospy.get_param('/gripper_limits')))

# Create the publisher. Name the topic "joint_angles_desired", with message type "JointState"
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)

# Create the message
cmds = [0., -np.pi/2., np.pi/2., 0., 0., 0., np.pi/2.]
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper'];
joint_angles_desired_msg.position = cmds      # upright neutral position


def move_gripper(arm_angles, point_index):
    """ 
        Params: arm_angle - list of angles in radians corresponding to [alpha, b1, b2, g1, b3, g2]
                point_index - int from 0 to 5 indicating which point in the path

        Returns: ang_lim list of [alpha, b1, b2, g1, b3, g2, gripper] angles 
     """
    # determine gripper_angle - float angle of gripper from 0: open to 90: closed
    if point_index < 3:
        gripper_angle = np.pi/4 # open
    if point_index >= 3 and point_index <= 5: # transitioning piece 
        gripper_angle = 80/90 * np.pi/2 # 70 degrees (closed)
    if point_index >= 6:
        gripper_angle = np.pi/4 # open

    ang_lim = np.append(arm_angles, gripper_angle) ## append a Gripper angle. This needs updating. 
    joint_angles_desired_msg.position = ang_lim 
    joint_angles_desired_msg.header.stamp = rospy.Time.now()
    pub_joint_angles_desired.publish(joint_angles_desired_msg)
    rospy.loginfo('Moving to gripper to '+str(np.degrees(gripper_angle))+ ' degrees')
    # rospy.loginfo('Predicted location: \n{}'.format(xyz_pred))
    return ang_lim

def manual_endpoint_location(): 
    
    rospy.init_node('manual_endpoint_locations',anonymous=False)
    
    current_position = np.array([0.1, 0.0, 0.08])  # Initialize current position, update with actual robot's initial position

    while not rospy.is_shutdown(): 

        try:
            # Load the chessboard configuration and calculate the move path
            start_square = input("Enter the starting square (e.g., A7): ")
            end_square = input("Enter the ending square (e.g., D3): ")
        except:
            rospy.loginfo('Bad Entry, try again!')
            continue 
        print('Moving from '+str(start_square)+' to '+str(end_square))

        config = load_config('robot.yaml')
        move_path = make_move_path(start_square, end_square, config)


        # Loop through the move path and compute IK for each point
        j = 0 # index that will go from 0 to 5 to indicate point in path.
        for xyz_goal in move_path:
            
            # Generate a minimum jerk trajectory to the new point
            _, trajectory = smooth.minimum_jerk_interpolation(current_position, xyz_goal, 
                                                            endpoint_speed=0.5, command_frequency=45)
            #print(trajectory)
            r = rospy.Rate(45)  # Adjust the rate as needed for smoothness
            for point in trajectory:
                # Compute Inverse Kinematics for each intermediate point
                #ang = IK.armrobinvkin(np.array(xyz_goal))
                ang = NIK.nico_IK(np.array(point))
                # Compute limited joint angles. 
                ang_lim = ang
                ang_lim[0] = np.clip(ang[0], np.min(rotlim_01), np.max(rotlim_01))
                ang_lim[1] = np.clip(ang[1], np.min(rotlim_12), np.max(rotlim_12))
                ang_lim[2] = np.clip(ang[2], np.min(rotlim_23), np.max(rotlim_23))
                ang_lim[3] = np.clip(ang[3], np.min(rotlim_34), np.max(rotlim_34))
                ang_lim[4] = np.clip(ang[4], np.min(rotlim_45), np.max(rotlim_45))
                ang_lim[5] = np.clip(ang[5], np.min(rotlim_56), np.max(rotlim_56))
                
                # Predict where the "limited" angles will get you. 
                xyz_pred = FK.armrobfwdkin(ang_lim)
                xyz_err_pred = point-xyz_pred  
                xyz_err_norm = np.sqrt(  np.sum(  np.power(xyz_err_pred, 2) ) )
                if np.isnan(xyz_err_norm) or (xyz_err_norm > 0.001):
                    rospy.loginfo('Unreachable Endpoint!')
                    if np.isnan(xyz_err_norm):
                        go_anyway = 'N'
                    else:
                        go_anyway = input('Move to nearest point {} - Y or N?\n')
                    
                    if not (go_anyway[0].upper() == 'Y') : 
                        rospy.loginfo('Not moving - try again.')
                        continue
                    
                # If the program gets here it has been told to go ahead. 
                # Move to endpoint. 
                
                #ang_lim = np.append(ang_lim, 0.) ## append a Gripper angle. This needs updating. 
                ang_lim = move_gripper(ang_lim, j)
                joint_angles_desired_msg.position = ang_lim 
                joint_angles_desired_msg.header.stamp = rospy.Time.now()
                pub_joint_angles_desired.publish(joint_angles_desired_msg)
                rospy.loginfo('Moving to {}'.format(ang_lim))
                rospy.loginfo('Predicted location: \n{}'.format(xyz_pred))

                r.sleep()

            j += 1
            
            current_position = xyz_goal  # Update the current position to the new endpoint
                

if __name__ == "__main__":
    try:
        manual_endpoint_location()
    except:
        traceback.print_exc()
        pass
    
    
