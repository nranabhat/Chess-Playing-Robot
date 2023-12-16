#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from chessboard_utils import make_move_path, load_config

class InverseKinematicsNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('inverse_kinematics_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "your_robot_arm"  # Replace with your robot arm group name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def compute_ik(self, position, orientation):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = orientation.w
        pose_goal.orientation.x = orientation.x
        pose_goal.orientation.y = orientation.y
        pose_goal.orientation.z = orientation.z
        pose_goal.position.x = position.x
        pose_goal.position.y = position.y
        pose_goal.position.z = position.z

        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.plan()

        # You can execute the plan on a real robot or in a simulation like this:
        # self.move_group.go(wait=True)

        # Clear targets after planning
        self.move_group.clear_pose_targets()

        return plan

if __name__ == '__main__':
    ik_node = InverseKinematicsNode()

    try:
        # Load the chessboard configuration and calculate the move path
        start_square = input("Enter the starting square (e.g., A7): ")
        end_square = input("Enter the ending square (e.g., D3): ")
        config = load_config('robot.yaml')
        move_path = make_move_path(start_square, end_square, config)

        # Loop through the move path and compute IK for each point
        for point in move_path:
            position = geometry_msgs.msg.Point(*point)
            orientation = geometry_msgs.msg.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)  # Fixed orientation, adjust as needed
            plan = ik_node.compute_ik(position, orientation)
            # Execute or visualize the plan as needed
            print(plan)

    except rospy.ROSInterruptException:
        pass

