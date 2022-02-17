from __future__ import print_function

# System
import argparse

import time
import rospy
import actionlib
from tue_manipulation_msgs.msg import GraspPrecomputeAction, GraspPrecomputeGoal

from robot_skills.arm.gripper import ParrallelGripper
from robot_skills.robot_part import RobotPart

def manipulation_client(x, y, z):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient("hero/arm_center/grasp_precompute", GraspPrecomputeAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    grasp_precompute_goal = GraspPrecomputeGoal()
    grasp_precompute_goal.goal.header.frame_id = "base_link"
    grasp_precompute_goal.goal.header.stamp = rospy.Time.now()

    grasp_precompute_goal.PERFORM_PRE_GRASP = False
    grasp_precompute_goal.FIRST_JOINT_POS_ONLY = False

    grasp_precompute_goal.allowed_touch_objects = list()

    grasp_precompute_goal.goal.x = x
    grasp_precompute_goal.goal.y = y
    grasp_precompute_goal.goal.z = z

    grasp_precompute_goal.goal.roll = 0
    grasp_precompute_goal.goal.pitch = 0
    grasp_precompute_goal.goal.yaw = 0

    # Sends the goal to the action server.
    client.send_goal(grasp_precompute_goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
        parser = argparse.ArgumentParser(
                description="Put an imaginary object in the world model and grasp it using the "
                            "'Grab' smach state")
        parser.add_argument("x", type=float, help="x-coordinate (in map) of the imaginary object")
        parser.add_argument("y", type=float, help="y-coordinate (in map) of the imaginary object")
        parser.add_argument("z", type=float, help="z-coordinate (in map) of the imaginary object")
        args = parser.parse_args()
        try:
                # Initializes a rospy node so that the SimpleActionClient can
                # publish and subscribe over ROS.
                rospy.init_node('fibonacci_client_py')
                result = manipulation_client(args.x, args.y, args.z)
                print("Result:", ', '.join([str(n) for n in result.sequence]))
        except rospy.ROSInterruptException:
                print("program interrupted before completion", file=sys.stderr)
