from __future__ import print_function

# System
import argparse

import time
import rospy
import actionlib
from tue_manipulation_msgs.msg import GraspPrecomputeAction, GraspPrecomputeGoal

from robot_skills.arm.gripper import ParrallelGripper
from robot_skills.robot_part import RobotPart

if __name__ == "__main__":
        parser = argparse.ArgumentParser(description="Put an imaginary object in the world model and grasp it using the "
                                                         "'Grab' smach state")
        parser.add_argument("x", type=float, help="x-coordinate (in map) of the imaginary object")
        parser.add_argument("y", type=float, help="y-coordinate (in map) of the imaginary object")
        parser.add_argument("z", type=float, help="z-coordinate (in map) of the imaginary object")
        args = parser.parse_args()

        rospy.init_node("test_manipulation_bridge")
        ac_grasp_precompute = actionlib.SimpleActionClient("hero/arm_center/grasp_precompute", GraspPrecomputeAction)

        grasp_precompute_goal = GraspPrecomputeGoal()
        grasp_precompute_goal.goal.header.frame_id = "base_link"
        grasp_precompute_goal.goal.header.stamp = rospy.Time.now()

        grasp_precompute_goal.PERFORM_PRE_GRASP = False
        grasp_precompute_goal.FIRST_JOINT_POS_ONLY = False

        grasp_precompute_goal.allowed_touch_objects = list()

        grasp_precompute_goal.goal.x = args.x
        grasp_precompute_goal.goal.y = args.y
        grasp_precompute_goal.goal.z = args.z

        grasp_precompute_goal.goal.roll = 0
        grasp_precompute_goal.goal.pitch = 0
        grasp_precompute_goal.goal.yaw = 0

        rospy.loginfo("sending goal:")
        ac_grasp_precompute.send_goal(grasp_precompute_goal)
        ac_grasp_precompute.wait_for_result()
        result = ac_grasp_precompute.get_result()

        rospy.loginfo("action_result: {}".format(result))
        rospy.spin()
