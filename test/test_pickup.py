from __future__ import print_function

# System
import argparse

import rospy
import actionlib
import sys
import tf2_ros

from moveit_msgs.msg import PickupAction, PickupGoal, Grasp

from tf.transformations import quaternion_from_euler


def create_grasp(x, y, z):
    grasp = Grasp()
    grasp.grasp_pose.header.frame_id = "base_link"
    grasp.grasp_pose.pose.position.x = x
    grasp.grasp_pose.pose.position.y = y
    grasp.grasp_pose.pose.position.z = z

    # hero grasp offset: roll: 3.14159265359, pitch: -1.57079632679, yaw: 0.0
    q = quaternion_from_euler(3.14159265359, -1.5707, 0)

    grasp.grasp_pose.pose.orientation.x = q[0]
    grasp.grasp_pose.pose.orientation.y = q[1]
    grasp.grasp_pose.pose.orientation.z = q[2]
    grasp.grasp_pose.pose.orientation.w = q[3]

    grasp.pre_grasp_approach.direction.header.frame_id = "base_link"

    grasp.post_grasp_retreat.direction.header.frame_id = "base_link"

    return grasp


def create_pickup_goal(x, y, z):
    grasp = create_grasp(x, y, z)
    pickupgoal = PickupGoal()
    pickupgoal.group_name = "whole_body"
    pickupgoal.possible_grasps = [grasp]

    pickupgoal.allowed_planning_time = 20.0

    return pickupgoal


def manipulation_client(x, y, z):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient("hero/pickup", PickupAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    grasp_precompute_goal = create_pickup_goal(x, y, z)
    rospy.loginfo("pickpup goal: {}".format(grasp_precompute_goal))

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
        print("Result: {}".format(result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
