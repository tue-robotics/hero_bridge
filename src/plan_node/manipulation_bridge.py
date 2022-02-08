#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''
import moveit_commander
import rospy
import actionlib

from tf.transformations import quaternion_from_euler, quaternion_multiply
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest

from std_srvs.srv import Trigger
from tue_manipulation_msgs.msg import GraspPrecomputeAction
from geometry_msgs.msg import PoseStamped, Point, Pose
from tmc_manipulation_msgs.msg import BaseMovementType, ArmManipulationErrorCodes

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry
import sys


class ManipulationBridge(object):
    def __init__(self):
        # server
        self.srv_manipulation = actionlib.SimpleActionServer('arm_center/grasp_precompute',
                                                             GraspPrecomputeAction,
                                                             execute_cb=self.manipulation_srv_inst,
                                                             auto_start=False)
        self.srv_manipulation.start()
        moveit_commander.roscpp_initialize(sys.argv + ['__ns:=/hero'])
        self.group_name = "whole_body"

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(ns='/hero', synchronous=True)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_workspace([0, 0, 0, 0])

    def manipulation_srv_inst(self, action):
        success = self.manipulation_srv(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation.set_succeeded()

    def manipulation_srv(self, action):
        """
        Here the grasp precompute action (TU/e) is sent to the robot through moveit
        :param action: the GraspPrecomputeAction type
        """
        success = True
        rospy.wait_for_service('ed/moveit_scene')
        try:
            moveit_call = rospy.ServiceProxy('ed/moveit_scene', Trigger)
            moveit_call()
        except rospy.ServiceException as e:
            rospy.logerr('The service call to ed/moveit_scene breaks, check that this service exists!')
            success = False

        pose_goal = PoseStamped()
        point = Point()
        pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw) #-3.1415927410125732, -1.5707963705062866, 0.680213212966919)
        point.x = action.goal.x #0.6055647134780884
        point.y = action.goal.y # 0.1297377049922943
        point.z = action.goal.z #0.699999988079071
        pose_goal.pose.position = point
        pose_goal.pose.orientation.x = pose_quaternion[0]
        pose_goal.pose.orientation.y = pose_quaternion[1]
        pose_goal.pose.orientation.z = pose_quaternion[2]
        pose_goal.pose.orientation.w = pose_quaternion[3]
        pose_goal.header.frame_id = 'odom'
        pose_goal.header.stamp = rospy.Time.now()

        rospy.logerr(pose_goal)
        # pose_goal = self.move_group.get_random_pose()
        # rospy.logerr(pose_goal)

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        # todo: logging/debugging stuff
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return success


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
