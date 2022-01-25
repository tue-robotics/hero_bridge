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
import geometry_msgs.msg
from tmc_manipulation_msgs.msg import BaseMovementType, ArmManipulationErrorCodes

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry
import sys


class ManipulationBridge(object):
    def __init__(self):
        # robot
        # self.robot = Robot()
        # self.whole_body = self.robot.try_get('whole_body')
        #TODO(Lars): look at namespacing

        # server
        self.srv_manipulation = actionlib.SimpleActionServer('arm_center/grasp_precompute',
                                                             GraspPrecomputeAction,
                                                             execute_cb=self.manipulation_srv_inst,
                                                             auto_start=False)
        self.srv_manipulation.start()
        moveit_commander.roscpp_initialize(sys.argv + ['__ns:=/hero'])
        #TODO(Lars): make it whole_body
        #TODO(Lars): fix base and whole body
        self.group_name = "arm"

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(ns='/hero', synchronous=True)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def manipulation_srv_inst(self, action):
        success = self.manipulation_srv(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation.set_succeeded()

    def manipulation_srv(self, action):
        """
        Here the grasp precompute action (TU/e) is translated to a PlanWithHandGoals (TMC) and send as goal to the robot
        :param action: the GraspPrecomputeAction type
        """
        success = True
        # pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        # pose = [action.goal.x, action.goal.y, action.goal.z], pose_quaternion

        # # Plan to a joint goal
        # joint_goal = self.move_group.get_current_joint_values()
        # joint_goal[0] = 0.3532629789664938
        # joint_goal[1] = -0.4293987981401086
        # joint_goal[2] = 1.1836900769463248
        # joint_goal[3] = -0.840112834258244
        # joint_goal[4] = 2.5121319061568474
        # joint_goal[5] = 0.0
        # self.move_group.go(joint_goal, wait=True)
        # self.move_group.stop()
        #TODO(Lars): change reference frame from odom to base_link?

        rospy.wait_for_service('ed/moveit_scene')
        try:
            moveit_call = rospy.ServiceProxy('ed/moveit_scene', Trigger)
            moveit_call()
        except rospy.ServiceException as e:
            rospy.logerr('This is broken!')

        rospy.logerr("This works!")
        pose_goal = self.move_group.get_random_pose()
        # cur_pose = move_group.get_current_pose()
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return success


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
