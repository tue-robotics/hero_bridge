#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from tmc_manipulation_msgs.srv import SafeJointChange
from tmc_planning_msgs.srv import PlanWithJointGoals
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
import hsrb_interface
import sys

reload(sys)
sys.setdefaultencoding('utf8')

class JointTrajectory(object):
    def __init__(self):
        rospy.logdebug("Getting robot")
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.try_get('whole_body')
        rospy.logdebug("Got whole_body")

        # server
        self.srv_safe_joint_change = actionlib.SimpleActionServer('/hero/body/joint_trajectory_action',
                                                                  FollowJointTrajectoryAction,
                                                                  execute_cb=self.moveit_joint_change_action,
                                                                  auto_start=False)

        # clients
        self.client_moveit_joint_change = rospy.ServiceProxy('/plan_with_joint_goals', PlanWithJointGoals)

        self.srv_safe_joint_change.start()

    def moveit_joint_change_action(self, goal):
        """
        :param goal: the FollowJointTrajectoryAction type
        :return:
        """
        success = True

        rospy.logdebug('points: {}'.format(goal.trajectory.points))
        for j, point in enumerate(goal.trajectory.points):
            rospy.logdebug("Process point {}: {}".format(j, point))
            joints_goal = {}
            for i, n in enumerate(goal.trajectory.joint_names):
                rospy.logdebug("Process joint {}: {}".format(i, n))
                joints_goal[n] = point.positions[i]
            self.client_moveit_joint_change.wait_for_service()
            rospy.logdebug('joints_goal is {}'.format(joints_goal))
            try:
                rospy.logdebug('Call whole_body.move_to_joint_positions(joint_goal)')
                self._whole_body.move_to_joint_positions(joints_goal)
                rospy.logdebug('Call whole_body.move_to_joint_positions(joint_goal) DONE')
                success = True
            except Exception as e: # TODO: catch specific exception, probably hsrb_interface.exceptions.MotionPlanningError
                success = False
                rospy.logerr('Move_to_joint_positions raised exception "{}"'.format(e))

            rospy.logdebug("self.client_moveit_joint_change.wait_for_service()")
            self.client_moveit_joint_change.wait_for_service()
            if not success:
                rospy.logwarn('Could not successfully move to specified joint goal, '
                              'failure occurred at point({}/{}) {}'.format(j, len(goal.trajectory.points), point))
                # break

        rospy.logdebug("All positions processed, success = {}".format(success))

        if success:
            self.srv_safe_joint_change.set_succeeded()
        else:
            self.srv_safe_joint_change.set_aborted()

        rospy.logdebug("moveit_joint_change_action finished")

if __name__ == "__main__":
    rospy.init_node('joint_trajectory_action')
    joint_trajectory = JointTrajectory()

    rospy.spin()
