# !/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from tf.transformations import quaternion_from_euler
from tmc_planning_msgs.srv import PlanWithHandGoals
from tue_manipulation_msgs.msg import GraspPrecomputeAction

# Preparation to use robot functions
from hsrb_interface import Robot
robot = Robot()
whole_body = robot.try_get('whole_body')

class ManipulationBridge(object):
    def __init__(self):

        # server
        self.srv_manipulation = actionlib.SimpleActionServer('/hero/left_arm/grasp_precompute', GraspPrecomputeAction,
                                                             execute_cb=self.manipulation_srv, auto_start=False)
        self.srv_manipulation.start()

        # clients
        self.client_manipulation = rospy.ServiceProxy('/plan_with_hand_goals', PlanWithHandGoals)

    def manipulation_srv(self, action):
        """
        Here the follow joint trajectory action is translated to a GraspPrecomputeAction message type
        :param action: the FollowJointTrajectoryAction type
        :return: the SafeJointChange message type
        """
        # helper variables
        # r = rospy.Rate(1)
        success = True

        # append the seeds for the fibonacci sequence
        point_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        point = [action.goal.x, action.goal.y, action.goal.z], point_quaternion

        try:
            whole_body.move_end_effector_pose(point)
        except:
            rospy.logerr('Fail to reach end effector goal')

        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation.set_succeeded()


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
