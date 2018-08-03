#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from tmc_manipulation_msgs.msg import SafeJointChange
from control_msgs.msg import FollowJointTrajectoryAction
import sys

reload(sys)
sys.setdefaultencoding('utf8')

class JointTrajectory(object):
    def __init__(self):

        # topics
        # self.pub_speak = rospy.Publisher("/talk_request", Voice, queue_size=10)

        # server
        self.srv_safe_joint_change = actionlib.SimpleActionServer('hero/body/joint_trajectory_action',
                                                                  FollowJointTrajectoryAction,
                                                                  self.safe_joint_change_srv())

        # clients
        self.client_safe_joint_change = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)

    def do_joint_trajectory(self, out):
        rospy.loginfo('Joint Goal: Toyota safe pose changer, through bridge node.')

        self.client_safe_joint_change(out)

    def safe_joint_change_srv(self, action):
        '''
        Here the follow joint trajectory action is translated to a SafeJointChange message type
        :param action: the FollowJointTrajectoryAction type
        :return: the SafeJointChange message type
        '''
        out = SafeJointChange()
        out.name = action.action_goal.trajectory.joint_names
        out.position = action.action_goal.trajectory.points.positions
        self.do_joint_trajectory(out)
        return ""

if __name__ == "__main__":
    rospy.init_node('joint_trajectory_action')
    joint_trajectory = JointTrajectory()

    rospy.spin()
