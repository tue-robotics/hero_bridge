#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from tmc_manipulation_msgs.srv import SafeJointChange
from sensor_msgs.msg import JointState


class JointTrajectory(object):
    def __init__(self):
        self.prev_data = JointState()
        self.prev_data.position = [0.0, 1.0]
        # topics
        self.sub_speak = rospy.Subscriber("/hero/neck/references", JointState, self.callback())

        # clients
        self.client_safe_joint_change = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)

    def callback(self, data):
        """
        Here the follow joint trajectory action is translated to a SafeJointChange message type
        :param data: the FollowJointTrajectoryAction type
        :return: the SafeJointChange message type
        """
        # helper variables
        success = True

        if not (data.position[0] is self.prev_data.position[0] and data.position[1] is self.prev_data.position[1]):
            safeJointChange = JointState()
            safeJointChange.header.seq = 0
            safeJointChange.header.stamp.secs = 0
            safeJointChange.header.stamp.nsecs = 0
            safeJointChange.header.frame_id = ''

            safeJointChange.name = ['head_pan_joint', 'head_tilt_joint']
            safeJointChange.position = data.position
            safeJointChange.velocity = [0, 0]
            safeJointChange.effort = [0, 0]

            self.client_safe_joint_change(safeJointChange)

            self.prev_data.position = data.position
             
            if success:
                rospy.loginfo('Trajectory bridge: Succeeded')
                self.srv_safe_joint_change.set_succeeded()
       

if __name__ == "__main__":
    rospy.init_node('head_ref_bridge')
    joint_trajectory = JointTrajectory()

    rospy.spin()
