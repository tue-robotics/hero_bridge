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
from control_msgs.msg import FollowJointTrajectoryAction
import sys

reload(sys)
sys.setdefaultencoding('utf8')

class JointTrajectory(object):
    def __init__(self):

        # server
        self.srv_safe_joint_change = actionlib.SimpleActionServer('/hero/body/joint_trajectory_action',
                                                                  FollowJointTrajectoryAction,
                                                                  execute_cb=self.safe_joint_change_srv,
                                                                  auto_start=False)
        self.srv_safe_joint_change.start()

        # clients
        self.client_safe_joint_change = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)

    def safe_joint_change_srv(self, goal):
        """
        Here the follow joint trajectory action is translated to a SafeJointChange message type
        :param goal: the FollowJointTrajectoryAction type
        :return: the SafeJointChange message type
        """

        # helper variables
        # r = rospy.Rate(1)
        success = True
           
        # append the seeds for the fibonacci sequence
        safeJointChange = JointState()
        safeJointChange.header.seq = 0 
        safeJointChange.header.stamp.secs = 0
        safeJointChange.header.stamp.nsecs = 0
        safeJointChange.header.frame_id = ''

        safeJointChange.name = goal.trajectory.joint_names
        safeJointChange.position = [0 for name in safeJointChange.name]
        safeJointChange.velocity = [0 for name in safeJointChange.name]
        safeJointChange.effort = [0 for name in safeJointChange.name]
           
        # start executing the action
        for point in goal.trajectory.points:
        # check that preempt has not been requested by the client
            if self.srv_safe_joint_change.is_preempt_requested():
                rospy.loginfo('Trajectory bridge: Preempted')
                self.srv_safe_joint_change.set_preempted()
                success = False
                break
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            #r.sleep()
            safeJointChange.position = point.positions
            self.client_safe_joint_change.wait_for_service()
            success=self.client_safe_joint_change(safeJointChange)
            if not success:
                rospy.logerr('Trajectory bridge failed to change pose')
                break
            self.client_safe_joint_change.wait_for_service()
            rospy.loginfo('position [{}]'.format(point.positions))            
 
        if success:
#            rospy.loginfo('Trajectory bridge: Succeeded')
            self.srv_safe_joint_change.set_succeeded()
       

if __name__ == "__main__":
    rospy.init_node('joint_trajectory_action')
    joint_trajectory = JointTrajectory()

    rospy.spin()
