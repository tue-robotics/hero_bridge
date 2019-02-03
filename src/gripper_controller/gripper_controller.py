#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from tmc_manipulation_msgs.srv import SafeJointChange
from sensor_msgs.msg import JointState
from tue_manipulation_msgs.msg import GripperCommandAction

from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal


class JointTrajectory(object):
    def __init__(self):
        self.success = True

        # server
        self.srv_safe_joint_change_left = actionlib.SimpleActionServer('/hero/left_arm/gripper/action',
                                                                       GripperCommandAction,
                                                                       execute_cb=self.gripper_left,
                                                                       auto_start=False)
        self.srv_safe_joint_change_right = actionlib.SimpleActionServer('/hero/right_arm/gripper/action',
                                                                        GripperCommandAction,
                                                                        execute_cb=self.gripper_right,
                                                                        auto_start=False)
        self.srv_safe_joint_change_left.start()
        self.srv_safe_joint_change_right.start()

        # clients
        self.client_safe_joint_change = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)
        self._grasp_client = actionlib.SimpleActionClient('gripper_controller/grasp', GripperApplyEffortAction)

    def gripper_left(self, goal):
        self.success = self.safe_joint_change_srv(goal)
        if self.success:
            self.srv_safe_joint_change_left.set_succeeded()

    def gripper_right(self, goal):
        self.success = self.safe_joint_change_srv(goal)
        if self.success:
            self.srv_safe_joint_change_right.set_succeeded()

    def safe_joint_change_srv(self, goal):
        """
        Here the follow joint trajectory action is translated to a SafeJointChange message type
        :param goal: the FollowJointTrajectoryAction type
        :return: the SafeJointChange message type
        """
           
        if goal.command.direction is goal.command.OPEN:
            return self._open_gripper()
        elif goal.command.direction is goal.command.CLOSE:
            return self._close_gripper()
        else:
            rospy.loginfo('Trajectory bridge: received gripper goal that is nor OPEN or CLOSE')
            return False
       
    def _open_gripper(self):
        safeJointChange = JointState()
        safeJointChange.header.seq = 0
        safeJointChange.header.stamp.secs = 0
        safeJointChange.header.stamp.nsecs = 0
        safeJointChange.header.frame_id = ''

        safeJointChange.name = ['hand_motor_joint']
        safeJointChange.position = [0]
        safeJointChange.velocity = [0]
        safeJointChange.effort = [0]

        safeJointChange.position = [1.0]  # this equals the gripper being open

        self.client_safe_joint_change.wait_for_service()
        out = self.client_safe_joint_change(safeJointChange)
        self.client_safe_joint_change.wait_for_service()
        return out

    def _close_gripper(self):
        effort = 0.1  # 0.1 Newton is enough for nearly everything
        _HAND_MOMENT_ARM_LENGTH = 0.07

        goal = GripperApplyEffortGoal()
        goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        self._grasp_client.send_goal(goal)

        timeout = rospy.Duration(5)
        if self._grasp_client.wait_for_result(timeout):
            self._grasp_client.get_result()
            state = self._grasp_client.get_state()
            if state != actionlib.GoalStatus.SUCCEEDED:
                return False
        else:
            self._grasp_client.cancel_goal()
            return False

        return True


if __name__ == "__main__":
    rospy.init_node('gripper_bridge')
    joint_trajectory = JointTrajectory()

    rospy.spin()
