#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These are bridged directly to the command of the arm controller.
# '''

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTrajectoryControllerState, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF


class JointTrajectoryNode(object):
    def __init__(self):

        # server
        self.srv_safe_joint_change = actionlib.SimpleActionServer('body/joint_trajectory_action',
                                                                  FollowJointTrajectoryAction,
                                                                  execute_cb=self.safe_joint_change_srv,
                                                                  auto_start=False)
        self.srv_safe_joint_change.start()

        self.pub_arm_ref = rospy.Publisher('arm_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.sub_arm_state = rospy.Subscriber('arm_trajectory_controller/state', JointTrajectoryControllerState,
                                              self.controller_state_callback)
        self._hero_urdf = URDF.from_parameter_server('robot_description')
        self._arm_joint_names = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                                'wrist_flex_joint', 'wrist_roll_joint']
        self.last_controller_state = JointTrajectoryPoint()
        self.current_goal_tolerance = []

    def safe_joint_change_srv(self, goal):
        """
        Here the follow joint trajectory action is translated to a JointTrajectory message type and published
        :param goal: the FollowJointTrajectoryAction type
        """
        try:
            self.validate_goal(goal.trajectory)
        except Exception as e:
            self.srv_safe_joint_change.set_aborted(
                result=FollowJointTrajectoryResult(
                    error_code=FollowJointTrajectoryResult.INVALID_GOAL,
                    error_string=str(e)
                ), text=str(e)
            )
            return

        last_timestamp = rospy.Time.now() + goal.trajectory.points[-1].time_from_start

        if not goal.goal_tolerance:
            self.current_goal_tolerance = [0.07]*5
        else:
            self.current_goal_tolerance = goal.goal_tolerance

        joint_trajectory = JointTrajectory()
        joint_trajectory.header = goal.trajectory.header

        joint_trajectory.joint_names = goal.trajectory.joint_names
        if len(joint_trajectory.joint_names) != 5:
            rospy.logwarn("You should include all joints")
            return

        joint_trajectory.points = goal.trajectory.points

        self.pub_arm_ref.publish(joint_trajectory)

        while not rospy.is_shutdown():
            if self.srv_safe_joint_change.is_preempt_requested():
                rospy.logwarn("Preempt requested")
                self.set_to_current_pose(goal.trajectory.header)
                self.srv_safe_joint_change.set_preempted()
                return
            if rospy.Time.now() > last_timestamp + rospy.Duration(10):
                msg = "das"
                self.srv_safe_joint_change.set_aborted(FollowJointTrajectoryResult(
                    error_code=-6,
                    error_string=msg
                ), msg)
                return
            if self.monitor_joint_state_within_tolerance(goal.trajectory.points[-1].positions):
                self.srv_safe_joint_change.set_succeeded()
                return
        self.srv_safe_joint_change.set_aborted()
        self.set_to_current_pose(goal.trajectory.header)
        return

    def validate_goal(self, trajectory):
        for point in trajectory.points:
            for count, name in enumerate(trajectory.joint_names):
                if name not in self._arm_joint_names:
                    raise ValueError("")
                big_enough = self._hero_urdf.joint_map[name].limit.lower < point.positions[count]
                small_enough = self._hero_urdf.joint_map[name].limit.upper > point.positions[count]
                if not big_enough or not small_enough:
                    raise ValueError("")
        rospy.loginfo("Validated goal")

        # check not empty

    def set_to_current_pose(self, header):
        stop_trajectory = JointTrajectory()
        stop_trajectory.header = header
        stop_trajectory.joint_names = self._arm_joint_names
        stop_trajectory.points.append(self.last_controller_state)
        self.pub_arm_ref.publish(stop_trajectory)

    def controller_state_callback(self, msg):
        self.last_controller_state = msg.actual

    def monitor_joint_state_within_tolerance(self, goal):
        for num, pose in enumerate(goal):
            if abs(pose - self.last_controller_state.positions[num]) > self.current_goal_tolerance[num]:
                # rospy.loginfo("pose is {}".format(pose))
                return False
        rospy.loginfo("Close enough to goal pose")
        return True


if __name__ == "__main__":
    rospy.init_node('joint_trajectory_action')
    joint_trajectory = JointTrajectoryNode()

    rospy.spin()


