#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

from copy import deepcopy

import rospy

from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tmc_manipulation_msgs.srv import SafeJointChange


class HeadBridge(object):
    def __init__(self):
        self.data = JointState()
        self.data.position = [0.0, 0.0]
        self.prev_data = JointState()
        self.prev_data.position = [0.0, 0.0]

        # topics
        self.sub_refs = rospy.Subscriber("neck/references", JointState, self.callback)

        # clients
        self.client_trajectory = SimpleActionClient(
            "head_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        self.client_trajectory.wait_for_server()

        self.client_safe_joint_change = rospy.ServiceProxy("safe_pose_changer/change_joint", SafeJointChange)

    def callback(self, msg: JointState):
        self.data.position = msg.position

    def process_references(self):
        rospy.logdebug(f"current goal: {self.data.position}\nprevious goal: {self.prev_data.position}")
        if self.reference_changed(self.data):
            self.prev_data.position = self.data.position
            positions = [self.prev_data.position[0], self.prev_data.position[1]]  # self.data.position can be written
            positions[0] = min(max(positions[0], -3.84), 1.75)
            positions[1] = min(max(positions[1], -1.57), 0.52)

            # fill ROS message
            goal = FollowJointTrajectoryGoal()
            traj = JointTrajectory()
            traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
            p = JointTrajectoryPoint()
            p.positions = positions
            p.velocities = [0, 0]
            p.accelerations = [0, 0]
            p.effort = [0, 0]
            p.time_from_start = rospy.Duration(1.5)
            traj.points = [p]
            goal.trajectory = traj

            # send message to the action server
            self.client_trajectory.send_goal(goal)
            # wait for the action server to complete the order
            self.client_trajectory.wait_for_result()

            rospy.loginfo("Head bridge: Succeeded")

    def reference_changed(self, data, threshold=0.05):
        return not (
            abs(data.position[0] - self.prev_data.position[0]) < threshold
            and abs(data.position[1] - self.prev_data.position[1]) < threshold
        )


if __name__ == "__main__":
    rospy.init_node("head_ref_bridge")
    head_bridge = HeadBridge()
    r = rospy.Rate(25)

    try:
        while not rospy.is_shutdown():
            head_bridge.process_references()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
