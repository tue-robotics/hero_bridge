#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a reference topic
# These will be bridged to a command controller
# '''

import rospy

# from tmc_manipulation_msgs.srv import SafeJointChange
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadBridge(object):
    def __init__(self):
        self.data = JointState()
        self.data.position = [0.0, 0.0]
        self.prev_data = JointState()
        self.prev_data.position = [0.0, 0.0]

        # topics
        self.sub_neck_ref = rospy.Subscriber('neck/references', JointState, self.callback)
        self.pub_neck_ref = rospy.Publisher('head_trajectory_controller/command', JointTrajectory, queue_size=1)

    def callback(self, data):
        self.data.position = data.position

    def process_references(self):
        if self.reference_changed(self.data):
            self.prev_data.position = self.data.position

            jointChange = JointTrajectory()

            position = [self.data.position[0], self.data.position[1]]
            position[0] = min(max(position[0], -3.84), 1.75)
            position[1] = min(max(position[1], -1.57), 0.52)

            goal_point = JointTrajectoryPoint()
            goal_point.positions = position
            goal_point.velocities = [0, 0]
            goal_point.accelerations = [0, 0]
            goal_point.effort = [0, 0]
            goal_point.time_from_start = rospy.Time.now()

            jointChange.header = self.data.header
            jointChange.joint_names = ['head_pan_joint', 'head_tilt_joint']
            jointChange.points.append(goal_point)
            self.pub_neck_ref.publish(jointChange)

            rospy.loginfo('Head bridge: Succeeded')

    def reference_changed(self, data):
        return not (abs(data.position[0] - self.prev_data.position[0]) < 0.05 and
                    abs(data.position[1] - self.prev_data.position[1]) < 0.05)


if __name__ == "__main__":
    rospy.init_node('head_ref_bridge')
    head_bridge = HeadBridge()
    r = rospy.Rate(25)

    try:
        while not rospy.is_shutdown():
            head_bridge.process_references()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
