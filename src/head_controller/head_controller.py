#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy

from tmc_manipulation_msgs.srv import SafeJointChange
from sensor_msgs.msg import JointState


class HeadBridge(object):
    def __init__(self):
        self.data = JointState()
        self.data.position = [0.0, 0.0]
        self.prev_data = JointState()
        self.prev_data.position = [0.0, 0.0]

        # topics
        self.sub_speak = rospy.Subscriber("neck/references", JointState, self.callback)

        # clients
        self.client_safe_joint_change = rospy.ServiceProxy('safe_pose_changer/change_joint', SafeJointChange)

    def callback(self, data):
        self.data.position = data.position

    def process_references(self):
        # rospy.loginfo("current:  {}".format(self.data.position))
        # rospy.loginfo("previous: {}".format(self.prev_data.position))
        if self.reference_changed(self.data):
            self.prev_data.position = self.data.position
            safeJointChange = JointState()
            safeJointChange.header.seq = 0
            safeJointChange.header.stamp.secs = 0
            safeJointChange.header.stamp.nsecs = 0
            safeJointChange.header.frame_id = ''

            safeJointChange.name = ['head_pan_joint', 'head_tilt_joint']
            position = [self.data.position[0], self.data.position[1]]

            position[0] = min(max(position[0], -3.84), 1.75)
            position[1] = min(max(position[1], -1.57), 0.52)
            safeJointChange.position = position
            safeJointChange.velocity = [0, 0]
            safeJointChange.effort = [0, 0]
            self.client_safe_joint_change.wait_for_service()
            self.client_safe_joint_change(safeJointChange)

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
