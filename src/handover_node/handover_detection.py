#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math

from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped

import sys

reload(sys)
sys.setdefaultencoding('utf8')


class HandoverDetector(object):
    def __init__(self, side):
        self.force_threshold = 5
        self.timeout = 10

        self.prev_force_x = 0
        self.prev_force_y = 0
        self.prev_force_z = 0

        self.force_x = 0
        self.force_y = 0
        self.force_z = 0

        # server
        self.sub_r2h = rospy.Subscriber("/hero/handoverdetector_" + side + "/toggle_robot2human",
                                        Bool, self.detect_handover)
        self.sub_h2r = rospy.Subscriber("/hero/handoverdetector_" + side + "/toggle_human2robot",
                                        Bool, self.detect_handover)
        self.sub_wrist_wrench = rospy.Subscriber("/hsrb/wrist_wrench/raw", WrenchStamped, self.update_forces)

        self.pub_result = rospy.Publisher("/hero/handoverdetector_" + side + "/result", Bool, queue_size=1)

    def detect_handover(self, data):
        if data.data:
            success = False

            self.prev_force_x = self.force_x
            self.prev_force_y = self.force_y
            self.prev_force_z = self.force_z

            start = rospy.Time.now()
            while rospy.Time.now() - start < rospy.Duration(self.timeout):
                if self.handover():
		    success = True
                    self.pub_result.publish(Bool(success))
		    break
                else:
                    rospy.sleep(0.1)
	else:
            rospy.loginfo("toggle sent which is not True")
	

    def handover(self):
        dx = self.force_x - self.prev_force_x
        dy = self.force_y - self.prev_force_y
        dz = self.force_z - self.prev_force_z

        diff = math.sqrt( pow(dx,2) + pow(dy,2) + pow(dz,2))
        return diff > self.force_threshold

    def update_forces(self, wrenchstamped):
        self.force_x = wrenchstamped.wrench.force.x
        self.force_y = wrenchstamped.wrench.force.y
        self.force_z = wrenchstamped.wrench.force.z




if __name__ == "__main__":
    rospy.init_node('hand_over_detector')
    handover_detection = HandoverDetector("left")
    handover_detection = HandoverDetector("right")

    rospy.spin()
