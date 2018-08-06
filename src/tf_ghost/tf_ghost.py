#!/usr/bin/env python
# -*- coding: utf-8 -*-

#'''
#This node listens to a service call and a topic for text to speech
#requests. These will be processed by the festival or the philips tts module.
#'''

import rospy
import tf

from std_msgs.msg import String


class bcolors:
    OKBLUE = '\033[94m'
    ENDC = '\033[0m'


class Ghost(object):
    def __init__(self):
        # topics
        self.prefix = "hero"
        rate = rospy.Rate(10)
        self.broadcaster = tf.TransformBroadcaster()

	while not rospy.is_shutdown():
           self.create_ghosts()
           rate.sleep()

    def create_ghosts(self):
        self.add_prefix("base_link")

        self.remap_frame("torso_lift_link", self.prefix + "/head_mount")
        self.remap_frame("head_tilt_link", self.prefix + "/neck_tilt")
        self.remap_frame("head_rgbd_sensor_link", self.prefix + "/top_kinect/openni_camera")

    def add_prefix(self, frame):
        ghost_frame = self.prefix + "/" + frame
        self.remap_frame(frame,ghost_frame)

    def remap_frame(self, frame, ghost_frame):
        self.broadcaster.sendTransform((0,0,0),(1,0,0,0),rospy.Time.now(), ghost_frame, frame)

if __name__ == "__main__":
    rospy.init_node('tf_ghost_publisher')
    try:
       ghost = Ghost()
    except rospy.ROSInterruptException:
       pass
