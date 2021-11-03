#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import TransformStamped
from math import pi
import rospy
import tf_conversions
import tf2_ros


class Ghost(object):
    def __init__(self):
        self.prefix = "hero"
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.transforms = []

    def create_ghosts(self):
        self.add_prefix("base_link")
        self.remap_frame("head_tilt_link", "neck_tilt")
        self.remap_frame("head_rgbd_sensor_gazebo_frame", "top_kinect/openni_camera")
        self.remap_frame("base_range_sensor_link", self.prefix + "/base_laser")

        self.add_transform((0, 0, 0), tf_conversions.transformations.quaternion_from_euler(pi, -pi/2, 0),
                           rospy.Time.now(), "head_mount", "torso_lift_link")

    def add_prefix(self, frame):
        ghost_frame = self.prefix + "/" + frame
        self.remap_frame(frame, ghost_frame)

    def remap_frame(self, frame, ghost_frame):
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.add_transform((0, 0, 0), quat, rospy.Time.now(), ghost_frame, frame)

    def add_transform(self, translation, rotation, time, child, parent):
        transform = TransformStamped()

        transform.header.stamp = time
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self.transforms.append(transform)

    def publish_transforms(self):
        self.broadcaster.sendTransform(self.transforms)


if __name__ == "__main__":
    rospy.init_node('tf_ghost_publisher')

    ghost = Ghost()
    ghost.create_ghosts()
    ghost.publish_transforms()
    rospy.spin()
