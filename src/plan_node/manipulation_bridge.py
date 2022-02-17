#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''
import moveit_commander
import rospy
import actionlib
import sys
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import quaternion_from_euler

from std_srvs.srv import Trigger
from tue_manipulation_msgs.msg import GraspPrecomputeAction
from geometry_msgs.msg import PoseStamped, Point



class ManipulationBridge(object):
    def __init__(self):
        # server
        self.srv_manipulation = actionlib.SimpleActionServer('arm_center/grasp_precompute',
                                                             GraspPrecomputeAction,
                                                             execute_cb=self.manipulation_srv_inst,
                                                             auto_start=False)
        self.srv_manipulation.start()
        moveit_commander.roscpp_initialize(sys.argv + ['__ns:=/hero'])
        #TODO: make changeable?
        self.group_name = "whole_body"

        self.robot = moveit_commander.RobotCommander()
        #TODO: dynamic hero
        self.scene = moveit_commander.PlanningSceneInterface(ns='/hero', synchronous=True)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        #TODO Check
        self.move_group.set_workspace([0, 0, 0, 0])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def manipulation_srv_inst(self, action):
        success = self.manipulation_srv(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation.set_succeeded()
        else:
            rospy.loginfo('Manipulation bridge: failed')
            self.srv_manipulation.set_aborted()

    def manipulation_srv(self, action):
        """
        Here the grasp precompute action (TU/e) is sent to the robot through moveit
        :param action: the GraspPrecomputeAction type
        """
        success = True
        #todo move to init, put it in a timeout
        rospy.wait_for_service('ed/moveit_scene')
        try:
            moveit_call = rospy.ServiceProxy('ed/moveit_scene', Trigger)
            moveit_call()
        except rospy.ServiceException as e:
            rospy.logerr('The service call to ed/moveit_scene breaks, check that this service exists!')
            success = False

        pose_goal = PoseStamped()
        point = Point()
        pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        point.x = action.goal.x
        point.y = action.goal.y
        point.z = action.goal.z
        pose_goal.pose.position = point
        pose_goal.pose.orientation.x = pose_quaternion[0]
        pose_goal.pose.orientation.y = pose_quaternion[1]
        pose_goal.pose.orientation.z = pose_quaternion[2]
        pose_goal.pose.orientation.w = pose_quaternion[3]
        #todo: check whether this should be base_link or odom
        pose_goal.header.frame_id = "base_link"
        pose_goal.header.stamp = rospy.Time.now()

        try:
            transform = self.tf_buffer.transform(pose_goal, "odom", timeout=rospy.Duration(1))
            rospy.logerr(transform)
        except Exception as e:
            rospy.logerr(e)

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        # todo: logging/debugging stuff
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return success


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
