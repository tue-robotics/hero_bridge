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
import tf2_geometry_msgs  # Do not remove! This contains vital transform functions.

from tf.transformations import quaternion_from_euler

from std_srvs.srv import Trigger
from tue_manipulation_msgs.msg import GraspPrecomputeAction
from geometry_msgs.msg import PoseStamped


class ManipulationBridge(object):
    def __init__(self, group_name="whole_body_weighted"):
        # initialise moveit commander
        moveit_commander.roscpp_initialize(sys.argv + ['__ns:=/hero'])
        self.group_name = group_name
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name, wait_for_servers=10.0)
        self.scene = moveit_commander.PlanningSceneInterface(ns='/hero', synchronous=True)
        self.move_group.set_max_velocity_scaling_factor(0.7)
        self.move_group.set_max_acceleration_scaling_factor(0.7)

        self.moveit_scene_srv = rospy.ServiceProxy('ed/moveit_scene', Trigger)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # server
        self.srv_manipulation = actionlib.SimpleActionServer('arm_center/grasp_precompute',
                                                             GraspPrecomputeAction,
                                                             execute_cb=self.manipulation_srv_inst,
                                                             auto_start=False)
        self.srv_manipulation.start()

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
        # call planning scene service
        try:
            self.moveit_scene_srv.call()
        except rospy.ServiceException as e:
            rospy.logerr('The service call to ed/moveit_scene breaks, check that this service exists!')

        # fill pose message
        pose_goal = PoseStamped()
        pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        pose_goal.pose.position.x = action.goal.x
        pose_goal.pose.position.y = action.goal.y
        pose_goal.pose.position.z = action.goal.z
        pose_goal.pose.orientation.x = pose_quaternion[0]
        pose_goal.pose.orientation.y = pose_quaternion[1]
        pose_goal.pose.orientation.z = pose_quaternion[2]
        pose_goal.pose.orientation.w = pose_quaternion[3]
        pose_goal.header.frame_id = action.goal.header.frame_id
        pose_goal.header.stamp = rospy.Time.now()

        # transform to odom frame
        try:
            transformed_goal = self.tf_buffer.transform(pose_goal, "odom", timeout=rospy.Duration(1))
            rospy.loginfo("transformed goal in odom frame: {}".format(transformed_goal))
        except tf2_ros.TransformException as e:
            rospy.logerr("Could not transform the goal pose to odom frame {}".format(e))
            return False

        # set the workspace
        try:
            base_pose = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            rospy.loginfo("base pose: {}".format(base_pose))
            self.move_group.set_workspace([base_pose.transform.translation.x - 1,
                                          base_pose.transform.translation.y - 1,
                                          0.0,
                                          base_pose.transform.translation.x + 1,
                                          base_pose.transform.translation.y + 1,
                                          2.0])

        except tf2_ros.TransformException as e:
            rospy.logerr("Could not get base pose in base_link frame: {}".format(e))
            return False

        # execute motion
        self.move_group.set_pose_target(transformed_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return True


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
