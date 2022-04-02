#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''
from copy import deepcopy

import geometry_msgs
import moveit_commander
import moveit_msgs
import rospy
import actionlib
import sys
import tf2_ros
import tf2_geometry_msgs  # Do not remove! This contains vital transform functions.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf.transformations import quaternion_from_euler, quaternion_multiply

from std_srvs.srv import Trigger
from tue_manipulation_msgs.msg import GraspPrecomputeAction
from geometry_msgs.msg import PoseStamped


class ManipulationBridge(object):
    def __init__(self, group_name="whole_body_weighted"):
        # initialise moveit commander
        moveit_commander.roscpp_initialize(sys.argv + ['__ns:=/hero'])
        self.group_name = group_name
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name, wait_for_servers=0.0)
        self.move_group.set_planner_id('SBLkConfigDefault')
        self.scene = moveit_commander.PlanningSceneInterface(ns='/hero', synchronous=True)
        self.move_group.set_max_velocity_scaling_factor(0.7)
        self.move_group.set_max_acceleration_scaling_factor(0.7)

        self.moveit_scene_srv = rospy.ServiceProxy('ed/moveit_scene', Trigger)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.end_effector = self.move_group.get_end_effector_link()
        self.reference_frame = "odom"

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

    def set_workspace(self):
        base_pose = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
        self.move_group.set_workspace([base_pose.transform.translation.x - 1,
                                       base_pose.transform.translation.y - 1,
                                       0.0,
                                       base_pose.transform.translation.x + 1,
                                       base_pose.transform.translation.y + 1,
                                       2.0])

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
        except tf2_ros.TransformException as e: # TODO proper import
            rospy.logerr("Could not transform the goal pose to odom frame {}".format(e))
            return False

        # set the workspace
        try:
            self.set_workspace()
        except tf2_ros.TransformException as e:
            rospy.logerr("Could not get base pose in base_link frame: {}".format(e))
            return False

        grasps = self.make_grasps(action.goal.allowed_touch_objects,
                                  (0.707, 0.0, 0.707, 0.0),
                                  quality=lambda x, y, z, roll, pitch, yaw: 1 - abs(pitch),  # noqa
                                  x=[-0.07],
                                  pitch=[-0.2, -0.1, 0, 0.1, 0.2])
        
        result = self.move_group.pick(action.goal.allowed_touch_objects, grasps)
        # execute motion
        # self.move_group.set_pose_target(transformed_goal)
        # plan = self.move_group.go(wait=True)
        # rospy.logwarn("plan: {}".format(plan))
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()
        return True

    def make_gripper_posture(self, pos, effort=0.0):
        t = JointTrajectory()
        t.joint_names = ["hand_motor_joint"]
        tp = JointTrajectoryPoint()
        tp.positions = [pos]
        tp.effort = [effort]
        tp.time_from_start = rospy.Duration(2.0)
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, vector, frame=None):
        g = moveit_msgs.msg.GripperTranslation()
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        if frame is None:
            g.direction.header.frame_id = self.end_effector
        else:
            g.direction.header.frame_id = frame
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self, target, init,
                    quality=None,
                    x=[0], y=[0], z=[0],
                    roll=[0], pitch=[0], yaw=[0]):
        poses = self.scene.get_object_poses([target])
        pose = poses[target]
        g = moveit_msgs.msg.Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(0.8)
        g.grasp_posture = self.make_gripper_posture(0.2, -0.01)
        g.pre_grasp_approach \
            = self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0])
        g.post_grasp_retreat \
            = self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0],
                                            "base_footprint")
        grasps = []
        for ix in x:
            for iy in y:
                for iz in z:
                    for iroll in roll:
                        for ipitch in pitch:
                            for iyaw in yaw:
                                x = pose.position.x + ix
                                y = pose.position.y + iy
                                z = pose.position.z + iz
                                g.grasp_pose = self.make_pose(init,
                                                              x, y, z,
                                                              iroll,
                                                              ipitch,
                                                              iyaw)
            g.id = str(len(grasps))
            g.allowed_touch_objects = ["target1"]
            g.max_contact_force = 0
            if quality is None:
                g.grasp_quality = 1.0
            else:
                g.grasp_quality = quality(ix, iy, iz, iroll, ipitch, iyaw)
            grasps.append(deepcopy(g))
        return grasps

    def make_pose(self, init, x, y, z, roll, pitch, yaw):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.reference_frame
        q = quaternion_from_euler(roll, pitch, yaw)
        q = quaternion_multiply(init, q)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
