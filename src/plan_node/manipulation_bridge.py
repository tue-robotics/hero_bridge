#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from ed_tmc_collision_msgs.srv import (
    GetCollisionEnvironment,
    GetCollisionEnvironmentRequest,
    GetCollisionEnvironmentResponse,
)

from tf.transformations import quaternion_from_euler
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest
from tue_manipulation_msgs.msg import GraspPrecomputeAction, GraspPrecomputeGoal
from tmc_manipulation_msgs.msg import BaseMovementType, ArmManipulationErrorCodes

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry
from hsrb_interface.joint_group import JointGroup

import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs


class ManipulationBridge:
    def __init__(self):

        # robot
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')  # type: JointGroup
        self.whole_body.linear_weight = 100.0  # Do move as little as possible

        self.collision_avoidance = True

        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # server
        self.manipulation_as = actionlib.SimpleActionServer("arm_center/grasp_precompute",
                                                            GraspPrecomputeAction,
                                                            execute_cb=self.manipulation_as_exec,
                                                            auto_start=False)
        self.collisions_env_srv = rospy.ServiceProxy("ed/tmc_collision/get_collision_environment",
                                                     GetCollisionEnvironment)
        # Waiting for srv is useless as bridge is started much earlier than the service in ED
        self.manipulation_as.start()

    def manipulation_as_exec(self, action: GraspPrecomputeGoal):
        success = self.manipulation_cb(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.manipulation_as.set_succeeded()
        else:
            rospy.loginfo('Manipulation bridge: Failed')
            self.manipulation_as.set_aborted()

    def manipulation_cb(self, action: GraspPrecomputeGoal):
        """
        Here the grasp precompute action (TU/e) is translated to a PlanWithHandGoals (TMC) and send as goal to the robot
        :param action: the GraspPrecomputeAction type
        """
        success = True

        pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        pose = [action.goal.x, action.goal.y, action.goal.z], pose_quaternion

        ################################################################################################################
        # This piece of code is partially copied from Toyota software, it also uses the private functions (we're very
        # sorry). Setting base_movement_type.val allows for adapting the allowed movements during manipulation.

        ref_frame_id = settings.get_frame('base')

        ref_to_hand_poses = [pose]

        odom_to_ref_pose = self.whole_body._lookup_odom_to_ref(ref_frame_id)
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_hand_poses = []
        for ref_to_hand in ref_to_hand_poses:
            odom_to_hand = geometry.multiply_tuples(odom_to_ref, ref_to_hand)
            odom_to_hand_poses.append(geometry.tuples_to_pose(odom_to_hand))

        req = self.whole_body._generate_planning_request(PlanWithHandGoalsRequest)  # type: PlanWithHandGoalsRequest

        if self.collision_avoidance:
            env_req = GetCollisionEnvironmentRequest()
            env_req.frame_id = settings.get_frame('odom')
            env_resp = self.collisions_env_srv(env_req)  # type: GetCollisionEnvironmentResponse
            req.environment_before_planning = env_resp.collision_environment

        req.origin_to_hand_goals = odom_to_hand_poses
        req.ref_frame_id = self.whole_body._end_effector_frame
        req.base_movement_type.val = (
            BaseMovementType.PLANAR if self.collision_avoidance else BaseMovementType.ROTATION_Z
        )

        service_name = self.whole_body._setting['plan_with_hand_goals_service']
        plan_service = rospy.ServiceProxy(service_name, PlanWithHandGoals)
        start = rospy.Time.now()
        res = plan_service.call(req)
        end = rospy.Time.now()
        rospy.loginfo('Plan with hand goals: %f', (end - start).to_sec())
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            rospy.logerr(f'Fail to plan move_endpoint({res.error_code.val})')
            success = False
        else:
            res.base_solution.header.frame_id = settings.get_frame('odom')
            start = rospy.Time.now()
            constrained_traj = self.whole_body._constrain_trajectories(res.solution, res.base_solution)
            end = rospy.Time.now()
            rospy.loginfo('Constrain trajectories: %f', (end - start).to_sec())
            start = rospy.Time.now()
            self.whole_body._execute_trajectory(constrained_traj)
            end = rospy.Time.now()
            rospy.loginfo('Execute trajectory: %f', (end - start).to_sec())

        return success


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
