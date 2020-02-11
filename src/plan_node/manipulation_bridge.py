#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''
import math

import rospy
import actionlib
import geometry_msgs.msg
import hero_msgs.srv
import PyKDL as kdl

from tf.transformations import quaternion_from_euler, quaternion_multiply
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest
from tue_manipulation_msgs.msg import GraspPrecomputeAction, GraspPrecomputeGoal
from tmc_manipulation_msgs.msg import BaseMovementType, ArmManipulationErrorCodes

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry, trajectory


class ManipulationBridge(object):
    def __init__(self):

        # robot
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

        # server
        self.srv_manipulation_left = actionlib.SimpleActionServer('left_arm/grasp_precompute',
                                                                  GraspPrecomputeAction,
                                                                  execute_cb=self.manipulation_srv_left,
                                                                  auto_start=False)
        self.srv_manipulation_right = actionlib.SimpleActionServer('right_arm/grasp_precompute',
                                                                   GraspPrecomputeAction,
                                                                   execute_cb=self.manipulation_srv_right,
                                                                   auto_start=False)
        self.srv_manipulation_left.start()
        self.srv_manipulation_right.start()

        # Service interface to compute the base pose corresponding to a grasp pose
        rospy.Service('compute_base_pose', hero_msgs.srv.GetBasePose, self.compute_base_pose)

    def manipulation_srv_left(self, action):
        success = self.manipulation_srv(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation_left.set_succeeded()

    def manipulation_srv_right(self, action):
        success = self.manipulation_srv(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation_right.set_succeeded()

    def _convert_frames(self, input_pose, frame_id):
        # type: (kdl.Frame, str) -> geometry_msgs.msg.Pose
        """
        Converts an end-effector pose (TU/e definition, x-axis pointing forward) w.r.t. the provided frame_id to the
        Toyota HSRB end-effector pose (Toyota definition, z-axis pointing forward) w.r.t. the HSRB odom frame so that
        it can be used directly in planning requests.

        :param input_pose: kdl Frame with (desired) end-effector pose
        :param frame_id: frame w.r.t. which the pose is defined
        :return: converted frame as geometry_msgs.Pose
        """
        # In much of the TU/e code, it is 'assumed' that a gripper pointing forward with roll and pitch level
        # has the x-axis pointing forward and z-axis pointing upward.
        # However, the frame used for planning in the HSR config is the hand_palm_link, with x-axis pointing
        # upwards and z-axis poinint forwards. Therefore, we have to convert this.
        kdl_pose = kdl.Frame(input_pose)
        kdl_pose.M.DoRotY(-0.5 * math.pi)
        kdl_pose.M.DoRotX(math.pi)
        ref_to_hand = [kdl_pose.p.x(), kdl_pose.p.y(), kdl_pose.p.z()], list(kdl_pose.M.GetQuaternion())

        # Compute transformation between odom and reference
        ref_frame_id = frame_id.lstrip("/")  # Typically: base_link. Make sure to remove leading /
        odom_to_ref_pose = self.whole_body._lookup_odom_to_ref(ref_frame_id)
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)

        # Compute the desired transformation from odom to hand
        odom_to_hand_pose = geometry.tuples_to_pose(geometry.multiply_tuples(odom_to_ref, ref_to_hand))
        return odom_to_hand_pose

    def manipulation_srv(self, action):
        # type: (GraspPrecomputeGoal) -> bool
        """
        Here the grasp precompute action (TU/e) is translated to a PlanWithHandGoals (TMC) and send as goal to the robot
        :param action: the GraspPrecomputeAction type
        """
        success = True

        # pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        # static_quaternion = quaternion_from_euler(3.14159265359, -1.57079632679, 0)
        # final_quaternion = quaternion_multiply(pose_quaternion, static_quaternion)
        # pose = [action.goal.x, action.goal.y, action.goal.z], final_quaternion
        #
        # ################################################################################################################
        # # This piece of code is partially copied from Toyota software, it also uses the private functions (we're very
        # # sorry). Setting base_movement_type.val allows for adapting the allowed movements during manipulation.
        #
        # ref_frame_id = settings.get_frame('base')
        #
        # ref_to_hand_poses = [pose]
        #
        # odom_to_ref_pose = self.whole_body._lookup_odom_to_ref(ref_frame_id)
        # odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        # odom_to_hand_poses = []
        # for ref_to_hand in ref_to_hand_poses:
        #     odom_to_hand = geometry.multiply_tuples(odom_to_ref, ref_to_hand)
        #     odom_to_hand_poses.append(geometry.tuples_to_pose(odom_to_hand))
        input_pose_kdl = kdl.Frame(kdl.Rotation.RPY(action.goal.roll, action.goal.pitch, action.goal.yaw),
                                   kdl.Vector(action.goal.x, action.goal.y, action.goal.z)
                                   )
        odom_to_hand_pose = self._convert_frames(input_pose_kdl, action.goal.header.frame_id)

        req = self.whole_body._generate_planning_request(PlanWithHandGoalsRequest)
        # req.origin_to_hand_goals = odom_to_hand_poses
        req.origin_to_hand_goals = [odom_to_hand_pose]
        req.ref_frame_id = self.whole_body._end_effector_frame
        req.base_movement_type.val = BaseMovementType.ROTATION_Z

        service_name = self.whole_body._setting['plan_with_hand_goals_service']
        plan_service = rospy.ServiceProxy(service_name, PlanWithHandGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            rospy.logerr('Fail to plan move_endpoint')
            success = False
        else:
            res.base_solution.header.frame_id = settings.get_frame('odom')
            constrained_traj = self.whole_body._constrain_trajectories(res.solution, res.base_solution)
            self.whole_body._execute_trajectory(constrained_traj)

        return success

    def compute_base_pose(self, request):
        # type: (hero_msgs.srv.GetBasePoseRequest) -> hero_msgs.srv.GetBasePoseResponse
        """
        Computes the base pose corresponding to a desired grasp pose

        :param request: Request (from ROS service call)
        :return: corresponding Response
        """
        raise rospy.ServiceException("GetBasePose not yet implemented")


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')

    # hsrb_interface can't handle prefix added by other toyota software
    settings._SETTINGS['frame']['odom']['frame_id'] = 'hero/odom'
    settings._SETTINGS['frame']['base']['frame_id'] = 'hero/base_footprint'
    trajectory._BASE_TRAJECTORY_ORIGIN = 'hero/odom'

    manipulation_bridge = ManipulationBridge()

    rospy.spin()
