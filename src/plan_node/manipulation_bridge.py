#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from tf.transformations import quaternion_from_euler, quaternion_multiply
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest
from tue_manipulation_msgs.msg import GraspPrecomputeAction
from tmc_manipulation_msgs.msg import BaseMovementType, ArmManipulationErrorCodes

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry, trajectory


class ManipulationBridge(object):
    def __init__(self):

        # robot
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

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

    def manipulation_srv(self, action):
        """
        Here the grasp precompute action (TU/e) is translated to a PlanWithHandGoals (TMC) and send as goal to the robot
        :param action: the GraspPrecomputeAction type
        """
        success = True

        pose_quaternion = quaternion_from_euler(action.goal.roll, action.goal.pitch, action.goal.yaw)
        static_quaternion = quaternion_from_euler(3.14159265359, -1.57079632679, 0)
        final_quaternion = quaternion_multiply(pose_quaternion, static_quaternion)
        pose = [action.goal.x, action.goal.y, action.goal.z], final_quaternion

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

        req = self.whole_body._generate_planning_request(PlanWithHandGoalsRequest)
        req.origin_to_hand_goals = odom_to_hand_poses
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


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')

    # hsrb_interface can't handle prefix added by other toyota software
    settings._SETTINGS['frame']['odom']['frame_id'] = 'hero/odom'
    settings._SETTINGS['frame']['base']['frame_id'] = 'hero/base_footprint'
    trajectory._BASE_TRAJECTORY_ORIGIN = 'hero/odom'

    manipulation_bridge = ManipulationBridge()

    rospy.spin()
