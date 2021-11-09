#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from moveit_msgs.msg import CollisionObject, PlanningSceneWorld
from shape_msgs.msg import Mesh, MeshTriangle
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_manipulation_msgs.msg import CollisionObject as CollisionObjectTMC, CollisionEnvironment
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest, PlanWithHandGoalsResponse
from tue_manipulation_msgs.msg import GraspPrecomputeAction
from tmc_manipulation_msgs.msg import BaseMovementType, ArmManipulationErrorCodes

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry


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

        self.srv_proxy_moveit_scene = rospy.ServiceProxy('ed/moveit_scene', Trigger)
        self.sub_moveit_scene = rospy.Subscriber('planning_scene_world', PlanningSceneWorld, self.planning_scene_cb,
                                                 queue_size=1)
        self._collision_environment = None

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

        req = self.whole_body._generate_planning_request(PlanWithHandGoalsRequest)
        req.origin_to_hand_goals = odom_to_hand_poses
        req.ref_frame_id = self.whole_body._end_effector_frame
        req.base_movement_type.val = BaseMovementType.ROTATION_Z
        if self.srv_proxy_moveit_scene() and self._collision_environment:
            req.environment_before_planning = self._collision_environment

        rospy.loginfo(req.environment_before_planning)
        service_name = self.whole_body._setting['plan_with_hand_goals_service']
        plan_service = rospy.ServiceProxy(service_name, PlanWithHandGoals)
        res = plan_service.call(req)  # type: PlanWithHandGoalsResponse
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            rospy.logerr('Fail to plan move_endpoint')
            success = False
        else:
            rospy.loginfo(res.environment_after_planning)
            res.base_solution.header.frame_id = settings.get_frame('odom')
            constrained_traj = self.whole_body._constrain_trajectories(res.solution, res.base_solution)
            self.whole_body._execute_trajectory(constrained_traj)

        return success

    def planning_scene_cb(self, msg: PlanningSceneWorld) -> None:
        self._collision_environment = None
        tmc_world = CollisionEnvironment()
        tmc_world.header.frame_id = 'map'
        tmc_world.header.stamp = rospy.Time.now()
        object_id = 1
        for coll_object in msg.collision_objects:  # type: CollisionObject
            tmc_coll_object = CollisionObjectTMC()

            tmc_coll_object.header = coll_object.header
            tmc_coll_object.id.name = coll_object.id
            tmc_coll_object.id.object_id = object_id
            object_id += 1
            tmc_coll_object.poses = coll_object.mesh_poses

            for mesh in coll_object.meshes:  # type: Mesh
                tmc_shape = Shape()
                tmc_shape.vertices = mesh.vertices
                for triangle in mesh.triangles:  # type: MeshTriangle
                    tmc_shape.triangles.extend(triangle.vertex_indices)

                tmc_coll_object.shapes.append(tmc_shape)

            tmc_world.known_objects.append(tmc_coll_object)

        self._collision_environment = tmc_world


if __name__ == "__main__":
    rospy.init_node('manipulation_bridge')
    manipulation_bridge = ManipulationBridge()

    rospy.spin()
