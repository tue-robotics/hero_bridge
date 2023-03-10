#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''
from random import randint

import rospy
import actionlib

from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest
from tue_manipulation_msgs.msg import GraspPrecomputeAction, GraspPrecomputeGoal
from tmc_manipulation_msgs.msg import CollisionObject, CollisionObjectOperation, BaseMovementType, ArmManipulationErrorCodes, CollisionEnvironment
from tmc_geometric_shapes_msgs.msg import Shape

# Preparation to use robot functions
from hsrb_interface import Robot, settings, geometry
from hsrb_interface.collision_world import CollisionWorld
from hsrb_interface.joint_group import JointGroup

import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs


def addBox(x=0.1, y=0.1, z=0.1, pose=geometry.pose(), frame_id='map', name='box', timeout=1.0):
    """
    Add a box to the collision world

    :param x: x dimension of the box
    :param y: y dimension of the box
    :param z: z dimension of the box
    :param pose: pose of the box
    :param frame_id: frame_id of the box
    :param name: name of the box
    :param timeout: timeout for the service call
    """
    box = CollisionObject()
    shape = Shape()
    shape.type = Shape.BOX
    shape.dimensions = [x, y, z]
    pose = geometry.tuples_to_pose(pose)
    box.operation.operation = CollisionObjectOperation.ADD
    box.id.object_id = randint(1000000, 9999999)
    box.id.name = name
    box.shapes.append(shape)
    box.poses.append(pose)
    box.header.frame_id = frame_id
    box.header.stamp = rospy.Time.now()
    return box


class ManipulationBridge(object):
    def __init__(self):

        # robot
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')  # type: JointGroup

        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.collision_world = self.robot.try_get('global_collision_world')  # type: CollisionWorld
        # self.whole_body.collision_world = self.collision_world

        # self.collision_world.remove_all()
        # self.collision_world.add_box(1.2, 0.8, 0.06, pose=geometry.pose(x=0., y=0., z=0.73), frame_id='dinner_table',
        #                              name='box', timeout=30.0)
        # self.collision_world.add_box(1.2, 0.8, 0.06, pose=geometry.pose(x=0., y=0., z=0.73), frame_id='dinner_table',
        #                              name='box2', timeout=30.0)
        # rospy.sleep(10)
        # r = rospy.Rate(5)
        # # while not rospy.is_shutdown():
        # #     rospy.wait_for_message()
        # snapshot = self.collision_world.snapshot(settings.get_frame('odom'))
        # rospy.logwarn(f"{snapshot=}")
        # selfgenerated = CollisionEnvironment()
        # selfgenerated.header.frame_id = settings.get_frame('odom')
        # selfgenerated.header.stamp = rospy.Time.now()
        # selfgenerated.known_objects.append(
        #     addBox(x=1.2, y=0.8, z=0.06, pose=geometry.pose(x=0., y=0., z=0.73), frame_id='dinner_table', name='box'))
        # rospy.logwarn(f"{selfgenerated=}")

        # server
        self.srv_manipulation = actionlib.SimpleActionServer('arm_center/grasp_precompute',
                                                                  GraspPrecomputeAction,
                                                                  execute_cb=self.manipulation_srv_inst,
                                                                  auto_start=False)
        self.srv_manipulation.start()

    def manipulation_srv_inst(self, action: GraspPrecomputeGoal):
        success = self.manipulation_srv(action)
        if success:
            rospy.loginfo('Manipulation bridge: Succeeded')
            self.srv_manipulation.set_succeeded()
        else:
            rospy.loginfo('Manipulation bridge: Failed')
            self.srv_manipulation.set_aborted()

    def manipulation_srv(self, action: GraspPrecomputeGoal):
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

        # self.collision_world.remove_all()
        # self.collision_world.add_box(1.2, 0.8, 0.06, pose=geometry.pose(x=0., y=0., z=0.73), frame_id='dinner_table',
        #                              name='box', timeout=15.0)
        req = self.whole_body._generate_planning_request(PlanWithHandGoalsRequest)  # type: PlanWithHandGoalsRequest
        req.environment_before_planning.header.frame_id = settings.get_frame('odom')
        req.environment_before_planning.header.stamp = rospy.Time.now()
        req.environment_before_planning.known_objects.append(
            addBox(x=1.2, y=0.8, z=0.06, pose=geometry.pose(x=0., y=0., z=0.73), frame_id='dinner_table', name='box'))
        transform = self.tf_buffer.lookup_transform(settings.get_frame('odom'), 'dinner_table', rospy.Time(0), rospy.Duration(1))
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        req.environment_before_planning.poses.append(pose)
        rospy.loginfo(f"{req.environment_before_planning=}")
        req.origin_to_hand_goals = odom_to_hand_poses
        req.ref_frame_id = self.whole_body._end_effector_frame
        req.base_movement_type.val = BaseMovementType.PLANAR

        service_name = self.whole_body._setting['plan_with_hand_goals_service']
        plan_service = rospy.ServiceProxy(service_name, PlanWithHandGoals)
        rospy.loginfo(f"{req=}")
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
