#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# This node listens to a service call for a joint trajectory planner.
# These will be bridged to the Toyota safe pose changer.
# '''

import rospy
import actionlib

from tmc_manipulation_msgs.srv import SafeJointChange
from tmc_planning_msgs.srv import PlanWithJointGoals
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
import hsrb_interface
import sys

reload(sys)
sys.setdefaultencoding('utf8')

class JointTrajectory(object):
    def __init__(self):

        # server
        self.srv_safe_joint_change = actionlib.SimpleActionServer('/hero/body/joint_trajectory_action',
                                                                  FollowJointTrajectoryAction,
                                                                  execute_cb=self.moveit_joint_change_srv,
                                                                  auto_start=False)
        self.srv_safe_joint_change.start()

        # clients
        self.client_safe_joint_change = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)
        self.client_moveit_joint_change = rospy.ServiceProxy('/plan_with_joint_goals', PlanWithJointGoals)

    def safe_joint_change_srv(self, goal):
        """
        Here the follow joint trajectory action is translated to a SafeJointChange message type
        :param goal: the FollowJointTrajectoryAction type
        :return: the SafeJointChange message type
        """

        # helper variables
        # r = rospy.Rate(1)
        success = True
           
        # append the seeds for the fibonacci sequence
        safeJointChange = JointState()
        safeJointChange.header.seq = 0 
        safeJointChange.header.stamp.secs = 0
        safeJointChange.header.stamp.nsecs = 0
        safeJointChange.header.frame_id = ''

        safeJointChange.name = goal.trajectory.joint_names
        safeJointChange.position = [0 for name in safeJointChange.name]
        safeJointChange.velocity = [0 for name in safeJointChange.name]
        safeJointChange.effort = [0 for name in safeJointChange.name]
           
        # start executing the action
        for point in goal.trajectory.points:
            rospy.loginfo("started pose in trajectory bridge")
        # check that preempt has not been requested by the client
            if self.srv_safe_joint_change.is_preempt_requested():
                rospy.loginfo('Trajectory bridge: Preempted')
                self.srv_safe_joint_change.set_preempted()
                success = False
                break
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            #r.sleep()
            safeJointChange.position = point.positions
            self.client_safe_joint_change.wait_for_service()
            success=self.client_safe_joint_change(safeJointChange)
            if not success:
                rospy.logerr('Trajectory bridge failed to change pose')
                break
            self.client_safe_joint_change.wait_for_service()
            rospy.loginfo('position [{}]'.format(point.positions))            
 
        if success:
            rospy.loginfo('Trajectory bridge: Succeeded')
            self.srv_safe_joint_change.set_succeeded()

    def moveit_joint_change_srv(self, goal):
        """
        :param goal: the FollowJointTrajectoryAction type
        :return:
        """
        success = True

        with hsrb_interface.Robot() as robot:
            whole_body = robot.get('whole_body')
            goals = {}
            rospy.loginfo('points: {}'.format(goal.trajectory.points))
            i = 0
            for n in goal.trajectory.joint_names:
                goals[n] = goal.trajectory.points[0].positions[i]
                i+=1
            self.client_moveit_joint_change.wait_for_service()
            success = whole_body.move_to_joint_positions(goals)

            if success:
                self.srv_safe_joint_change.set_succeeded()
            self.client_moveit_joint_change.wait_for_service()
# raise ROSException("publish() to a closed topic")
# 'required argument is not a float' when writing '' in goal_state

if __name__ == "__main__":
    rospy.init_node('joint_trajectory_action')
    joint_trajectory = JointTrajectory()

    rospy.spin()


###########################################################
    # def move_to_joint_positions(self, goals={}, **kwargs):
    #     """
    #     import math
    #     import hsrb_interface
    #
    #     with hsrb_interface.Robot() as robot:
    #        whole_body = robot.get('whole_body')
    #        goals = {
    #            'arm_lift_joint': 0.5,
    #            'arm_flex_joint': math.radians(-90)
    #        }
    #        whole_body.move_to_joint_positions(goals)
    #
    #        # The method also accept keyword arguments
    #        whole_body.move_to_joint_positions(
    #            head_tilt_joint=math.radians(30)
    #        )
    #
    #     """
    #     if goals is None:
    #         goals = {}
    #     goals.update(kwargs)
    #     if not goals:
    #         return
    #     goal_state = JointState()
    #     for k, v in goals.items():
    #         goal_state.name.append(k)
    #         goal_state.position.append(v)
    #     self._change_joint_state(goal_state)
    #
    # def _change_joint_state(self, goal_state):
    #     """Move joints to specified joint state while checking self collision.
    #
    #     Args:
    #         goal_state (sensor_msgs.msg.JointState): Target joint state
    #     Returns:
    #         None
    #     Raises:
    #         ValueError: Some specified joints are not found.
    #         ValueError: Target joints include some uncontrollable joints.
    #     """
    #     # Validate joint names
    #     initial_joint_state = self._get_joint_state()
    #     active_joint_set = set(initial_joint_state.name)
    #     target_joint_set = set(goal_state.name)
    #     if not target_joint_set.issubset(active_joint_set):
    #         unknown_set = target_joint_set.difference(active_joint_set)
    #         msg = "No such joint(s): [{0}]".format(', '.join(unknown_set))
    #         raise ValueError(msg)
    #     if 'base_roll_joint' in target_joint_set:
    #         raise ValueError(
    #             "base_roll_joint is not supported in change_joint_state")
    #     passive_joint_set = set(self._passive_joints)
    #     if not target_joint_set.isdisjoint(passive_joint_set):
    #         intersected = target_joint_set.intersection(passive_joint_set)
    #         msg = "Passive joint(s): [{0}]".format(', '.join(intersected))
    #         raise ValueError(msg)
    #
    #     req = self._generate_planning_request(PlanWithJointGoalsRequest)
    #     goal_position = JointPosition()
    #     goal_position.position = goal_state.position
    #     req.use_joints = goal_state.name
    #     req.goal_joint_states.append(goal_position)
    #
    #     service_name = self._setting['plan_with_joint_goals_service']
    #     plan_service = rospy.ServiceProxy(service_name, PlanWithJointGoals)
    #     res = plan_service.call(req)
    #     if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
    #         msg = "Fail to plan change_joint_state"
    #         raise exceptions.MotionPlanningError(msg, res.error_code)
    #     res.base_solution.header.frame_id = settings.get_frame('odom')
    #     constrained_traj = self._constrain_trajectories(res.solution)
    #     self._execute_trajectory(constrained_traj)
