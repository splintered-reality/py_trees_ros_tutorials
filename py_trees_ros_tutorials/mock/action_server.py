#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a battery provider.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import rclpy
import rclpy.parameter
import sys

##############################################################################
# Class
##############################################################################


class ActionServer(object):
    """
    Generic action server that can be subclassed to quickly create action
    servers of varying types in a mock simulation.

    Dynamic Reconfigure:
        * **~duration** (:obj:`float`)

          * reonfigure the duration to be used for the next goal execution

    Args:
        action_name (:obj:`str`): name of the action server (e.g. move_base)
        action_type (:obj:`any`): type of the action server (e.g. move_base_msgs.msg.MoveBaseAction
        worker (:obj:`func`): callback to be executed inside the execute loop, no args
        goal_recieved_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): forcibly override the dyn reconf time for a goal to complete (seconds)

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self,
                 action_name,
                 action_type,
                 worker,
                 goal_received_callback=None,
                 duration=None):
        self.node = rclpy.create_node(
            action_name,
            initial_parameters=[
                rclpy.parameter.Parameter(
                    'duration',
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    5.0  # seconds
                ),
            ]
        )

        self.worker = worker
        self.goal_received_callback = goal_received_callback
#         self.action_server = actionlib.SimpleActionServer(action_name,
#                                                           action_type,
#                                                           execute_cb=self.execute,
#                                                           auto_start=False
#                                                           )
        self.percent_completed = 0
        self.title = action_name.replace('_', ' ').title()
        self.action = action_type()

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        # self.action_server.start()
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()

    def execute(self, goal):
        """
        Check for pre-emption, but otherwise just spin around gradually incrementing
        a hypothetical 'percent' done.

        Args:
            goal (:obj:`any`): goal of type specified by the action_type in the constructor.
        """
        if self.goal_received_callback:
            self.goal_received_callback(goal)
        # goal.target_pose = don't care
        frequency = 3.0  # hz
        increment = 100 / (frequency * self.parameters.duration)
        self.percent_completed = 0
        rate = rospy.Rate(frequency)  # hz
        rospy.loginfo("{title}: received a goal".format(title=self.title))
        # if we just received a goal, we erase any previous pre-emption
        self.action_server.preempt_request = False
        while True:
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                rospy.loginfo("{title}: goal preempted".format(title=self.title))
                self.action_server.set_preempted(self.action.action_result.result, "goal was preempted")
                success = False
                break
            if self.percent_completed >= 100:
                rospy.loginfo("{title}: feedback 100%".format(title=self.title))
                success = True
                break
            else:
                rospy.loginfo("{title}: feedback {percent:.2f}%".format(title=self.title, percent=self.percent_completed))
                self.percent_completed += increment
                self.worker()
            rate.sleep()
        if success:
            rospy.loginfo("{title}: goal success".format(title=self.title))
            self.action_server.set_succeeded(self.action.action_result.result, "goal reached")
