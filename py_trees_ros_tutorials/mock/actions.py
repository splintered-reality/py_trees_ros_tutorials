#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks an action server.
"""


##############################################################################
# Imports
##############################################################################

import action_msgs.msg as action_msgs  # GoalStatus
import action_msgs.srv as action_srvs
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.executors
import rclpy.parameter
import rclpy.qos
import time

##############################################################################
# Fake
##############################################################################
#
# References:
#  rclpy/action/client.py: https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/action/client.py
#  action server PR      : https://github.com/ros2/rclpy/pull/270
#  action client example : https://github.com/ros2/examples/pull/222
#
# Note:
#  currently:  py_trees_ros_interfaces.action.Dock_Goal.Request()
#  to come(?): py_trees_ros_interfaces.action.Dock.GoalRequestService.Request()


class GenericServer(object):
    """
    Generic action server that can be subclassed to quickly create action
    servers of varying types in a mock simulation.

    Dynamic Reconfigure:
        * **~duration** (:obj:`float`)

          * reconfigure the duration to be used for the next goal execution

    Args:
        action_name (:obj:`str`): name of the action server (e.g. move_base)
        action_type (:obj:`any`): type of the action server (e.g. move_base_msgs.msg.MoveBaseAction
        custom_execute_callback (:obj:`func`): callback to be executed inside the execute loop, no args
        goal_recieved_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): forcibly override the dyn reconf time for a goal to complete (seconds)

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self,
                 action_name,
                 action_type,  # e.g. (py_trees_ros_interfaces.action, "Dock")
                 generate_feedback_message=None,
                 goal_received_callback=lambda request: None,
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

        self.frequency = 3.0  # hz
        self.percent_completed = 0
        self.goal_received = None
        self.title = ""  # action_name.replace('_', ' ').title()
        self.duration = self.node.get_parameter("duration").value
        self.prefix = "[" + self.title + "] " if self.title else ""

        self.action_type = action_type
        if generate_feedback_message is None:
            self.generate_feedback_message = lambda: self.action_type.Feedback()
        else:
            self.generate_feedback_message = generate_feedback_message
        self.goal_received_callback = goal_received_callback

        self.action_server = rclpy.action.ActionServer(
            self.node,
            self.action_type,
            action_name,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),  # needed?
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_goal_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            result_timeout=10
        )
        print("DJS: Setup")

    def goal_callback(self, goal_request):
        if self.goal_received:
            self.node.get_logger().info(
                "{prefix}not permitting pre-emptions, rejecting".format(
                    prefix=self.prefix
                )
            )
            return rclpy.action.server.GoalResponse.REJECT
        else:
            self.node.get_logger().info("{prefix}received a goal".format(prefix=self.prefix))
            self.goal_received = goal_request
            self.goal_received_callback(goal_request)
            self.percent_completed = 0
            self.duration = self.node.get_parameter("duration").value
            return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """No cancellations."""
        self.node.get_logger().info("{prefix}no cancellations accepted".format(prefix=self.prefix))
        return rclpy.action.CancelResponse.REJECT

#     def result_service_callback(self, request, response):
#         """
#         Bit wierd, using this to trigger the actual work. Alternative
#         would be to asyncio this one and await execute()
#         """
#         self.node.get_logger().info("received a result request\n    %s" % request)
#         self.execute()
#         response.action_status = action_msgs.GoalStatus.STATUS_SUCCEEDED
#         return response

    def execute_goal_callback(self, goal_handle):
        """
        Check for pre-emption, but otherwise just spin around gradually incrementing
        a hypothetical 'percent' done.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        # TODO: put this all under a timer?
        # If you want to check for cancellations, check goal_handle.is_cancel_requested

        # goal.details (e.g. pose) = don't care
        self.node.get_logger().info(
            "{prefix}executing a goal".format(
                prefix=self.prefix
            )
        )
        success = False
        while not success:
            if self.goal_received:
                increment = 100 / (self.frequency * self.duration)
                if self.percent_completed >= 100.0:
                    self.percent_completed = 100.0
                    self.node.get_logger().info(
                        "{prefix}feedback 100%%".format(
                            prefix=self.prefix
                        )
                    )
                    success = True
                else:
                    self.node.get_logger().info(
                        "{prefix}feedback {percent:.2f}%%".format(
                            prefix=self.prefix, percent=self.percent_completed
                        )
                    )
                    self.percent_completed += increment
                    goal_handle.publish_feedback(
                        self.generate_feedback_message()
                    )

            # TODO: use a rate when they have it
            time.sleep(1.0 / self.frequency)
        if success:
            result = self.action_type.Result()
            result.message = "{prefix}goal executed with success".format(prefix=self.prefix)
            self.node.get_logger().info(result.message)
            goal_handle.set_succeeded()
            self.goal_received = None
            return result

    def handle_accepted_callback(self, goal_handle):
        self.node.get_logger().info("{prefix}handle accepted".format(prefix=self.prefix))
        goal_handle.execute()

    def shutdown(self):
        """
        Cleanup
        """
        self.action_server.destroy()
        self.node.destroy_node()
