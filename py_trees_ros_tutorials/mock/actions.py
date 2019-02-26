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
Mocks an action server.
"""


##############################################################################
# Imports
##############################################################################

import action_msgs.srv as action_srvs
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
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
        worker (:obj:`func`): callback to be executed inside the execute loop, no args
        goal_recieved_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): forcibly override the dyn reconf time for a goal to complete (seconds)

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self,
                 action_name,
                 action_type_string,  # "Dock" or "Rotate"
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
        self.frequency = 3.0  # hz
        self.percent_completed = 0
        self.goal_received = None
        self.title = action_name.replace('_', ' ').title()
        self.duration = self.node.get_parameter("duration").value

        self.custom_goal_callback = goal_received_callback

        self.service_callbacks = {}
        self.service_callbacks["goal"] = {
            "module": py_trees_actions,
            "callback": self.goal_service_callback,
            "class": action_type_string + "_Goal"
        }
        self.service_callbacks["result"] = {
            "module": py_trees_actions,
            "callback": self.result_service_callback,
            "class": action_type_string + "_Result"
        }
        self.service_callbacks["cancel"] = {
            "module": action_srvs,
            "callback": self.cancel_service_callback,
            "class": "CancelGoal"
        }

        self.services = {}
        self.service_types = {}
        for service_name in ["goal", "result", "cancel"]:
            self.service_types[service_name] = getattr(
                self.service_callbacks[service_name]["module"],
                self.service_callbacks[service_name]["class"]
            )
            self.services[service_name] = self.node.create_service(
                self.service_types[service_name],
                "~/" + service_name,
                self.service_callbacks[service_name],
                qos_profile=rclpy.qos.qos_profile_services_default
            )

        self.services["cancel"] = self.node.create_service(
            self.service_types["cancel"],
            "~/" + service_name,
            self.service_callbacks[service_name],
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        
        self.feedback_message_type = getattr(
            py_trees_actions,
            action_type_string + "_Feedback"
        )
        self.feedback_publisher = self.node.create_publisher(
            msg_type=self.feedback_message_type,
            topic="~/feedback")

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        # TODO: replace when rclpy has a Rate
        # rate = rclpy.Rate(frequency)  # hz
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node)
                self.execute_once()
                time.sleep(1.0/self.frequency)
                # TODO: once we have rates
                # rate.sleep()
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()

    def goal_service_callback(self, goal_request):
        if self.goal_received:
            self.node.get_logger().info("[{title}] pre-empted a goal".format(title=self.title))
            # TODO: send back current goal's result with ERROR
            pass
        else:
            self.node.get_logger().info("[{title}] received a goal".format(title=self.title))
        self.goal_received = goal_request
        self.custom_goal_callback(goal_request)
        self.percent_completed = 0
        self.duration = self.node.get_parameter("duration").value
        # TODO: some response

    def cancel_service_callback(self, cancel_request):
        pass

    def result_service_callback(self, result_request):
        pass

    def execute_once(self, goal):
        """
        Check for pre-emption, but otherwise just spin around gradually incrementing
        a hypothetical 'percent' done.

        Args:
            goal (:obj:`any`): goal of type specified by the action_type in the constructor.
        """
        # goal.details (e.g. pose) = don't care
        success = False
        if self.goal_received:
            increment = 100 / (self.frequency * self.duration)
            while True:
                if self.percent_completed >= 100:
                    self.node.get_logger().info("[{title}] feedback 100%".format(title=self.title))
                    success = True
                    break
                else:
                    self.node.get_logger().info("[{title}] feedback {percent:.2f}%".format(title=self.title, percent=self.percent_completed))
                    self.percent_completed += increment
                    self.worker()
        if success:
            self.node.get_logger().info("[{title}] goal success".format(title=self.title))
            # TODO: send goal result with SUCCESS
            self.goal_received = None
