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

import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import rclpy.parameter

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
                 goal_callback=None,
                 result_callback=None,
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
        self.percent_completed = 0
        # self.title = action_name.replace('_', ' ').title()

        self.callbacks = {}
        self.callbacks["goal"] = goal_callback
        self.callbacks["result"] = result_callback
        self.services = {}
        self.service_types = {}
        for service_name in ["goal", "result"]:
            self.service_types[service_name] = getattr(
                py_trees_actions,
                action_type_string + "_" + service_name.capitalize()
            )
            self.services[service_name] = self.node.create_service(
                self.service_types[service_name],
                "~/" + service_name,
                self.callbacks[service_name]
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
        self.node.get_logger().info("received a goal")
        # if we just received a goal, we erase any previous pre-emption
        self.action_server.preempt_request = False
        while True:
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                self.node.get_logger().info("goal preempted")
                self.action_server.set_preempted(self.action.action_result.result, "goal was preempted")
                success = False
                break
            if self.percent_completed >= 100:
                self.node.get_logger().info("{feedback 100%")
                success = True
                break
            else:
                self.node.get_logger().info("feedback {percent:.2f}%".format(percent=self.percent_completed))
                self.percent_completed += increment
                self.worker()
            rate.sleep()
        if success:
            self.node.get_logger().info("goal success")
            self.action_server.set_succeeded(self.action.action_result.result, "goal reached")
