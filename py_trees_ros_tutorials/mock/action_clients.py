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

import action_msgs.msg as action_msgs  # GoalStatus
import action_msgs.srv as action_srvs  # CancelGoal
import py_trees_ros_interfaces.action as py_trees_actions  # Dock, Rotate
import rclpy
# import rclpy.action  # ActionClient

##############################################################################
# Class
##############################################################################


class ActionClient(object):
    """
    Generic action client that can be used to test the mock action servers.

    Args:
        node_name (:obj:`str`): name of the action server (e.g. move_base)
        action_name (:obj:`str`): name of the action server (e.g. move_base)
        action_type (:obj:`any`): type of the action server (e.g. move_base_msgs.msg.MoveBaseAction
        worker (:obj:`func`): callback to be executed inside the execute loop, no args
        goal_recieved_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): forcibly override the dyn reconf time for a goal to complete (seconds)

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self,
                 node_name,
                 action_name,
                 action_type_string):
        self.node = rclpy.create_node(node_name)
        self.action_name = action_name
        self.action_type = action_type_string
        self.goal_type = getattr(
            py_trees_actions,
            action_type_string + "_Goal"
        )
        self.result_type = getattr(
            py_trees_actions,
            action_type_string + "_Result"
        )
        self.clients = {
            "goal": self.node.create_client(
                self.goal_type,
                "docking_controller/goal"
            ),
            "result": self.node.create_client(
                self.goal_type,
                "docking_controller/result"
            ),
            "cancel": self.node.create_client(
                action_srvs.CancelGoal,
                "docking_controller/cancel"
            )
        }
        # self.action_client = rclpy.action.ActionClient(self.node, action_type, action_name)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.node.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

    def get_result_callback(self, future):
        status = future.result().action_status
        if status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Goal succeeded! Result: {0}'.format(future.result().sequence))
        else:
            self.node.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        # self.node.get_logger().info('waiting for action server...')
        # self._action_client.wait_for_server()

        goal_msg = self.goal_type()
        goal_msg.order = 10

        self.node.get_logger().info('sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def spin(self):
        initialised = {name: False for name in self.clients.keys()}
        try:
            while rclpy.ok() and not all(value for value in initialised.values()):
                for name, client in self.clients.items():
                    if client.wait_for_service(timeout_sec=0.1):
                        initialised[name] = True
                        self.node.get_logger().info("'{}' initialised".format(name))
                    else:
                        self.node.get_logger().info("service '' unavailable, waiting...".format(name))
            # self.send_goal()
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()


def dock(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="dock_client",
        action_name="dock",
        action_type_string="Dock"  # py_trees_actions.Dock
        )
    action_client.spin()


def rotate(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="rotate_client",
        action_name="rotate",
        action_type_string="Rotate"  # py_trees_actions.Rotate
        )
    action_client.spin()
