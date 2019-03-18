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
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_ros_actions  # Dock, Rotate
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
                 action_type_string,
                 action_server_namespace):
        self.node = rclpy.create_node(node_name)
        self.action_name = action_name
        self.action_type = action_type_string
        self.action_server_namespace = action_server_namespace
        self.goal_type = getattr(
            py_trees_ros_actions,
            action_type_string + "_Goal"
        )
        self.result_type = getattr(
            py_trees_ros_actions,
            action_type_string + "_Result"
        )
        self.clients = {
            "goal": self.node.create_client(
                self.goal_type,
                "{}/goal".format(self.action_server_namespace)
            ),
            "result": self.node.create_client(
                self.result_type,
                "{}/result".format(self.action_server_namespace)
            ),
            "cancel": self.node.create_client(
                action_srvs.CancelGoal,
                "{}/cancel".format(self.action_server_namespace)
            )
        }
        # self.action_client = rclpy.action.ActionClient(self.node, action_type, action_name)

    def feedback_callback(self, feedback):
        self.node.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

    def process_result(self, future):
        status = future.result().action_status
        if status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('goal success!')
        else:
            self.node.get_logger().info('goal failed with status {0}'.format(status))

    def send_goal(self, uuid_msg):
        # send and wait for the goal response
        self.node.get_logger().info('sending goal request...')
        request = self.goal_type.Request(action_goal_id=uuid_msg)
        self.node.get_logger().info("goal request: %s" % request)
        future = self.clients["goal"].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None and future.result().accepted is True:
            self.node.get_logger().info("...goal accepted %s" % future.result())
        else:
            self.node.get_logger().info('...goal rejected [%r]' % (future.exception()))

    def spin(self):
        initialised = {name: False for name in self.clients.keys()}
        try:
            while rclpy.ok() and not all(value for value in initialised.values()):
                for name, client in self.clients.items():
                    if client.wait_for_service(timeout_sec=0.1):
                        initialised[name] = True
                        self.node.get_logger().info("'{}' initialised".format(name))
                    else:
                        self.node.get_logger().info("service '{}' unavailable, waiting...".format(self.action_server_namespace + "/" + name))

            uuid_msg = py_trees_ros.conversions.uuid4_to_msg()
            self.send_goal(uuid_msg)

            request = self.result_type.Request(action_goal_id=uuid_msg)
            self.node.get_logger().info("result request %s" % request)
            future = self.clients["result"].call_async(request)
            self.node.get_logger().info("   future %s" % future)
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if future.done():
                    self.process_result(future)
                    break
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()


def dock(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="dock_client",
        action_name="dock",
        action_type_string="Dock",  # py_trees_ros_actions.Dock
        action_server_namespace="/docking_controller"
        )
    action_client.spin()


def rotate(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="rotate_client",
        action_name="rotate",
        action_type_string="Rotate",  # py_trees_ros_actions.Rotate
        action_server_namespace="/rotation_controller"
        )
    action_client.spin()


def move_base(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="move_base_client",
        action_name="move_base",
        action_type_string="MoveBase",  # py_trees_ros_actions.MoveBase
        action_server_namespace="/move_base"
        )
    action_client.spin()
