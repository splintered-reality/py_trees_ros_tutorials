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
Mocks a battery provider.
"""


##############################################################################
# Imports
##############################################################################


import action_msgs.msg as action_msgs  # GoalStatus
import action_msgs.srv as action_srvs  # CancelGoal
import argparse
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_ros_actions  # Dock, Rotate
import rclpy
import rclpy.action  # ActionClient
import sys
import unique_identifier_msgs.msg as unique_identifier_msgs
import uuid

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
                 action_type,
                 action_server_namespace,
                 cancel=False,
                 generate_feedback_message=None
                 ):
        self.action_type = action_type
        self.action_name = action_name
        self.action_server_namespace = action_server_namespace
        self.node_name = node_name
        if generate_feedback_message is None:
            self.generate_feedback_message = lambda msg: str(msg)
        else:
            self.generate_feedback_message = generate_feedback_message

    def setup(self):
        self.node = rclpy.create_node(self.node_name)
        self.action_client = rclpy.action.ActionClient(
            node=self.node,
            action_type=self.action_type,
            action_name=self.action_name
        )
        self.node.get_logger().info(
            'waiting for server [{}]'.format(
                self.action_server_namespace
            )
        )
        result = self.action_client.wait_for_server(timeout_sec=2.0)
        if not result:
            message = "timed out waiting for the server [{}]".format(
                    self.action_server_namespace
                )
            self.node.get_logger().error(message)
            raise RuntimeError(message)

    def feedback_callback(self, msg):
        self.node.get_logger().info('Feedback: {0}'.format(self.generate_feedback_message(msg)))

    def process_result(self, future):
        status = future.result().action_status
        if status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('goal success!')
        else:
            self.node.get_logger().info('goal failed with status {0}'.format(status))

    def send_goal(self):
        goal_uuid = unique_identifier_msgs.UUID(
            uuid=list(uuid.uuid4().bytes)
        )
        self.node.get_logger().info('sending goal...')
        future = self.action_client.send_goal_async(
                self.action_type.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        if future.result() is not None and future.result().accepted is True:
            self.node.get_logger().info("...goal accepted %s" % future.result())
        else:
            self.node.get_logger().info('...goal rejected [%r]' % (future.exception()))
        return future.result()

    def spin(self):
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        goal_handle = self.send_goal()
        if goal_handle is None:
            return
        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        # Status: https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatus.msg
        status_strings = {
            0: "STATUS_UNKNOWN",
            1: "STATUS_ACCEPTED",
            2: "STATUS_EXECUTING",
            3: "STATUS_CANCELING",
            4: "STATUS_SUCCEEDED",
            5: "STATUS_CANCELED",
            6: "STATUS_ABORTED"
        }
        self.node.get_logger().info("Result")
        self.node.get_logger().info("  status: {}".format(status_strings[future.result().action_status]))
        self.node.get_logger().info("  message: {}".format(future.result().message))

    def shutdown(self):
        self.action_client.destroy()
        self.node.destroy_node()


##############################################################################
# Programs
##############################################################################

def command_line_argument_parser():
    parser = argparse.ArgumentParser(
        description="action client",
        epilog="And his noodly appendage reached forth to tickle the blessed...\n"
    )
    parser.add_argument(
        '-c', '--cancel',
        action='store_true',
        default=False,
        help='send a cancel request a short period after sending the goal request')
    return parser.parse_args(sys.argv[1:])


def dock(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="dock_client",
        action_name="dock",
        action_type=py_trees_ros_actions.Dock,
        action_server_namespace="/docking_controller"
        )
    try:
        action_client.setup()
        action_client.spin()
    except KeyboardInterrupt:
        pass
    action_client.shutdown()


def rotate(args=None):
    rclpy.init(args=args)
    action_client = ActionClient(
        node_name="rotate_client",
        action_name="rotate",
        action_type=py_trees_ros_actions.Rotate,
        action_server_namespace="/rotation_controller"
        )
    try:
        action_client.setup()
        action_client.spin()
    except KeyboardInterrupt:
        pass
    action_client.shutdown()


def move_base(args=None):
    rclpy.init(args=args)
    args = command_line_argument_parser()
    action_client = ActionClient(
        node_name="move_base_client",
        action_name="move_base",
        action_type=py_trees_ros_actions.MoveBase,
        action_server_namespace="/move_base",
        cancel=args.cancel,
        generate_feedback_message=lambda msg: "x={0:.2f}m".format(msg.base_position.pose.position.x),
    )
    try:
        action_client.setup()
        action_client.spin()
    except KeyboardInterrupt:
        pass
    action_client.shutdown()
