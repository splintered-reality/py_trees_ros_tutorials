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
Action servers and clients
"""

##############################################################################
# Imports
##############################################################################

import action_msgs.msg as action_msgs  # GoalStatus
import argparse
import py_trees_ros.exceptions
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import rclpy.action
import sys

##############################################################################
# Action Client
##############################################################################


class GenericClient(object):
    """
    Generic action client that can be used to test the mock action servers.

    Args:
        node_name (:obj:`str`): name of the action server (e.g. move_base)
        action_name (:obj:`str`): name of the action server (e.g. move_base)
        action_type (:obj:`any`): type of the action server (e.g. move_base_msgs.msg.MoveBaseAction
        generate_feedback_message(:obj:`func`): format the feedback message

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self,
                 node_name,
                 action_name,
                 action_type,
                 generate_feedback_message=None
                 ):
        self.action_type = action_type
        self.action_name = action_name
        self.node_name = node_name
        if generate_feedback_message is None:
            self.generate_feedback_message = lambda msg: str(msg)
        else:
            self.generate_feedback_message = generate_feedback_message

        self.status_strings = {
                action_msgs.GoalStatus.STATUS_UNKNOWN : "STATUS_UNKNOWN",  # noqa
                action_msgs.GoalStatus.STATUS_ACCEPTED : "STATUS_ACCEPTED",  # noqa
                action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
                action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELED : "STATUS_CANCELED",  # noqa
                action_msgs.GoalStatus.STATUS_ABORTED  : "STATUS_ABORTED"  # noqa
            }

        ####################
        # ROS Setup
        ####################
        self.node = rclpy.create_node(self.node_name)
        self.action_client = rclpy.action.ActionClient(
            node=self.node,
            action_type=self.action_type,
            action_name=self.action_name
        )
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

    def setup(self):
        self.node.get_logger().info(
            'waiting for server [{}]'.format(
                self.action_name
            )
        )
        result = self.action_client.wait_for_server(timeout_sec=2.0)
        if not result:
            message = "timed out waiting for the server [{}]".format(
                    self.action_name
                )
            self.node.get_logger().error(message)
            raise py_trees_ros.exceptions.TimedOutError(message)
        else:
            self.node.get_logger().info("...connected")

    def feedback_callback(self, msg):
        self.node.get_logger().info('feedback: {0}'.format(self.generate_feedback_message(msg)))

    def send_cancel_request(self):

        self.node.get_logger().info('Cancelling goal')

        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)

        self._timer.cancel()

    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Goal successfully cancelled')
        else:
            self.node.get_logger().info('Goal failed to cancel')

    def send_goal(self):
        """
        Send the goal and get a future back, but don't do any
        spinning here to await the future result.

        Returns:
            :class:`rclpy.task.Future`
        """
        self.node.get_logger().info('sending goal...')
        self._send_goal_future = self.action_client.send_goal_async(
                self.action_type.Goal(),
                feedback_callback=self.feedback_callback,
                # A random uuid is always generated
                # goal_uuid=unique_identifier_msgs.UUID(
                #     uuid=list(uuid.uuid4().bytes)
                # )
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    def goal_response_callback(self, future):
        """
        Handle goal response, proceed to listen for the result if accepted.
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().info('...goal rejected :( \n%r' % (future.exception()))
            return
        self.node.get_logger().info("...goal accepted :)\n%s" % future.result())

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle result.
        """
        message = future.result().message
        status = future.result().action_status
        status_string = self.status_strings[status]

        if status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
            self.node.get_logger().info("Result")
            self.node.get_logger().info("  status: {}".format(status_string))
            self.node.get_logger().info("  message: {}".format(message))
        else:
            self.node.get_logger().info('Goal failed with status: {0}'.format(status_string))

    def spin(self, cancel=False):
        """
        Common spin method for clients.

        Args:
            cancel (:obj:`bool`): send a cancel request shortly after sending the goal request
        """
        self.send_goal()
        if cancel:
            self._timer = self.node.create_timer(1.0, self.send_cancel_request)
        rclpy.spin(self.node)

    def shutdown(self):
        self.action_client.destroy()
        self.node.destroy_node()


##############################################################################
# Action Clients
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


def generic_client(client_class):
    rclpy.init(args=None)
    args = command_line_argument_parser()
    action_client = client_class()
    try:
        action_client.setup()
        action_client.spin(cancel=args.cancel)
    except (py_trees_ros.exceptions.TimedOutError, KeyboardInterrupt):
        pass
    action_client.shutdown()


class DockClient(GenericClient):
    def __init__(self):
        super().__init__(
            node_name="docking_client",
            action_name="dock",
            action_type=py_trees_actions.Dock,
            generate_feedback_message=lambda msg: "{}".format(msg.percentage_completed)
        )


def dock_client():
    generic_client(DockClient)


class RotateClient(GenericClient):
    def __init__(self):
        super().__init__(
            node_name="rotate_client",
            action_name="rotate",
            action_type=py_trees_actions.Rotate
        )


def rotate_client(args=None):
    rclpy.init(args=args)
    action_client = RotateClient()
    try:
        action_client.spin()
    except KeyboardInterrupt:
        pass
    action_client.shutdown()


class MoveBaseClient(GenericClient):
    def __init__(self):
        super().__init__(
            node_name="move_base_client",
            action_name="move_base",
            action_type=py_trees_actions.MoveBase,
            generate_feedback_message=lambda msg: "x={0:.2f}m".format(msg.base_position.pose.position.x),
        )


def move_base_client(args=None):
    rclpy.init(args=args)
    args = command_line_argument_parser()
    action_client = MoveBaseClient()
    try:
        action_client.spin(cancel=args.cancel)
    except KeyboardInterrupt:
        pass
    action_client.shutdown()
