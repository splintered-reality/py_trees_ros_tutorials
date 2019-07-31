#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a simple action server that rotates the robot 360 degrees.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import math
import py_trees_ros.mock.actions
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import sys

##############################################################################
# Class
##############################################################################


class Rotate(py_trees_ros.mock.actions.GenericServer):
    """
    Simple server that controls a full rotation of the robot.

    Node Name:
        * **rotation_controller**

    Action Servers:
        * **/rotate** (:class:`py_trees_ros_interfaces.action.Dock`)

          * motion primitives - rotation server

    Args:
        rotation_rate (:obj:`float`): rate of rotation (rad/s)
    """
    def __init__(self, rotation_rate: float=1.57):
        super().__init__(node_name="rotation_controller",
                         action_name="rotate",
                         action_type=py_trees_actions.Rotate,
                         generate_feedback_message=self.generate_feedback_message,
                         duration=2.0 * math.pi / rotation_rate
                         )

    def generate_feedback_message(self):
        """
        Create a feedback message that populates the percent completed.

        Returns:
            :class:`py_trees_actions.Rotate.Feedback`: the populated feedback message
        """
        # TODO: send some feedback message
        msg = py_trees_actions.Rotate.Feedback()  # Rotate.Feedback() works, but the indexer can't find it
        msg.percentage_completed = self.percent_completed
        msg.angle_rotated = 2*math.pi*self.percent_completed/100.0
        return msg


def main():
    """
    Entry point for the mock rotation controller node.
    """
    parser = argparse.ArgumentParser(description='Mock a rotation controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    rotation = Rotate()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(rotation.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        rotation.abort()
        # caveat: often broken, whether with spin_once multiple times or this, the
        # usual mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits

    rotation.shutdown()
    rclpy.shutdown()
