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
Mocks a simple action server that rotates the robot 360 degrees.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import math
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import sys

from . import actions

##############################################################################
# Class
##############################################################################


class Rotate(actions.GenericServer):
    """
    Simple server that controls a full rotation of the robot.

    Args:
        rotation_rate (:obj:`float`): rate of rotation )rad/s)
    """
    def __init__(self, rotation_rate=1.57):
        super().__init__(node_name="rotation_controller",
                         action_name="rotate",
                         action_type=py_trees_actions.Rotate,
                         generate_feedback_message=self.generate_feedback_message,
                         duration=2.0 * math.pi / rotation_rate
                         )

    def generate_feedback_message(self):
        """
        Create some appropriate feedback.
        """
        # TODO: send some feedback message
        msg = py_trees_actions.Rotate_Feedback()  # Rotate.Feedback() works, but the indexer can't find it
        msg.percentage_completed = self.percent_completed
        msg.angle_rotated = 2*math.pi*self.percent_completed/100.0
        return msg


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a docking controller')
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
