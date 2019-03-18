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
        super().__init__(action_name="rotation_controller",
                         action_type=(py_trees_actions, "Rotate"),
                         custom_execute_callback=self.custom_execute_callback,
                         duration=2.0 * math.pi / rotation_rate
                         )

    def custom_execute_callback(self):
        """
        Create some appropriate feedback.
        """
        # TODO: send some feedback message
        self.feedback_publisher.publish(
            py_trees_actions.Rotate_Feedback(
                percentage_completed=self.percent_completed,
                angle_rotated=2*math.pi*self.percent_completed/100.0
            )
        )


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a docking controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    rotation_controller = Rotate()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(rotation_controller.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rotation_controller.shutdown()
    executor.shutdown()
    rclpy.shutdown()
