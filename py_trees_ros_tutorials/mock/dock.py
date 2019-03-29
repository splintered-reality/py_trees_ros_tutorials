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
Mocks a docking controller
"""


##############################################################################
# Imports
##############################################################################

import argparse
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import sys

from . import actions

##############################################################################
# Class
##############################################################################


class Dock(actions.GenericServer):
    """
    Simple server that docks if the goal is true, undocks otherwise.
    """
    def __init__(self):
        super().__init__(
            action_name="docking_controller",
            action_type=py_trees_actions.Dock,
            generate_feedback_message=self.generate_feedback_message,
            goal_received_callback=self.goal_received_callback,
            duration=2.0
        )

    def goal_received_callback(self, goal):
        if goal.dock:
            self.title = "Dock"
        else:
            self.title = "UnDock"

    def generate_feedback_message(self):
        """
        Create some appropriate feedback.
        """
        # TODO: send some feedback message
        msg = py_trees_actions.Dock_Feedback(
            percentage_completed=self.percent_completed
        )
        self.feedback_publisher.publish(msg)
        return msg


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a docking controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    docking = Dock()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(docking.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        docking.abort()
        # caveat: often broken, whether with spin_once multiple times or this, the
        # usual mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits

    docking.shutdown()
    rclpy.shutdown()
