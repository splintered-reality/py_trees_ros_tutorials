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
Mocks a docking controller
"""


##############################################################################
# Imports
##############################################################################

import argparse
import py_trees_ros.mock.actions
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import sys

##############################################################################
# Class
##############################################################################


class Dock(py_trees_ros.mock.actions.GenericServer):
    """
    Simple action server that docks/undocks depending on the instructions
    in the goal requests.

    Node Name:
        * **docking_controller**

    Action Servers:
        * **/dock** (:class:`py_trees_ros_interfaces.action.Dock`)

          * docking/undocking control

    Args:
        duration: mocked duration of a successful docking/undocking action
    """
    def __init__(self, duration: float=2.0):
        super().__init__(
            node_name="docking_controller",
            action_name="dock",
            action_type=py_trees_actions.Dock,
            generate_feedback_message=self.generate_feedback_message,
            goal_received_callback=self.goal_received_callback,
            duration=duration
        )

    def goal_received_callback(self, goal):
        """
        Set the title of the action depending on whether a docking
        or undocking action was requestions ('Dock'/'UnDock')
        """
        if goal.dock:
            self.title = "Dock"
        else:
            self.title = "UnDock"

    def generate_feedback_message(self) -> py_trees_actions.Dock.Feedback:
        """
        Create a feedback message that populates the percent completed.

        Returns:
            :class:`py_trees_actions.Dock_Feedback`: the populated feedback message
        """
        msg = py_trees_actions.Dock.Feedback(
            percentage_completed=self.percent_completed
        )
        return msg


def main():
    """
    Entry point for the mocked docking controller.
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
