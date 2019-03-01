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


class MoveBase(actions.GenericServer):
    """
    Simulates:

    * move base interface
    * publishing on /odom (nav_msgs.msg.Odometry)
    * publishing on /pose (geometry_msgs.msg.PoseWithCovarianceStamped)

    Args:
        odometry_topic (:obj:`str`): name of the odometry topic
        pose_topic (:obj:`str`): name of the pose (with covariance stamped) topic
        duration (:obj:`int`): time for a goal to complete (seconds)
    """
    def __init__(self, odometry_topic='/odom', pose_topic='/pose', duration=None):
        super().__init__(action_name="move_base",
                         action_type_string=move_base_msgs.MoveBaseAction,
                         custom_execute_callback=self.custom_execute_callback,
                         duration=duration
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
    rotation_controller.spin()
    rclpy.shutdown()
