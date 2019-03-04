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
import geometry_msgs.msg as geometry_msgs
import math
import py_trees_ros.utilities
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
                         action_type=(py_trees_actions, "MoveBase"),
                         custom_execute_callback=self.custom_execute_callback,
                         duration=duration
                         )
        # self.odometry = nav_msgs.Odometry()
        # self.odometry.pose.pose.position = geometry_msgs.Point(0, 0, 0)
        self.pose = geometry_msgs.PoseWithCovarianceStamped()
        self.pose.pose.pose.position = geometry_msgs.Point(x=0.0, y=0.0, z=0.0)

        # publishers
        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('pose', pose_topic, geometry_msgs.PoseWithCovarianceStamped, not_latched),
                # ('odometry', odometry_topic, nav_msgs.Odometry, not_latched)
            ]
        )
        self.publishers.pose.publish(self.pose)
        # self.publishers.odometry.publish(self.odometry)

        self.publish_timer = self.node.create_timer(
            timer_period_sec=0.5,
            callback=self.publish
        )

    def custom_execute_callback(self):
        """
        Increment the odometry and pose towards the goal.
        """
        # actually doesn't go to the goal right now...
        # but we could take the feedback from the action
        # and increment this to that proportion
        # self.odometry.pose.pose.position.x += 0.01
        self.pose.pose.pose.position.x += 0.01

    def publish(self, unused_event):
        """
        Most things expect a continous stream of odometry/pose messages, so we
        run this from a timer.
        """
        # self.publishers.odometry.publish(self.odometry)
        self.publishers.pose.publish(self.pose)


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a docking controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    move_base_controller = MoveBase()
    move_base_controller.spin()
    rclpy.shutdown()
