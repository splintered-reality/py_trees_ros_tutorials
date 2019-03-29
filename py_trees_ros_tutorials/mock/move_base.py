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
                         action_type=py_trees_actions.MoveBase,
                         generate_feedback_message=self.generate_feedback_message,
                         duration=duration
                         )
        # self.odometry = nav_msgs.Odometry()
        # self.odometry.pose.pose.position = geometry_msgs.Point(0, 0, 0)
        self.pose = geometry_msgs.PoseStamped()
        self.pose.pose.position = geometry_msgs.Point(x=0.0, y=0.0, z=0.0)

        # publishers
        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('pose', pose_topic, geometry_msgs.PoseStamped, not_latched),
                # ('odometry', odometry_topic, nav_msgs.Odometry, not_latched)
            ]
        )
        self.publishers.pose.publish(self.pose)
        # self.publishers.odometry.publish(self.odometry)

        self.publish_timer = self.node.create_timer(
            timer_period_sec=0.5,
            callback=self.publish
        )

    def generate_feedback_message(self):
        """
        Increment the odometry and pose towards the goal.
        """
        # actually doesn't go to the goal right now...
        # but we could take the feedback from the action
        # and increment this to that proportion
        # self.odometry.pose.pose.position.x += 0.01
        self.pose.pose.position.x += 0.01
        msg = py_trees_actions.MoveBase_Feedback()  # .Feedback() is more proper, but indexing can't find it
        msg.base_position = self.pose
        return msg

    def publish(self):
        """
        Most things expect a continous stream of odometry/pose messages, so we
        run this from a timer.
        """
        # self.publishers.odometry.publish(self.odometry)
        self.publishers.pose.publish(self.pose)

    def shutdown(self):
        if self.publish_timer is not None:
            self.publish_timer.cancel()
            self.node.destroy_timer(self.publish_timer)
        super().shutdown()


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a docking controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)

    rclpy.init()  # picks up sys.argv automagically internally
    move_base = MoveBase()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(move_base.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        move_base.abort()
        # caveat: often broken, whether with spin_once multiple times or this, the
        # usual mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits

    print("move base")
    move_base.shutdown()
    print("rclpy")
    rclpy.shutdown()
