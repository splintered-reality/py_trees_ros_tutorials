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
Mocks a battery provider.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import rclpy
import rclpy.parameter
import sys

##############################################################################
# Class
##############################################################################


class SafetySensors(object):
    """
    Mocks the ability to enable/disable a safety sensor processing pipeline.
    This emulates a component which needs to be enabled contextually so that
    cpu resources can be efficiently optimised or to resolve contextual
    conflicts in the usage of the sensors.

    Node Name:
        * **safety_sensors**

    Dynamic Parameters:
        * **~enable** (:obj:`bool`)

          * enable/disable the safety sensor pipeline (default: False)

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self):
        # node
        self.node = rclpy.create_node(
            "safety_sensors",
            parameter_overrides=[
                rclpy.parameter.Parameter('enabled', rclpy.parameter.Parameter.Type.BOOL, False),
            ],
            automatically_declare_parameters_from_overrides=True
        )

    def shutdown(self):
        """
        Cleanup ROS components.
        """
        # currently complains with:
        #  RuntimeWarning: Failed to fini publisher: rcl node implementation is invalid, at /tmp/binarydeb/ros-dashing-rcl-0.7.5/src/rcl/node.c:462
        # Q: should rlcpy.shutdown() automagically handle descruction of nodes implicitly?
        self.node.destroy_node()


def main():
    """
    Entry point for the mock safety sensors node.
    """
    parser = argparse.ArgumentParser(description='Mock the safety sensors')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    safety_sensors = SafetySensors()
    try:
        rclpy.spin(safety_sensors.node)
    except KeyboardInterrupt:
        pass
    safety_sensors.shutdown()
    rclpy.shutdown()
