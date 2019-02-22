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

    Dynamic Reconfigure:
        * **~enable** (:obj:`bool`)

          * enable/disable the safety sensor pipeline (default: False)

    Use the ``dashboard`` to dynamically reconfigure the parameters.
    """
    def __init__(self):
        # node
        self.node = rclpy.create_node(
            "safety_sensors",
            initial_parameters=[
                rclpy.parameter.Parameter('enabled', rclpy.parameter.Parameter.Type.BOOL, False),
            ]
        )

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock the safety sensors')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    safety_sensors = SafetySensors()
    safety_sensors.spin()
    rclpy.shutdown()
