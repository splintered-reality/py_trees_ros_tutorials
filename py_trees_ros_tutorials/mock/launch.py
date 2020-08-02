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
Launch the mock robot.
"""
##############################################################################
# Imports
##############################################################################

import typing

import launch
import launch_ros.actions

##############################################################################
# Helpers
##############################################################################


def generate_launch_nodes() -> typing.List[launch_ros.actions.Node]:
    """
    Generate an action node for launch.

    Returns:
        a list of the mock robot ros nodes as actions for launch
    """
    launch_nodes = []
    for node_name in ['battery', 'dashboard', 'docking_controller',
                      'led_strip', 'move_base', 'rotation_controller',
                      'safety_sensors']:
        executable = "mock-{}".format(node_name.replace('_', '-'))
        launch_nodes.append(
            launch_ros.actions.Node(
                package='py_trees_ros_tutorials',
                name=node_name,
                executable=executable,
                output='screen',
                emulate_tty=True
            )
        )
    launch_nodes.append(
        launch.actions.LogInfo(msg=["Bob the robot, at your service. Need a colander?"])
    )
    return launch_nodes


def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch the mock robot (i.e. launch all mocked components).

    Returns:
        the launch description
    """

    return launch.LaunchDescription(generate_launch_nodes())
