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
Launch the mock robot.
"""
##############################################################################
# Imports
##############################################################################

import launch
# import launch.actions
# import launch.substitutions
import launch_ros.actions
import py_trees.console as console

##############################################################################
# Helpers
##############################################################################


def generate_launch_description():
    """Launch the mock robot."""

    launch_description = launch.LaunchDescription()

    ##########################################################################
    # Mock Robot Nodes
    ##########################################################################
    for node_name in ['battery', 'dashboard', 'docking_controller',
                      'led_strip', 'move_base', 'rotation_controller',
                      'safety_sensors']:
        node_executable = "mock-{}".format(node_name.replace('_', '-'))
        launch_description.add_action(
            launch_ros.actions.Node(
                package='py_trees_ros_tutorials',
                node_name=node_name,
                node_executable=node_executable,
                output='screen'  # screen is awkward, it's after the fact
            )
        )
    launch_description.add_action(
        launch.actions.LogInfo(msg=["I'm froody, you should be too."]),
    )
    return launch_description

##############################################################################
# Main
##############################################################################


def main():
    """Inspired by launch_ros/examples"""
    launch_description = generate_launch_description()

    print('')
    print(console.green + 'Introspection' + console.reset)
    print('')

    print(launch.LaunchIntrospector().format_launch_description(launch_description))

    print('')
    print(console.green + 'Launch' + console.reset)
    print('')

    # ls = LaunchService(debug=True)
    ls = launch.LaunchService()
    ls.include_launch_description(
        launch_ros.get_default_launch_description(
            prefix_output_with_name=False
        )
    )
    ls.include_launch_description(launch_description)
    return ls.run()
