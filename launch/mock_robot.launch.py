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
import launch_ros.actions

##############################################################################
# Helpers
##############################################################################


##############################################################################
# Main
##############################################################################

def generate_launch_description():
    """Launch the mock robot."""
    launch_description = launch_ros.get_default_launch_description()
    launch_description.add_action(
        launch_ros.actions.Node(
            package='py_trees_ros_tutorials', node_executable='mock-dashboard', output='screen',
            node_name='dashboard')
    )
    launch_description.add_action(
        launch_ros.actions.Node(
            package='py_trees_ros_tutorials', node_executable='mock-led-strip',  output='screen',  # screen is awkward, it's after the fact
            node_name='led_strip')
    )
    print("Dude")
    return launch_description
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='py_trees_ros_tutorials', node_executable='mock-dashboard', output='screen',
#             node_name='dashboard'),
#         launch_ros.actions.Node(
#             package='py_trees_ros_tutorials', node_executable='mock-led-strip',  output='screen',  # screen is awkward, it's after the fact
#             node_name='led_strip'),
#     ])
