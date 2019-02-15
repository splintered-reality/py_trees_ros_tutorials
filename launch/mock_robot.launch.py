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

import functools
import launch
import launch.event_handlers
import launch_ros
import launch_ros.actions

##############################################################################
# Helpers
##############################################################################


##############################################################################
# Main
##############################################################################


def generate_launch_description():
    """Launch the mock robot."""
    # launch_description = launch_ros.get_default_launch_description()  # broken

    launch_description = launch.LaunchDescription()

    ##########################################################################
    # Default ROS Launch Description Components
    #   launch_ros.default_launch_description.get_default_launch_description()
    ##########################################################################
    # BROKEN!
    # launch_description.add_action(
    #     launch_ros.default_launch_description.ROSSpecificLaunchStartup()
    # )
    launch_description.add_action(
        # Handle process starts.
        launch.actions.RegisterEventHandler(launch.EventHandler(
            matcher=lambda event: isinstance(event, launch.events.process.ProcessStarted),
            entities=[launch.actions.OpaqueFunction(
                function=launch_ros.default_launch_description._on_process_started)
            ],
        ))
    )
    # Add default handler for output from processes.
    prefix_output_with_name = False
    launch_description.add_action(
        launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
            on_stdout=functools.partial(
                launch_ros.default_launch_description._on_process_output,
                file_name='stdout', prefix_output=prefix_output_with_name),
            on_stderr=functools.partial(
                launch_ros.default_launch_description._on_process_output,
                file_name='stderr',
                prefix_output=prefix_output_with_name),
        ))
    )
    # Handle process exit.
    launch_description.add_action(
        launch.actions.RegisterEventHandler(launch.EventHandler(
            matcher=lambda event: isinstance(event, launch.events.process.ProcessExited),
            entities=[launch.actions.OpaqueFunction(
                function=launch_ros.default_launch_description._on_process_exited)
            ]
        ))
    )

    ##########################################################################
    # Mock Robot Nodes
    ##########################################################################
    for node_name in ['battery', 'dashboard', 'led_strip']:
        node_executable = "mock-{}".format(node_name.replace('_', '-'))
        launch_description.add_action(
            launch_ros.actions.Node(
                package='py_trees_ros_tutorials',
                node_name=node_name,
                node_executable=node_executable,
                output='screen'  # screen is awkward, it's after the fact
            )
        )
    return launch_description
