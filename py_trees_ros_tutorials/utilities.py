#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Utilities for the tutorials and launchers
"""

##############################################################################
# Imports
##############################################################################

import launch
import launch_ros
import os
import py_trees.console as console
from typing import List

##############################################################################
# Methods
##############################################################################


def generate_tree_launch_description(runnable: str) -> launch.LaunchDescription:
    """
    Generate a single launch description for the specified tree script.

    Args:
        runnable: name of the console script to launch

    Returns:
        the single launch description
    """
    python_unbuffered_environment = os.environ.copy()
    python_unbuffered_environment['PYTHONUNBUFFERED'] = '1'

    launch_description = launch.LaunchDescription()
    launch_description.add_action(
        launch_ros.actions.Node(
            package='py_trees_ros_tutorials',
            # node_name="one", # ha, it's a multi-node process
            node_executable=runnable,
            output='screen',
            # workaround to print to stdout till https://github.com/ros2/launch/issues/188
            # but...this fails too - https://github.com/ros2/launch/issues/203
            # env=python_unbuffered_environment
        )
    )
    return launch_description


def generate_ros_launch_service(
        launch_descriptions: List[launch.LaunchDescription],
        debug: bool
     ) -> launch.LaunchService:
    """
    Convenience method that Assembles a ros-style launch service so that
    standalone launchers can be executed via ros2 run.

    Args:
        launch_descriptions: launch descriptions to include
        bool: enable debugging on the launch service

    Returns:
        :class:`launch.LaunchService`: service to execute
    """
    print('')
    print(console.green + 'Introspection' + console.reset)
    print('')

    for launch_description in launch_descriptions:
        print(launch.LaunchIntrospector().format_launch_description(launch_description))

    print('')
    print(console.green + 'Launch' + console.reset)
    print('')

    launch_service = launch.LaunchService(debug=debug)
    launch_service.include_launch_description(
        launch_ros.get_default_launch_description(
            prefix_output_with_name=False
        )
    )
    for launch_description in launch_descriptions:
        launch_service.include_launch_description(launch_description)

    im_froody = launch.LaunchDescription()
    im_froody.add_action(
        launch.actions.LogInfo(msg=["I'm froody, you should be too."])
    )
    launch_service.include_launch_description(im_froody)
    return launch_service
