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
import py_trees.console as console
from typing import List

##############################################################################
# Methods
##############################################################################


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
