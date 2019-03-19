#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import launch
import launch_ros.actions


def main():
    launch_description = launch.LaunchDescription()
    launch_description.add_action(
        launch_ros.actions.Node(
            package='py_trees_ros_tutorials',
            node_name="one",
            node_executable="testies",
            output='screen',
        )
    )
    ls = launch.LaunchService()
    ls.include_launch_description(
        launch_ros.get_default_launch_description(
            prefix_output_with_name=False
        )
    )
    ls.include_launch_description(launch_description)
    return ls.run()
