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
About
^^^^^

In this, the first of the tutorials, we start out by using a behaviour to
collect battery data from a ros subscriber and cache the result on the
blackboard for other behaviours to utilise.

Data gathering up front via subscribers is a useful practice that can
by justified by one of a variety of reasons. In order of priority:

* Lock incoming data for the remainder of the tick so that decision making is consistent across the entire tree
* Avoid invoking the rospy threading model when it is not necessary
* Python access to the blackboard is easier than ROS api handling

In short, it takes the asynchronicity out of
the subscriber callbacks and when it comes to sharing the data, it is
far simpler to access the blackboard than to manage multiple subscribers
spread across the entire behaviour tree.

Usually you will end up with a collection
of data gatherers at the front end of your behaviour tree which will always
run and will happen before any decision branching can occur in the tree.

Tree
^^^^

.. graphviz:: dot/tutorial-one.dot

.. literalinclude:: ../py_trees_ros_tutorials/one.py
   :language: python
   :linenos:
   :lines: 95-117
   :caption: py_trees_ros_tutorials/one.py#create_root

Along with the data gathering side, you'll also notice the dummy branch for
priority jobs (complete with idle behaviour that is always
:attr:`~py_trees.common.Status.RUNNING`). This configuration is typical
of the :term:`data gathering` pattern.

Behaviours
^^^^^^^^^^

The tree makes use of the :class:`py_trees_ros.battery.ToBlackboard` behaviour.

.. literalinclude:: ../py_trees_ros/battery.py
   :language: python
   :linenos:
   :lines: 29-79
   :caption: py_trees_ros/battery.py

This behaviour will cause the entire tree will tick over with
:attr:`~py_trees.common.Status.SUCCESS` so long as there is data incoming.
If there is no data incoming, it will simply
:term:`block` and prevent the rest of the tree from acting.


Running
^^^^^^^

.. code-block:: bash

    $ roslaunch py_trees_ros tutorial_one.launch --screen

A glimpse of the blackboard with battery updates:

.. image:: images/tutorial-one-blackboard.gif
"""

##############################################################################
# Imports
##############################################################################

import functools
import launch
import launch_ros.actions
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

from . import mock

##############################################################################
# Launcher
##############################################################################


def generate_launch_description():
    """Launch the tutorial"""
    launch_description = launch.LaunchDescription()
    launch_description.add_action(
        launch_ros.actions.Node(
            package='py_trees_ros_tutorials',
            node_name="one",
            node_executable="tutorial-one-data-gathering",
            output='screen'
        )
    )
    launch_description.add_action(
        launch.actions.LogInfo(msg=["I'm froody, you should be too."]),
    )
    return launch_description


def launch_main():
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
    ls.include_launch_description(mock.launch.generate_launch_description())
    return ls.run()

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel("Tutorial")

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    # TODO: battery2bb = py_trees_ros.battery.ToBlackboard(name="Battery2BB",
    #                                                topic_name="/battery/state",
    #                                                threshold=30.0
    #                                                )
    priorities = py_trees.composites.Selector("Priorities")
    # idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(
        name="Flipper",
        n=2)

    root.add_child(topics2bb)
    # TODO: topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(flipper)
    return root


def tutorial_shutdown(tree):
    tree.interrupt()


def tutorial_main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        ascii_tree_debug=True
    )
    # TODO: rospy.on_shutdown(functools.partial(tutorial_shutdown, behaviour_tree))
    try:
        tree.setup(timeout=15)
    except Exception as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        sys.exit(1)

    timer = tree.node.create_timer(0.5, tree.tick)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.node.destroy_timer(timer)
    tree.node.destroy_node()
    rclpy.shutdown()
