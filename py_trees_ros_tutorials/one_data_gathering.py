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

In this, the first of the tutorials, we start out with a behaviour that
collects battery data from a subscriber and stores the result on the
blackboard for other behaviours to utilise.

Data gathering up front via subscribers is a useful convention for
a number of reasons:

* Freeze incoming data for remaining behaviours in the tree tick so that decision making is consistent across the entire tree
* Avoid redundantly invoking multiple subscribers to the same topic when not necessary
* Python access to the blackboard is easier than ROS middleware handling

Typically data gatherers will be assembled underneath a parallel at or near
the very root of the tree so they may always trigger their update() method
and be processed before any decision making behaviours elsewhere in the tree.

Tree
^^^^

.. graphviz:: dot/tutorial-one-data-gathering.dot

.. literalinclude:: ../py_trees_ros_tutorials/one_data_gathering.py
   :language: python
   :linenos:
   :lines: 95-117
   :caption: py_trees_ros_tutorials/one_data_gathering.py#tutorial_create_root

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

    $ ros2 run py_trees_ros_tutorials tutorial-one-data-gathering

Battery updates on the blackboard:

.. image:: images/tutorial-one-blackboard.gif
"""

##############################################################################
# Imports
##############################################################################

import launch
import launch_ros.actions
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

from . import mock
from . import utilities

##############################################################################
# Launcher
##############################################################################


def generate_tree_launch_description():
    """Launch the tutorial"""
    launch_description = launch.LaunchDescription()
    launch_description.add_action(
        launch_ros.actions.Node(
            package='py_trees_ros_tutorials',
            # node_name="one", # ha, it's a multi-node process
            node_executable="tree-data-gathering",
            output='screen',
            # workaround to print to stdout till https://github.com/ros2/launch/issues/188
            # but...this fails too - https://github.com/ros2/launch/issues/203
            # env={'PYTHONUNBUFFERED': '1'}
        )
    )
    return launch_description


def launch_main():
    """A rosrunnable launch."""
    launch_descriptions = []
    launch_descriptions.append(mock.launch.generate_launch_description())
    launch_descriptions.append(generate_tree_launch_description())
    launch_service = utilities.generate_ros_launch_service(
        launch_descriptions=launch_descriptions,
        debug=False
    )
    return launch_service.run()

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        threshold=30.0
    )
    priorities = py_trees.composites.Selector("Tasks")
    idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(flipper)
    priorities.add_child(idle)
    return root


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
    try:
        tree.setup(timeout=15)
    except Exception as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()
