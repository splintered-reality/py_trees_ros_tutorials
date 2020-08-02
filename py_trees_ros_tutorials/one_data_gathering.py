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

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_tutorials.one_data_gathering.tutorial_create_root

.. graphviz:: dot/tutorial-one-data-gathering.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/one_data_gathering.py
   :language: python
   :linenos:
   :lines: 121-153
   :caption: one_data_gathering.py#tutorial_create_root

Along with the data gathering side, you'll also notice the dummy branch for
priority jobs (complete with idle behaviour that is always
:attr:`~py_trees.common.Status.RUNNING`). This configuration is typical
of the :term:`data gathering` pattern.

Behaviours
^^^^^^^^^^

The tree makes use of the :class:`py_trees_ros.battery.ToBlackboard` behaviour.

This behaviour will cause the entire tree will tick over with
:attr:`~py_trees.common.Status.SUCCESS` so long as there is data incoming.
If there is no data incoming, it will simply
:term:`block` and prevent the rest of the tree from acting.


Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 launch py_trees_ros_tutorials tutorial_one_data_gathering_launch.py
    # In a different shell, introspect the entire blackboard
    $ py-trees-blackboard-watcher
    # Or selectively get the battery percentage
    $ py-trees-blackboard-watcher --list
    $ py-trees-blackboard-watcher /battery.percentage

.. image:: images/tutorial-one-data-gathering.gif
"""

##############################################################################
# Imports
##############################################################################

import launch
import launch_ros
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
    """
    Launcher for the tutorial.

    Returns:
        the launch description
    """
    return launch.LaunchDescription(
        mock.launch.generate_launch_nodes() +
        [
            launch_ros.actions.Node(
                package='py_trees_ros_tutorials',
                executable="tree-data-gathering",
                output='screen',
                emulate_tty=True,
            )
        ]
    )

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial One",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
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
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()
