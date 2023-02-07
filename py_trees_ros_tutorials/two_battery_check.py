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

Here we add the first decision. What to do if the battery is low? For this,
we’ll get the mocked robot to flash a notification over it’s led strip.

Tree
^^^^

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_tutorials.two_battery_check.tutorial_create_root

.. graphviz:: dot/tutorial-two-battery-check.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/two_battery_check.py
   :language: python
   :linenos:
   :lines: 122-166
   :caption: two_battery_check.py#tutorial_create_root

Here we’ve added a high priority branch for dealing with a low battery
that causes the hardware strip to flash. The :class:`py_trees.decorators.EternalGuard`
enables a continuous check of the battery reading and subsequent termination of
the flashing strip as soon as the battery level has recovered sufficiently.
We could have equivalently made use of the :class:`py_trees.idioms.eternal_guard` idiom,
which yields a more verbose, but explicit tree and would also allow direct use of
the :class:`py_trees.blackboard.CheckBlackboardVariable` class as the conditional check.

Behaviours
^^^^^^^^^^

This tree makes use of the :class:`py_trees_ros_tutorials.behaviours.FlashLedStrip` behaviour.

.. literalinclude:: ../py_trees_ros_tutorials/behaviours.py
   :language: python
   :linenos:
   :lines: 29-110
   :caption: behaviours.py#FlashLedStrip

This is a typical ROS behaviour that accepts a ROS node on setup. This delayed style is
preferred since it allows simple construction of the behaviour, in a tree, sans all of the
ROS plumbing - useful when rendering dot graphs of the tree without having a ROS runtime
around.

The rest of the behaviour too, is fairly conventional:

* ROS plumbing (i.e. the publisher) instantiated in setup()
* Flashing notifications published in update()
* The reset notification published when the behaviour is terminated

Running
^^^^^^^

.. code-block:: bash

    $ ros2 launch py_trees_ros_tutorials tutorial_two_battery_check_launch.py

Then play with the battery slider in the qt dashboard to trigger the decision
branching in the tree.

.. image:: images/tutorial-two-battery-check.png
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

from . import behaviours
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
                executable="tree-battery-check",
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
    Create a basic tree with a battery to blackboard writer and a
    battery check that flashes the LEDs on the mock robot if the
    battery level goes low.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial Two",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks", memory=False)
    flash_led_strip = behaviours.FlashLedStrip(
        name="FlashLEDs",
        colour="red"
    )

    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_led_strip
    )
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(tasks)
    tasks.add_children([battery_emergency, idle])
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
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
