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

This tutorial inserts a context switching behaviour to run in tandem with the
scan rotation. A context switching behaviour will alter the runtime system
in some way when it is entered (i.e. in :meth:`~py_trees.behaviour.Behaviour.initialise`)
and reset the runtime system to it's original context
on :meth:`~py_trees.behaviour.Behaviour.terminate`). Refer to :term:`context switch`
for more detail.

In this example it will enable a hypothetical safety sensor pipeline, necessary
necessary for dangerous but slow moving rotational maneuvres not required for
normal modes of travel (suppose we have a large rectangular robot that is
ordinarily blind to the sides - it may need to take advantage of noisy
sonars to the sides or rotate forward facing sensing into position before
engaging).


Tree
^^^^

.. code-block:: bash

   $ py-trees-render py_trees_ros_tutorials.six_context_switching.tutorial_create_root

.. graphviz:: dot/tutorial-six-context-switching.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/six_context_switching.py
   :language: python
   :linenos:
   :lines: 123-215
   :caption: six_context_switching.py#tutorial_create_root

Behaviour
---------

The :class:`py_trees_ros_tutorials.behaviours.ScanContext` is the
context switching behaviour constructed for this tutorial.

* :meth:`~py_trees_ros_tutorials.behaviours.ScanContext.initialise()`: trigger a sequence service calls to cache and set the /safety_sensors/enabled parameter to True
* :meth:`~py_trees_ros_tutorials.behaviours.ScanContext.update()`: complete the chain of service calls & maintain the context
* :meth:`~py_trees_ros_tutorials.behaviours.ScanContext.terminate()`: reset the parameter to the cached value


Context Switching
-----------------

.. graphviz:: dot/tutorial-six-context-switching-subtree.dot
   :align: center

On entry into the parallel, the :class:`~py_trees_ros_tutorials.behaviours.ScanContext`
behaviour will cache and switch
the safety sensors parameter. While in the parallel it will return with
:data:`~py_trees.common.Status.RUNNING` indefinitely. When the rotation
action succeeds or fails, it will terminate the parallel and subsequently
the :class:`~py_trees_ros_tutorials.behaviours.ScanContext` will terminate,
resetting the safety sensors parameter to it's original value.

Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 run py_trees_ros_tutorials tutorial-six-context-switching
    # In another shell, watch the parameter as a context switch occurs
    $ watch -n 1 ros2 param get /safety_sensors enabled
    # Trigger scan requests from the qt dashboard

.. image:: images/tutorial-six-context-switching.png
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import rclpy
import sys

from . import behaviours
from . import mock
from . import utilities

##############################################################################
# Launcher
##############################################################################


def launch_main():
    """
    A rosrunnable launch for the tutorial.
    """
    launch_descriptions = []
    launch_descriptions.append(mock.launch.generate_launch_description())
    launch_descriptions.append(utilities.generate_tree_launch_description("tree-context-switching"))
    launch_service = utilities.generate_ros_launch_service(
        launch_descriptions=launch_descriptions,
        debug=False
    )
    return launch_service.run()

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Insert a task between battery emergency and idle behaviours that
    controls a rotation action controller and notifications simultaenously
    to scan a room.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial Six",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        variable_name="event_scan_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks")
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )

    # Emergency Tasks
    def check_battery_low_on_blackboard():
        blackboard = py_trees.blackboard.Blackboard()
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        child=flash_red
    )
    # Worker Tasks
    scan = py_trees.composites.Sequence(name="Scan")
    is_scan_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Scan?",
        variable_name='event_scan_button',
        expected_value=True
    )
    scan_preempt = py_trees.composites.Selector(name="Preempt?")
    is_scan_requested_two = py_trees.decorators.SuccessIsRunning(
        name="SuccessIsRunning",
        child=py_trees.blackboard.CheckBlackboardVariable(
            name="Scan?",
            variable_name='event_scan_button',
            expected_value=True
        )
    )
    scanning = py_trees.composites.Parallel(
        name="Scanning",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    scan_context_switch = behaviours.ScanContext("Context Switch")
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_type=py_trees_actions.Rotate,
        action_name="rotate",
        action_goal=py_trees_actions.Rotate.Goal(),
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
    )
    flash_blue = behaviours.FlashLedStrip(
        name="Flash Blue",
        colour="blue"
    )
    scan_celebrate = py_trees.composites.Parallel(
        name="Celebrate",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    flash_green = behaviours.FlashLedStrip(name="Flash Green", colour="green")
    scan_pause = py_trees.timers.Timer("Pause", duration=3.0)
    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, scan, idle])
    scan.add_children([is_scan_requested, scan_preempt, scan_celebrate])
    scan_preempt.add_children([is_scan_requested_two, scanning])
    scanning.add_children([scan_context_switch, scan_rotate, flash_blue])
    scan_celebrate.add_children([flash_green, scan_pause])
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
