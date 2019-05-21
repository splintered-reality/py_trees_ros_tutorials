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

This tutorial inserts a task between emergency and fallback (idle)
behaviours to perform some actual work - rotate 360 degrees in place to
scan a room whilst simultaneously notifying the user (via flashing led strip)
of it's actions. The task is triggered from the qt dashboard.

The rotation is performed with a ROS action. Actions are almost the defacto
means of interfacing with the control systems of
ROS robots. Here we introduces the :class:`py_trees_ros.actions.ActionClient`
behaviour - a simple means of sequentially interacting with an action server such
that a goal always executes to completion or is cancelled before another
goal is sent (a client-side kind of preemption).

It also demonstrates the value of coordinating subsystems from the behaviour tree.
In this case, both action controllers and notification subsystems are
managed from the tree to perform a task. This frees control subsystems from
having to be dependent on each other and simultaneously aware of higher level
application logic. Instead, the application logic is centralised in one place,
the tree, where it can be easily monitored, logged, and reconstructed in
a slightly different form for another application without requiring changes
in the underlying control subsystems.

Tree
^^^^

.. code-block:: bash

   $ py-trees-render py_trees_ros_tutorials.five_action_clients.tutorial_create_root

.. graphviz:: dot/tutorial-five-action-clients.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/five_action_clients.py
   :language: python
   :linenos:
   :lines: 115-158
   :caption: five_action_clients.py#tutorial_create_root

A few things to note here:

* A :term:`data gathering` behaviour (Scan2BB) collects incoming requests from the qt dashboard
* The scanning behaviour is a subtree inserted between battery low and idel behaviours.
* Scan requests cause the scanning subtree to go live so long as the battery is not low
* Scanning will be interrupted if the battery should go low

More details on the action client and the nature of it's client side preemptions below.

Action Client
-------------

This tree makes use of the :class:`py_trees_ros.actions.ActionClient` behaviour.
Key characteristics of this behaviour include:

* Goal details are supplied at construction time and cannot be changed thereafter
* New goals are sent on `initialise()`
* Monitoring of feedback and result response occurs in `update()`
* If the behaviour is interrupted, the goal will be cancelled in `terminate()`

This obviously places constraints on it's usage. In particular, goal details
cannot be assembled dynamically/elsewhere, nor can it send a new goal while
a preceding goal is still active - the behaviour lifecycle forces it through
`terminate()` before a new goal can be sent.

These constraints however, are fine in most situations and result in a
very simple behaviour that almost always does what you need without
perspiring inordinately on tree design ramifications.

.. note:

    If you are looking for more complex logic, e.g. enabling interactions with
    a manipulation action server with which you would like to leave preemptions
    up to the server, then this will require either decomposing the separate parts
    of this action client behaviour (separate send goal, from monitoring, from
    cancelling) or a more intelligent behaviour that does all the things itself
    (please send a PR if you have such a behaviour serving your purposes well).

Instantiatinog the action client, configured for rotations:

.. literalinclude:: ../py_trees_ros/tutorials/five.py
   :language: python
   :linenos:
   :lines: 158-163
   :caption: py_trees_ros_tutorials/five_actino_clients.py#instantiation

Guards
------

.. graphviz:: dot/tutorial-five-guard.dot

The entire scan branch is protected by a :term:`guard` (note that the blackbox
in the above diagram is exactly that, a black box representing the lower
part of the tree). Once the scan event is received, this branch gets to work
until it either finishes, or is pre-empted by the higher priority low battery
branch.

A Kind of Preemption
--------------------

.. graphviz:: dot/tutorial-five-preempt.dot

The second part of the tree enables a kind of pre-emption on the scanning action.
If a new request comes in, it will trigger the secondary scan event check, invalidating
whatever scanning action was currently running. This will clear the led command and
cancel the rotate action. On the next tick, the scan event check will fail (it was
consumed on the last tick) and the scanning will restart.
.. note::
    This is not true pre-emption since it cancels the rotate action and restarts it. It is
    however, exactly the pattern that is required in many instances. For true pre-emption
    you could bundle both scan check and rotation action in the same behaviour or dynamically
    insert action goals on the fly from the parent class.

Handling Failure
----------------

If the rotate action should fail, then the whole branch will also fail. Subsequently
dropping the robot back to its idle state. A failure event could be generated by
simply watching either the 'Scanning' parallel or the :meth:`~py_trees.trees.BehaviourTree.tip`
of the tree and reacting to it's state change.

Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 run py_trees_ros_tutorials tutorial-five-action-clients

Send a scan request(s) from the qt dashboard.

.. image:: images/tutorial-five-action-clients.png
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
    launch_descriptions.append(utilities.generate_tree_launch_description("tree-action-clients"))
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
        name="Tutorial Five",
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
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_type=py_trees_actions.Rotate,
        action_name="rotate",
        action_goal=py_trees_actions.Rotate.Goal(),
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.percentage_completed)
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
    scanning.add_children([scan_rotate, flash_blue])
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
