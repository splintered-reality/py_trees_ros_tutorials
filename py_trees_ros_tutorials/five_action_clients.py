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

This tutorial inserts a task between emergency and fallback (idle)
behaviours to perform some actual work - rotate 360 degrees in place to
scan a room whilst simultaneously notifying the user (via flashing led strip)
of it's actions. The task is triggered from the qt dashboard.

The rotation is performed with a ROS action which are almost the defacto
means of interfacing with the control systems of ROS robots.
Here we introduce the :class:`py_trees_ros.actions.ActionClient`
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

   $ py-trees-render -b py_trees_ros_tutorials.five_action_clients.tutorial_create_root

.. graphviz:: dot/tutorial-five-action-clients.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/five_action_clients.py
   :language: python
   :linenos:
   :lines: 208-307
   :caption: five_action_clients.py#tutorial_create_root

Data Gathering
--------------

.. graphviz:: dot/tutorial-five-data-gathering.dot
   :align: center

The Scan2BB behaviour collects incoming requests from the qt dashboard and drops them
onto the blackboard. This is your usual :class:`py_trees_ros.subscribers.EventToBlackboard`
behaviour which will only register the result `True` on the blackboard if
there was an incoming message between the last and the current tick.

The Scanning Branch
-------------------

.. graphviz:: dot/tutorial-five-scan-branch.dot
   :align: center

The entire scanning branch is protected by a :term:`guard` (the blackbox
represents the lower part of the tree) which checks the blackboard to
determine whether Scan2BB had recorded an incoming rqeuest.
Once the scan event is received, this branch proceeds to work
until it either finishes, or is pre-empted by the higher
priority low battery branch.

Action Clients
--------------

.. graphviz:: dot/tutorial-five-action-client.dot
   :align: center

This tree makes use of the :class:`py_trees_ros.actions.ActionClient`
for the 'Rotate' behaviour.

* Goal details are configured at construction and cannot be changed thereafter
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

Instantiating the action client, configured for rotations:

.. literalinclude:: ../py_trees_ros_tutorials/five_action_clients.py
   :language: python
   :linenos:
   :lines: 276-282
   :caption: five_action_clients.py#instantiate

The notification behaviour (FlashLedStrip) runs in parallel with the
action. Composing in this manner from the behaviour tree centralises
design concepts (in this case, notifications) and decouples the need
for the control subsystems to be aware each other and the application
logic.

A Kind of Preemption
--------------------

.. graphviz:: dot/tutorial-five-preemption.dot
   :align: center

The higher priority branch in the scanning action enables a kind of
pre-emption on the scanning action from the client side.
If a new request comes in, it will trigger the secondary scan event check, invalidating
whatever scanning action was currently running. This will clear the led command and
cancel the rotate action. On the next tick, the scan event check will fail (it was
consumed on the last tick) and the scanning will restart.

The decorator is used to signal farther up in the tree that the action
is still running, even when being preempted.

.. note::
    This is not true pre-emption since it cancels the rotate action and
    restarts it. It is however, exactly the pattern that is required in
    many instances. If you are looking for more complex logic, e.g.
    enabling interactions with a manipulation action server with which
    you would like to leave pre-emptions up to the server, then this
    will require either decomposing the separate parts of the action
    client behaviour (i.e. separate send goal, monitoring and
    cancelling) into separate behaviours or construct a more complex
    behaviour that manages the entire process itself. PR's welcome!

Handling Failure
----------------

If the rotate action should fail, then the whole branch will also fail,
subsequently dropping the robot back to its idle state. A failure
event could be generated by monitoring either the status of the 'Scanning'
parallel or the :meth:`py_trees.trees.BehaviourTree.tip` of the
tree and reacting to it's state change.

Running
^^^^^^^

.. code-block:: bash

    $ ros2 launch py_trees_ros_tutorials tutorial_five_action_clients_launch.py

Send scan requests from the qt dashboard.

.. image:: images/tutorial-five-action-clients.png
"""

##############################################################################
# Imports
##############################################################################

import operator
import sys

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import rclpy

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
                executable="tree-action-clients",
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
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_scan_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks")
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )

    # Emergency Tasks
    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )
    # Worker Tasks
    scan = py_trees.composites.Sequence(name="Scan")
    is_scan_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Scan?",
        check=py_trees.common.ComparisonExpression(
            variable="event_scan_button",
            value=True,
            operator=operator.eq
        )
    )
    scan_preempt = py_trees.composites.Selector(name="Preempt?")
    is_scan_requested_two = py_trees.decorators.SuccessIsRunning(
        name="SuccessIsRunning",
        child=py_trees.behaviours.CheckBlackboardVariableValue(
            name="Scan?",
            check=py_trees.common.ComparisonExpression(
                variable="event_scan_button",
                value=True,
                operator=operator.eq
            )
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
        action_goal=py_trees_actions.Rotate.Goal(),  # noqa
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
