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

This tutorial adds additional complexity to the scanning application in order to
introduce a few patterns typical of most applications - cancellations, recovery
and result handling.

Specifically, there is now an undocking-move combination pre-scanning and
a move-docking combination post-scanning. When cancelling, the robot should
recover it's initial state so it is ready to accept future requests. In this
case, the robot must move home and dock, even when cancelled.

Additionally, the application should report out on it's result upon completion.

.. note::

    Preemption has been dropped from the application for simplicity. It could
    be reinserted, but care would be required to handle undocking and docking
    appropriately.

Tree
^^^^

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_tutorials.seven_docking_cancelling_failing.tutorial_create_root

.. graphviz:: dot/tutorial-seven-docking-cancelling-failing.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/seven_docking_cancelling_failing.py
   :language: python
   :linenos:
   :lines: 211-398
   :caption: seven_docking_cancelling_failing.py#tutorial_create_root

Succeeding
----------

.. graphviz:: dot/tutorial-seven-ere-we-go.dot
   :align: center

Assuming everything works perfectly, then the subtree will sequentially progress to completion
through undocking, move out, rotate, move home and docking actions as illustrated in the
dot graph above. However, nothing ever works perfectly, so ...

Failing
-------

.. image:: images/tutorial-seven-failure_paths.svg
   :align: center

If any step of the 'Ere we Go' sequence fails the mock robot robot will simply stop, drop
into the post-failure ('Die') subtree and commence post-failure actions. In this case
this consists of both an alarm signal (flashing red) and communication of failure to
the user (echoes to the screen, but could have been, for example, a middleware response
to the user's application).

These actions are merely post-failure notifications that would ostensibly result in
manual (human assisted) recovery of the situation. To attempt an automated recovery,
there are two options:

   1. Global Recovery - use the blackboard as a means of transferring information about the
      failure from the relevant behaviour (UnDock, Move Out, Move Home, Dock) to the
      post-failure subtree. Introspect the data and determine the right course of action in
      the post-failure subtree.

   2. Local Recovery - use a selector with each of the individual behaviours to immediately
      generate a recovery subtree specifically adapted to the behaviour that failed. This
      recovery subtree should also return :attr:`~py_trees.common.Status.FAILURE` so the
      parent sequence also returns :attr:`~py_trees.common.Status.FAILURE`. The
      'Die' subtree is then merely for common post-failure actions (e.g. notification and
      response).

The latter is technically preferable as the decision logic is entirely visible in the tree
connections, but it does cause an explosion in the scale of the tree and it's maintenance.

.. note::

   It is interesting to observe that although the application is considered to have
   failed, the 'Scan or Die' operation will return with :attr:`~py_trees.common.Status.SUCCESS`
   after which post-failure actions will kick in.
   Here, application failure is recorded in the 'Result2BB' behaviour which is later
   transmitted back to the user in the final stages of the application.

   Application failure is handled via the actions of behaviours,
   not the state of the tree.

.. tip::

   Decision logic in the tree is for routing decision making,
   not routing application failure/success, nor logical errors. Overloading
   tree decision logic with more than one purpose will constrain your
   application design to the point of non-usefulness.

Cancelling
----------

In this tutorial, the application listens continuously for cancellation requests and
will cancel the operation if it is currently between undocking and docking actions.

.. note::

   The approach demonstrated in this tutorial is simple, but sufficient as an example.
   Interactions are only one-way - from the user to the application.
   It neither prevents the user from requesting nor does it provide an informative
   response if the request is invalid (i.e. if the application is not running or already
   cancelling). It also falls short of caching and handling
   cancel requests across the entire application.
   These cases are easy to handle with additional logic in the tree - consider it
   a homework exercise :)

.. graphviz:: dot/tutorial-seven-cancel2bb.dot
   :align: center

Cancelling begins with catching incoming cancel requests:

.. image:: images/tutorial-seven-cancelling.svg
   :align: center

Cancelling is a high priority subtree, but here we make sure that the post-cancelling
workflow integrates with the non-cancelling workflow so that the robot returns to
it's initial location and state.


Results
-------

.. image:: images/tutorial-seven-result.svg
   :align: center

As noted earlier, it is typically important to keep application result logic
separate from the decision tree logic. To do so, the blackboard is used to
record the application result and an application result agnostic behaviour
is used to communicate the result back to the user in the final stage of the
application's lifecycle.


Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 launch py_trees_ros_tutorials tutorial_seven_docking_cancelling_failing_launch.py
    # In another shell
    $ py-trees-tree-watcher -b
    # Trigger scan/cancel requests from the qt dashboard

.. image:: images/tutorial-seven-docking-cancelling-failing.png
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
                executable="tree-docking-cancelling-failing",
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
        name="Tutorial Seven",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_scan_button"
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_cancel_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector(name="Tasks", memory=False)
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
    scan = py_trees.composites.Sequence(name="Scan", memory=True)
    is_scan_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Scan?",
        check=py_trees.common.ComparisonExpression(
            variable="event_scan_button",
            value=True,
            operator=operator.eq
        )
    )
    scan_or_die = py_trees.composites.Selector(name="Scan or Die", memory=False)
    die = py_trees.composites.Sequence(name="Die", memory=True)
    failed_notification = py_trees.composites.Parallel(
        name="Notification",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    failed_flash_green = behaviours.FlashLedStrip(name="Flash Red", colour="red")
    failed_pause = py_trees.timers.Timer("Pause", duration=3.0)
    result_failed_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'failed'",
        variable_name='scan_result',
        variable_value='failed',
        overwrite=True
    )
    ere_we_go = py_trees.composites.Sequence(name="Ere we Go", memory=True)
    undock = py_trees_ros.actions.ActionClient(
        name="UnDock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=False),
        generate_feedback_message=lambda msg: "undocking"
    )
    scan_or_be_cancelled = py_trees.composites.Selector(name="Scan or Be Cancelled", memory=False)
    cancelling = py_trees.composites.Sequence(name="Cancelling?", memory=True)
    is_cancel_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Cancel?",
        check=py_trees.common.ComparisonExpression(
            variable="event_cancel_button",
            value=True,
            operator=operator.eq
        )
    )
    move_home_after_cancel = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_cancelled_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'cancelled'",
        variable_name='scan_result',
        variable_value='cancelled',
        overwrite=True
    )
    move_out_and_scan = py_trees.composites.Sequence(name="Move Out and Scan", memory=True)
    move_base = py_trees_ros.actions.ActionClient(
        name="Move Out",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving out"
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
    scan_flash_blue = behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'succeeded'",
        variable_name='scan_result',
        variable_value='succeeded',
        overwrite=True
    )
    celebrate = py_trees.composites.Parallel(
        name="Celebrate",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    celebrate_flash_green = behaviours.FlashLedStrip(name="Flash Green", colour="green")
    celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)
    dock = py_trees_ros.actions.ActionClient(
        name="Dock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=True),  # noqa
        generate_feedback_message=lambda msg: "docking"
    )

    class SendResult(py_trees.behaviour.Behaviour):

        def __init__(self, name: str):
            super().__init__(name="Send Result")
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="scan_result",
                access=py_trees.common.Access.READ
            )

        def update(self):
            print(console.green +
                  "********** Result: {} **********".format(self.blackboard.scan_result) +
                  console.reset
                  )
            return py_trees.common.Status.SUCCESS

    send_result = SendResult(name="Send Result")

    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, scan, idle])
    scan.add_children([is_scan_requested, scan_or_die, send_result])
    scan_or_die.add_children([ere_we_go, die])
    die.add_children([failed_notification, result_failed_to_bb])
    failed_notification.add_children([failed_flash_green, failed_pause])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children([is_cancel_requested, move_home_after_cancel, result_cancelled_to_bb])
    move_out_and_scan.add_children([move_base, scanning, move_home_after_scan, result_succeeded_to_bb])
    scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
    celebrate.add_children([celebrate_flash_green, celebrate_pause])
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
