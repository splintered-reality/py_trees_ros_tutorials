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

   $ py-trees-render py_trees_ros_tutorials.seven_docking_cancelling_failing.tutorial_create_root

.. graphviz:: dot/tutorial-seven-docking-cancelling-failing.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/seven_docking_cancelling_failing.py
   :language: python
   :linenos:
   :lines: 123-215
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
    $ ros2 run py_trees_ros_tutorials tutorial-seven-docking-cancelling-failing
    # In another shell, watch the parameter as a context switch occurs
    # Trigger scan/cancel requests from the qt dashboard

.. image:: images/tutorial-seven-docking-cancelling-failing.png
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rclpy
import std_msgs.msg as std_msgs
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
    launch_descriptions.append(
        utilities.generate_tree_launch_description(
            "tree-dynamic-job-handling"
        )
    )
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
        name="Tutorial Eight",
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
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        variable_name="event_cancel_button"
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
    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, idle])
    return root


def tutorial_create_scan_subtree() -> py_trees.behaviour.Behaviour:
    """
    Create the job subtree based on the incoming goal specification.
    Args:
        goal (:class:`~std_msgs.msg.Empty`): incoming goal specification
    Returns:
       :class:`~py_trees.behaviour.Behaviour`: subtree root
    """
    # behaviours
    scan = py_trees.composites.Sequence(name="Scan")
    is_scan_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Scan?",
        variable_name='event_scan_button',
        expected_value=True
    )
    scan_or_die = py_trees.composites.Selector(name="Scan or Die")
    die = py_trees.composites.Sequence(name="Die")
    failed_notification = py_trees.composites.Parallel(
        name="Notification",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    failed_flash_green = behaviours.FlashLedStrip(name="Flash Red", colour="red")
    failed_pause = py_trees.timers.Timer("Pause", duration=3.0)
    result_failed_to_bb = py_trees.blackboard.SetBlackboardVariable(
        name="Result2BB\n'failed'",
        variable_name='scan_result',
        variable_value='failed'
    )
    ere_we_go = py_trees.composites.Sequence(name="Ere we Go")
    undock = py_trees_ros.actions.ActionClient(
        name="UnDock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=False),
        generate_feedback_message=lambda msg: "undocking"
    )
    scan_or_be_cancelled = py_trees.composites.Selector("Scan or Be Cancelled")
    cancelling = py_trees.composites.Sequence("Cancelling?")
    is_cancel_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Cancel?",
        variable_name='event_cancel_button',
        expected_value=True
    )
    move_home_after_cancel = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_cancelled_to_bb = py_trees.blackboard.SetBlackboardVariable(
        name="Result2BB\n'cancelled'",
        variable_name='scan_result',
        variable_value='cancelled'
    )
    move_out_and_scan = py_trees.composites.Sequence("Move Out and Scan")
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
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.percentage_completed)
    )
    scan_flash_blue = behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_succeeded_to_bb = py_trees.blackboard.SetBlackboardVariable(
        name="Result2BB\n'succeeded'",
        variable_name='scan_result',
        variable_value='succeeded'
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
        action_goal=py_trees_actions.Dock.Goal(dock=True),
        generate_feedback_message=lambda msg: "docking"
    )

    def send_result_to_screen(self):
        blackboard = py_trees.blackboard.Blackboard()
        print(console.green +
              "********** Result: {} **********".format(blackboard.scan_result) +
              console.reset
              )
        return py_trees.common.Status.SUCCESS

    send_result = py_trees.behaviours.meta.create_behaviour_from_function(send_result_to_screen)(
        name="Send Result"
    )
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
    return scan


class DynamicJobHandlingTree(py_trees_ros.trees.BehaviourTree):
    """
    Wraps the ROS behaviour tree manager in a class that manages loading
    and unloading of jobs.
    """

    def __init__(self):
        super().__init__(
            root=tutorial_create_root(),
            unicode_tree_debug=True
        )
        self.add_post_tick_handler(
            self.prune_application_subtree_if_done)

    def setup(self, timeout: float):
        """
        Redirect the setup function"
        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        super().setup(timeout=15)
        self._report_service = self.node.create_service(
            srv_type=py_trees_srvs.StatusReport,
            srv_name="~/report",
            callback=self.deliver_status_report
        )
        self._job_subscriber = self.node.create_subscription(
            msg_type=std_msgs.Empty,
            topic="/dashboard/scan",
            callback=self.receive_incoming_job
        )

    def receive_incoming_job(self, msg: std_msgs.Empty):
        """
        Incoming job callback.

        Args:
            msg: incoming goal message

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
        """
        if self.busy():
            self._node.get_logger().info("rejecting new job, last job is still active")
        else:
            scan_subtree = tutorial_create_scan_subtree()
            try:
                py_trees.trees.setup(
                    root=scan_subtree,
                    node=self.node
                )
            except Exception as e:
                console.logerror(console.red + "failed to setup the scan subtree, aborting [{}]".format(str(e)) + console.reset)
                sys.exit(1)
            self.insert_subtree(scan_subtree, self.priorities.id, 1)
            self.node.get_logger().info("inserted job subtree")

    def deliver_status_report(
            self,
            unused_request: py_trees_srvs.StatusReport.Request,
            response: py_trees_srvs.StatusReport.Response
         ):
        """
        Prepare a status report for an external service client.

        Args:
            unused_request: empty request message
        """
        # last result value or none
        last_result = self.blackboard_exchange.blackboard.get(name="scan_result")
        if self.busy():
            response.report = "executing"
        elif self.root.tip().has_parent_with_name("Battery Emergency"):
            response.report = "battery [last result: {}]".format(last_result)
        else:
            response.report = "idle [last result: {}]".format(last_result)
        return response

    def prune_application_subtree_if_done(self, tree):
        """
        Check if a job is running and if it has finished. If so, prune the job subtree from the tree.
        Additionally, make a status report upon introspection of the tree.
        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        # executing
        if self.busy():
            job = self.priorities.children[-2]
            # finished
            if job.status == py_trees.common.Status.SUCCESS or job.status == py_trees.common.Status.FAILURE:
                self.node.get_logger().info("{0}: finished [{1}]".format(job.name, job.status))
                tree.prune_subtree(job.id)

    def busy(self):
        """
        Check if a job subtree exists and is running. Only one job is permitted at
        a time, so it is sufficient to just check that the priority task selector
        is of length three (note: there is always emergency and idle tasks
        alongside the active job). When the job is not active, it is
        pruned from the tree, leaving just two prioritised tasks (emergency and idle).

        Returns:
            :obj:`bool`: whether it is busy with a job subtree or not
        """
        return len(self.priorities.children) == 3

    @property
    def priorities(self) -> py_trees.composites.Selector:
        """
        Returns the composite (:class:`~py_trees.composites.Selector`) that is
        home to the prioritised list of tasks.
        """
        return self.root.children[-1]


def tutorial_main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    tree = DynamicJobHandlingTree()
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
