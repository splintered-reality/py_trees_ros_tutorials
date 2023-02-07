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

The previous tutorial enables execution of a specific job upon
request. You will inevitably grow the functionality of the robot beyond this
and a very common use case for the trees is to switch the context of the robot
between 'applications' - calibration, tests, demos, scheduled tasks from
a fleet server, etc.

While these contexts could be entirely managed by the tree simultaneously,
the exclusivity of the applications lends itself far more easily to the following
paradigm:

1. Construct a tree on bringup for ticking over basic functionality while idling
2. Dynamically insert/prune application subtrees on demand, rejecting requests when already busy

This mirrors both the way smart phones operate (which also happens to be a reasonable
mode of operation for robots due to similar resource contention arguments) and the
conventional use of roslaunch files to bringup a core and later bootstrap / tear
down application level processes on demand.

This tutorial uses a wrapper class around :class:`py_trees_ros.trees.BehaviourTree` to handle:

1. Construction of the core tree
2. A job (application) request callback
3. Insertion of the application subtree in the request callback (if not busy)
4. Pruning of the application subtree in a post-tick handler (if finished)
5. A status report service for external clients of the tree

.. note::

    Only the basics are demonstrated here, but you could imagine extensions
    to this class that would make it truly useful in an application driven robotics
    system - abstractions so application modules need not be known in advance,
    application subtrees delivered as python code, more
    detailed tree introspection in status reports (given it's responsibility
    to be the decision making engine for the robot, it is the best snapshot of the
    robot's current activity). You're only limited by your imagination!

Core Tree (Dot Graph)
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_tutorials.eight_dynamic_application_loading.tutorial_create_root

.. graphviz:: dot/tutorial-eight-core-tree.dot
   :align: center
   :caption: py_trees_ros_tutorials.eight_dynamic_application_loading.tutorial_create_root

Application Subtree (Dot Graph)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   $ py-trees-render --with-blackboard-variables py_trees_ros_tutorials.eight_dynamic_application_loading.tutorial_create_scan_subtree

.. graphviz:: dot/tutorial-eight-application-subtree.dot
   :align: center
   :caption: py_trees_ros_tutorials.eight_dynamic_application_loading.tutorial_create_scan_subtree

Dynamic Application Tree (Class)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../py_trees_ros_tutorials/eight_dynamic_application_loading.py
   :language: python
   :linenos:
   :lines: 380-384
   :caption: Dynamic Application Tree

.. literalinclude:: ../py_trees_ros_tutorials/eight_dynamic_application_loading.py
   :language: python
   :linenos:
   :lines: 386-397
   :caption: Init - Create the Root Tree

.. literalinclude:: ../py_trees_ros_tutorials/eight_dynamic_application_loading.py
   :language: python
   :linenos:
   :lines: 399-419
   :caption: Setup - Application Subscribers & Services

.. literalinclude:: ../py_trees_ros_tutorials/eight_dynamic_application_loading.py
   :language: python
   :linenos:
   :lines: 421-444
   :caption: Requests - Inserting Application Subtrees

.. literalinclude:: ../py_trees_ros_tutorials/eight_dynamic_application_loading.py
   :language: python
   :linenos:
   :lines: 467-482
   :caption: Post-Execution - Pruning Application Subtrees

.. literalinclude:: ../py_trees_ros_tutorials/eight_dynamic_application_loading.py
   :language: python
   :linenos:
   :lines: 445-465
   :caption: Status Reports

.. note::

   In the code above, there is a conspicuous absence of thread locks. This is
   possible due to the use of ROS2's single threaded executors to handle service and
   subscriber callbacks along with the tree's tick tock that operates from within
   ROS2 timer callbacks. If using a behaviour tree, as is exemplified here,
   to handle robot application logic, you should never need to go beyond single
   threaded execution and thus avoid the complexity and bugs that come along with
   having to handle concurrency (this is a considerable improvement on the situation
   for ROS1).

Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 launch py_trees_ros_tutorials tutorial_eight_dynamic_application_loading_launch.py
    # In another shell, catch the tree snapshots
    $ py-trees-tree-watcher -b
    # Trigger scan/cancel requests from the qt dashboard

.. image:: images/tutorial-eight-dynamic-application-loading.png
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
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rclpy
import std_msgs.msg as std_msgs

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
                executable="tree-dynamic-application-loading",
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
        name="Tutorial Eight",
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
    scan = py_trees.composites.Sequence(name="Scan", memory=True)
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
        action_goal=py_trees_actions.Dock.Goal(dock=False),  # noqa
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
        action_goal=py_trees_actions.MoveBase.Goal(),  # noqa
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
        action_goal=py_trees_actions.MoveBase.Goal(),  # noqa
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
        action_goal=py_trees_actions.Rotate.Goal(),  # noqa
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
    )
    scan_flash_blue = behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),  # noqa
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

    scan.add_children([scan_or_die, send_result])
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


class DynamicApplicationTree(py_trees_ros.trees.BehaviourTree):
    """
    Wraps the ROS behaviour tree manager in a class that manages loading
    and unloading of jobs.
    """

    def __init__(self):
        """
        Create the core tree and add post tick handlers for post-execution
        management of the tree.
        """
        super().__init__(
            root=tutorial_create_root(),
            unicode_tree_debug=True
        )
        self.add_post_tick_handler(
            self.prune_application_subtree_if_done
        )

    def setup(self, timeout: float):
        """
        Setup the tree and connect additional application management / status
        report subscribers and services.

        Args:
            timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
        """
        super().setup(timeout=timeout)
        self._report_service = self.node.create_service(
            srv_type=py_trees_srvs.StatusReport,
            srv_name="~/report",
            callback=self.deliver_status_report,
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        self._job_subscriber = self.node.create_subscription(
            msg_type=std_msgs.Empty,
            topic="/dashboard/scan",
            callback=self.receive_incoming_job,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
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
            self.node.get_logger().warning("rejecting new job, last job is still active")
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
            unused_request: py_trees_srvs.StatusReport.Request,  # noqa
            response: py_trees_srvs.StatusReport.Response  # noqa
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
                for node in job.iterate():
                    node.shutdown()
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
    tree = DynamicApplicationTree()
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
