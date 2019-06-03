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
Job subtrees and handlers for the tutorials.
"""
##############################################################################
# Imports
##############################################################################

import copy
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.msg as py_trees_msgs  # noqa
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros
import rclpy
import std_msgs.msg as std_msgs

from typing import Union

from . import behaviours

##############################################################################
# Classes
##############################################################################


class Scan(object):
    """
    A job handler for the scanning application.
    """

    def __init__(self):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        self._goal = None
        self._root = None

    def initialise(self, node: rclpy.node.Node):
        """
        Initialise ros communications.
        """
        self._node = node
        self._subscriber = node.create_subscription(
            msg_type=std_msgs.Empty,
            topic="/dashboard/scan",
            callback=self.receive_incoming_job
        )

    @property
    def goal(self) -> Union(std_msgs.Empty, None):
        """
        Will return a goal if there is one active, otherwise None.
        """
        with self._lock:
            g = copy.copy(self._goal) or self._goal
        return g

    def receive_incoming_job(self, msg):
        """
        Incoming goal callback.
        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().info("Scan: rejecting new goal, last goal is still active")
        else:
            self.goal = msg

    def create_report_string(self, root):
        """
        Introspect the tree to determine an appropriate human readable status report string.
        Args:
            subtree_root (:class:`~py_trees.behaviour.Behaviour`): introspect the subtree
        Returns:
            :obj:`str`: human readable substring
        """
        if root.tip().has_parent_with_name("Cancelling?"):
            return "cancelling"
        else:
            return "scanning"

    def create_subtree(self, goal=std_msgs.Empty()):
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

        self._root = scan
        return scan
