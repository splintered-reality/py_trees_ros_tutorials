#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros_tutorials

if __name__ == '__main__':

    # Worker Tasks
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
    failed_notification.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
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
    scanning.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
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
    celebrate.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
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

    # Fallback task

    scan.add_children([is_scan_requested, scan_or_die, send_result])
    scan_or_die.add_children([ere_we_go, die])
    die.add_children([failed_notification, result_failed_to_bb])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children([is_cancel_requested, move_home_after_cancel, result_cancelled_to_bb])
    move_out_and_scan.add_children([move_base, scanning, move_home_after_scan, result_succeeded_to_bb])

    py_trees.display.render_dot_tree(
        scan,
        py_trees.common.string_to_visibility_level("detail")
    )
