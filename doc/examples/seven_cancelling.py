#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros_tutorials

if __name__ == '__main__':

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
    celebrate_flash_green = py_trees_ros_tutorials.behaviours.FlashLedStrip(name="Flash Green", colour="green")
    celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)
    dock = py_trees_ros.actions.ActionClient(
        name="Dock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=True),
        generate_feedback_message=lambda msg: "docking"
    )
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children([is_cancel_requested, move_home_after_cancel, result_cancelled_to_bb])
    move_out_and_scan.add_children([move_base, scanning, move_home_after_scan, result_succeeded_to_bb])
    celebrate.add_children([celebrate_flash_green, celebrate_pause])

    py_trees.display.render_dot_tree(
        ere_we_go,
        py_trees.common.string_to_visibility_level("detail")
    )
