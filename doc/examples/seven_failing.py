#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_actions
import py_trees_ros_tutorials

if __name__ == '__main__':

    scan_or_die = py_trees.composites.Selector(name="Scan or Die", memory=False)
    die = py_trees.composites.Sequence(name="Die", memory=True)
    failed_notification = py_trees.composites.Parallel(
        name="Notification",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    failed_flash_green = py_trees_ros_tutorials.behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )
    failed_pause = py_trees.timers.Timer("Pause", duration=3.0)
    result_failed_to_bb = py_trees.blackboard.SetBlackboardVariable(
        name="Result2BB\n'failed'",
        variable_name='scan_result',
        variable_value='failed'
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
    cancelling.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    move_out_and_scan = py_trees.composites.Sequence(name="Move Out and Scan", memory=True)
    move_out_and_scan.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
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

    scan_or_die.add_children([ere_we_go, die])
    die.add_children([failed_notification, result_failed_to_bb])
    failed_notification.add_children([failed_flash_green, failed_pause])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])

    py_trees.display.render_dot_tree(
        scan_or_die,
        py_trees.common.string_to_visibility_level("detail")
    )
