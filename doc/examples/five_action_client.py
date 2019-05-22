#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_actions
import py_trees_ros_tutorials

if __name__ == '__main__':

    scan_preempt = py_trees.composites.Selector(name="Preempt?")
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
    flash_blue = py_trees_ros_tutorials.behaviours.FlashLedStrip(
        name="Flash Blue",
        colour="blue"
    )

    scan_preempt.add_children([scanning])
    scanning.add_children([scan_rotate, flash_blue])
    py_trees.display.render_dot_tree(
        scan_preempt,
        py_trees.common.string_to_visibility_level("all"))
