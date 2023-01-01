#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_actions
import py_trees_ros_tutorials

if __name__ == '__main__':

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        variable_name="event_scan_button"
    )
    scan2bb.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
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
    battery2bb.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])

    py_trees.display.render_dot_tree(
        topics2bb,
        py_trees.common.string_to_visibility_level("detail")
    )
