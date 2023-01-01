#!/usr/bin/env python

import py_trees
import py_trees_ros

if __name__ == '__main__':
    root = py_trees.composites.Sequence(name="Topics2BB", memory=True)

    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        variable_name="event_scan_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        threshold=30.0
    )
    root.add_children([scan2bb, battery2bb])
    py_trees.display.render_dot_tree(
        root,
        py_trees.common.string_to_visibility_level("all"))
