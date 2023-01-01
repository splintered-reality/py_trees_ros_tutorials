#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':

    scan = py_trees.composites.Sequence(name="Scan", memory=True)
    is_scan_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Scan?",
        variable_name='event_scan_button',
        expected_value=True
    )
    scan_preempt = py_trees.composites.Selector(name="Preempt?", memory=False)
    scan_preempt.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    scan.add_children([is_scan_requested, scan_preempt])
    py_trees.display.render_dot_tree(
        scan,
        py_trees.common.string_to_visibility_level("detail"))
