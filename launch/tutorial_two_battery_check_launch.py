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
Tutorial 2 - Battery Check
"""
##############################################################################
# Imports
##############################################################################

import py_trees_ros_tutorials.two_battery_check as tutorial

##############################################################################
# Launch Service
##############################################################################


def generate_launch_description():
    """
    Launch description for the tutorial.
    """
    return tutorial.generate_launch_description()
