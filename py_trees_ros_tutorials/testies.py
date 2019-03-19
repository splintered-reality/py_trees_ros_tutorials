#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rclpy
import rclpy.executors

##############################################################################
# Launcher
##############################################################################


def foo():
    print("foo")


def main():
    rclpy.init()
    foo_node = rclpy.create_node(node_name="foo")
    bar_node = rclpy.create_node(node_name="bar")
    timer = foo_node.create_timer(timer_period_sec=0.5, callback=foo)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(foo_node)
    executor.add_node(bar_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    timer.cancel()
    foo_node.destroy_timer(timer)
    foo_node.destroy_node()
    bar_node.destroy_node()
    rclpy.shutdown()
