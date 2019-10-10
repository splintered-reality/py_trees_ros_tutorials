#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import py_trees_ros_tutorials
import rclpy
import rclpy.executors

##############################################################################
# Helpers
##############################################################################


def assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def assert_details(text, expected, result):
    print(console.green + text +
          "." * (40 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)


def setup_module(module):
    console.banner("ROS Init")
    rclpy.init()


def teardown_module(module):
    console.banner("ROS Shutdown")
    rclpy.shutdown()


def timeout():
    return 3.0


def number_of_iterations():
    return 40

##############################################################################
# Tests
##############################################################################


def test_led_strip():
    console.banner("Client Success")

    mock_led_strip = py_trees_ros_tutorials.mock.led_strip.LEDStrip()
    tree_node = rclpy.create_node("tree")
    flash_led_strip = py_trees_ros_tutorials.behaviours.FlashLedStrip(name="Flash")
    flash_led_strip.setup(node=tree_node)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(mock_led_strip.node)
    executor.add_node(tree_node)

    assert_banner()

    # send flashing led
    spin_iterations = 0
    while spin_iterations < number_of_iterations() and flash_led_strip.colour not in mock_led_strip.last_text:
        flash_led_strip.tick_once()
        executor.spin_once(timeout_sec=0.05)
        spin_iterations += 1

    assert_details("flashing", flash_led_strip.colour, flash_led_strip.colour if flash_led_strip.colour in mock_led_strip.last_text else mock_led_strip.last_text)
    assert(flash_led_strip.colour in mock_led_strip.last_text)

    # cancel
    flash_led_strip.stop(new_status=py_trees.common.Status.INVALID)
    spin_iterations = 0
    while spin_iterations < number_of_iterations() and mock_led_strip.last_text:
        executor.spin_once(timeout_sec=0.05)
        spin_iterations += 1

    assert_details("cancelled", "", mock_led_strip.last_text)
    assert("" == mock_led_strip.last_text)

    executor.shutdown()
    tree_node.destroy_node()
    mock_led_strip.node.destroy_node()
