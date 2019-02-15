#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Launch a qt dashboard for the tutorials.
"""
##############################################################################
# Imports
##############################################################################

import functools
import py_trees_ros
import rclpy
import signal
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import sys
import threading

import PyQt5.QtCore as qt_core
import PyQt5.QtWidgets as qt_widgets

# To use generated files instead of loading ui's directly
from . import gui

##############################################################################
# Helpers
##############################################################################


class Backend(qt_core.QObject):

    led_colour_changed = qt_core.pyqtSignal(str, name="ledColourChanged")
    battery_percentage_changed = qt_core.pyqtSignal(float, name="batteryPercentageChanged")

    def __init__(self, dashboard_group_box):
        super().__init__()

        self.ui = dashboard_group_box
        self.node = rclpy.create_node("dashboard")

        self.shutdown_requested = False

        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('scan', "~/scan", std_msgs.Empty, not_latched),
                ('cancel', "~/cancel", std_msgs.Empty, not_latched),
            ]
        )

        self.ui.ui.scan_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.scan)
        )

        self.ui.ui.cancel_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.cancel)
        )

        latched = True
        unlatched = False
        self.subscribers = py_trees_ros.utilities.Subscribers(
            self.node,
            [
                ("report", "/tree/report", std_msgs.String, latched, self.reality_report_callback),
                ("led_strip", "/led_strip/display", std_msgs.String, latched, self.led_strip_display_callback),
                ("battery_state", "/battery/state", sensor_msgs.BatteryState, unlatched, self.battery_state_callback)
            ]
        )

    def spin(self):
        try:
            while rclpy.ok() and not self.shutdown_requested:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()

    def publish_button_message(self, publisher):
        publisher.publish(std_msgs.Empty())

    def shutdown_requested_callback(self):
        self.shutdown_requested = True

    # TODO: shift to the ui
    def reality_report_callback(self, msg):
        if msg.data == "cancelling":
            self.set_scanning_colour(False)
            self.ui.set_cancel_push_button_colour(True)
            self.ui.ui.cancel_push_button.setEnabled(True)
        elif msg.data == "scanning":
            self.set_scanning_colour(True)
            self.ui.set_cancel_push_button_colour(False)
            self.ui.ui.cancel_push_button.setEnabled(True)
        else:
            self.ui.set_scan_push_button_colour(False)
            self.ui.set_cancel_push_button_colour(False)
            self.ui.ui.cancel_push_button.setEnabled(False)

    def led_strip_display_callback(self, msg):
        colour = "grey"
        if not msg.data:
            self.node.get_logger().info("no colour specified, setting '{}'".format(colour))
        elif msg.data not in ["grey", "blue", "red", "green"]:
            self.node.get_logger().info("received unsupported LED colour '{0}', setting '{1}'".format(msg.data, colour))
        else:
            colour = msg.data
        self.led_colour_changed.emit(colour)

    def battery_state_callback(self, msg):
        """
        Args:
            msg (:class:`sensor_msgs.msg.BatteryState`): battery state
        """
        print("Got callback")
        self.battery_percentage_changed.emit(msg.percentage)


##############################################################################
# Main
##############################################################################


def main():
    # picks up sys.argv automagically internally
    rclpy.init()
    # enable handling of ctrl-c (from roslaunch as well)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # the players
    app = qt_widgets.QApplication(sys.argv)
    main_window = gui.main_window.MainWindow()
    backend = Backend(main_window.ui.dashboard_group_box)

    # sigslots
    backend.led_colour_changed.connect(
        main_window.ui.dashboard_group_box.set_led_strip_colour
    )
    backend.battery_percentage_changed.connect(
        main_window.ui.configuration_group_box.set_battery_percentage
    )
    main_window.request_shutdown.connect(
        backend.shutdown_requested_callback
    )

    # qt ... up
    ros_thread = threading.Thread(target=backend.spin)
    ros_thread.start()
    main_window.show()
    result = app.exec_()

    # shutdown
    ros_thread.join()
    rclpy.shutdown()
    sys.exit(result)
