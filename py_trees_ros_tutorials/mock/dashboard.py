#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
A qt dashboard for interactions with the mock robot
"""
##############################################################################
# Imports
##############################################################################

import functools
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
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
    battery_charging_status_changed = qt_core.pyqtSignal(float, name="batteryChargingStatusChanged")

    def __init__(self, dashboard_group_box):
        super().__init__()

        self.ui = dashboard_group_box
        self.node = rclpy.create_node("dashboard")

        self.shutdown_requested = False
        self.last_battery_charging_status = None

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

        # dynamic parameter clients
        self.parameter_clients = {
            'battery': self.node.create_client(
                rcl_srvs.SetParameters,
                '/battery/set_parameters'
            ),
            'safety_sensors': self.node.create_client(
                rcl_srvs.SetParameters,
                '/safety_sensors/set_parameters'
            )
        }

    def spin(self):
        initialised = {name: False for name in self.parameter_clients.keys()}
        while rclpy.ok() and not self.shutdown_requested and not all(value for value in initialised.values()):
            for name, client in self.parameter_clients.items():
                if client.wait_for_service(timeout_sec=0.1):
                    initialised[name] = True
                    self.node.get_logger().info("'{}' initialised".format(name))
                else:
                    self.node.get_logger().info("service '/{}/set_parameters' unavailable, waiting...".format(name))
        while rclpy.ok() and not self.shutdown_requested:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()

    def publish_button_message(self, publisher):
        publisher.publish(std_msgs.Empty())

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested")
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
        self.battery_percentage_changed.emit(msg.percentage)
        if msg.power_supply_status == sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
            charging = False
        else:
            charging = True
        if charging != self.last_battery_charging_status:
            self.battery_charging_status_changed.emit(charging)
        self.last_battery_charging_status = charging

    def update_battery_percentage(self, percentage):
        request = rcl_srvs.SetParameters.Request()
        parameter = rcl_msgs.Parameter()
        parameter.name = "charging_percentage"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = percentage
        request.parameters.append(parameter)
        unused_future = self.parameter_clients['battery'].call_async(request)

    def update_battery_charging_status(self, charging):
        request = rcl_srvs.SetParameters.Request()
        parameter = rcl_msgs.Parameter()
        parameter.name = "charging"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = charging
        request.parameters.append(parameter)
        unused_future = self.parameter_clients['battery'].call_async(request)

        # no need to check for the response, though do note that
        # if you do, you're probably in the wrong thread if you're
        # checking for futures
        #
        # rclpy.spin_until_future_complete(self.node, future, executor=rclpy.executors.SingleThreadedExecutor())
        # if future.result() is not None:
        #     self.node.get_logger().info('result of set charging status parameter: %s' % future.result())
        # else:
        #     self.node.get_logger().error('exception while calling service: %r' % future.exception())

    def update_safety_sensors_enabled(self, enabled):
        request = rcl_srvs.SetParameters.Request()
        parameter = rcl_msgs.Parameter()
        parameter.name = "enabled"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = enabled
        request.parameters.append(parameter)
        unused_future = self.parameter_clients['safety_sensors'].call_async(request)


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
    backend.battery_charging_status_changed.connect(
        main_window.ui.configuration_group_box.set_charging_status
    )
    main_window.ui.configuration_group_box.change_battery_percentage.connect(
        backend.update_battery_percentage
    )
    main_window.ui.configuration_group_box.change_battery_charging_status.connect(
        backend.update_battery_charging_status
    )
    main_window.ui.configuration_group_box.change_safety_sensors_enabled.connect(
        backend.update_safety_sensors_enabled
    )
    main_window.request_shutdown.connect(
        backend.terminate_ros_spinner
    )

    # sig interrupt handling
    signal.signal(
        signal.SIGINT,
        lambda unused_signal, unused_frame: main_window.close()
    )

    # qt ... up
    ros_thread = threading.Thread(target=backend.spin)
    ros_thread.start()
    main_window.show()
    result = app.exec_()

    # shutdown
    backend.node.get_logger().info("joining")
    ros_thread.join()
    backend.node.get_logger().info("joined")
    rclpy.shutdown()
    backend.node.get_logger().info("rclpy shutdown")
    sys.exit(result)
