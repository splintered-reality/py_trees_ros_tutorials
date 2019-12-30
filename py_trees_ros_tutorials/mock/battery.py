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
Mock the state of a battery component.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import py_trees_ros
import rclpy
import rclpy.parameter
import sensor_msgs.msg as sensor_msgs
import sys

##############################################################################
# Class
##############################################################################


class Battery(object):
    """
    Mocks the processed battery state for a robot (/battery/sensor_state).

    Node Name:
        * **battery**

    Publishers:
        * **~state** (:class:`sensor_msgs.msg.BatteryState`)

          * full battery state information

    Dynamic Parameters:
        * **~charging_percentage** (:obj:`float`)

          * one-shot setter of the current battery percentage
        * **~charging** (:obj:`bool`)

          * charging or discharging
        * **~charging_increment** (:obj:`float`)

          * the current charging/discharging increment

    On startup it is in a DISCHARGING state and updates every 200ms.
    Use the ``dashboard`` to dynamically reconfigure parameters.
    """
    def __init__(self):
        # node
        self.node = rclpy.create_node(
            node_name="battery",
            parameter_overrides=[
                rclpy.parameter.Parameter('charging_percentage', rclpy.parameter.Parameter.Type.DOUBLE, 100.0),
                rclpy.parameter.Parameter('charging_increment', rclpy.parameter.Parameter.Type.DOUBLE, 0.1),
                rclpy.parameter.Parameter('charging', rclpy.parameter.Parameter.Type.BOOL, False),
            ],
            automatically_declare_parameters_from_overrides=True
        )

        # publishers
        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('state', "~/state", sensor_msgs.BatteryState, not_latched),
            ]
        )

        # initialisations
        self.battery = sensor_msgs.BatteryState()
        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.battery.voltage = float('nan')
        self.battery.current = float('nan')
        self.battery.charge = float('nan')
        self.battery.capacity = float('nan')
        self.battery.design_capacity = float('nan')
        self.battery.percentage = 100.0
        self.battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        self.battery.present = True
        self.battery.location = ""
        self.battery.serial_number = ""

        self.timer = self.node.create_timer(
            timer_period_sec=0.2,
            callback=self.update_and_publish
        )

    def update_and_publish(self):
        """
        Timer callback that processes the battery state update and publishes.
        """
        # parameters
        charging = self.node.get_parameter("charging").value
        charging_increment = self.node.get_parameter("charging_increment").value
        charging_percentage = self.node.get_parameter("charging_percentage").value

        # update state
        if charging:
            charging_percentage = min(100.0, charging_percentage + charging_increment)
            if charging_percentage % 5.0 < 0.1:
                self.node.get_logger().debug("Charging...{:.1f}%%".format(charging_percentage))
        else:
            charging_percentage = max(0.0, charging_percentage - charging_increment)
            if charging_percentage % 2.5 < 0.1:
                self.node.get_logger().debug("Discharging...{:.1f}%%".format(charging_percentage))

        # update parameters (TODO: need a guard?)
        self.node.set_parameters([
            rclpy.parameter.Parameter(
                'charging_percentage',
                rclpy.parameter.Parameter.Type.DOUBLE,
                float(charging_percentage)
            )
        ])

        # publish
        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        charging_percentage = min(100.0, charging_percentage)
        self.battery.percentage = charging_percentage
        if charging_percentage == 100.0:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        elif charging:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.publishers.state.publish(msg=self.battery)

    def shutdown(self):
        """
        Cleanup ROS components.
        """
        # currently complains with:
        #  RuntimeWarning: Failed to fini publisher: rcl node implementation is invalid, at /tmp/binarydeb/ros-dashing-rcl-0.7.5/src/rcl/node.c:462
        # Q: should rlcpy.shutdown() automagically handle descruction of nodes implicitly?
        self.node.destroy_node()


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock the state of a battery component')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    battery = Battery()
    try:
        rclpy.spin(battery.node)
    except KeyboardInterrupt:
        pass
    battery.shutdown()
    rclpy.shutdown()
