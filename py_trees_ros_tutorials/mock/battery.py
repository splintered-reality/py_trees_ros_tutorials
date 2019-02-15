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
Mocks a battery provider.
"""


##############################################################################
# Imports
##############################################################################

import argparse
# import dynamic_reconfigure.server
import py_trees_ros
import rclpy
import rclpy.parameter
import sensor_msgs.msg as sensor_msgs
import sys

# from py_trees_msgs.cfg import BatteryConfig

##############################################################################
# Class
##############################################################################


class Battery:
    """
    Mocks the processed battery state for a robot (/battery/sensor_state)
    as well as a possible charging source (/battery/charging_source).

    ROS Publishers:
        * **~state** (:class:`sensor_msgs.msg.BatteryState`)

          * full battery state information

    Dynamic Reconfigure:
        * **~charging_percentage** (:obj:`float`)

          * one-step setting of the current battery percentage
        * **~charging** (:obj:`bool`)

          * whether it is currently charging or not
        * **~charging_increment** (:obj:`float`)

          * how fast it charges/discharges

    On startup it is in a DISCHARGING state. Use ``rqt_reconfigure`` to change the battery state.
    """
    def __init__(self):
        # node
        self.node = rclpy.create_node(
            "battery",
            initial_parameters=[
                rclpy.parameter.Parameter('charging_percentage', rclpy.parameter.Parameter.Type.DOUBLE, 100.0),
                rclpy.parameter.Parameter('charging_increment', rclpy.parameter.Parameter.Type.DOUBLE, 0.1),
                rclpy.parameter.Parameter('charging', rclpy.parameter.Parameter.Type.BOOL, False),
            ]
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

    def update_parameters(self, parameters):
        """
        Args:
            parameters ([:class:`rclpy.parameters.Parameter`]): incoming configuration
            level (:obj:`int`):
        """
        self.node.set_parameters(parameters)

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        # TODO: with rate and spin_once, once rate is implemented in rclpy
        unused_timer = self.node.create_timer(
            timer_period_sec=0.2,
            callback=self.update_and_publish
        )
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()

    def update_and_publish(self):
        """
        Update and publish.
        """
        # parameters
        charging = self.node.get_parameter("charging").value
        charging_increment = self.node.get_parameter("charging_increment").value
        charging_percentage = self.node.get_parameter("charging_percentage").value

        # update state
        if charging:
            charging_percentage = min(100, charging_percentage + charging_increment)
            self.node.get_logger().info("Charging...{:.1f}%%".format(charging_percentage))
        else:
            charging_percentage = max(0, charging_percentage - charging_increment)
            self.node.get_logger().info("Discharging...{:.1f}%%".format(charging_percentage))

        # update parameters (TODO: need a guard?)
        self.node.set_parameters([
            rclpy.parameter.Parameter(
                'charging_percentage',
                rclpy.parameter.Parameter.Type.DOUBLE,
                charging_percentage
            )
        ])

        # publish
        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.battery.percentage = charging_percentage
        if self.battery.percentage >= 100:
            self.battery.percentage = 100
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        elif charging:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.publishers.state.publish(msg=self.battery)


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a battery/charging source')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    battery = Battery()
    battery.spin()
    rclpy.shutdown()
