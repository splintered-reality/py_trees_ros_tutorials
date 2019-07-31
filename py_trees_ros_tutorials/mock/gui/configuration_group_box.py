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
Launch a qt dashboard for the tutorials.
"""
##############################################################################
# Imports
##############################################################################

import PyQt5.QtWidgets as qt_widgets
import PyQt5.QtCore as qt_core

from . import configuration_group_box_ui

##############################################################################
# Helpers
##############################################################################


class ConfigurationGroupBox(qt_widgets.QGroupBox):
    """
    Convenience class that Designer can use to promote
    elements for layouts in applications.
    """

    change_battery_percentage = qt_core.pyqtSignal(float, name="changeBatteryPercentage")
    change_battery_charging_status = qt_core.pyqtSignal(bool, name="changeBatteryChargingStatus")
    change_safety_sensors_enabled = qt_core.pyqtSignal(bool, name="safetySensorsEnabled")

    def __init__(self, parent):
        super().__init__(parent)
        self.ui = configuration_group_box_ui.Ui_ConfigurationGroupBox()
        self.ui.setupUi(self)

        self.ui.battery_charging_check_box.clicked.connect(
            self.battery_charging_status_checkbox_clicked
        )
        self.ui.battery_percentage_slider.sliderReleased.connect(
            self.battery_percentage_slider_updated
        )
        self.ui.safety_sensors_enabled_check_box.clicked.connect(
            self.safety_sensors_enabled_checkbox_clicked
        )

    def set_battery_percentage(self, percentage):
        if not self.ui.battery_percentage_slider.isSliderDown():
            self.ui.battery_percentage_slider.setValue(int(percentage))

    def set_charging_status(self, charging_status):
        self.ui.battery_charging_check_box.setChecked(charging_status)

    def set_safety_sensors_enabled(self, enabled_status: bool):
        self.ui.safety_sensors_enabled_check_box.setChecked(enabled_status)

    def battery_percentage_slider_updated(self):
        percentage = self.ui.battery_percentage_slider.value()
        self.change_battery_percentage.emit(percentage)

    def battery_charging_status_checkbox_clicked(self, checked):
        self.change_battery_charging_status.emit(checked)

    def safety_sensors_enabled_checkbox_clicked(self, checked):
        self.change_safety_sensors_enabled.emit(checked)
