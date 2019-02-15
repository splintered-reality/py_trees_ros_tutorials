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

import PyQt5.QtWidgets as qt_widgets

from . import configuration_group_box_ui

##############################################################################
# Helpers
##############################################################################


class ConfigurationGroupBox(qt_widgets.QGroupBox):
    """
    Convenience class that Designer can use to promote
    elements for layouts in applications.
    """
    def __init__(self, parent):
        super().__init__(parent)
        self.ui = configuration_group_box_ui.Ui_ConfigurationGroupBox()
        self.ui.setupUi(self)

    def set_battery_percentage(self, percentage):
        print("Battery Percentage -> Ui {}".format(percentage))
        self.ui.battery_percentage_slider.setValue(int(percentage))
