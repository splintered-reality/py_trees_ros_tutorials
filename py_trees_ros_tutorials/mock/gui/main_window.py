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
Launch a qt dashboard for the tutorials.
"""
##############################################################################
# Imports
##############################################################################

import PyQt5.QtCore as qt_core
import PyQt5.QtWidgets as qt_widgets

from . import main_window_ui

##############################################################################
# Helpers
##############################################################################


class MainWindow(qt_widgets.QMainWindow):

    request_shutdown = qt_core.pyqtSignal(name="requestShutdown")

    def __init__(self):
        super().__init__()
        self.ui = main_window_ui.Ui_MainWindow()
        self.ui.setupUi(self)

    def closeEvent(self, unused_event):
        self.request_shutdown.emit()
