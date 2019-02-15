# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dashboard_group_box.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_DashboardGroupBox(object):
    def setupUi(self, DashboardGroupBox):
        DashboardGroupBox.setObjectName("DashboardGroupBox")
        DashboardGroupBox.resize(400, 300)
        self.verticalLayout = QtWidgets.QVBoxLayout(DashboardGroupBox)
        self.verticalLayout.setObjectName("verticalLayout")
        self.scan_push_button = QtWidgets.QPushButton(DashboardGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scan_push_button.sizePolicy().hasHeightForWidth())
        self.scan_push_button.setSizePolicy(sizePolicy)
        self.scan_push_button.setStyleSheet("font-size: 30pt;")
        self.scan_push_button.setObjectName("scan_push_button")
        self.verticalLayout.addWidget(self.scan_push_button)
        self.cancel_push_button = QtWidgets.QPushButton(DashboardGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cancel_push_button.sizePolicy().hasHeightForWidth())
        self.cancel_push_button.setSizePolicy(sizePolicy)
        self.cancel_push_button.setAutoFillBackground(False)
        self.cancel_push_button.setStyleSheet("font-size: 30pt;")
        self.cancel_push_button.setFlat(False)
        self.cancel_push_button.setObjectName("cancel_push_button")
        self.verticalLayout.addWidget(self.cancel_push_button)
        self.led_strip_label = QtWidgets.QLabel(DashboardGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.led_strip_label.sizePolicy().hasHeightForWidth())
        self.led_strip_label.setSizePolicy(sizePolicy)
        self.led_strip_label.setAutoFillBackground(False)
        self.led_strip_label.setStyleSheet("font-size: 30pt;")
        self.led_strip_label.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.led_strip_label.setFrameShadow(QtWidgets.QFrame.Raised)
        self.led_strip_label.setAlignment(QtCore.Qt.AlignCenter)
        self.led_strip_label.setObjectName("led_strip_label")
        self.verticalLayout.addWidget(self.led_strip_label)

        self.retranslateUi(DashboardGroupBox)
        QtCore.QMetaObject.connectSlotsByName(DashboardGroupBox)

    def retranslateUi(self, DashboardGroupBox):
        _translate = QtCore.QCoreApplication.translate
        DashboardGroupBox.setWindowTitle(_translate("DashboardGroupBox", "Dashboard"))
        DashboardGroupBox.setTitle(_translate("DashboardGroupBox", "Dashboard"))
        self.scan_push_button.setText(_translate("DashboardGroupBox", "Scan"))
        self.cancel_push_button.setText(_translate("DashboardGroupBox", "Cancel"))
        self.led_strip_label.setText(_translate("DashboardGroupBox", "Led Strip"))

