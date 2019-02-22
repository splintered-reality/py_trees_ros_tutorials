# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'configuration_group_box.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_ConfigurationGroupBox(object):
    def setupUi(self, ConfigurationGroupBox):
        ConfigurationGroupBox.setObjectName("ConfigurationGroupBox")
        ConfigurationGroupBox.resize(400, 300)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(ConfigurationGroupBox)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.battery_group_box = QtWidgets.QGroupBox(ConfigurationGroupBox)
        self.battery_group_box.setObjectName("battery_group_box")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.battery_group_box)
        self.verticalLayout.setObjectName("verticalLayout")
        self.battery_charging_check_box = QtWidgets.QCheckBox(self.battery_group_box)
        self.battery_charging_check_box.setObjectName("battery_charging_check_box")
        self.verticalLayout.addWidget(self.battery_charging_check_box)
        self.battery_percentage_slider = QtWidgets.QSlider(self.battery_group_box)
        self.battery_percentage_slider.setMaximum(100)
        self.battery_percentage_slider.setSliderPosition(50)
        self.battery_percentage_slider.setOrientation(QtCore.Qt.Horizontal)
        self.battery_percentage_slider.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.battery_percentage_slider.setObjectName("battery_percentage_slider")
        self.verticalLayout.addWidget(self.battery_percentage_slider)
        spacerItem = QtWidgets.QSpacerItem(20, 41, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.verticalLayout_2.addWidget(self.battery_group_box)
        self.safety_sensors_group_box = QtWidgets.QGroupBox(ConfigurationGroupBox)
        self.safety_sensors_group_box.setObjectName("safety_sensors_group_box")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.safety_sensors_group_box)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.safety_sensors_enabled_check_box = QtWidgets.QCheckBox(self.safety_sensors_group_box)
        self.safety_sensors_enabled_check_box.setObjectName("safety_sensors_enabled_check_box")
        self.verticalLayout_3.addWidget(self.safety_sensors_enabled_check_box)
        spacerItem1 = QtWidgets.QSpacerItem(20, 97, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.verticalLayout_2.addWidget(self.safety_sensors_group_box)

        self.retranslateUi(ConfigurationGroupBox)
        QtCore.QMetaObject.connectSlotsByName(ConfigurationGroupBox)

    def retranslateUi(self, ConfigurationGroupBox):
        _translate = QtCore.QCoreApplication.translate
        ConfigurationGroupBox.setWindowTitle(_translate("ConfigurationGroupBox", "GroupBox"))
        ConfigurationGroupBox.setTitle(_translate("ConfigurationGroupBox", "Configuration"))
        self.battery_group_box.setTitle(_translate("ConfigurationGroupBox", "Battery"))
        self.battery_charging_check_box.setText(_translate("ConfigurationGroupBox", "Charging"))
        self.safety_sensors_group_box.setTitle(_translate("ConfigurationGroupBox", "Safety Sensors"))
        self.safety_sensors_enabled_check_box.setText(_translate("ConfigurationGroupBox", "Enabled"))

