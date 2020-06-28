#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from PyQt5.QtWidgets import QTabWidget
from modular_framework_api.config_widgets.robot_interface_widget import RobotInterfaceWidget
from modular_framework_api.config_widgets.hardware_config_widget import HardwareConfigWidget


class RobotIntegrationArea(QTabWidget):

    """
        Widget gathering all the widgets required to integrate a robot to the framework
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different widgets

            @param parent: parent of the widget
        """
        super(RobotIntegrationArea, self).__init__(parent=parent)
        # Set the object name to be able to look it up and restore it
        self.setObjectName("Robot integration area")
        self.init_ui()

    def init_ui(self):
        """
            Create the different widgets and store them into tabs
        """
        # Widget taking care of robot interfacing
        self.robot_interface_widget = RobotInterfaceWidget(self)
        # Widget taking care of configuring the potential robot arm
        self.arm_config_widget = HardwareConfigWidget("Arm", self)
        # Widget taking care of configuring the potential robot hand
        self.hand_config_widget = HardwareConfigWidget("Hand", self)
        # Widget taking care of configuring the experimental environemnt
        # self.setup_config_widget = SetupConfigWidget(self)
        self.addTab(self.robot_interface_widget, "Robot interface")
        self.addTab(self.arm_config_widget, "Arm configuration")
        self.addTab(self.hand_config_widget, "Hand configuration")
        # self.addTab(self.setup_config_widget, "Setup configuration")
        # By default the hand and arm tabs are disabled since we don't know the robot's composure
        self.setTabEnabled(1, False)
        self.setTabEnabled(2, False)

        # Update the display according to the simulation check box
        self.robot_interface_widget.simulation_config.check_box.toggled.connect(self.update_simulation_availability)
        # Update the tab view according to the robot_interface_widget's spin box value
        self.robot_interface_widget.robot_config.arm_spin_box.spin_box.valueChanged.connect(self.update_tab_view)
        self.robot_interface_widget.robot_config.hand_spin_box.spin_box.valueChanged.connect(self.update_tab_view)

    def update_simulation_availability(self):
        """
            Enables or disables some widgets according to the selected mode (simulation or physical platform)
        """
        is_checked = self.sender().isChecked()
        self.arm_config_widget.hardware_connection_config.setEnabled(not is_checked)
        self.hand_config_widget.hardware_connection_config.setEnabled(not is_checked)
        self.robot_interface_widget.robot_config.enable_user_entry("Collision scene", not is_checked)
        self.robot_interface_widget.simulation_config.enable_user_entry("Gazebo world file", is_checked)
        self.robot_interface_widget.simulation_config.enable_user_entry("Gazebo model folder", is_checked)
        self.robot_interface_widget.simulation_config.enable_user_entry("Starting pose", is_checked)

    def update_tab_view(self):
        """
            Enables or disables the hardware tabs according to the current configuration
        """
        # TODO: Handle the sensor part
        triggering_widget = self.sender()
        should_enable = triggering_widget.value() > 0
        tab_index = 1 if triggering_widget.objectName() == "arm" else 2
        self.setTabEnabled(tab_index, should_enable)

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for tab_index in range(self.count()):
            widget_in_tab = self.widget(tab_index)
            widget_in_tab.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for tab_index in range(self.count()):
            widget_in_tab = self.widget(tab_index)
            widget_in_tab.restore_config(settings)
        settings.endGroup()
