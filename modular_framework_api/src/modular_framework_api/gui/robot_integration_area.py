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
from modular_framework_api.config_widgets.settings_config_widget import SettingsConfigWidget


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
        self.can_be_saved = False
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
        self.settings_config_widget = SettingsConfigWidget(self)
        self.addTab(self.robot_interface_widget, "Robot interface")
        self.addTab(self.arm_config_widget, "Arm configuration")
        self.addTab(self.hand_config_widget, "Hand configuration")
        self.addTab(self.settings_config_widget, "Settings")
        # By default the hand and arm tabs are disabled since we don't know the robot's composure
        self.setTabEnabled(1, False)
        self.setTabEnabled(2, False)
        self.setTabEnabled(3, False)

        # Update the display according to the simulation check box
        self.robot_interface_widget.simulation_config.check_box.toggled.connect(self.update_simulation_availability)
        # Update the tab view according to the robot_interface_widget's spin box value
        self.robot_interface_widget.robot_config.arm_spin_box.spin_box.valueChanged.connect(self.update_view)
        self.robot_interface_widget.robot_config.hand_spin_box.spin_box.valueChanged.connect(self.update_view)
        self.robot_interface_widget.robot_config.sensor_spin_box.spin_box.valueChanged.connect(self.update_view)
        # Change the content and availability of the MoveIt! related widgets in hardware configuration
        self.robot_interface_widget.moveit_config.moveit_package_entry_widget.entry_edit_line.textChanged.connect(self.update_widgets)
        # Update whether something has changed within the widget's children or not
        self.robot_interface_widget.interfaceChanged.connect(self.make_savable)

    def make_savable(self):
        """
            Change the attribute stating if a children has been modified to True
        """
        self.can_be_saved = True

    def update_widgets(self):
        """
            Enables or disables configuration widgets for the arm and hand depending on the MoveIt! configuration.
        """
        should_enable = self.robot_interface_widget.moveit_config.moveit_package_entry_widget.valid_input is not None
        self.arm_config_widget.moveit_planners_config.setEnabled(should_enable)
        self.hand_config_widget.moveit_planners_config.setEnabled(should_enable)
        # If a MoveIt! package has been provided then parse and set the proper information to
        # the ROS controllers and MoveIt! planners editors
        if should_enable:
            parsed_controllers, parsed_planners = self.robot_interface_widget.moveit_config.get_parsed_info()
            self.arm_config_widget.moveit_planners_config.set_planners_information(parsed_planners)
            self.arm_config_widget.ros_controllers.set_controllers_information(parsed_controllers)
            self.hand_config_widget.moveit_planners_config.set_planners_information(parsed_planners)
            self.hand_config_widget.ros_controllers.set_controllers_information(parsed_controllers)

            # Stack all the joints from the parsed controllers to set the autocompletion for the named joint states
            stacked_joints = []
            for joints in parsed_controllers.values():
                stacked_joints += joints
            # Get unique instance of each joints
            stacked_joints = set(stacked_joints)
            # Set the autocompletion
            self.settings_config_widget.named_joint_states.code_editor.set_autocompletion(stacked_joints)

    def update_simulation_availability(self):
        """
            Enables or disables some widgets according to the selected mode (simulation or physical platform)
        """
        is_checked = self.sender().isChecked()
        self.arm_config_widget.hardware_connection_config.setEnabled(not is_checked)
        self.hand_config_widget.hardware_connection_config.setEnabled(not is_checked)
        self.robot_interface_widget.robot_config.collision_scene_entry_widget.setEnabled(not is_checked)
        self.robot_interface_widget.simulation_config.gazebo_file_entry_widget.setEnabled(is_checked)
        self.robot_interface_widget.simulation_config.gazebo_folder_entry_widget.setEnabled(is_checked)
        self.robot_interface_widget.simulation_config.starting_pose_entry_widget.setEnabled(is_checked)

    def update_view(self):
        """
            Enables/disables the widgets and tabs according to the state of the spin boxes
        """
        # Modify only the hardware tab view
        # Get the widget that triggers this method
        triggering_widget = self.sender()
        widget_name = triggering_widget.objectName()
        # If the input of the spin box is > 0 then at least tab needs to be enabled
        enable_tab = triggering_widget.value() > 0
        # If the triggering widget is not the sensor's spin box, enables/disables the corresponding tab view
        if widget_name != "sensor":
            tab_index = 1 if widget_name == "arm" else 2
            self.setTabEnabled(tab_index, enable_tab)

        # Is the robot composed of at least one hand or one arm
        one_hardware_is_on = self.isTabEnabled(1) or self.isTabEnabled(2)
        # Is sensor's spin box is not set to 0
        sensor_is_on = self.robot_interface_widget.robot_config.sensor_spin_box.spin_box.value() > 0
        # Here we make sure that regardless of which spin box has been triggered we modify the settings tab
        if one_hardware_is_on or (widget_name == "sensor" and enable_tab):
            self.setTabEnabled(3, True)
        # Equivalent to no spin box activated
        elif not (one_hardware_is_on or sensor_is_on):
            self.setTabEnabled(3, False)
        # Here we make sure that regardless of which spin box has been triggered we modify the widgets
        # inside the different tabs
        self.arm_config_widget.set_default_enabled()
        self.hand_config_widget.set_default_enabled()
        self.settings_config_widget.named_poses.setEnabled(one_hardware_is_on)
        self.settings_config_widget.named_joint_states.setEnabled(one_hardware_is_on)
        self.settings_config_widget.named_trajectories.setEnabled(one_hardware_is_on)
        self.settings_config_widget.sensor_configs.setEnabled(sensor_is_on)
        self.settings_config_widget.sensor_plugins.setEnabled(sensor_is_on)
        self.settings_config_widget.methods_parameters.setEnabled(sensor_is_on or one_hardware_is_on)

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
        # If removed then will always ask to save even though nothing has been modified
        self.can_be_saved = False
