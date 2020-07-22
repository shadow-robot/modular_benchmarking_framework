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

from PyQt5.QtWidgets import QTabWidget, QPushButton
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
        self.config_changed = dict()
        self. can_be_saved = False
        self.init_ui()
        self.connect_slots()

    def init_ui(self):
        """
            Create the different widgets and store them into tabs
        """
        # Widget taking care of robot interfacing
        self.robot_interface = RobotInterfaceWidget(self)
        # Widget taking care of configuring the potential robot arm
        self.arm_config_widget = HardwareConfigWidget("Arm", self)
        # Widget taking care of configuring the potential robot hand
        self.hand_config_widget = HardwareConfigWidget("Hand", self)
        # Widget taking care of configuring the experimental environemnt
        self.settings_config_widget = SettingsConfigWidget(self)
        self.addTab(self.robot_interface, "Robot interface")
        self.addTab(self.arm_config_widget, "Arm configuration")
        self.addTab(self.hand_config_widget, "Hand configuration")
        self.addTab(self.settings_config_widget, "Settings")
        # By default the hand and arm tabs are disabled since we don't know the robot's composure
        self.setTabEnabled(1, False)
        self.setTabEnabled(2, False)
        self.setTabEnabled(3, False)
        # Create a button (disabled by default) allowing to launch the configured robot
        self.launch_button = QPushButton("Launch robot", enabled=False)
        # Compute the max height of the widget to get nice visual
        maximum_height = self.launch_button.fontMetrics().boundingRect(self.launch_button.text()).height() + 5
        self.launch_button.setMaximumHeight(maximum_height)
        # self.launch_button.clicked.connect(self.launch_robot)
        self.setCornerWidget(self.launch_button)

    def connect_slots(self):
        """
            Remap signals coming from different widgets to have a reactive behavior
        """
        # Update the display according to the simulation check box
        self.robot_interface.simulation_config.check_box.toggled.connect(self.update_simulation_availability)
        # Update the tab view according to the robot_interface_widget's spin box value
        self.robot_interface.robot_config.arm_spin_box.spin_box.valueChanged.connect(self.update_view)
        self.robot_interface.robot_config.hand_spin_box.spin_box.valueChanged.connect(self.update_view)
        self.robot_interface.robot_config.sensor_spin_box.spin_box.valueChanged.connect(self.update_view)
        # Change the content and availability of the MoveIt! related widgets in hardware configuration
        self.robot_interface.moveit_config.moveit_package_entry_widget.validInputChanged.connect(self.update_widgets)
        # Update whether something has changed within the widget's children or not
        self.robot_interface.interfaceChanged.connect(self.update_savable)
        self.arm_config_widget.hardwareChanged.connect(self.update_savable)
        self.settings_config_widget.settingsChanged.connect(self.update_savable)

    def update_savable(self, is_savable=True):
        """
            Change the attribute stating whether children widgets can be saved

            @param is_savable: Boolean specifying whether the widget has been saved
        """
        # Since each object has got an unique name, store it in a dictionary
        self.config_changed[self.sender().objectName()] = is_savable
        # Take into account all the different configs
        self.can_be_saved = any(self.config_changed.values())
        self.test()

    def test(self):
        """
        """
        # ======================= Basics
        # If everything is 0 then it's bad
        nb_arm = self.robot_interface.robot_config.arm_spin_box.get_value()
        nb_hand = self.robot_interface.robot_config.hand_spin_box.get_value()
        nb_sensors = self.robot_interface.robot_config.sensor_spin_box.get_value()
        # None if not good
        urdf_file = self.robot_interface.robot_config.robot_urdf_entry_widget.valid_input
        if nb_arm + nb_hand + nb_sensors == 0 or urdf_file is None:
            self.launch_button.setEnabled(False)
            print("Not the basics")
            return
        # ===================== Pure simulation
        is_simu = self.robot_interface.simulation_config.check_box.isChecked()
        # If None and simu not good
        gazebo_world_file = self.robot_interface.simulation_config.gazebo_file_entry_widget.valid_input
        gazebo_model_folder = self.robot_interface.simulation_config.gazebo_folder_entry_widget.valid_input
        is_gazebo_configured = gazebo_world_file is not None and gazebo_model_folder is not None
        # Check the starting pose is good
        pose_line_text = self.robot_interface.simulation_config.starting_pose_entry_widget.entry_edit_line.text()
        pose = self.robot_interface.simulation_config.starting_pose_entry_widget.valid_input
        is_starting_pose_correct = pose is not None or pose is None and not pose_line_text
        if is_simu and not (is_gazebo_configured and is_starting_pose_correct):
            self.launch_button.setEnabled(False)
            print("Bad simulation")
            return
        # ======================= Arm
        # Empty is not good
        arm_external_controller = self.arm_config_widget.external_controller.valid_input
        arm_ros_controller = self.arm_config_widget.ros_controllers.valid_input
        arm_moveit_planner = self.arm_config_widget.moveit_planners_config.valid_input
        arm_moveit_planner = self.arm_config_widget.moveit_planners_config.valid_input
        arm_hardware_connection = self.arm_config_widget.hardware_connection_config.code_editor.parsed_content
        if self.arm_config_widget.moveit_planners_config.file_path is not None and not arm_moveit_planner:
            self.launch_button.setEnabled(False)
            print("Wrong moveit planner config")
            return
        if self.arm_config_widget.ros_controllers.file_path is not None and not arm_ros_controller:
            self.launch_button.setEnabled(False)
            print("Wrong ROS controller config")
            return
        if self.arm_config_widget.external_controller.file_path is not None and not arm_external_controller:
            self.launch_button.setEnabled(False)
            print("Wrong external controller config")
            return
        if nb_arm and not (arm_ros_controller or arm_external_controller):
            self.launch_button.setEnabled(False)
            print("Unconfigured arm controller")
            return
        if nb_arm and is_simu and not arm_ros_controller:
            self.launch_button.setEnabled(False)
            print("No ROS controller with embedded simu")
            return
        if nb_arm and arm_moveit_planner and not arm_ros_controller:
            self.launch_button.setEnabled(False)
            print("No ROS controller for given planning group")
            return
        if nb_arm and not is_simu and not arm_external_controller and not arm_hardware_connection:
            self.launch_button.setEnabled(False)
            print("No way to connect to arm provided")
            return
        self.launch_button.setEnabled(True)

        # # If empty then wrong
        # sensor_configs = self.settings_config_widget.sensor_configs.valid_input
        # # ======================= For sensor
        # if nb_sensors and not sensor_configs:
        #     self.launch_button.setEnabled(False)
        #     print("Sensor unconfigured")
        #     return
        # # None if not good
        # moveit_package = self.robot_interface.moveit_config.moveit_package_entry_widget.valid_input
        # # None (but should have package filled) or not empty list when good
        # move_group_args = self.robot_interface.moveit_config.move_group_editor.valid_input
        # rviz_args = self.robot_interface.moveit_config.rviz_editor.valid_input
        # # ======================== For arm
        # # Empty is not good
        # arm_external_controller = self.arm_config_widget.external_controller.valid_input
        # arm_ros_controller = self.arm_config_widget.ros_controllers.valid_input
        # arm_moveit_planner = self.arm_config_widget.moveit_planners_config.valid_input
        # arm_hardware_connection = self.arm_config_widget.hardware_connection_config.code_editor.parsed_content
        # if not (arm_ros_controller or arm_external_controller):
        #     self.launch_button.setEnabled(False)
        #     return
        # if arm_moveit_planner and not arm_ros_controller:
        #     self.launch_button.setEnabled(False)
        #     return
        # if arm_ros_controller and not is_simu and not arm_hardware_connection:
        #     self.launch_button.setEnabled(False)
        #     return
        # ========================================================= Moveit route
        # None if not good
        # moveit_package = self.robot_interface.moveit_config.moveit_package_entry_widget.valid_input
        # if moveit_package is None:
        #     self.launch_button.setEnabled(False)
        #     return
        # print("c")
        # # None (but should have package filled) or not empty list when good
        # move_group_args = self.robot_interface.moveit_config.move_group_editor.valid_input
        # rviz_args = self.robot_interface.moveit_config.rviz_editor.valid_input
        # if not((move_group_args is None or move_group_args) and (rviz_args is None or rviz_args)):
        #     self.launch_button.setEnabled(False)
        #     return
        # print("d")
        # # If empty then bad!
        # arm_ros_controller = self.arm_config_widget.ros_controllers.valid_input
        # # print(self.arm_config_widget.ros_controllers.valid_input)
        # if nb_arm and not arm_ros_controller:
        #     self.launch_button.setEnabled(False)
        #     return
        # print("e")
        # hand_ros_controller = self.hand_config_widget.ros_controllers.valid_input
        # if nb_hand and not hand_ros_controller:
        #     self.launch_button.setEnabled(False)
        #     return
        # print("f")
        # ========================================================= Custom launch file
        # None if not good
        # custom_launch_file = self.robot_interface.robot_config.launch_file_entry_widget.valid_input
        # if custom_launch_file is None:
        #     self.launch_button.setEnabled(False)
        #     return
        # # None (but should have launch file filled) or not empty list when good
        # launch_args = self.robot_interface.robot_config.launch_file_editor.valid_input
        # if not (launch_args is None or launch_args):
        #     self.launch_button.setEnabled(False)
        #     return

    def update_widgets(self):
        """
            Update the availability and attributes of all the widgets related to MoveIt!
        """
        # Check if a valid moveit package has bee provided
        should_enable = self.sender().valid_input is not None
        # Update availability of moveit planners widgets
        self.arm_config_widget.moveit_planners_config.setEnabled(should_enable)
        self.hand_config_widget.moveit_planners_config.setEnabled(should_enable)
        # If a moveit config package is provided
        if should_enable:
            # Get information from the moveit package
            parsed_controllers, parsed_planners = self.robot_interface.moveit_config.get_parsed_info()
        else:
            # Otherwise turn off the autocompletion and exit the method
            self.settings_config_widget.named_joint_states.code_editor.turn_off_autocompletion()
            return
        # Set the parsed infromation to the proper widgets
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

    def update_simulation_availability(self, is_checked):
        """
            Enables or disables some widgets according to the selected mode (simulation or physical platform)

            @param is_checked: Boolean provided by the signal stating whether the checkbox is ticked or not
        """
        self.arm_config_widget.hardware_connection_config.setEnabled(not is_checked)
        self.hand_config_widget.hardware_connection_config.setEnabled(not is_checked)

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
        sensor_is_on = self.robot_interface.robot_config.sensor_spin_box.spin_box.value() > 0
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

            @param: settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for tab_index in range(self.count()):
            widget_in_tab = self.widget(tab_index)
            widget_in_tab.save_config(settings)
        settings.endGroup()
        self.can_be_saved = False

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param: settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for tab_index in range(self.count()):
            widget_in_tab = self.widget(tab_index)
            widget_in_tab.restore_config(settings)
        settings.endGroup()
        # If removed then will always ask to save even though nothing has been modified
        self.can_be_saved = False
