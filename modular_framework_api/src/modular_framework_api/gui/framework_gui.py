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

import sys
import rospy
import signal
from PyQt5.QtWidgets import QMainWindow, QAction, QApplication, QTabWidget, QFileDialog
from PyQt5.QtCore import QFileInfo, QSettings
from modular_framework_core.utils.common_paths import (GUI_CONFIGS_FOLDER, ROBOT_INTEGRATION_MAIN_CONFIG_FILE,
                                                       ROBOT_INTEGRATION_DEFAULT_CONFIG_FILE
                                                       )
from robot_integration_area import RobotIntegrationArea
from modular_framework_api.utils.common_dialog_boxes import can_save_warning_message


class FrameworkGui(QMainWindow):

    """
        Main window containing the GUI of the framework
    """

    # Main settings of the UI
    settings = QSettings(ROBOT_INTEGRATION_MAIN_CONFIG_FILE, QSettings.IniFormat)

    def __init__(self):
        """
            Initialize the class by generating the UI
        """
        super(FrameworkGui, self).__init__()
        self.init_ui()
        # Contains the widget configurations allowing to load previous robot integration config
        self.config_file_path = ROBOT_INTEGRATION_DEFAULT_CONFIG_FILE
        self.init_config()

    def init_ui(self):
        """
            Set up the geometry of the main window and create the main widgets
        """
        self.init_main_widget()

        # status bar
        self.initialize_status_bar()
        # actions
        self.create_actions()
        # menus
        self.create_menus()
        # set window properties
        self.setGeometry(200, 200, 1000, 800)
        self.setWindowTitle("Modular Benchmarking Framework GUI")
        self.showMaximized()
        self.show()

    def init_main_widget(self):
        """
            Initialize the main widget that will contain all the others
        """
        # Initialize the tab widget (central widget)
        self.tab_container = QTabWidget(self)
        # Widget containing all the components required to configure and launch a robot
        self.robot_integration_area = RobotIntegrationArea(self)
        # Update the menu related to the robot
        self.robot_integration_area.robotCanBeLaunched.connect(self.update_robot_launch_action)
        self.robot_integration_area.robotCanBeStopped.connect(self.update_robot_stop_action)
        # Add the widget to the first tab
        self.tab_container.addTab(self.robot_integration_area, "Integrate a robot")
        self.setCentralWidget(self.tab_container)

    def initialize_status_bar(self):
        """
            Create and set a welcome message to the status bar
        """
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("GUI launched -- Welcome!", 10000)

    def create_actions(self):
        """
            Create the different actions composing menus
        """
        # Allows to open a robot integration config
        self.action_open = QAction('&Open', self, shortcut='Ctrl+O', statusTip="Open file", triggered=self.open_file)
        # Save the current robot integration config
        self.action_save = QAction('&Save', self, shortcut='Ctrl+S', statusTip="Save file", triggered=self.save_file)
        # Save the current robot integration as
        self.action_save_as = QAction('Save &As...', self, shortcut='Ctrl+Shift+S', statusTip="Save file as...",
                                      triggered=self.save_file_as)
        # Exit the application
        self.action_exit = QAction('E&xit', self, shortcut='Ctrl+Q', statusTip="Exit application", triggered=self.exit)

        # Launch the robot
        self.launch_robot = QAction("&Launch", self, shortcut="Ctrl+L", statusTip="Launch the robot", enabled=False,
                                    triggered=self.robot_integration_area.launch_robot)
        self.stop_robot = QAction("Sto&p", self, shortcut="Ctrl+P", statusTip="Stop the robot", enabled=False,
                                  triggered=self.robot_integration_area.stop_robot)

    def create_menus(self):
        """
            Create the "File" menu allowing to manage the robot integration config
        """
        # Create a menu bar
        menubar = self.menuBar()
        # Add the "File" menu
        self.file_menu = menubar.addMenu('&File')
        # Add the different actions
        self.file_menu.addAction(self.action_open)
        self.file_menu.addAction(self.action_save)
        self.file_menu.addAction(self.action_save_as)
        self.file_menu.addSeparator()
        self.file_menu.addAction(self.action_exit)

        # Add the "Robot" menu
        self.robot_menu = menubar.addMenu('&Robot')
        self.robot_menu.addAction(self.launch_robot)
        self.robot_menu.addAction(self.stop_robot)

    def update_robot_launch_action(self, is_robot_launchable):
        """
            Enable/Disable the action allowing to launch the robot

            @param is_robot_launchable: Boolean coming from the signal and stating whether the robot can be launched
        """
        self.launch_robot.setEnabled(is_robot_launchable)

    def update_robot_stop_action(self, is_robot_running):
        """
            Enable/Disable the action allowing to stop the robot

            @param is_robot_running: Boolean coming from the signal and stating whether the robot can be stopped
        """
        self.stop_robot.setEnabled(is_robot_running)

    def open_file(self):
        """
            Open an already created robot integration config file
        """

        robot_config_path, _ = QFileDialog.getOpenFileName(self, "Open robot integration config file",
                                                           filter="ini(*.ini)",
                                                           directory=GUI_CONFIGS_FOLDER)
        if robot_config_path:
            self.config_file_path = robot_config_path
            self.settings.setValue("latest_config", robot_config_path)
            self.init_config()

    def save_file(self):
        """
            Save the current robot integration config file
        """
        current_widget = self.robot_integration_area
        current_widget.save_config(self.latest_config)
        self.settings.setValue("latest_config", self.config_file_path)

    def save_file_as(self):
        """
            Save the current robot integration config file with a specific name provided by the user
        """
        robot_config_path, _ = QFileDialog.getSaveFileName(self, "Save robot integration config file as",
                                                           filter="ini(*.ini)",
                                                           directory=GUI_CONFIGS_FOLDER)
        # Ensures we get the correct extension for the config file
        if not robot_config_path.endswith(".ini"):
            robot_config_path += ".ini"

        self.config_file_path = robot_config_path
        self.latest_config = QSettings(self.config_file_path, QSettings.IniFormat)
        self.save_file()

    def check_if_save(self):
        """
            Check whether some changes are not saved. If it is the case ask the user what to do

            @return: True saving has been perofrmed by the user request, False changes are not to be saved and None
                     if user clicked on Cancel
        """
        should_save = False
        # Notifies the user that there are unsaved changes that can be lost
        if self.robot_integration_area.can_be_saved:
            should_save = can_save_warning_message("Before leaving...", "The robot configuration has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file()
                # Must call sync to force Qt to remain open, otherwise close without generating the files
                self.settings.sync()
                self.latest_config.sync()
        return should_save

    def exit(self):
        """
            Exit the GUI
        """
        if self.check_if_save() is None:
            return
        # If a robot is running, kill the process before exiting
        self.robot_integration_area.stop_robot()
        QApplication.exit()

    def closeEvent(self, event):
        """
            Overwrites the default behaviour by calling the check_if_save function before proceeding
        """
        if self.check_if_save() is not None:
            event.accept()
        else:
            event.ignore()

    def init_config(self):
        """
            Restore if possible all the widgets to their state saved into a given configuration
        """
        # If a configuration has already been saved in a file, get its path
        if self.settings.contains("latest_config"):
            self.config_file_path = self.settings.value("latest_config")

        # Create a Qt info file required to save the state of each widget
        info_file = QFileInfo(self.config_file_path)
        # Initialize the Qt settings file
        self.latest_config = QSettings(self.config_file_path, QSettings.IniFormat)
        # Introspect all the children widgets and call their restore_config() function
        if info_file.exists() and info_file.isFile():
            widget_names = self.latest_config.childGroups()
            for widget_name in widget_names:
                widget = self.findChild(self.str_to_class(widget_name + "/type"), widget_name)
                widget.restore_config(self.latest_config)

    def str_to_class(self, classname):
        """
            Turns a string (name of a class) into into its class type

            @param classname: String corresponding to the name of a class
            @return: Type of class
        """
        return getattr(sys.modules[__name__], self.latest_config.value(classname))


if __name__ == "__main__":
    rospy.init_node("framework_GUI")
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    framework_gui = FrameworkGui()
    framework_gui.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
