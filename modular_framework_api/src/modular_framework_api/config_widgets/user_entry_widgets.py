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

import os
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QLineEdit, QToolButton, QFileDialog
import rospkg
from modular_framework_core.utils.common_paths import CATKIN_WS
from modular_framework_api.utils.files_specifics import FILE_TO_EXTENSION


class GenericUserEntryWidget(QWidget):

    """
        # TODO: doc and see if not possible to align beginning of entry line
    """

    def __init__(self, entry_name, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(GenericUserEntryWidget, self).__init__(parent=parent)
        self.entry_name = entry_name
        self.valid_input = None
        # Set the name of the object (UE stands for user entry)
        self.setObjectName("UE {}".format(entry_name))
        self.init_ui()
        self.create_entry(browser_button, placeholder_text, enabled)

    def init_ui(self):
        """
            Set the layout of the widget
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_entry(self, browser_button, placeholder_text, enabled):
        """
        """
        # Label to display what the entry is about
        self.entry_label = QLabel(self.entry_name + ":", objectName="label {}".format(self.entry_name))
        # Line edit entry to let the user write and read the content of the entry
        self.entry_edit_line = QLineEdit(objectName="entry_edit_line {}".format(self.entry_name))
        # Set a small button allowing to clear the whole line
        self.entry_edit_line.setClearButtonEnabled(True)
        self.entry_edit_line.textChanged.connect(self.check_input_validity)
        # If specified, set a placeholder text to show for isntance what is expected in this entry
        if placeholder_text is not None:
            self.entry_edit_line.setPlaceholderText(placeholder_text)
        # Enables or disables access to this entry
        for widget in (self.entry_label, self.entry_edit_line):
            widget.setEnabled(enabled)

        # If the user is expected to select a file or folder, create the corresponding button to do so
        self.layout.addWidget(self.entry_label, 0, 0)
        self.layout.addWidget(self.entry_edit_line, 0, 1)
        if browser_button:
            self.browser_tool_button = QToolButton(objectName="browser {}".format(self.entry_name))
            self.browser_tool_button.setText("...")
            self.browser_tool_button.setEnabled(enabled)
            self.browser_tool_button.clicked.connect(self.fill_line_with_browsing)
            self.layout.addWidget(self.browser_tool_button, 0, 2)
        else:
            self.browser_tool_button = None
        self.setLayout(self.layout)

    def fill_line_with_browsing(self):
        """
            Open a window allowing the user to browse through the file system to provide either a file or a directory
            and automatically set it to the entry edit line
        """
        # Get the filter information for the given user entry
        filter_info = FILE_TO_EXTENSION[self.entry_name]
        # If it's not empty then open a dialog box allowing the user to select a file
        if filter_info:
            description, name, extension = filter_info
            returned_path, _ = QFileDialog.getOpenFileName(self, "Select the {} file".format(description),
                                                           filter="{}(*{})".format(name, extension),
                                                           directory=CATKIN_WS)
        # Otherwise it means what is expected is a folder
        else:
            returned_path = QFileDialog.getExistingDirectory(self, "Select the {} ".format(self.entry_name.lower()),
                                                             directory=CATKIN_WS)
        # If a folder or file has been selected then set the text with its path
        if returned_path:
            self.entry_edit_line.setText(returned_path)

    def set_enabled(self, should_enable):
        """
        """
        self.entry_label.setEnabled(should_enable)
        self.entry_edit_line.setEnabled(should_enable)
        if self.browser_tool_button is not None:
            self.browser_tool_button.setEnabled(should_enable)

    def check_input_validity(self):
        """
        """
        pass

    def set_valid_entry_line_background(self, valid):
        """
        """
        if valid:
            self.entry_edit_line.setStyleSheet("background-color: rgb(255, 255, 255)")
        else:
            self.entry_edit_line.setStyleSheet("background-color: rgba(255, 0, 0, 75)")


class UrdfEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(UrdfEntryWidget, self).__init__("Robot's URDF file", browser_button, placeholder_text, enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        is_valid_file = current_input.endswith(".urdf") or current_input.endswith(".urdf.xacro")
        if current_input and not(os.path.isfile(current_input) and is_valid_file):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)


class UrdfArgumentsEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(UrdfArgumentsEntryWidget, self).__init__("URDF arguments (optional)", browser_button, placeholder_text,
                                                       enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        space_split_input = current_input.split(" ")
        if current_input and any(":=" not in x for x in space_split_input):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)


class LaunchFileEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(LaunchFileEntryWidget, self).__init__("Custom launch file", browser_button, placeholder_text,
                                                    enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        is_extension_valid = current_input.endswith(".launch")
        is_in_ros_pkg = rospkg.get_package_name(current_input) is not None
        if current_input and not(is_extension_valid and is_in_ros_pkg):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)


class CollisionFileEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(CollisionFileEntryWidget, self).__init__("Collision scene", browser_button, placeholder_text,
                                                       enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        is_valid_file = current_input.endswith(".scene")
        if current_input and not(os.path.isfile(current_input) and is_valid_file):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)


class GazeboWorldEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(GazeboWorldEntryWidget, self).__init__("Gazebo world file", browser_button, placeholder_text,
                                                     enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        is_valid_file = current_input.endswith(".world")
        if current_input and not(os.path.isfile(current_input) and is_valid_file):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)


class GazeboFolderEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(GazeboFolderEntryWidget, self).__init__("Gazebo model folder", browser_button, placeholder_text,
                                                      enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        if current_input and not os.path.isdir(current_input):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)


class StartingPoseEntryWidget(GenericUserEntryWidget):

    """
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
        """
        super(StartingPoseEntryWidget, self).__init__("Starting pose", browser_button, placeholder_text,
                                                      enabled, parent)

    def check_input_validity(self):
        """
        """
        current_input = self.entry_edit_line.text()
        if current_input and not os.path.isdir(current_input):
            self.set_valid_entry_line_background(False)
        else:
            self.set_valid_entry_line_background(True)
