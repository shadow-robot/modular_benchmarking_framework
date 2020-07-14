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

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QPushButton, QSpacerItem, QFileDialog, QInputDialog, QLineEdit
from modular_framework_core.utils.common_paths import CATKIN_WS
from modular_framework_api.utils.common_dialog_boxes import error_message, can_save_warning_message
from code_editors import GenericCodeEditor, YamlCodeEditor, XmlCodeEditor
import re
import os
from collections import OrderedDict
import copy


class GenericEditorWidget(QWidget):

    """
        Generic widget allowing the user to create new configuration files that can be modified in an editor
    """
    # Signal used to notify that the input of the editor becomes valid/invalid
    validEditorChanged = pyqtSignal(bool)
    fileEditorChanged = pyqtSignal(bool)

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(GenericEditorWidget, self).__init__(objectName="Editor {}".format(name), parent=parent)
        self.init_ui()
        self.name = name
        self.create_header()
        self.create_editor()
        self.setEnabled(enabled)

    def init_ui(self):
        """
            Set the layout
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def create_header(self):
        """
            Create a header containing the name of the widget
        """
        self.title = QLabel(self.name)
        self.layout.addWidget(self.title)

    def create_editor(self):
        """
            Initialize the code editor. Will be used by the derivated classes
        """
        self.code_editor = GenericCodeEditor()

    def set_editor_content(self, content):
        """
            Set the content of the editor

            @param content: String to be displayed in the editor
        """
        # Make sure the editor is lexed before setting any text
        if not self.code_editor.is_lexed:
            self.code_editor.set_lexer()
        self.code_editor.setText(content)
        self.setEnabled(True)


class YAMLEditorWidget(GenericEditorWidget):

    """
        Widget containing the header and editor required to work with YAML files.
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        self.file_path = None
        self.initial_path = None
        super(YAMLEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)
        self.initial_input = OrderedDict()
        self.reinit_widget_state = False

    def get_file_path(self):
        """
            Returns the path of the file linked to the editor

            @return: Path to the file corresponding to the editor's content
        """
        return self.file_path

    def create_header(self):
        """
            Create the header allowing to create, open, save, and close a new YAML file
        """
        super(YAMLEditorWidget, self).create_header()
        self.new_button = QPushButton("New")
        # Allows to have button that fit the text width
        self.new_button.setMaximumWidth(self.new_button.fontMetrics().boundingRect(self.new_button.text()).width() + 10)
        self.new_button.clicked.connect(self.new_file)
        self.open_button = QPushButton("Open")
        self.open_button.setMaximumWidth(
            self.open_button.fontMetrics().boundingRect(self.open_button.text()).width() + 10)
        self.open_button.clicked.connect(self.open_file)
        self.save_button = QPushButton("Save")
        self.save_button.setMaximumWidth(
            self.save_button.fontMetrics().boundingRect(self.save_button.text()).width() + 10)
        self.save_button.clicked.connect(self.save_file)
        self.save_as_button = QPushButton("Save as")
        self.save_as_button.setMaximumWidth(
            self.save_as_button.fontMetrics().boundingRect(self.save_as_button.text()).width() + 10)
        self.save_as_button.clicked.connect(self.save_file_as)
        self.close_button = QPushButton("Close")
        self.close_button.setMaximumWidth(
            self.close_button.fontMetrics().boundingRect(self.close_button.text()).width() + 10)
        self.close_button.clicked.connect(self.close_file)
        # Set widgets to the layout
        self.layout.addItem(QSpacerItem(30, 0), 0, 1)
        self.layout.addWidget(self.new_button, 0, 2)
        self.layout.addWidget(self.open_button, 0, 3)
        self.layout.addWidget(self.save_button, 0, 4)
        self.layout.addWidget(self.save_as_button, 0, 5)
        self.layout.addWidget(self.close_button, 0, 6)

    def create_editor(self):
        """
            Initialize and set a YAML editor to the layout
        """
        self.code_editor = YamlCodeEditor()
        self.layout.addWidget(self.code_editor, 1, 0, 1, 7)
        self.code_editor.contentIsModified.connect(self.check_arguments_validity)

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid for the given editor (will be overloaded by children)

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        if self.reinit_widget_state:
            self.reset_widget()
            return

        # Since it is a simple YAML editor we don't need to carry out more in-depth checks about the content
        self.validEditorChanged.emit(is_different)
        self.title.setText(self.name + "*" if is_different else self.name)

    def reset_widget(self):
        """
            # TODO: docu
        """
        self.code_editor.reset_init_content()
        self.reinit_widget_state = False
        self.title.setText(self.name)

    def load_file(self):
        """
            Display the content of the file linked to the editor
        """
        with open(self.file_path, "r") as file_:
            yaml_content = "".join(file_.readlines())
        self.reinit_widget_state = True
        self.set_editor_content(yaml_content)

    def open_file(self):
        """
            Open an already existing YAML file selected by the user
        """
        # TODO: put this in a common
        # If a file is already open make sure the user would not loose unaved changes
        if self.file_path and "*" in self.title.text():
            should_save = can_save_warning_message("Before closing...", "The current file has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file()
            elif should_save is None:
                return

        returned_path, _ = QFileDialog.getOpenFileName(self, "Select the {} file".format(self.name),
                                                       filter="{}(*{})".format("YAML", ".yaml"), directory=CATKIN_WS)
        if returned_path:
            self.file_path = returned_path
            self.load_file()
            self.fileEditorChanged.emit(returned_path != self.initial_path)

    def save_file(self):
        """
            Save the content of the editor to the linked file
        """
        if self.file_path is not None:
            with open(self.file_path, "w") as file_:
                file_.writelines(self.code_editor.text())
            self.reset_widget()
            self.initial_path = self.file_path

    def save_file_path(self, message):
        """
            Makes sure the user specifies the path where the file must be saved

            @param message: Message to display when asking the user where the file should be saved

            @return: Boolean stating whether the required name as been provided
        """
        file_path, _ = QFileDialog.getSaveFileName(self, message, filter="YAML(*.yaml)", directory=CATKIN_WS)
        print(repr(file_path))
        if file_path:
            if not file_path.endswith(".yaml"):
                file_path += ".yaml"
            self.file_path = file_path
        return file_path != ""

    def save_file_as(self):
        """
            Save the content of the editor to a file specified by the user
        """
        if self.save_file_path("Save configuration file as"):
            self.save_file()

    def new_file(self):
        """
            Create a new YAML file
        """
        # If a file is already open make sure the user would not loose unaved changes
        if self.file_path and "*" in self.title.text():
            should_save = can_save_warning_message("Before closing...", "The current file has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file()
            elif should_save is None:
                return
        if self.save_file_path("Save new configuration file as"):
            self.code_editor.set_lexer()
            self.code_editor.reset()
            self.fileEditorChanged.emit(True)

    def close_file(self):
        """
            Reset the editor and unlinks the editor to any file
        """
        self.reinit_widget_state = True
        self.code_editor.reinitialize()
        self.file_path = None
        self.fileEditorChanged.emit(self.file_path != self.initial_path)

    def get_yaml_formatted_content(self):
        """
            Returns the content of the editor as a dictionary

            @return: None if the content is empty, otherwise a YAML-formated dictionary
        """
        if not self.code_editor.text():
            return None

        return self.code_editor.parsed_content

    def update_number_of_elements(self):
        """
            Update the number of elements comtained in the editor by taking into account potential user input
        """
        yaml_formatted = self.get_yaml_formatted_content()
        if yaml_formatted is None:
            self.number_components = 0
        else:
            self.number_components = len(yaml_formatted)

    def save_config(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        self.save_file()
        object_name = self.objectName()
        settings.beginGroup(object_name)
        settings.setValue("type", self.metaObject().className())
        settings.setValue("enabled", self.isEnabled())
        if self.file_path is not None:
            settings.setValue("file_path", self.file_path)
            self.initial_path = self.file_path
        else:
            settings.remove("file_path")
        settings.endGroup()

    def restore_config(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        settings.beginGroup(self.objectName())
        if settings.contains("file_path"):
            self.file_path = settings.value("file_path")
            self.initial_path = self.file_path
            self.load_file()
            self.initial_path = self.file_path
        else:
            self.code_editor.clear()
        self.setEnabled(settings.value("enabled", type=bool))
        settings.endGroup()


class XMLEditorWidget(GenericEditorWidget):

    """
        Widget containing the header and editor required to work with XML files.
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(XMLEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)
        # Will contain the input arguments set by the user in the editor
        self.input_arguments = None
        # Will specify the state of the arguments (valid, etc.)
        self.arguments_status = None
        self.valid_input = None
        self.bad_format = None
        self.initial_input = []

    def get_formated_arguments(self):
        """
            Return the arguments with proper format

            @return: String containing the formated arguments
        """
        if self.input_arguments is not None:
            formated_arguments = ""
            for argument in self.input_arguments:
                formated_arguments += "<arg " + argument + "/>\n\t"
            formated_arguments = formated_arguments.rsplit("\t", 1)[0]
            return formated_arguments

    def create_editor(self):
        """
            Initialize and set a XML compatible editor to the layout
        """
        self.code_editor = XmlCodeEditor()
        self.code_editor.textChanged.connect(self.check_arguments_validity)
        self.layout.addWidget(self.code_editor)

    def check_arguments_validity(self):
        """

        """
        previous_valid = self.valid_input
        self.parse_input()
        self.update_background()
        if previous_valid != self.valid_input and self.valid_input is not None:
            # TODO: put this somewhere else
            is_different_from_initial = self.valid_input != self.initial_input
            self.validEditorChanged.emit(is_different_from_initial)
            self.title.setText(self.name + "*" if is_different_from_initial else self.name)

    def update_background(self):
        """

        """
        self.code_editor.markerDeleteAll()
        if self.valid_input is None and self.bad_format is None:
            lines = range(self.code_editor.lines())
        elif self.valid_input is None or self.bad_format is not None:
            lines = self.bad_format
        else:
            return
        for line in lines:
            self.code_editor.markerAdd(line, 1)

    def parse_input(self):
        """
            # TODO: change docu
        """
        editor_content = self.code_editor.text()
        if not editor_content:
            self.valid_input = None
            self.bad_input = None
            return
        raw_arguments = re.search("\<include file=.*?\>(.*?)\<\/include\>", editor_content, re.DOTALL)
        if raw_arguments is None:
            self.valid_input = None
            self.bad_input = None
            return

        raw_arguments = re.sub("<!-- You can add any options you want to the file -->", "", raw_arguments.group(1))
        # Strip is used to remove possible spaces at the head and tail of the string
        arguments_list = re.split("\n", raw_arguments.strip())
        filtered_arguments = [x.strip() for x in arguments_list if x]

        editor_list = re.split("\n", editor_content.strip())
        filtered_editor = [x.strip() for x in editor_list if x]

        self.valid_input = []
        self.bad_format = []
        if not filtered_arguments:
            return

        for argument in filtered_arguments:
            template_search = re.search("\<arg name\s?=\s?(.*?) value\s?=\s?(.*?)\s?\/\>", argument)
            if template_search is None:
                self.bad_format.append(filtered_editor.index(argument))
            else:
                self.valid_input.append(argument)

        if not self.valid_input:
            self.valid_input = None

    def reset_init_input(self):
        """
            # TODO: docu
        """
        self.initial_input = self.valid_input[:] if self.valid_input is not None else None
        self.title.setText(self.name)

    def save_config(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        object_name = self.objectName()
        settings.beginGroup(object_name)
        settings.setValue("type", self.metaObject().className())
        settings.setValue("enabled", self.isEnabled())
        settings.setValue("value", self.valid_input)
        settings.endGroup()
        self.reset_init_input()

    def restore_config(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        settings.beginGroup(self.objectName())
        self.valid_input = settings.value("value")
        self.setEnabled(settings.value("enabled", type=bool))
        settings.endGroup()
        # TODO: put this in a function
        if isinstance(self.valid_input, list):
            for index, input in enumerate(self.valid_input):
                self.code_editor.insertAt("  " + input + "\n", 2 + index, 0)
        self.reset_init_input()


class ComponentEditorWidget(YAMLEditorWidget):

    """
        Generic widget allowing to add components to the framework
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(ComponentEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)
        # Number of components to integrate
        self.number_components = 0
        self.valid_input = OrderedDict()

    def create_editor(self):
        """
            Create a YAML editor with a marker in the margin to add a new component
        """
        super(ComponentEditorWidget, self).create_editor()
        self.code_editor.marginClicked.connect(self.on_margin_click)
        self.code_editor.set_margin_marker()

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid to successfully integrate a new component

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """

        filtered_input = OrderedDict()
        for component_name, component_args in self.code_editor.parsed_content.items():
            if not set(["file", "action/service", "server_name", "node_name"]).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            elif all(x for x in component_args.values()):
                filtered_input[component_name] = component_args

        if self.reinit_widget_state:
            self.reset_widget()
            return
        if self.valid_input == filtered_input:
            return
        self.valid_input = copy.deepcopy(filtered_input)
        is_different_from_initial = self.valid_input != self.initial_input
        self.validEditorChanged.emit(is_different_from_initial)
        self.title.setText(self.name + "*" if is_different_from_initial else self.name)

    def on_margin_click(self, margin_index, line_index, state):
        """
            Function called when the margin is clicked
        """
        # If the click is on the marker then add a component
        if not self.code_editor.markersAtLine(line_index):
            return
        self.add_component()

    def add_component(self):
        """
            Asks the user a set of information required to successfully add a component to the framework
        """
        component_name, ok = QInputDialog().getText(self, "Input name", "Name of the component:", QLineEdit.Normal)
        # If no input is provided then exit
        if not (component_name and ok):
            return
        returned_server_path, _ = QFileDialog.getOpenFileName(self, "Select the action/service server",
                                                              filter="python(*.py);;C++(*.cpp)", directory=CATKIN_WS)
        # If no input is provided then exit
        if not returned_server_path:
            return
        # Extract the filename to display
        server_name = os.path.basename(returned_server_path)

        returned_file_path, _ = QFileDialog.getOpenFileName(self, "Select the action/service file",
                                                            filter="action(*.action);;service(*.srv)",
                                                            directory=CATKIN_WS)
        if not returned_file_path:
            error_message("Error message", "An action or service file must be provided", parent=self)
            return

        if returned_file_path.endswith(".srv"):
            filename = os.path.basename(returned_file_path).strip(".srv")
        else:
            filename = os.path.basename(returned_file_path).replace(".a", "A")

        text_to_display = "{}:\n  file: {}\n  action/service: {}\n  server_name: \n  node_name: \n  "\
                          "# You can add other parameters to configure the server here".format(component_name,
                                                                                               server_name, filename)
        self.update_number_of_elements()
        # If some components have already been added then append the text
        if self.number_components >= 1:
            text_to_display = "\n\n" + text_to_display
            self.code_editor.append(text_to_display)
        else:
            # Otherwise set the text
            self.set_editor_content(text_to_display)
        self.number_components += 1

    def load_file(self):
        """
            Loads a configuration file integrating components to the framework
        """
        super(ComponentEditorWidget, self).load_file()
        # Get the number of components contained in the file
        self.number_components = len(self.valid_input)
        self.code_editor.markerAdd(0, 1)

    def new_file(self):
        """
            Create a new file for integrating components
        """
        super(ComponentEditorWidget, self).new_file()
        self.code_editor.markerAdd(0, 1)
        self.number_components = 0

    def reset_widget(self):
        """
            # TODO: docu
        """
        super(ComponentEditorWidget, self).reset_widget()
        self.initial_input = copy.deepcopy(self.valid_input) if self.valid_input else OrderedDict()

    def save_config(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        super(ComponentEditorWidget, self).save_config(settings)
        self.reset_widget()

    def restore_config(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        super(ComponentEditorWidget, self).restore_config(settings)
        self.reset_widget()


class ROSComponentEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to integrate MoveIt! related components
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(ROSComponentEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)
        self.number_components = 0
        self.controllers_info = None
        self.planners_info = None

    def set_controllers_information(self, controllers_info):
        """
            Set information about MoveIt! controllers

            @param controllers_info: Dictionary containing information about the MoveIt! controllers. The dictionary
                                     must be formated as follow {"controller_name": [joint_name_1, joint_name_2,..], ..}
        """
        self.controllers_info = controllers_info

    def set_planners_information(self, planners_info):
        """
            Set information about MoveIt! planners

            @param planners_info: Dictionary containing information about the MoveIt! planners. The dictionary must be
                                  formated as follow {"group_name": [planner_name_1, planner_name2, ...], ...}
        """
        self.planners_info = planners_info

    def check_arguments_validity(self, is_different):
        """
            # TODO: double check whole docu
            Make sure the parsed arguments are valid to successfully integrate a new component

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        filtered_input = OrderedDict()
        # print("Parsed: {}".format(self.code_editor.parsed_content))
        for component_name, component_args in self.code_editor.parsed_content.items():
            if "controller" in self.name and "type" not in component_args:
                self.code_editor.mark_component(component_name)
            elif "planners" in self.name and not set(["planner_name", "robot_speed_factor", "number_plan_attempt", "planning_max_time"]).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            elif all(x for x in component_args.values()):
                filtered_input[component_name] = component_args
        # print("Filtered: {}".format(filtered_input))
        if self.reinit_widget_state:
            self.reset_widget()
            return
        # print("valid is {}".format(self.valid_input))
        if self.valid_input == filtered_input:
            return
        self.valid_input = copy.deepcopy(filtered_input)
        is_different_from_initial = self.valid_input != self.initial_input
        self.validEditorChanged.emit(is_different_from_initial)
        self.title.setText(self.name + "*" if is_different_from_initial else self.name)

    def add_component(self):
        """
            Add a component to the editor
        """
        if "controller" in self.name:
            message = "controller"
            items_to_display = [""] + self.controllers_info.keys() if self.controllers_info else [""]
        else:
            message = "group"
            items_to_display = self.planners_info.keys() if self.planners_info else [""]

        component_name, ok = QInputDialog().getItem(self, "Input name", "Name of the {}:".format(message),
                                                    items_to_display)
        if not (component_name and ok):
            return

        if "controller" in self.name:
            content = self.get_controller_template(component_name)
        else:
            template = "{}:\n  planner_name: \n  robot_speed_factor: \n  number_plan_attempt: \n  planning_max_time: "
            content = template.format(component_name)
            # Set autocompletion to help the user to easily find which planners are available
            if self.planners_info is not None and component_name in self.planners_info:
                self.code_editor.set_autocompletion(self.planners_info[component_name])

        self.update_number_of_elements()
        # If some components have already been added then append the text
        if self.number_components >= 1:
            content = "\n\n" + content
            self.code_editor.append(content)
        # Otherwise sets the text
        else:
            self.set_editor_content(content)
        self.number_components += 1

    def get_controller_template(self, controller_name):
        """
            Returns the text to display that provides the different inputs required to add a ROS controller

            @param controller_name: Name of the controller to add
            @return: String corresponding to the input of a ROS controller
        """
        template = "{}:\n\ttype: \n\tjoints:\n\t\t{}\n\tconstraints:\n\t\tgoal_time: "\
                   "\n\t\tstopped_velocity_tolerance: \n\t\t{}\n\tstop_trajectory_duration: "\
                   "\n\tstate_publish_rate: \n\taction_monitor_rate: \n\tallow_partial_joints_goal: "
        # Replace the "\t" by spaces so it doesn't appear in red in the editor
        template = template.replace("\t", "  ")
        if self.controllers_info and controller_name in self.controllers_info:
            joint_names = self.controllers_info[controller_name]
            first_formated_joints = "\n    ".join(list(map(lambda x: "- " + x, joint_names)))
            second_formated_joints = "\n    ".join(list(map(lambda x: x + ":", joint_names)))
        else:
            first_formated_joints = ""
            second_formated_joints = ""
        return template.format(controller_name, first_formated_joints, second_formated_joints)


class SensorEditorWidget(ComponentEditorWidget):

    """

    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(SensorEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)

    def check_arguments_validity(self, is_different):
        """
            # TODO: double check whole docu
            Make sure the parsed arguments are valid to successfully integrate a new component

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        filtered_input = OrderedDict()
        # print("Parsed: {}".format(self.code_editor.parsed_content))
        for component_name, component_args in self.code_editor.parsed_content.items():
            if not all(x for x in component_args.values()):
                self.code_editor.mark_component(component_name)
            elif not set(["data_topics", "initial_pose"]).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            elif isinstance(component_args["initial_pose"], OrderedDict):
                if not (set(["frame_id", "parent_frame_id", "position_x", "position_z", "position_z"]).issubset(set(component_args["initial_pose"])) and (set(["orientation_roll", "orientation_pitch", "orientation_yaw"]).issubset(set(component_args["initial_pose"])) or set(["quaternion_x", "quaternion_y", "quaternion_z"]).issubset(set(component_args["initial_pose"])))):
                    self.code_editor.mark_component(component_name)
            else:
                filtered_input[component_name] = component_args
        # print("Filtered: {}".format(filtered_input))
        if self.reinit_widget_state:
            self.reset_widget()
            return
        # print("valid is {}".format(self.valid_input))
        if self.valid_input == filtered_input:
            return
        self.valid_input = copy.deepcopy(filtered_input)
        is_different_from_initial = self.valid_input != self.initial_input
        self.validEditorChanged.emit(is_different_from_initial)
        self.title.setText(self.name + "*" if is_different_from_initial else self.name)

    def add_component(self):
        """
            Asks the user a set of information required to add a new sensor to the framework
        """
        component_name, ok = QInputDialog().getText(self, "Input name", "Name of the sensor:", QLineEdit.Normal)
        # If no input is provided then exit
        if not (component_name and ok):
            return

        text_to_display = self.get_sensor_template(component_name)
        self.update_number_of_elements()
        # If some components have already been added then append the text
        if self.number_components >= 1:
            text_to_display = "\n\n" + text_to_display
            self.code_editor.append(text_to_display)
        else:
            # Otherwise set the text
            self.set_editor_content(text_to_display)
        self.number_components += 1

    def get_sensor_template(self, sensor_name):
        """
            Returns the text to display that provides the different inputs required to add a sensor to the framework

            @param controller_name: Name of the controller to add
            @return: String corresponding to the input of a sensor
        """
        template = "{}:\n\tdata_topics: \n\tinitial_pose: \n\t\t"\
                   "# You can simplify this part by defining a pose in the pose editor\n\t\tframe_id: \n\t\t"\
                   "parent_frame_id: \n\t\tposition_x: \n\t\tposition_y: \n\t\tposition_z: \n\t\t"\
                   "# You can also define the orientation using quaternion\n\t\torientation_roll: \n\t\t"\
                   "orientation_pitch: \n\t\torientation_yaw: "
        # Replace the "\t" by spaces so it doesn't appear in red in the editor
        template = template.replace("\t", "  ")
        return template.format(sensor_name)
