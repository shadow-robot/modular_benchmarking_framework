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

from PyQt5.QtWidgets import QFileDialog, QInputDialog, QLineEdit
from modular_framework_core.utils.common_paths import CATKIN_WS
from modular_framework_api.utils.common_dialog_boxes import error_message
from plain_editor_widgets import YAMLEditorWidget
import os
from collections import OrderedDict
import copy


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
        # Fields a components must contain to be integrated to the framework
        self.mandatory_fields = ["file", "action/service", "server_name", "node_name"]
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
            is_dict = isinstance(component_args, OrderedDict)
            # If the argument is not a dict or does not contain the mandatory fields when mark it as wrong
            if not is_dict or not set(self.mandatory_fields).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            # If every field has a non empty value add it to the filtered_input
            elif all(x for x in component_args.values()):
                filtered_input[component_name] = component_args
        self.handle_valid_input_change(filtered_input)

    def handle_valid_input_change(self, new_input):
        """
            Given a new input update potentially hte valid_input attribute and emit a signal if needed

            @param new_input: Dictionary containing the new valid input corresponding to the editor's content
        """
        # If the valid input has not changed then quit
        if self.valid_input == new_input:
            return
        # Set the new valid input
        self.valid_input = copy.deepcopy(new_input)
        # If the widget needs to be reset do it and quit
        if self.reinit_widget_state:
            self.reset_widget()
            return
        # Otherwise check that the changed valid input is different than the initial
        is_different_from_initial = self.valid_input != self.initial_input
        # Emit the signal and set a * to show that a valid change has occured
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
            Set the current state of the widget as the initial one
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
        if "controller" in self.name:
            self.mandatory_fields = "type"
        else:
            self.mandatory_fields = ["planner_name", "robot_speed_factor", "number_plan_attempt", "planning_max_time"]

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
            Make sure the parsed arguments are valid to successfully integrate a new ROS component

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        filtered_input = OrderedDict()
        for component_name, component_args in self.code_editor.parsed_content.items():
            is_dict = isinstance(component_args, OrderedDict)
            if not is_dict:
                self.code_editor.mark_component(component_name)
            elif "controller" in self.name and self.mandatory_fields not in component_args:
                self.code_editor.mark_component(component_name)
            elif "planners" in self.name and not set(self.mandatory_fields).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            elif all(x for x in component_args.values()):
                filtered_input[component_name] = component_args
        self.handle_valid_input_change(filtered_input)

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
