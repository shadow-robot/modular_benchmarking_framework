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
from modular_framework_api.utils.common_checks import is_pose_valid, is_topic_valid, is_moveit_planner_valid
from plain_editor_widgets import YAMLEditorWidget
import os
from collections import OrderedDict
import copy


class ComponentEditorWidget(YAMLEditorWidget):

    """
        Generic widget allowing to add components to the framework
    """

    def __init__(self, name, enabled=False, margin_marker=True, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param margin_marker: Boolean stating if the editor should contain a marker helping adding a component
            @param parent: parent of the widget
        """
        self.margin_marker = margin_marker
        super(ComponentEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)
        # Number of components to integrate
        self.number_components = 0
        # Fields a components must contain to be integrated to the framework
        self.mandatory_fields = ["file", "action/service", "server_name", "node_name"]

    def create_editor(self):
        """
            Create a YAML editor with a marker in the margin to add a new component
        """
        super(ComponentEditorWidget, self).create_editor()
        if self.margin_marker:
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
            elif all(isinstance(x, str) for x in component_args.values()):
                filtered_input[component_name] = component_args
            else:
                self.code_editor.mark_component(component_name)
        self.handle_valid_input_change(filtered_input, is_different)

    def handle_valid_input_change(self, new_input, is_different):
        """
            Given a new input, update potentially the valid_input attribute and emit a signal if needed

            @param new_input: Dictionary containing the new valid input corresponding to the editor's content
        """
        current_valid = new_input if not self.code_editor.wrong_format_lines else None
        if not self.should_emit_signal:
            self.initial_input = copy.deepcopy(current_valid) if current_valid is not None else None
            self.should_emit_signal = True
        if current_valid != self.valid_input:
            self.valid_input = current_valid
            self.canBeSaved.emit(self.valid_input != self.initial_input and self.valid_input is not None)

        if self.code_editor.wrong_format_lines:
            test = True
        else:
            test = is_different
        # print("current: {}".format(current_valid))
        # print("init: {}".format(self.initial_input))
        # print("valid: {}".format(self.valid_input))
        # print("test: {}".format(test))
        # print("===================================================================================")
        # Since it is a simple YAML editor we don't need to carry out more in-depth checks about the content
        self.title.setText(self.name + "*" if test and self.file_path else self.name)
        # self.title.setText(self.name + "*" if self.valid_input != self.initial_input and self.file_path else self.name)
        if self.update_init_state:
            self.update_init_widget()

        # If the valid input has not changed then quit
        # if self.valid_input == new_input:
        #     return
        # # Set the new valid input
        # self.valid_input = copy.deepcopy(new_input)
        # # Otherwise check that the changed valid input is different than the initial
        # is_different_from_initial = self.valid_input != self.initial_input
        # if not self.should_emit_signal:
        #     self.initial_input = copy.deepcopy(self.code_editor.parsed_content)
        #     self.should_emit_signal = True
        # # if self.should_emit_signal:
        #     # Emit the signal and set a * to show that a valid change has occured
        # self.validEditorChanged.emit(is_different_from_initial)
        # # else:
        #     # self.initial_input = copy.deepcopy(self.valid_input)
        #     # self.should_emit_signal = True
        # # self.validEditorChanged.emit(is_different_from_initial)
        # self.title.setText(self.name + "*" if is_different_from_initial and self.file_path else self.name)
        # # If the widget needs to be reset do it and quit
        # if self.update_init_state:
        #     self.update_init_widget()

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

        self.append_template(text_to_display)

    def append_template(self, template):
        """
            Method that properly appends the template of a new component to the editor

            @param template: String corresponding to the text to set
        """
        self.update_number_of_elements()
        # If some components have already been added then append the text
        if self.number_components >= 1:
            template = "\n\n" + template
            self.code_editor.append(template)
        else:
            # Otherwise set the text
            self.set_editor_content(template)
        self.number_components += 1

    def load_file(self):
        """
            Loads a configuration file integrating components to the framework
        """
        super(ComponentEditorWidget, self).load_file()
        # Get the number of components contained in the file
        self.number_components = len(self.valid_input)
        if self.file_path and self.margin_marker:
            self.code_editor.markerAdd(0, 1)

    def new_file(self):
        """
            Create a new file for integrating components
        """
        super(ComponentEditorWidget, self).new_file()
        self.number_components = 0
        if self.file_path and self.margin_marker:
            self.code_editor.markerAdd(0, 1)

    # def update_init_widget(self):
    #     """
    #         Set the current state of the widget as the initial one
    #     """
    #     super(ComponentEditorWidget, self).update_init_widget()
    #     self.initial_input = copy.deepcopy(self.valid_input) if self.valid_input else OrderedDict()

    # def save_config(self, settings):
    #     """
    #         Save the current state of the widget
    #
    #         @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
    #     """
    #     super(ComponentEditorWidget, self).save_config(settings)
    #     self.reset_widget()
    #
    # def restore_config(self, settings):
    #     """
    #         Set the different components of the widget according to a specified configuration
    #
    #         @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
    #     """
    #     super(ComponentEditorWidget, self).restore_config(settings)
    #     self.reset_widget()


class MoveItPlannerEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to configure MoveIt! planners
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(MoveItPlannerEditorWidget, self).__init__(name=name, enabled=enabled, margin_marker=True, parent=parent)
        self.number_components = 0
        self.planners_info = None
        self.mandatory_fields = ["planner_name", "robot_speed_factor", "number_plan_attempt", "planning_max_time"]

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
            elif not set(self.mandatory_fields).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            elif is_moveit_planner_valid(component_args):
                filtered_input[component_name] = component_args
            else:
                self.code_editor.mark_component(component_name)
        self.handle_valid_input_change(filtered_input, is_different)

    def add_component(self):
        """
            Add a component to the editor
        """
        items_to_display = self.planners_info.keys() if self.planners_info else [""]

        component_name, ok = QInputDialog().getItem(self, "Input name", "Name of the group:", items_to_display)
        if not (component_name and ok):
            return

        template = "{}:\n  planner_name: \n  robot_speed_factor: \n  number_plan_attempt: \n  planning_max_time: "
        content = template.format(component_name)
        # Set autocompletion to help the user to easily find which planners are available
        if self.planners_info is not None and component_name in self.planners_info:
            self.code_editor.set_autocompletion(self.planners_info[component_name])

        self.append_template(content)


class RosControllersEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to integrate ROS controllers for the given hardware
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(RosControllersEditorWidget, self).__init__(name=name, enabled=enabled, margin_marker=True, parent=parent)
        self.number_components = 0
        self.controllers_info = None
        self.mandatory_fields = "type"

    def set_controllers_information(self, controllers_info):
        """
            Set information about MoveIt! controllers

            @param controllers_info: Dictionary containing information about the MoveIt! controllers. The dictionary
                                     must be formated as follow {"controller_name": [joint_name_1, joint_name_2,..], ..}
        """
        self.controllers_info = controllers_info

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
            elif self.mandatory_fields not in component_args:
                self.code_editor.mark_component(component_name)
            elif all(x for x in component_args.values()):
                filtered_input[component_name] = component_args
        self.handle_valid_input_change(filtered_input, is_different)

    def add_component(self):
        """
            Add a controller to the editor
        """
        items_to_display = self.controllers_info.keys() if self.controllers_info else [""]

        component_name, ok = QInputDialog().getItem(self, "Input name", "Name of the controller:", items_to_display)
        if not (component_name and ok):
            return

        content = self.get_controller_template(component_name)

        self.append_template(content)

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


class JointStateEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to define pre-recorded joint states that can be used in the state machine
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(JointStateEditorWidget, self).__init__(name=name, enabled=enabled, margin_marker=False, parent=parent)
        # Number of components to integrate
        self.number_components = 0
        self.valid_input = OrderedDict()

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid to successfully integrate a new component

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        filtered_input = OrderedDict()
        for component_name, component_args in self.code_editor.parsed_content.items():
            # If the argument is not a dict then mark it as wrong
            if not isinstance(component_args, OrderedDict):
                self.code_editor.mark_component(component_name)
            # If every field has a non empty value and has the proper type of data add it to the filtered_input
            elif all(isinstance(x, float) or isinstance(x, int) for x in component_args.values()):
                filtered_input[component_name] = component_args
            else:
                self.code_editor.mark_component(component_name)
        self.handle_valid_input_change(filtered_input, is_different)


class PoseEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to add pre-recorded poses and cartesian poses
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(PoseEditorWidget, self).__init__(name=name, enabled=enabled, margin_marker=True, parent=parent)
        self.mandatory_fields = "reference_frame"
        self.poses = OrderedDict()
        self.cartesian_poses = OrderedDict()

    def add_component(self):
        """
            Asks the user a set of information required to successfully add a pose to the framework
        """
        pose_name, ok = QInputDialog().getText(self, "Input name", "Name of the pose:", QLineEdit.Normal)
        # If no input is provided then exit
        if not (pose_name and ok):
            return

        text_to_display = "{}:\n  reference_frame: \n  position: {{x: , y: , z: }}\n  "\
                          "# You can change r,p,y to x,y,z,w for quaternion\n  "\
                          "orientation: {{r: , p: , y: }}".format(pose_name)

        self.append_template(text_to_display)

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid to successfully integrate a new component

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        # Reinitialise the two dictionaries
        self.poses = OrderedDict()
        self.cartesian_poses = OrderedDict()
        # Keep only the properly formated elements
        filtered_input = OrderedDict()
        for component_name, component_args in self.code_editor.parsed_content.items():
            is_dict = isinstance(component_args, OrderedDict)
            # If the argument is not a dict or does not contain a valid mandatory field then mark it as wrong
            if not is_dict or self.mandatory_fields not in component_args:
                self.code_editor.mark_component(component_name)
                continue
            # If the component is a pose (not a cartesian pose)
            if set(["reference_frame", "position", "orientation"]) == set(component_args):
                # If the pose is valid add it to the filtered
                if is_pose_valid(component_args):
                    filtered_input[component_name] = component_args
                    self.poses[component_name] = component_args
                # Otherwise mark the component as wrong
                else:
                    self.code_editor.mark_component(component_name)
                continue
            # If only the reference frame is properly set
            elif set(["reference_frame"]) == set(component_args):
                self.code_editor.mark_component(component_name)
                continue
            # If the pose is a potential cartesian pose
            is_cartesian_pose = True
            # Make sure the component is a cartesian pose
            for end_effector_name, ee_args in component_args.items():
                # Ignore the reference_frame field
                if end_effector_name == "reference_frame":
                    continue
                # If the corresponding value is not a dictionary or is not properly formatted then switch the boolean
                # to false and mark the component as wrong
                is_bad_format = not is_pose_valid(ee_args, add_reference_frame=False)
                if not isinstance(ee_args, OrderedDict) or is_bad_format:
                    is_cartesian_pose = False
                    self.code_editor.mark_component(component_name)
                    break
            # If the component is indeed a cartesian pose add it to the filtered elements
            if is_cartesian_pose:
                filtered_input[component_name] = component_args
                self.cartesian_poses[component_name] = component_args

        self.handle_valid_input_change(filtered_input, is_different)


class TrajectoryEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to create trajectories from pre-recorded joint states
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(TrajectoryEditorWidget, self).__init__(name=name, enabled=enabled, margin_marker=True, parent=parent)
        self.mandatory_fields = ["name", "interpolate_time", "pause_time"]
        self.known_checkpoints = list()

    def add_component(self):
        """
            Asks the user a set of information required to successfully add a pose to the framework
        """
        pose_name, ok = QInputDialog().getText(self, "Input name", "Name of the trajectory:", QLineEdit.Normal)
        # If no input is provided then exit
        if not (pose_name and ok):
            return

        text_to_display = "{}:\n  - {{name: , interpolate_time: , pause_time: }}\n  "\
                          "- {{name: , interpolate_time: , pause_time: }}".format(pose_name)
        self.append_template(text_to_display)

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid to successfully add a new trajectory

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        filtered_input = OrderedDict()
        for component_name, component_args in self.code_editor.parsed_content.items():
            is_list = isinstance(component_args, list)
            # If the argument is not a list then mark the component as wrong
            if not is_list:
                self.code_editor.mark_component(component_name)
                continue
            # If any element of the list is not a dictionary then mark the component as wrong
            if any(not isinstance(x, OrderedDict) for x in component_args):
                self.code_editor.mark_component(component_name)
                continue
            is_trajectory_valid = True
            for checkpoint in component_args:
                if not self.is_checkpoint_valid(checkpoint):
                    is_trajectory_valid = False
                    self.code_editor.mark_component(component_name)
                    break
            if is_trajectory_valid:
                filtered_input[component_name] = component_args

        self.handle_valid_input_change(filtered_input, is_different)

    def is_checkpoint_valid(self, checkpoint):
        """
            Check whether a dictionary representing a checkpoint is properly formatted

            @param checkpoint: Dictionary that describe a point of the trajectory
            @return: True if the dictionary has the proper format, False otherwise
        """
        if set(self.mandatory_fields) != set(checkpoint):
            return False
        elif not isinstance(checkpoint["name"], str) or checkpoint["name"] not in self.known_checkpoints:
            return False
        elif not (isinstance(checkpoint["interpolate_time"], float) or isinstance(checkpoint["interpolate_time"], int)):
            return False
        elif not (isinstance(checkpoint["pause_time"], float) or isinstance(checkpoint["pause_time"], int)):
            return False
        return True

    def set_known_checkpoints(self, checkpoints):
        """
            Update the known checkpoints defined in other editors and rerun a check on the current editor's content

            @param checkpoints: List of valid named joint states defined in the corresponding editor
        """
        self.known_checkpoints = checkpoints[:]
        self.code_editor.parse_and_format_editor()


class SensorEditorWidget(ComponentEditorWidget):

    """
        Widget allowing to integrate sensors to the framework
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(SensorEditorWidget, self).__init__(name=name, enabled=enabled, margin_marker=True, parent=parent)
        self.mandatory_fields = ["data_topics", "initial_pose"]
        self.known_poses = list()

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid to successfully integrate a new sensor

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        filtered_input = OrderedDict()
        for component_name, component_args in self.code_editor.parsed_content.items():
            is_dict = isinstance(component_args, OrderedDict)
            if not is_dict or not set(self.mandatory_fields).issubset(set(component_args)):
                self.code_editor.mark_component(component_name)
            elif not is_topic_valid(component_args["data_topics"]):
                self.code_editor.mark_component(component_name)
            elif isinstance(component_args["initial_pose"], str):
                if component_args["initial_pose"] in self.known_poses:
                    filtered_input[component_name] = component_args
                else:
                    self.code_editor.mark_component(component_name)
            elif not is_pose_valid(component_args["initial_pose"], add_frame_id=True):
                self.code_editor.mark_component(component_name)
            else:
                filtered_input[component_name] = component_args

        self.handle_valid_input_change(filtered_input, is_different)

    def add_component(self):
        """
            Asks the user a set of information required to add a new sensor to the framework
        """
        component_name, ok = QInputDialog().getText(self, "Input name", "Name of the sensor:", QLineEdit.Normal)
        # If no input is provided then exit
        if not (component_name and ok):
            return

        text_to_display = self.get_sensor_template(component_name)
        self.append_template(text_to_display)

    def get_sensor_template(self, sensor_name):
        """
            Returns the text to display that provides the different inputs required to add a sensor to the framework

            @param controller_name: Name of the controller to add
            @return: String corresponding to the input of a sensor
        """
        template = "{}:\n\tdata_topics: \n\tinitial_pose: \n\t\t"\
                   "# You can simplify this part by defining a pose in the pose editor\n\t\tframe_id: \n\t\t"\
                   "reference_frame: \n\t\tposition: {{x: , y: , z: }}\n\t\t# You can use z,y,z,w for quaternion\n\t\t"\
                   "orientation: {{r: , p: , y: }}"
        # Replace the "\t" by spaces so it doesn't appear in red in the editor
        template = template.replace("\t", "  ")
        return template.format(sensor_name)

    def set_known_poses(self, poses):
        """
            Update the known poses defined in other editors and rerun a check on the current editor's content

            @param checkpoints: List of valid named poses defined in the corresponding editor
        """
        self.known_poses = poses[:]
        self.code_editor.parse_and_format_editor()
