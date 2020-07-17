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

from plain_editor_widgets import YAMLEditorWidget
from component_editor_widgets import ComponentEditorWidget
from collections import OrderedDict
import copy


class JointStateEditorWidget(YAMLEditorWidget):

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
        super(JointStateEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)
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

    def load_file(self):
        """
            Loads a configuration file integrating components to the framework
        """
        super(JointStateEditorWidget, self).load_file()
        # Get the number of components contained in the file
        self.number_components = len(self.valid_input)

    def new_file(self):
        """
            Create a new file for integrating components
        """
        super(JointStateEditorWidget, self).new_file()
        self.number_components = 0

    def reset_widget(self):
        """
            Set the current state of the widget as the initial one
        """
        super(JointStateEditorWidget, self).reset_widget()
        self.initial_input = copy.deepcopy(self.valid_input) if self.valid_input else OrderedDict()

    def save_config(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        super(JointStateEditorWidget, self).save_config(settings)
        self.reset_widget()

    def restore_config(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        super(JointStateEditorWidget, self).restore_config(settings)
        self.reset_widget()


class PoseEditorWidget(ComponentEditorWidget):

    """

    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super(PoseEditorWidget, self).__init__(name=name, enabled=enabled, parent=parent)

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
