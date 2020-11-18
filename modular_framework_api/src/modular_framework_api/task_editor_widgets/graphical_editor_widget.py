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

from PyQt5.QtWidgets import QGridLayout, QWidget
from PyQt5.QtCore import Qt
from task_editor_scene import TaskEditorScene
from modular_framework_api.task_editor_graphics.view import TaskEditorView
from state_machine import StateMachine
from modular_framework_core.utils.file_parsers import (extract_state_machine_parameters_from_file,
                                                       AVAILABLE_STATEMACHINES)
from modular_framework_core.utils.common_paths import TASK_EDITOR_ROOT_TEMPLATE


class GraphicalEditorWidget(QWidget):

    """
        Widget gathering the scene and view allowing the user to edit state machines
    """

    def __init__(self, state_machine_name, state_machine_type, parent=None):
        """
            Initialize the widget

            @param state_machine_name: Name given to both the state machine and this widget
            @param state_machine_type: Type of the state machine to load
            @param parent: Parent of the widget
        """
        super(GraphicalEditorWidget, self).__init__(parent=parent)
        self.init_ui()
        self.set_base_state_machine(state_machine_type)
        self.set_name(state_machine_name)

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Main layout
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        # Create graphics scene
        self.scene = TaskEditorScene()
        # Create graphics view
        self.editor_view = TaskEditorView(self.scene.graphics_scene, self)
        self.layout.addWidget(self.editor_view)
        # Make sure that if the window containing the widget is deleted, remove this widget as well
        self.setAttribute(Qt.WA_DeleteOnClose)

    def set_base_state_machine(self, state_machine_type):
        """
            Set this widget's base state machine

            @param state_machine_type: Type of the state machine to be added
        """
        if state_machine_type == "base":
            state_machine_parameters = extract_state_machine_parameters_from_file(TASK_EDITOR_ROOT_TEMPLATE)
        else:
            state_machine_parameters = AVAILABLE_STATEMACHINES[state_machine_type]["parameters"]
        self.base_state_machine = StateMachine(graphical_editor_widget=self, parameters=state_machine_parameters)

    def set_name(self, name):
        """
            Set the name of both the subwindow and state machine

            @param name: Name given to the subwindow and state machine
        """
        self.setWindowTitle(name)
        self.base_state_machine.set_name(name)
