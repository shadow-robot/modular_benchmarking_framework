# !/usr/bin/env python

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

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QTabWidget
from modular_framework_api.task_editor_widgets.task_editor_mdi import TaskEditorMDIArea
from modular_framework_api.task_editor_widgets.side_displayers import StatesDisplayer


class TaskEditorArea(QWidget):

    """
        Widget containing the MDI area and state/state machine displayer required to design and execute tasks
    """

    def __init__(self, parent=None):
        """
            Initialise the different widgets

            @param parent: Parent of the widget
        """
        super(TaskEditorArea, self).__init__(parent=parent)
        self.init_ui()

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        # Add the Multi Document Interface area
        self.mdi_area = TaskEditorMDIArea(parent=self)
        layout.addWidget(self.mdi_area)
        # Create a tab widget to display both states and state machines
        displayers = QTabWidget(self)
        # Move the tab to the bottom so it looks better
        displayers.setTabPosition(QTabWidget.South)
        # Add the widget showing available states and to import some new ones
        self.state_displayer = StatesDisplayer(self)
        displayers.addTab(self.state_displayer, "States")
        layout.addWidget(displayers)
        self.setLayout(layout)
