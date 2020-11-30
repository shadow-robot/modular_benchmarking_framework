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

from PyQt5.QtWidgets import QMdiArea, QMdiSubWindow, QMenu
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon, QPixmap
from modular_framework_core.utils.common_paths import RED_CIRCLE, GREEN_CIRCLE
from graphical_editor_widget import GraphicalEditorWidget


class TaskEditorMDIArea(QMdiArea):

    """
        Widget managing the Multi Document Interface (MDI) required to design and execute tasks
    """

    def __init__(self, parent=None):
        """
            Initialise the MDI Area and automatically create the root state machine

            @param parent: Parent of the widget
        """
        super(TaskEditorMDIArea, self).__init__(parent=parent)
        self.init_ui()
        # Focused subwindow
        self.focused_subindow = None
        # Add a subwindow containing the base of all state machines
        self.add_subwindow("root", "base")

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setViewMode(QMdiArea.TabbedView)
        self.setDocumentMode(True)
        # Set it to False to avoid unexpected closure
        self.setTabsClosable(False)
        self.setTabsMovable(True)
        self.setWindowFlags(Qt.FramelessWindowHint)

    def add_subwindow(self, state_machine_name, state_machine_type):
        """
            Add a new custom made subwindow containing a GraphicalEditorWidget

            @param state_machine_name: Name of the state machine that will be contained in the subwindow
            @param state_machine_type: Type of the state machine that will be contained in the subwindow
        """
        # Create the subwindow
        subwindow = TaskEditorSubWindow(state_machine_name, state_machine_type, parent=self)
        # Add it to the MDI area
        self.addSubWindow(subwindow)
        self.setActiveSubWindow(subwindow)
        # Make sure to get a nice visualization
        subwindow.showMaximized()
        self.focused_subindow = subwindow


class TaskEditorSubWindow(QMdiSubWindow):
    """
        Subwindow that will contain a GraphicalEditorWidget in which a state machine can be configured
    """
    def __init__(self, state_machine_name, state_machine_type, parent=None):
        """
            Initialize the class by adding a state machine to the the GraphicalEditorWidget

            @param state_machine_name: Name of the state machine that will be set to the GraphicalEditorWidget
            @param state_machine_type: Type of the state machine that will be loaded to the GraphicalEditorWidget
            @param parent: Parent of the widget
        """
        super(TaskEditorSubWindow, self).__init__(parent=parent)
        self.init_ui()
        # Set the subwindow icon
        self.setWindowIcon(self.red_icon)
        # Set the widget
        self.setWidget(GraphicalEditorWidget(state_machine_name, state_machine_type, parent=self))
        # If the subwindow is the main, remove the possibility of removing it
        if state_machine_type == "base":
            self.setSystemMenu(QMenu(self))

    def init_ui(self):
        """
            Initialize the subwindow
        """
        # Define resources notifying whether the state machine is valid or not
        self.red_icon = QIcon(QPixmap(RED_CIRCLE))
        self.green_icon = QIcon(QPixmap(GREEN_CIRCLE))
        # This command makes sure that when the tab is removed, the subwindow is as well
        self.setAttribute(Qt.WA_DeleteOnClose)
