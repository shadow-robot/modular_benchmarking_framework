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

from PyQt5.QtWidgets import QGridLayout, QWidget, QInputDialog, QLineEdit, QMenu
from PyQt5.QtCore import Qt, QDataStream, QIODevice
from task_editor_scene import TaskEditorScene
from modular_framework_api.task_editor_graphics.view import TaskEditorView
from state_machine_container import StateMachineContainer
from state_machine import StateMachine
from modular_framework_api.utils.files_specifics import LISTITEM_MIMETYPE
from state import State


class GraphicalEditorWidget(QWidget):

    """
        Widget gathering the scene and view allowing the user to edit state machines
    """

    def __init__(self, state_machine_container_name, state_machine_container_type, parent=None):
        """
            Initialize the widget

            @param state_machine_container_name: Name given to both the container and this widget
            @param state_machine_container_type: Type of the state machine container to load
            @param parent: Parent of the widget
        """
        super(GraphicalEditorWidget, self).__init__(parent=parent)
        self.init_ui()
        # Set this widget's state machine container
        self.state_machine_container = StateMachineContainer(graphical_editor_widget=self,
                                                             container_type=state_machine_container_type)
        # Initialize the context menu
        self.init_context_menu()
        self.set_name(state_machine_container_name)

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Main layout
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        # Create graphics scene
        self.scene = TaskEditorScene(self)
        # Create graphics view
        self.editor_view = TaskEditorView(self.scene.graphics_scene, self)
        # Link the drag and drop event that occurs in the view to methods defined here
        self.editor_view.add_drag_enter_listener(self.on_drag_enter)
        self.editor_view.add_drop_listener(self.on_drop)
        # Add the graphics view in the layout
        self.layout.addWidget(self.editor_view)
        # Make sure that if the window containing the widget is deleted, this widget is properly removed as well
        self.setAttribute(Qt.WA_DeleteOnClose)

    def init_context_menu(self):
        """
            Initialize the proper context menu according to this widget's state machine container
        """
        self.context_menu = QMenu(self)
        if self.state_machine_container.type == "base":
            self.execute_action = self.context_menu.addAction("Execute")
        self.auto_connect_failures = self.context_menu.addAction("Auto-connect failures")

    def set_name(self, name):
        """
            Set the name of both the subwindow and state machine

            @param name: Name given to the subwindow and state machine
        """
        self.setWindowTitle(name)
        self.state_machine_container.set_name(name)

    def on_drag_enter(self, event):
        """
            Filter out what is accepted when receiving a drag enter event

            @param event: QDragEnterEvent sent py PyQt5
        """
        # If we have either a state or state machine entering accept the action otherwise ignore it
        if event.mimeData().hasFormat(LISTITEM_MIMETYPE):
            event.acceptProposedAction()
        else:
            event.setAccepted(False)

    def on_drop(self, event):
        """
            Handle the widget that is dropped to the graphical editor

            @param event: QDropEvent sent py PyQt5
        """
        # Check whether the MIME container has the proper format, otherwise ignore it
        if event.mimeData().hasFormat(LISTITEM_MIMETYPE):
            # Extract the data from the MIME container
            event_data = event.mimeData().data(LISTITEM_MIMETYPE)
            # Get the data stream
            data_stream = QDataStream(event_data, QIODevice.ReadOnly)
            # Extract the information from the stream
            is_state = data_stream.readBool()
            item_type = data_stream.readQString()
            # Get the position on which the object has been dropped
            mouse_position = event.pos()
            # Map it to the view coordinates
            view_position = self.scene.get_view().mapToScene(mouse_position)
            if is_state:
                # Create a State object from the extracted information
                dropped_state = State(self.scene, item_type)
                dropped_state.set_position(view_position.x(), view_position.y())
            else:
                state_machine_name, ok = QInputDialog().getText(self, "Input name", "Name of the state machine:",
                                                                QLineEdit.Normal)
                if state_machine_name and ok:
                    # Create another tab and extract the container
                    self.parent().mdiArea().add_subwindow(state_machine_name, item_type)
                    container = self.parent().mdiArea().focused_subwindow.widget().state_machine_container
                    # Create a state like representation to be displayed in the current widget
                    dropped_state_machine = StateMachine(self.scene, container)
                    dropped_state_machine.set_position(view_position.x(), view_position.y())
                    # Link it to the newly created container
                    container.set_state_like(dropped_state_machine)
            # Accept the drop action
            event.setDropAction(Qt.MoveAction)
            event.accept()
        else:
            event.ignore()

    def update_validity_icon(self):
        """
            Update the icon showing if this widget is properly configured
        """
        self.parent().change_icon(self.scene.is_scene_valid)

    def contextMenuEvent(self, event):
        """
            Function triggered when right click is pressed to make a context menu appear

            @param event: QContextMenuEvent sent by PyQt5
        """
        if self.state_machine_container.type == "base":
            self.execute_action.setEnabled(self.scene.is_scene_valid)
        # Execute the context menu
        action = self.context_menu.exec_(event.globalPos())
        if action == self.auto_connect_failures:
            self.state_machine_container.connect_failure_sockets()
