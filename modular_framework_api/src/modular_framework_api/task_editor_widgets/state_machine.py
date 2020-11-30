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

from graphical_editor_base import Serializable
from terminal_socket import TerminalSocket


class StateMachine(Serializable):

    """
        Object gathering the logic and graphical representation of a state machine in the task editor
    """

    def __init__(self, graphical_editor_widget, parameters):
        """
            Initialize the object

            @param graphical_editor_widget: GraphicalEditorWidget for which the state machine is the root
            @param parameters: Dictionary containing the description, init parameters and outcomes of the state machine
        """
        super(StateMachine, self).__init__()
        # Store the graphical editor widget
        self.editor_widget = graphical_editor_widget
        # Store the parameters
        self.parameters = parameters
        # Extract the outcomes
        self.outcomes = self.get_outcomes()
        # Set the terminal sockets of the state machine in the corresponding editor
        self.set_terminal_sockets()

    def get_outcomes(self):
        """
            Extract the outcomes from the parameters

            @return: A list of strings, each element being an outcome
        """
        # Get the outcomes from the parameters
        parsed_outcomes = self.parameters["outcomes"]
        # If nothing is provided, then set "success" and "failure" as default
        outcomes = ["success", "failure"] if not parsed_outcomes else parsed_outcomes
        return outcomes

    def set_terminal_sockets(self):
        """
            Create and add the terminal sockets of this state machine
        """
        # Will contain the terminal sockets
        self.terminal_sockets = list()

        # Create a terminal socket for the beginning of the state machine
        start_socket = TerminalSocket(parent=self, socket_name="Start", index=0, multi_connections=False)
        # Add it to the terminal sockets and add it to the graphics scene
        self.terminal_sockets.append(start_socket)
        self.editor_widget.scene.graphics_scene.addItem(start_socket.graphics_socket)

        # Create a terminal socket for each outcome
        for index, outcome in enumerate(self.outcomes):
            terminal_socket = TerminalSocket(parent=self, socket_name=outcome, index=index)
            self.terminal_sockets.append(terminal_socket)
            # Add the graphical socket to the graphics scene so it can be rendered
            self.editor_widget.scene.graphics_scene.addItem(terminal_socket.graphics_socket)

    def set_name(self, name):
        """
            Set the name of the state machine

            @param name: Name of the state machine
        """
        self.name = name
