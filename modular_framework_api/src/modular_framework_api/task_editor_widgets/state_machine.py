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
from socket import Socket


class StateMachine(Serializable):

    """
        Object gathering the logic and graphical representation of a state machine in the Task Editor
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
        # Set the root sockets of the state machine in the corresponding editor
        self.set_root_sockets()

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

    def set_root_sockets(self):
        """
            Create and add the root sockets of this state machine
        """
        # Will contain the root sockets
        self.root_sockets = list()
        # Create a socket for each outcome
        for index, outcome in enumerate(self.outcomes):
            root_socket = Socket(parent=self, socket_name=outcome, index=index)
            self.root_sockets.append(root_socket)
            # Add the graphical socket to the graphics scene so it can be rendered
            self.editor_widget.scene.graphics_scene.addItem(root_socket.graphics_socket)

    def set_name(self, name):
        """
            Set the name of the state machine

            @param name: Name of the state machine
        """
        self.name = name
