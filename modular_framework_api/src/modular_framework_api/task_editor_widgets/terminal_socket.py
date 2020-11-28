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
from modular_framework_api.task_editor_graphics.terminal_socket import TerminalGraphicsSocket


class TerminalSocket(Serializable):

    """
        Object gathering the logic and graphical representation of a socket
    """

    def __init__(self, parent, socket_name, index, multi_edges=True):
        """
            Initialize the object

            @param parent: Parent of the object (can either be a StateMachine or a State)
            @param socket_name: Name of the outcome the socket represents
            @param index: Index of the socket
            @param multi_edges: State whether the socket can host several edges. Default to True
        """
        super(TerminalSocket, self).__init__()
        # Store the parent of the socket (can either be a StateMachine or a State)
        self.parent = parent
        # Name of the outcome corresponding to the socket
        self.name = socket_name
        self.index = index
        # Set the multi edge attribute
        self.is_multi_edges = multi_edges
        # Create and store the graphical socket to be displayed
        self.graphics_socket = TerminalGraphicsSocket(self)
        # Set the position of the terminal socket in the view
        self.graphics_socket.setPos(*self.get_position())
        # Get all the connectors connected to this terminal socket
        self.connectors = list()

    def get_position(self):
        """
            Return the initial position of the socket

            @return: List (x,y) corresponding to the position of the socket
        """
        # Number of sockets
        num_sockets = len(self.parent.outcomes)
        # View size
        view_size = self.parent.editor_widget.scene.get_view().sizeHint()
        # The (0,0) point is in the centre of the screen!
        # We want the text that goes with the socket to be close to the bottom
        y = view_size.height() / 4.
        # Compute the spacing between the sockets
        width = view_size.width()
        total_number_of_spaces = num_sockets - 1
        socket_spacing = width / (num_sockets * 3.)
        x = self.index * socket_spacing - (total_number_of_spaces) / 2. * socket_spacing
        # Return the corrdinates as a list
        return [x, y]

    def add_connector(self, connector):
        """
            Add a provided connector to this terminal socket

            @param connector: Connector object to be added to the attribute connectors
        """
        self.connectors.append(connector)

    def remove_connector(self, connector):
        """
            Remove a provided connector from this terminal socket

            @param connector: Connector object to be removed from the attribute connectors
        """
        # Make sure it is part of the socket to avoid any exception
        if connector in self.connectors:
            self.connectors.remove(connector)
