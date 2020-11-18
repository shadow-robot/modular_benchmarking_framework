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
from modular_framework_api.task_editor_graphics.root_socket import RootGraphicsSocket


class Socket(Serializable):
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
        super(Socket, self).__init__()
        # Store the parent of the socket (can either be a StateMachine or a State)
        self.parent = parent
        # Name of the outcome corresponding to the socket
        self.name = socket_name
        self.index = index
        self.position = self.get_initial_position()
        # Create and store the graphical socket to be displayed
        self.graphics_socket = RootGraphicsSocket(self)

    def get_initial_position(self):
        """
            Return the initial position of the socket

            @return: List (x,y) corresponding to the position of the socket
        """
        # Number of sockets
        num_sockets = len(self.parent.outcomes)
        # View size
        view_size = self.parent.editor_widget.scene.get_view().size()
        # We want the text that goes with the socket to be close to the bottom
        # TODO: get the 50 from the graphics
        y = view_size.height() / 2. - 50
        # Compute the spacing between the sockets
        width = view_size.width()
        total_number_of_spaces = num_sockets - 1
        socket_spacing = width / (num_sockets * 3.)
        x = self.index * socket_spacing - (total_number_of_spaces) / 2. * socket_spacing
        # Return the corrdinates as a list
        return [x, y]
