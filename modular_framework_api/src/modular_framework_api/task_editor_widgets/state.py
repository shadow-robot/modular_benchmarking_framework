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

from modular_framework_api.task_editor_graphics.state import GraphicsState
from graphical_editor_base import Serializable
from state_content_widget import StateContentWidget
from socket import Socket


class State(Serializable):

    """
        Object that gathers all the logic necessary to handle states
    """

    def __init__(self, scene, type="Undefined state"):
        """
            Initialize the widget and create the corresponding graphical representation

            @param scene: Object (scene) to which the node is added
            @param type: Type (string) of the state (e.g. Move, Plan, etc.)
        """
        super(State, self).__init__()
        self.scene = scene
        # When added, by default the name of the state is its type
        self.type = type
        self.name = type
        # Create the widget disaplyed inside the state to configure it
        self.content = StateContentWidget(self)
        # Create the graphical representation of the state
        self.graphics_state = GraphicsState(self)
        # Add the state to the scene
        self.scene.add_state(self)
        # Add the graphical item to the graphics scene
        self.scene.graphics_scene.addItem(self.graphics_state)
        # Parametrize the spacing between sockets
        self.socket_spacing = 80
        # Create the sockets
        self.init_sockets()

    def set_position(self, x, y):
        """
            Set the position of the object is in graphics scene

            @param x: x coordinate (float or integer) of the top left corner
            @param y: y coordinate (float or integer) of the top left corner
        """
        self.graphics_state.setPos(x, y)

    def init_sockets(self):
        """
            Create the sockets associated to the state
        """
        # Create a socket for input
        Socket(state=self, socket_name="input")
        # Get the initial outcomes
        outcomes = self.content.get_outcomes()
        # Create a socket for each outcome
        for counter, item in enumerate(outcomes):
            Socket(state=self, index=counter, socket_name=item, multi_edges=False, count_on_this_side=len(outcomes))
