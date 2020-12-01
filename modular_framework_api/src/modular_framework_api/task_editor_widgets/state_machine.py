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

from modular_framework_api.task_editor_graphics.state_machine import GraphicsStateMachine
from graphical_editor_base import Serializable
from state_content_widget import StateMachineContentWidget
from socket import Socket


class StateMachine(Serializable):

    """
        Object that gathers all the logic necessary to handle the state-like representation of state machines
    """

    def __init__(self, scene, container):
        """
            Initialize the widget and create the corresponding graphical representation

            @param scene: Object (TaskEditorScene) to which the state machine is added
            @param container: StateMachineContainer object linked to this representation
        """
        super(StateMachine, self).__init__()
        self.scene = scene
        self.container = container
        # By definition the name of the container is also the name of the state machine
        self.name = self.container.name
        # Create the widget displayed inside the state-like representation to configure it
        self.content = StateMachineContentWidget(self)
        # Create the graphical representation of the state machine
        self.graphics_state = GraphicsStateMachine(self)
        # Add the state machine to the scene
        self.scene.add_state_machine(self)
        # Add the graphical item to the graphics scene
        self.scene.graphics_scene.addItem(self.graphics_state)
        # Parametrize the spacing between sockets
        self.socket_spacing = 80
        # Will contain the input socket, set a list to make the update easier (see update_connectors)
        self.input_socket = list()
        # Will contain all the output sockets
        self.output_sockets = list()
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
        self.input_socket.append(Socket(state=self, socket_name="input"))
        # Get the initial outcomes
        outcomes = self.container.outcomes
        # Create a socket for each outcome
        for counter, item in enumerate(outcomes):
            self.output_sockets.append(Socket(state=self, index=counter, socket_name=item, multi_connections=False,
                                              count_on_this_side=len(outcomes)))

    def update_connectors(self):
        """
            Function called when the graphical representation is moved by the user so that the connectors get updated
        """
        # For all the sockets part of the state machine, update the connectors
        for socket in self.input_socket + self.output_sockets:
            for connector in socket.connectors:
                connector.update_positions()

    def remove(self):
        """
            Remove this object from the scene and graphics scene
        """
        # For each socket, remove all the linked connectors and sockets
        for socket in (self.input_socket + self.output_sockets):
            for connector in socket.connectors:
                connector.remove()
            # Remove the socket as well
            socket.remove()
        # Remove the graphics state machine from the graphics scene
        self.scene.graphics_scene.removeItem(self.graphics_state)
        self.graphics_state = None
        # Remove the state machine from the scene
        self.scene.remove_state_machine(self)
