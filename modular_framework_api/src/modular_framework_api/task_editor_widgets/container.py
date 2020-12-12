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
from modular_framework_core.utils.file_parsers import (extract_state_machine_parameters_from_file,
                                                       AVAILABLE_STATEMACHINES)
from modular_framework_core.utils.common_paths import TASK_EDITOR_ROOT_TEMPLATE
from modular_framework_api.task_editor_graphics.container import GraphicsContainer
from connector import Connector


class Container(Serializable):

    """
        Object keeping record of which widgets are currently composing a state machine.
    """

    def __init__(self, editor_widget, container_type):
        """
            Initialize the object

            @param editor_widget: GraphicalEditorWidget for which the state machine is the container
            @param container_type: Type of the container (string)
        """
        super(Container, self).__init__()
        # Store the graphical editor widget
        self.editor_widget = editor_widget
        # Set the type of the container
        self.type = container_type
        # Attribute recording whether the container is valid (i.e. can be executed)
        self.is_valid = False
        # Create the graphics representation of this widget
        self.graphics_container = GraphicsContainer(self)
        self.graphics_container.set_graphics_scene(64000, 64000)
        # Store the different states present in the container
        self.states = list()
        # Store the different connectors present in the container
        self.connectors = list()
        # Store the different state machines added to this container
        self.state_machines = list()
        # Extract the default outcomes of the state machine container
        self.outcomes = self.get_outcomes()
        # If applicable, will contain the state-like representation of this container in another GraphicalEditorWidget
        self.state_machine = None
        # Attribute tracking the current "layer" height required to be sure to have a widget overposing all the others
        self.z_tracker = 0

    def get_outcomes(self):
        """
            Extract the outcomes from the current parameters of the state machine container

            @return: A list of strings, each element being an outcome
        """
        # Get the outcomes from the parameters
        parsed_outcomes = self.parameters["outcomes"]
        # If nothing is provided, then set "success" and "failure" as default
        outcomes = ["success", "failure"] if not parsed_outcomes else parsed_outcomes
        return outcomes

    def create_terminal_sockets(self):
        """
            Create and add the terminal sockets of this state machine container
        """
        # Will contain the terminal sockets
        self.terminal_sockets = list()

        # Create a terminal socket for the beginning of the container. If the container is a concurrent one then the
        # starting socket support multi connections otherwise it does not.
        is_multi = self.type == "ConcurrentStateMachine"
        start_socket = TerminalSocket(container=self, socket_name="Start", index=0, multi_connections=is_multi)
        # Add it to the terminal sockets and add it to the graphical representation
        self.terminal_sockets.append(start_socket)
        self.graphics_container.addItem(start_socket.graphics_socket)

        # Create a terminal socket for each outcome
        for index, outcome in enumerate(self.outcomes):
            terminal_socket = TerminalSocket(container=self, socket_name=outcome, index=index)
            self.terminal_sockets.append(terminal_socket)
            # Add the graphical socket to the graphical representation so it can be rendered
            self.graphics_container.addItem(terminal_socket.graphics_socket)
        # By default, the default outcome is the last one
        self.default_socket = self.terminal_sockets[-1]

    def set_name(self, name):
        """
            Set the name of the container and if applicable corresponding state-like representation

            @param name: Name of the state machine container
        """
        self.name = name
        if self.state_machine is not None:
            self.state_machine.name = name

    def set_state_like(self, state_machine):
        """
            Set a state-like representation of the state machine, added into another editor widget

            @param state_machine: StateMachine object corresponding to this container in another editor widget
        """
        self.state_machine = state_machine

    def connect_to_default_socket(self):
        """
            Automatically connect all free sockets to the default terminal socket
        """
        # For each state-like object in the container
        for box in (self.states + self.state_machines):
            # If a socket is not connected, create a new connector between it and the default terminal socket
            for socket in box.output_sockets:
                if not socket.is_connected():
                    Connector(self, socket, self.default_socket)

    def get_view(self):
        """
            Return the view corresponding to the associated GraphicsScene

            @return: TaskEditorView linked to the container
        """
        return self.graphics_container.views()[0]

    def add_state(self, state):
        """
            Store a new state added to the container and potentially rename it if another one with the same name exists

            @param state: State object to be added to the container
        """
        # Get the unique name with which the state will be added
        updated_name = self.get_unique_name(state.name)
        # Update the name of the state
        state.name = updated_name
        # Register the state
        self.states.append(state)
        # Adding a state will make the container not valid, so update it
        self.update_validity()
        self.z_tracker += 1

    def add_connector(self, connector):
        """
            Store a new connector between two states, added in the view

            @param connector: Object (Connector) to be added in the container
        """
        self.connectors.append(connector)
        # When we add a new connector, it can make the container valid, so call the function to update this attribute
        self.update_validity()

    def add_state_machine(self, state_machine):
        """
            Store a new state machine added to the container

            @param state_machine: StateMachine object added to the container
        """
        self.state_machines.append(state_machine)
        # Adding a state machine will make the container not valid, so update it
        self.update_validity()

    def remove_state(self, state):
        """
            Remove the input state from the container

            @param state: State object to be removed from the container
        """
        # Make sure the state is still part of the container
        if state in self.states:
            self.states.remove(state)
            # Remove the state from the graphical view as well
            self.graphics_container.removeItem(state.graphics_state)
            # Removing a state can make the container valid, so update it
            self.update_validity()
            # Update the depth tracker
            self.z_tracker -= 1

    def remove_connector(self, connector):
        """
            Remove a given connector for the connectors attribute

            @param connector: Object (Connector) to be removed from the container
        """
        if connector in self.connectors:
            self.connectors.remove(connector)
            # Remove the connector from the graphical view as well (if not already removed by a socket)
            if connector.graphics_connector is not None:
                self.graphics_container.removeItem(connector.graphics_connector)
            # Removing a connector can make the container not valid, so update it
            self.update_validity()

    def remove_state_machine(self, state_machine):
        """
            Remove the provided state machine from the container

            @param state_machine: StateMachine object to be removed from the container
        """
        # Make sure the state machine is still part of the container
        if state_machine in self.state_machines:
            self.state_machines.remove(state_machine)
            # Remove the state from the graphical view as well
            self.graphics_container.removeItem(state_machine.graphics_state)
            # Removing a state machine can make the container valid, so update it
            self.update_validity()
            # Update the depth tracker
            self.z_tracker -= 1

    def get_unique_name(self, name):
        """
            Modify the input name by appending a digit at the end if needed to make sure two elements part of the
            container don't have the same name

            @param name: Candidate name (string) of the element to be add to the container
            @return: Unchanged name if it is unique, otherwise name+index, for instance name0 or name1
        """
        final_name = name
        counter = 0
        # As long as we find the given name in states or state machines, generate another one
        while any(final_name == x.name for x in (self.states + self.state_machines)):
            final_name = name + "{}".format(counter)
            counter += 1
        return final_name

    def update_validity(self):
        """
            Update the is_valid attribute according to the current container's content
        """
        # All the states should be fully connected
        are_states_valid = all(state.is_valid() for state in self.states)
        # Check that all connectors are linked to two sockets
        are_connectors_valid = all(connector.is_valid() for connector in self.connectors)
        # Run a check on the state machines
        are_state_machines_valid = all(state_machine.is_valid() for state_machine in self.state_machines)
        # Check whether the container is empty
        is_empty = (not self.states and not self.state_machines)
        # For the container to be valid we need all the above conditions to be True and empty to be False
        is_valid = are_connectors_valid and are_states_valid and are_state_machines_valid and not is_empty
        # If the validity of the container has changed, update the corresponding attribute and the validity icon
        if is_valid != self.is_valid:
            self.is_valid = is_valid
            self.editor_widget.update_validity_icon()
            # Update the related containers
            if self.state_machine is not None:
                self.state_machine.container.update_validity()

    @property
    def type(self):
        """
            Return the type of this container

            @return: String corresponding to the type of the container
        """
        return self._type

    @type.setter
    def type(self, container_type):
        """
            Set the type of the container and set the corresponding attributes

            @param container_type: Type of the container (string)
        """
        # Get the proper parameters
        if container_type == "base":
            self.parameters = extract_state_machine_parameters_from_file(TASK_EDITOR_ROOT_TEMPLATE)
        else:
            self.parameters = AVAILABLE_STATEMACHINES[container_type]["parameters"]
        # Set the type attribute
        self._type = container_type
