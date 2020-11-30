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
from modular_framework_api.task_editor_graphics.scene import TaskEditorGraphicsScene


class TaskEditorScene(Serializable):

    """
        Object managing the scene of a graphical editor. It keeps record of what widgets are present and also manage
        the graphics scene.
    """

    def __init__(self):
        """
            Initialize the object
        """
        super(TaskEditorScene, self).__init__()
        # Store the different states present in the scene
        self.states = list()
        # Store the different connectors present in the view
        self.connectors = list()
        # Create the graphics scene
        self.graphics_scene = TaskEditorGraphicsScene(self)
        self.graphics_scene.set_graphics_scene(64000, 64000)
        # Attribute tracking the current "layer" height required to have a widget on top of all the others
        self.z_tracker = 0

    def get_view(self):
        """
            Return the view corresponding to the GraphicsScene

            @return: Main view of the scene
        """
        return self.graphics_scene.views()[0]

    def get_item_at(self, pos):
        """
            Return the QGraphicsItem that is displayed in the view located at position pos

            @param pos: QPoint corresponding to the position
            @return: QGraphicsItem located at position pos
        """
        return self.get_view().itemAt(pos)

    def add_state(self, state):
        """
            Store a new state in scene and potentially rename it if another one with the same name has been added

            @param state: State object to be added to the scene
        """
        # Get the unique name with which the state will be added
        updated_name = self.get_unique_name(state.name)
        # Update the name of the state
        state.name = updated_name
        # Register the state
        self.states.append(state)
        self.z_tracker += 1

    def remove_state(self, state):
        """
            Remove the provided state from the states attribute

            @param state: State object to be removed from the scene
        """
        # Make sure the state is still part of the scene
        if state in self.states:
            self.states.remove(state)
            # Update the depth tracker
            self.z_tracker -= 1

    def add_connector(self, connector):
        """
            Store a new connector between two states, added in the view

            @param connector: Object (Connector) to be added in the scene
        """
        self.connectors.append(connector)

    def remove_connector(self, connector):
        """
            Remove a given connector for the connectors attribute

            @param connector: Object (Connector) to be removed from the scene
        """
        if connector in self.connectors:
            self.connectors.remove(connector)

    def get_unique_name(self, name):
        """
            Modify the input name by appending a digit at the end if needed to make sure two states don't have the same
            name

            @param name: Candidate name (string) of the state to be add to states
            @return: Unchanged name if it is unique, otherwise name+index, for instance name0 or name1
        """
        final_name = name
        counter = 0
        # As long as we find the given name in states, generate another one
        while any(final_name == x.name for x in self.states):
            final_name = name + "{}".format(counter)
            counter += 1
        return final_name
