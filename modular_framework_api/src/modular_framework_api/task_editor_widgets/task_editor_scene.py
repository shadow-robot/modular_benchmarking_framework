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
        self.nodes = []
        self.state_machines = []
        self.edges = []
        self._has_been_modified = False
        self._last_selected_items = []
        self._has_been_modified_listeners = []
        # Create the graphics scene
        self.graphics_scene = TaskEditorGraphicsScene(self)
        self.graphics_scene.set_graphics_scene(64000, 64000)

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
