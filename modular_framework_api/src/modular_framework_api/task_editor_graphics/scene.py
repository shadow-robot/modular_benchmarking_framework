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

from PyQt5.QtWidgets import QGraphicsScene
from PyQt5.QtGui import QColor


class TaskEditorGraphicsScene(QGraphicsScene):
    """
        Widget used as "background" of Graphical Editor Widgets
    """
    def __init__(self, scene, parent=None):
        """
            Initialize the widget
        """
        super(TaskEditorGraphicsScene, self).__init__(parent)
        self.scene = scene
        # Set background colour
        self.color_background = QColor("#535353")
        self.setBackgroundBrush(self.color_background)

    def set_graphics_scene(self, width, height):
        """
            Set a rectangle as the graphics scene

            @param width: Width of the rectangle
            @param height: Height of the rectangle
        """
        self.setSceneRect(-width // 2, -height // 2, width, height)
