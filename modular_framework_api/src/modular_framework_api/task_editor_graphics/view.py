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

from PyQt5.QtWidgets import QGraphicsView, QApplication
from PyQt5.QtCore import Qt, QEvent, QPointF
from PyQt5.QtGui import QPainter, QMouseEvent
from PyQt5.QtWidgets import QGraphicsProxyWidget

MODE_NOOP = 1
MODE_EDGE_DRAG = 2
MODE_EDGE_CUT = 3

EDGE_DRAG_START_THRESHOLD = 10


class TaskEditorView(QGraphicsView):
    """
        Widget allowing to visualize the content of the graphics scene
    """
    def __init__(self, graphics_scene, parent=None):
        """
            Initialize the widget

            @param graphics_scene: QGraphicsScene object linked to the widget
            @param parent: Parent of the widget
        """
        super(TaskEditorView, self).__init__(parent)
        # Store the QGraphicsScene
        self.graphics_scene = graphics_scene
        self.init_ui()
        # Set the QGraphicsScene
        self.setScene(self.graphics_scene)
        self.rubberBandDraggingRectangle = False

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Bunch of options to makes things look nice (not pixelized, etc.)
        self.setRenderHints(QPainter.Antialiasing | QPainter.HighQualityAntialiasing |
                            QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setAcceptDrops(True)
        self.rubberBandDraggingRectangle = False

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.middleMouseButtonPress(event)
        elif event.button() == Qt.LeftButton:
            self.leftMouseButtonPress(event)
        elif event.button() == Qt.RightButton:
            self.rightMouseButtonPress(event)
        else:
            super(TaskEditorView, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.middleMouseButtonRelease(event)
        elif event.button() == Qt.LeftButton:
            self.leftMouseButtonRelease(event)
        elif event.button() == Qt.RightButton:
            self.rightMouseButtonRelease(event)
        else:
            super(TaskEditorView, self).mouseReleaseEvent(event)

    def middleMouseButtonPress(self, event):
        releaseEvent = QMouseEvent(QEvent.MouseButtonRelease, event.localPos(), event.screenPos(),
                                   Qt.LeftButton, Qt.NoButton, event.modifiers())
        super(TaskEditorView, self).mouseReleaseEvent(releaseEvent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        fakeEvent = QMouseEvent(event.type(), event.localPos(), event.screenPos(),
                                Qt.LeftButton, event.buttons() | Qt.LeftButton, event.modifiers())
        super(TaskEditorView, self).mousePressEvent(fakeEvent)

    def middleMouseButtonRelease(self, event):
        fakeEvent = QMouseEvent(event.type(), event.localPos(), event.screenPos(),
                                Qt.LeftButton, event.buttons() & ~Qt.LeftButton, event.modifiers())
        super(TaskEditorView, self).mouseReleaseEvent(fakeEvent)
        self.setDragMode(QGraphicsView.RubberBandDrag)

    def leftMouseButtonPress(self, event):
        super(TaskEditorView, self).mousePressEvent(event)

    def leftMouseButtonRelease(self, event):
        super(TaskEditorView, self).mouseReleaseEvent(event)

    def rightMouseButtonPress(self, event):
        super(TaskEditorView, self).mousePressEvent(event)

    def rightMouseButtonRelease(self, event):
        super(TaskEditorView, self).mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        super(TaskEditorView, self).mouseMoveEvent(event)

    def keyPressEvent(self, event):
        super(TaskEditorView, self).keyPressEvent(event)

    def wheelEvent(self, event):
        super(TaskEditorView, self).wheelEvent(event)
