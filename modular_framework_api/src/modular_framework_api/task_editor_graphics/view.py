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

from PyQt5.QtWidgets import QGraphicsView
from PyQt5.QtCore import Qt, QEvent, pyqtSignal
from PyQt5.QtGui import QPainter, QMouseEvent
from modular_framework_api.task_editor_graphics.state import GraphicsStateContent


class TaskEditorView(QGraphicsView):
    """
        Widget allowing to visualize the content of the graphics scene
    """
    # Signal triggered when the view is scaled and gives the level of zoom
    viewScaled = pyqtSignal(int)

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
        # Create lists that will contain functions to be executed when drag enter and drop events occur
        self.drag_enter_listeners = list()
        self.drop_listeners = list()
        # Set the QGraphicsScene
        self.setScene(self.graphics_scene)
        # Set the zooming parameters
        self.zoom_in_multiplier = 1.1
        self.zoom_out_multiplier = 1/1.1
        self.current_zoom = 0
        self.zoom_range = [-15, 15]

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Bunch of options to make things look nice (not pixelized, etc.)
        self.setRenderHints(QPainter.Antialiasing | QPainter.HighQualityAntialiasing |
                            QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setAcceptDrops(True)
        self.rubberBandDraggingRectangle = False

    def add_drag_enter_listener(self, callback):
        """
            Add a function (callback) to be executed when a dragged object enters the view

            @param callback: Function or method to be executed
        """
        self.drag_enter_listeners.append(callback)

    def add_drop_listener(self, callback):
        """
            Add a function (callback) to be executed when an object is dropped onto the view

            @param callback: Function or method to be executed
        """
        self.drop_listeners.append(callback)

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

    def dragEnterEvent(self, event):
        """
            Function called when an item is dragged in this widget

            @param event: QDragEvent
        """
        for callback in self.drag_enter_listeners:
            callback(event)

    def dropEvent(self, event):
        """
            Function called when an item is dropped onto this widget

            @param event: QDropEvent
        """
        for callback in self.drop_listeners:
            callback(event)

    def wheelEvent(self, event):
        """
            Function triggered when the wheel of a mouse is activated

            @param event: QWheelEvent triggered by PyQt5
        """
        # Get the item under the mouse when the wheel event is triggered
        pointed_item = self.itemAt(event.pos())
        # Make sure to be able to scroll the content of the state with the wheel
        if isinstance(pointed_item, GraphicsStateContent):
            super(TaskEditorView, self).wheelEvent(event)
            return

        # Calculate zoom
        if event.angleDelta().y() > 0:
            zoom_to_apply = self.zoom_in_multiplier
            self.current_zoom += 1
        else:
            zoom_to_apply = self.zoom_out_multiplier
            self.current_zoom -= 1

        # Clamp the zoom if required
        clamped = False
        if self.current_zoom < self.zoom_range[0]:
            self.current_zoom, clamped = self.zoom_range[0], True
        if self.current_zoom > self.zoom_range[1]:
            self.current_zoom, clamped = self.zoom_range[1], True

        # Set view scale
        if not clamped:
            # Emit the signal giving the current zoom
            self.viewScaled.emit(self.current_zoom)
            self.scale(zoom_to_apply, zoom_to_apply)
