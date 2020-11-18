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

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem
from PyQt5.QtGui import QColor, QPen, QBrush, QFont
from PyQt5.QtCore import QRectF, Qt


class RootGraphicsSocket(QGraphicsItem):

    """
        Graphical representation of a socket
    """

    def __init__(self, socket, parent=None):
        """
            Initialize the widget

            @param socket: Socket linked to this graphical representation
            @param: Parent of this widget
        """
        super(RootGraphicsSocket, self).__init__(parent=parent)
        self.socket = socket
        # Check whether the socket is used as the input of a state or not
        self.is_input = not self.socket.name
        self.init_resources()
        self.init_ui()
        self.init_title()

    def init_resources(self):
        """
            Define the resources that will be used to render the widget
        """
        # Radius of the socket
        self.radius = 10.0
        # Width of the outline
        self.outline_width = 2.0
        # Space between the socket and its name
        self.title_padding = 3.0
        # Set the font and color of the name
        self.title_color = Qt.white
        self.title_font = QFont("Ubuntu", 14)
        # Color for input sockets
        self.input_socket_color = QColor("#FFdbe220")
        # Color for outcomes
        self.socket_type_colors = [QColor("#FF52e220"), QColor("#FFFF2a23"), QColor("#FF0056a6"), QColor("#FFFF7700"),
                                   QColor("#FFa86db1"), QColor("#FFb54747")]
        # Get the color of the socket
        self.socket_color = self.input_socket_color if self.is_input else self.socket_type_colors[self.socket.index]
        # Outline color
        self.color_outline = QColor("#FF000000")
        # Color of the outline when selected
        self.color_selected = QColor("#FFFFFFFF")
        # Define of the painter should draw the lines (= outlines)
        self.pen = QPen(self.color_outline)
        self.pen.setWidthF(self.outline_width)
        self.pen_selected = QPen(self.color_selected)
        self.pen_selected.setWidthF(self.outline_width)
        # Define the fill patern of the socket
        self.brush = QBrush(self.socket_color)

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Make the GraphicsSocket selectable and movable
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setAcceptHoverEvents(True)

    def init_title(self):
        """
            Add a text below the socket
        """
        self.title_item = QGraphicsTextItem(self)
        # Set the text
        self.title_item.setPlainText(self.socket.name)
        self.title_item.setDefaultTextColor(self.title_color)
        self.title_item.setFont(self.title_font)
        # Make sure the text takes the right amount of space
        self.title_item.adjustSize()
        # Center it below the socket
        self.title_item.setPos(-self.title_item.textWidth() / 2., self.radius +
                               self.outline_width + self.title_padding)

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        """
            Paint the content of the socket onto the QGraphicsView

            @param painter: QPainter that will render the widget
            @param QStyleOptionGraphicsItem: Options provividing style options for the item
            @param widget: Specify another QWidget on which this item will be painted on. Default to None
        """
        # For the first initialization, set the expected initial position
        if not self.pos():
            self.setPos(*self.socket.get_initial_position())
        # Set the brush and pen
        painter.setBrush(self.brush)
        painter.setPen(self.pen if not self.isSelected() else self.pen_selected)
        # Paint the socket
        painter.drawEllipse(-self.radius, -self.radius, 2 * self.radius, 2 * self.radius)

    def boundingRect(self):
        """
            Return the bounding rectangle of the widget. Must be override to avoid issues when interacting with other
            widgets

            @return: QRectF containing the origin and size of the rectangle (x,y,w,h)
        """
        return QRectF(- self.radius - self.outline_width, - self.radius - self.outline_width,
                      2 * (self.radius + self.outline_width), 2 * (self.radius + self.outline_width))
