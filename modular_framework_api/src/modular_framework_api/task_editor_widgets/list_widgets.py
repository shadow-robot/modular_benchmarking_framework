# !/usr/bin/env python

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

from PyQt5.QtWidgets import QListWidget, QAbstractItemView, QListWidgetItem
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from list_item_widgets import BoxItemContent
from modular_framework_core.utils.file_parsers import AVAILABLE_STATES


class CommonDraggableListWidget(QListWidget):

    """
        Common widget allowing to show a list of items that can be dragged
    """

    def __init__(self, items, parent=None):
        """
            Initialize the widget and fill the list

            @param items: Dictionary containing the name of elements to integrate as keys and the description as value
            @param parent: Parent of the widget
        """
        super(CommonDraggableListWidget, self).__init__(parent)
        self.init_ui()
        self.add_items(items)

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        # Make it draggable
        self.setDragEnabled(True)

    def add_items(self, items_to_fill):
        """
            Add the items to the list

            @param items_to_fill: Dictionary containing the elements to integrate
        """
        for item_name, item_parameters in items_to_fill.items():
            # Function defined in each children
            self.add_item(item_name, item_description=item_parameters["description"])

    # def startDrag(self, *args, **kwargs):
    #     """
    #         Embed all required information when dragging an element
    #     """
    #     try:
    #         item = self.currentItem()
    #         widget = self.itemWidget(item)
    #         is_state = widget.is_state
    #         item_type = widget.name
    #
    #         pixmap = QPixmap(item.data(Qt.UserRole))
    #
    #         itemData = QByteArray()
    #         dataStream = QDataStream(itemData, QIODevice.WriteOnly)
    #         dataStream.writeBool(is_state)
    #         dataStream.writeQString(item_type)
    #         dataStream << pixmap
    #
    #         mimeData = QMimeData()
    #         mimeData.setData(LISTITEM_MIMETYPE, itemData)
    #
    #         drag = QDrag(self)
    #         drag.setMimeData(mimeData)
    #         drag.setHotSpot(QPoint(pixmap.width() / 2, pixmap.height() / 2))
    #         drag.setPixmap(pixmap)
    #
    #         drag.exec_(Qt.MoveAction)
    #
    #     except Exception as e:
    #         print("Exception is {}".format(e))


class StateListWidget(CommonDraggableListWidget):

    """
        List widget gathering all the states that can be used in the task editor
    """

    def __init__(self, parent=None):
        """
            Initialize the widget
        """
        # Set the icon of each item
        # self.icon = QPixmap(STATE_MACHINE_ICON).scaledToHeight(32)
        self.icon = QPixmap(".")
        super(StateListWidget, self).__init__(items=AVAILABLE_STATES, parent=parent)

    # TODO add this to the common class!
    def add_item(self, item_name, item_description):
        """
            Add an item to the list widget

            @param item_name: Name of the item (string)
            @param item_description: Description of the item (string)
        """
        # Create a new QListWidgetItem
        list_item = QListWidgetItem()
        # Create a widget properly formatting the content of the item to display
        widget_item = BoxItemContent(item_name, item_description, parent=self)
        # Adjust the size of the list widget item
        list_item.setSizeHint(widget_item.size())
        list_item.setData(Qt.UserRole, self.icon)
        # Add the item to the list
        self.addItem(list_item)
        # Set our widget to the list  widget item
        self.setItemWidget(list_item, widget_item)

    def update_content(self):
        """
            Update the content of the list
        """
        self.clear()
        super(StateListWidget, self).add_items(AVAILABLE_STATES)
