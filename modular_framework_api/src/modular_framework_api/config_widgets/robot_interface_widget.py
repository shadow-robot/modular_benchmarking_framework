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

from PyQt5.QtWidgets import QHBoxLayout, QWidget, QVBoxLayout
from interface_config_widgets import RobotInterfaceConfig, SimulationConfig, MoveitConfig


class RobotInterfaceWidget(QWidget):

    """
        Widget containing other widgets allowing to interface a robot
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different widgets

            @param parent: parent of the widget
        """
        super(RobotInterfaceWidget, self).__init__(parent=parent)
        self.setObjectName("Robot interface widget")
        self.init_ui()
        self.create_widgets()

    # ----------------------------- Methods creating the different components of the widget ----------------------------
    def init_ui(self):
        """
            Create the layout of the widget
        """
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_widgets(self):
        """
            Create the different widgets and add them to the layout.
        """
        # Create a vertical layout first
        left_hand_side_layout = QVBoxLayout()
        left_hand_side_layout.setContentsMargins(0, 0, 0, 0)

        # Create the widgets that will be contained in this sub-layout
        self.robot_config = RobotInterfaceConfig(parent=self)
        self.simulation_config = SimulationConfig(parent=self)
        # Add both widgets to the sub-layout
        left_hand_side_layout.addWidget(self.robot_config)
        left_hand_side_layout.addWidget(self.simulation_config)

        # Create the widget composing the right hand side of the layout
        self.moveit_config = MoveitConfig(parent=self)
        # Add the two components of the layout so they can be displayed
        self.layout.addLayout(left_hand_side_layout, 1)
        self.layout.addWidget(self.moveit_config, 1)
        # Set the layout otherwise nothing will be displayed
        self.setLayout(self.layout)

    # ---------------------------------- Methods related to configuration restoration ----------------------------------
    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @settings: QSettings object in which widgets' information are stored
        """
        test = self.objectName()
        settings.beginGroup(test)
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for widget in self.children():
            if not isinstance(widget, QHBoxLayout):
                widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not isinstance(widget, QHBoxLayout):
                widget.restore_config(settings)
        settings.endGroup()
