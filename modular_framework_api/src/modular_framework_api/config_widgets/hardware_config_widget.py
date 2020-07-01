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

from PyQt5.QtWidgets import QWidget, QGridLayout
from code_editor_widgets import YAMLEditorWidget, ComponentEditorWidget, ROSComponentEditorWidget


class HardwareConfigWidget(QWidget):

    """
        Widget allowing the user to configure a robot arm or hand and integrate it to the framework
    """

    def __init__(self, hardware_part, parent=None):
        """
            Initialize the class by creating the layout and initializing the widgets

            @param hardware_part: String stating whether the widget is meant for an "Arm" or a "Hand"
            @param parent: parent of the widget
        """
        super(HardwareConfigWidget, self).__init__(parent=parent)
        self.hardware_part = hardware_part
        self.setObjectName("{} config widget".format(hardware_part))
        self.init_ui()
        self.create_widgets()

    def init_ui(self):
        """
            Set the widget's layout
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(5, 10, 5, 5)

    def create_widgets(self):
        """
            Initialize the editors allowing to configure the arm
        """
        self.hardware_connection_config = YAMLEditorWidget("{} hardware connection".format(self.hardware_part),
                                                           enabled=False, parent=self)
        self.ros_controllers = ROSComponentEditorWidget("ROS controllers", parent=self)
        self.moveit_planners_config = ROSComponentEditorWidget("MoveIt! planners", enabled=False, parent=self)
        self.kinematic_libraries_config = ComponentEditorWidget("External kinematics", parent=self)
        self.external_controller = ComponentEditorWidget("External controllers", parent=self)
        self.external_motion_planner = ComponentEditorWidget("External Motion Planners", parent=self)

        self.layout.addWidget(self.hardware_connection_config, 0, 0)
        self.layout.addWidget(self.ros_controllers, 0, 1)
        self.layout.addWidget(self.moveit_planners_config, 0, 2)
        self.layout.addWidget(self.kinematic_libraries_config, 1, 0)
        self.layout.addWidget(self.external_controller, 1, 1)
        self.layout.addWidget(self.external_motion_planner, 1, 2)
        self.setLayout(self.layout)

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
            # We don't consider the layout
            if not isinstance(widget, QGridLayout):
                widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not isinstance(widget, QGridLayout):
                widget.restore_config(settings)
        settings.endGroup()
