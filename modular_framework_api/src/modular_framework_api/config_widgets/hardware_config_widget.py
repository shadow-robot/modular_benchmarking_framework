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
from PyQt5.QtCore import pyqtSignal
from plain_editor_widgets import YAMLEditorWidget
from component_editor_widgets import ComponentEditorWidget, RosControllersEditorWidget, MoveItPlannerEditorWidget


class HardwareConfigWidget(QWidget):

    """
        Widget allowing the user to configure a robot arm or hand and integrate it to the framework
    """
    # Create a signal parametrized by a boolean specifying if the hardware config is in a different state as its initial
    hardwareChanged = pyqtSignal(bool)

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
        self.editor_content_changed = dict()
        # self.editor_file_changed = dict()
        self.connect_slots()

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
                                                           parent=self)
        self.ros_controllers = RosControllersEditorWidget("ROS controllers", parent=self)
        self.moveit_planners_config = MoveItPlannerEditorWidget("MoveIt! planners", parent=self)
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

    def connect_slots(self):
        """
            Remap signals coming from all this widget's children
        """
        # Remap signals coming from changes in editors' content
        self.hardware_connection_config.validEditorChanged.connect(self.handle_editor_content_signal)
        self.external_controller.validEditorChanged.connect(self.handle_editor_content_signal)
        self.kinematic_libraries_config.validEditorChanged.connect(self.handle_editor_content_signal)
        self.external_motion_planner.validEditorChanged.connect(self.handle_editor_content_signal)
        self.ros_controllers.validEditorChanged.connect(self.handle_editor_content_signal)
        self.moveit_planners_config.validEditorChanged.connect(self.handle_editor_content_signal)
        # Signal coming from changes in files linked to editors
        self.hardware_connection_config.fileEditorChanged.connect(self.handle_editor_file_signal)
        self.external_controller.fileEditorChanged.connect(self.handle_editor_file_signal)
        self.kinematic_libraries_config.fileEditorChanged.connect(self.handle_editor_file_signal)
        self.external_motion_planner.fileEditorChanged.connect(self.handle_editor_file_signal)
        self.ros_controllers.fileEditorChanged.connect(self.handle_editor_file_signal)
        self.moveit_planners_config.fileEditorChanged.connect(self.handle_editor_file_signal)

    def handle_editor_content_signal(self, has_widget_changed):
        """
            Emit a signal stating whether the content of one of the editors of hardware configuration has changed

            @param has_widget_changed: Boolean stating whether the content of the sender has changed
        """
        # Since each object has got an unique name, store it in a dictionary
        self.editor_content_changed[self.sender().objectName()] = has_widget_changed
        print("editor content changed: {}".format(self.editor_content_changed))
        # print("editor file changed: {}".format(self.editor_file_changed))
        # Emits the signal. If any of the children widgets has been changed then it tells that the interface has changed
        # It must include both editors' content and file changes
        self.hardwareChanged.emit(any(self.editor_content_changed.values()))

    # def handle_editor_file_signal(self, has_widget_changed):
    #     """
    #         Emit a signal stating whether the file set to a given widget has changed
    #
    #         @param has_widget_changed: Boolean stating if the file linked to the widget is different than the original
    #     """
    #     # Since each object has got an unique name, store it in a dictionary
    #     self.editor_file_changed[self.sender().objectName()] = has_widget_changed
    #     # Emits the signal. If any of the children widgets has been changed then it tells that the interface has changed
    #     print("editor content changed: {}".format(self.editor_content_changed))
    #     print("editor file changed: {}".format(self.editor_file_changed))
    #     # It must include both editors' content and file changes
    #     self.hardwareChanged.emit(any(self.editor_content_changed.values()) or any(self.editor_file_changed.values()))
    def handle_editor_file_signal(self):
        """
            Emit a signal stating whether the file set to a given widget has changed

            @param has_widget_changed: Boolean stating if the file linked to the widget is different than the original
        """
        self.hardwareChanged.emit(True)

    def set_default_enabled(self):
        """
            Enable/Disable the different widgets corresponding to the "default" configuration
        """
        is_widget_enabled = self.isEnabled()
        self.ros_controllers.setEnabled(is_widget_enabled)
        self.external_controller.setEnabled(is_widget_enabled)
        self.external_motion_planner.setEnabled(is_widget_enabled)
        self.kinematic_libraries_config.setEnabled(is_widget_enabled)

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        widget_name = self.objectName()
        settings.beginGroup(widget_name)
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

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not isinstance(widget, QGridLayout):
                widget.restore_config(settings)
        settings.endGroup()
