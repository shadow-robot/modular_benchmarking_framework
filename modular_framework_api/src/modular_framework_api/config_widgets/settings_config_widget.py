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
from code_editor_widgets import YAMLEditorWidget, SensorEditorWidget


class SettingsConfigWidget(QWidget):

    """
        Widget allowing the user to configure the settings (recorded joint states, sensor configs, etc.)
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the layout and initializing the widgets

            @param parent: parent of the widget
        """
        super(SettingsConfigWidget, self).__init__(parent=parent)
        self.setObjectName("Setup config widget")
        self.init_ui()
        self.create_widgets()
        # Boolean stating whether or not new checkpoints (named joint states and poses) are defined by the user
        self.new_checkpoint = False

    def init_ui(self):
        """
            Set the widget's layout
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(5, 10, 5, 5)
        self.setLayout(self.layout)

    def create_widgets(self):
        """
            Initialize the editors configuring the settings of the experiment
        """
        self.named_joint_states = YAMLEditorWidget("Named joint states", parent=self)
        self.layout.addWidget(self.named_joint_states, 0, 0)
        self.named_poses = YAMLEditorWidget("Named poses", parent=self)
        self.layout.addWidget(self.named_poses, 0, 1)
        self.named_trajectories = YAMLEditorWidget("Named trajectories", parent=self)
        self.layout.addWidget(self.named_trajectories, 0, 2)
        self.sensor_configs = SensorEditorWidget("Sensors config", parent=self)
        self.layout.addWidget(self.sensor_configs, 1, 0)
        self.sensor_plugins = YAMLEditorWidget("Sensor plugins", parent=self)
        self.layout.addWidget(self.sensor_plugins, 1, 1)
        self.methods_parameters = YAMLEditorWidget("Methods settings", parent=self)
        self.layout.addWidget(self.methods_parameters, 1, 2)

        self.named_joint_states.code_editor.linesChanged.connect(self.update_checkpoints_state)
        self.named_poses.code_editor.linesChanged.connect(self.update_checkpoints_state)
        self.named_trajectories.code_editor.selectionChanged.connect(self.update_trajectory_auto_completion)

    def update_trajectory_auto_completion(self):
        """
            Update the autocompletion's content of the trajectory editor
        """
        # Make sure we update the autocompletion only if changes have been brought to the joint states or poses editor
        if not self.new_checkpoint:
            return
        # Use a try except block to make up for potential non valid YAML syntax written by the user
        try:
            joint_states_content = self.named_joint_states.get_yaml_formatted_content()
            poses_content = self.named_poses.get_yaml_formatted_content()
            self.new_checkpoint = False
            if joint_states_content is None and poses_content is None:
                self.named_trajectories.code_editor.turn_off_autocompletion()
                return
            # Merge properly the entries of the autocompletion
            if joint_states_content is None:
                checkpoints = poses_content.keys()
            elif poses_content is None:
                checkpoints = joint_states_content.keys()
            else:
                checkpoints = poses_content.keys() + joint_states_content.keys()

            self.named_trajectories.code_editor.set_autocompletion(checkpoints)
        except:
            pass

    def update_checkpoints_state(self):
        """
            Update the class attribute stating whether a new checkpoint (named joint state) may be added by the user
        """
        self.new_checkpoint = True

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
