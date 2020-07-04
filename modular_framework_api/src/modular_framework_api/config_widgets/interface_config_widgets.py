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

from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QSpinBox, QHBoxLayout, QCheckBox
from code_editor_widgets import XMLEditorWidget
from user_entry_widgets import GenericUserEntryWidget, UrdfEntryWidget, UrdfArgumentsEntryWidget, LaunchFileEntryWidget, CollisionFileEntryWidget
from modular_framework_api.utils.common_dialog_boxes import display_error_message
import os
import rospkg
import re
import yaml


class GenericInterfaceConfigWidget(QWidget):

    """
        Generic widget gathering common properties of widgets used to interface a robot
    """

    def __init__(self, name, parent=None):
        """
            Initialize the class by setting up the layout and initializing the different attributes

            @param name: Name given to the object. Used to look it up when restoring a configuration
            @param parent: parent of the widget
        """
        super(GenericInterfaceConfigWidget, self).__init__(parent=parent)
        self.setObjectName(name)
        self.init_ui()
        # Index of the current row inside the widget
        self.current_row = 0
        # Maximum number of widgets that can be contained on a single row
        self.max_number_widget_per_row = 2
        # Store the name of all the user entries
        self.registered_entries = []
        # Store the name of all the sub-widgets (children)
        self.registered_subwidgets = []

    def init_ui(self):
        """
            Set up the layout
        """
        self.layout = QGridLayout()
        # These values prevent the different elements not to be stuck to the tab
        self.layout.setContentsMargins(5, 15, 5, 5)
        self.setLayout(self.layout)

    # ----------------- Methods realted to the widgets allowing the user to interact with the framework ----------------
    # def add_user_entry(self, entry_name, should_browse=True, placeholder_text=None, enabled=True):
    #     """
    #         Add a row to the widget with a label, an edit line and potentially a button to brose files or folders
    #
    #         @param entry_name: String to be displayed to specify what is expected
    #         @param should_browse: Boolean stating whether the browse button should be displayed
    #         @param placeholder_text: Optional text that can be displayed initially inside the edit line
    #         @param enabled: Boolean stating whether the entry should be initially enabled or not.
    #     """
    # Label to display what the entry is about
    #     entry_label = QLabel(entry_name + ":", objectName="label {}".format(entry_name))
    # Line edit entry to let the user write and read the content of the entry
    #     entry_edit_line = QLineEdit(objectName="entry_edit_line {}".format(entry_name))
    # Set a small button allowing to clear the whole line
    #     entry_edit_line.setClearButtonEnabled(True)
    # If specified, set a placeholder text to show for isntance what is expected in this entry
    #     if placeholder_text is not None:
    #         entry_edit_line.setPlaceholderText(placeholder_text)
    # Enables or disables access to this entry
    #     for widget in (entry_label, entry_edit_line):
    #         widget.setEnabled(enabled)
    #
    # If the user is expected to select a file or folder, create the corresponding button to do so
    #     self.layout.addWidget(entry_label, self.current_row, 0)
    #     if should_browse:
    #         self.max_number_widget_per_row = 3
    #         browser_button = QToolButton(objectName="browser {}".format(entry_name))
    #         browser_button.setText("...")
    #         browser_button.setEnabled(enabled)
    #         browser_button.clicked.connect(self.fill_line_with_browsing)
    #         self.layout.addWidget(entry_edit_line, self.current_row, 1)
    #         self.layout.addWidget(browser_button, self.current_row, 2)
    #     else:
    # Make the entry edit line span over two columns
    #         self.layout.addWidget(entry_edit_line, self.current_row, 1, 1, 2)
    # Increment the current row
    #     self.current_row += 1
    # Add the entry name to the regitered entries
    #     self.registered_entries.append(entry_name)

    # def add_xml_editor(self, entry_name, enabled=False):
    #     """
    #         Add an XMLEditorWidget object to the widget
    #
    #         @param entry_name: String to be displayed to specify what is expected
    #         @param enabled: Boolean stating whether the entry should be initially enabled or not.
    #     """
    #     xml_editor = XMLEditorWidget(entry_name, enabled=enabled, parent=self)
    #     self.layout.addWidget(xml_editor, self.current_row, 0, 1, self.max_number_widget_per_row)
    #     self.current_row += 1
    #     self.registered_subwidgets.append(entry_name)

    # def enable_user_entry(self, entry_name, should_enable=True):
    #     """
    #         Enables or disables a given user entry according to "should_enable"
    #
    #         @param entry_name: Name of the user entry to modify
    #         @param should_enable: Specifies whether the user_entry should be enabled or disabled
    #     """
    #     self.findChild(QLineEdit, "entry_edit_line {}".format(entry_name)).setEnabled(should_enable)
    #     self.findChild(QLabel, "label {}".format(entry_name)).setEnabled(should_enable)
    #     tool_button = self.findChild(QToolButton, "browser {}".format(entry_name))
    #     if tool_button is not None:
    #         tool_button.setEnabled(should_enable)

    # def get_entry_value(self, entry_name):
    #     """
    #         Returns the text contained in the edit line corresponding to the entry_name
    #
    #         @param entry_name: Name (string) of the entry containing the value to return
    #
    #         @return: Value of the edit line (string)
    #     """
    #     return self.findChild(QLineEdit, "entry_edit_line {}".format(entry_name)).text()

    # def set_entry_value(self, entry_name, value):
    #     """
    #         Set the text of a specified entry
    #
    #         @param entry_name: Name (string) of the entry
    #         @param value: Value to set (string)
    #     """
    #     self.findChild(QLineEdit, "entry_edit_line {}".format(entry_name)).setText(value)

    # ------------------------------ Function triggered when clicking on a browse button -------------------------------
    # def fill_line_with_browsing(self):
    #     """
    #         Open a window allowing the user to browse through the file system to provide either a file or a directory
    #         and automatically set it to the entry edit line
    #     """
    # Retrieve the entry_name of the user_entry that triggered the function
    #     entry_name = " ".join(self.sender().objectName().split(" ")[1:])
    # Get the filter information for the given user entry
    #     filter_info = FILE_TO_EXTENSION[entry_name]
    # If it's not empty then open a dialog box allowing the user to select a file
    #     if filter_info:
    #         description, name, extension = filter_info
    #         returned_path, _ = QFileDialog.getOpenFileName(self, "Select the {} file".format(description),
    #                                                        filter="{}(*{})".format(name, extension),
    #                                                        directory=CATKIN_WS)
    # Otherwise it means what is expected is a folder
    #     else:
    #         returned_path = QFileDialog.getExistingDirectory(self, "Select the {} ".format(entry_name.lower()),
    #                                                          directory=CATKIN_WS)
    # If a folder or file has been selected then set the text with its path
    #     if returned_path:
    #         self.findChild(QLineEdit, "entry_edit_line {}".format(entry_name)).setText(returned_path)

    # ---------------------------------- Methods related to configuration restoration ----------------------------------
    # def save_config(self, settings):
    #     """
    #         Save the current state of the widget
    #
    #         @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
    #     """
    #     object_name = self.objectName()
    #     settings.beginGroup(object_name)
    #     settings.setValue("type", self.metaObject().className())
    #     for registered_key in self.registered_entries:
    #         text_value = self.get_entry_value(registered_key)
    #         settings.setValue("{}".format(registered_key), text_value)
    #     for registered_subwidget in self.registered_subwidgets:
    #         widget = self.findChild(XMLEditorWidget, "Editor {}".format(registered_subwidget))
    #         widget.save_config(settings)
    #     settings.endGroup()
    #
    # def restore_config(self, settings):
    #     """
    #         Set the different components of the widget according to a specified configuration
    #
    #         @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
    #     """
    #     settings.beginGroup(self.objectName())
    #     for registered_key in self.registered_entries:
    #         self.set_entry_value(registered_key, settings.value(registered_key))
    #     for registered_subwidget in self.registered_subwidgets:
    #         widget = self.findChild(XMLEditorWidget, "Editor {}".format(registered_subwidget))
    #         widget.restore_config(settings)
    #     settings.endGroup()


class SimulationConfig(GenericInterfaceConfigWidget):

    """
        Widget allowing to set up the simulation parameters
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different user entries

            @param parent: parent of the widget
        """
        super(SimulationConfig, self).__init__("Simulation parameters", parent=parent)
        self.initialize_content()

    def initialize_content(self):
        """
            Create and add the user entries allowing to set up the simulation mode
        """
        # Add a check box to specify whether the simulation mode should be activated
        self.check_box = QCheckBox("Simulation")
        self.check_box.setChecked(True)
        self.layout.addWidget(self.check_box, 0, 0)
        self.gazebo_file_entry_widget = GazeboWorldEntryWidget("Gazebo world file", parent=self)
        self.gazebo_folder_entry_widget = GazeboFolderEntryWidget("Gazebo model folder", parent=self)
        self.starting_pose_entry_widget = StartingPoseEntryWidget("Starting pose", browser_button=False,
                                                                 placeholder_text="-J shoulder_pan_joint 0.5 "
                                                                 "-J shoulder_lift_joint 1.5 -J ...", parent=self)
        self.layout.addWidget(self.gazebo_file_entry_widget)
        self.layout.addWidget(self.gazebo_folder_entry_widget)
        self.layout.addWidget(self.starting_pose_entry_widget)

    # def save_config(self, settings):
    #     """
    #         Store the state of this widget and its children into settings
    #
    #         @settings: QSettings object in which widgets' information are stored
    #     """
    #     super(SimulationConfig, self).save_config(settings)
    #     settings.beginGroup(self.objectName())
    #     settings.setValue("is_checked", self.check_box.isChecked())
    #     settings.endGroup()
    #
    # def restore_config(self, settings):
    #     """
    #         Restore the children's widget from the configuration saved in settings
    #
    #         @settings: QSettings object that contains information of the widgets to restore
    #     """
    #     super(SimulationConfig, self).restore_config(settings)
    #     settings.beginGroup(self.objectName())
    #     self.check_box.setChecked(settings.value("is_checked", type=bool))
    #     settings.endGroup()


class MoveitConfig(GenericInterfaceConfigWidget):

    """
        Widget allowing the user to specify the optional MoveIt! configuration of a robot
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different user entries

            @param parent: parent of the widget
        """
        super(MoveitConfig, self).__init__("Moveit parameters", parent=parent)
        self.initialize_content()
        # self.retrieve_entries()
        # self.moveit_package_entry_line.textChanged.connect(self.update_xml_editors)
        # self.package_path = None
        # self.package_name = None

    def initialize_content(self):
        """
            Create and add the user entries allowing to set up the MoveIt! configuration
        """
        self.moveit_package_entry_widget = GenericUserEntryWidget("Moveit package", parent=self)
        self.move_group_editor = XMLEditorWidget("Move group arguments (optional)", enabled=False, parent=self)
        self.rviz_editor = XMLEditorWidget("RViz arguments (optional)", enabled=False, parent=self)
        self.layout.addWidget(self.moveit_package_entry_widget)
        self.layout.addWidget(self.move_group_editor)
        self.layout.addWidget(self.rviz_editor)

    # ------------------------------------ Methods related to setting up the editors -----------------------------------
    # def retrieve_entries(self):
    #     """
    #         Sets the two XMLEditorWidget objects and the LineEdit as attributes
    #     """
    #     self.move_group_xml_editor = self.findChild(
    #         XMLEditorWidget, "Editor {}".format("Move group arguments (optional)"))
    #     self.rviz_xml_editor = self.findChild(XMLEditorWidget, "Editor {}".format("RViz arguments (optional)"))
    #     self.moveit_package_entry_line = self.findChild(QLineEdit, "entry_edit_line {}".format("Moveit package"))
    #
    # def fill_line_with_browsing(self):
    #     """
    #         In addition to fill the edit line of the user entry, setup the two XML editors
    #     """
    #     super(MoveitConfig, self).fill_line_with_browsing()
    #     if "Moveit package" in self.sender().objectName():
    #         self.setup_editors()

    # def setup_editors(self):
    #     """
    #         Setup the editors allowing to modify the move group and rviz launch files
    #     """
    #     package_name = self.get_entry_value("Moveit package")
    #     # Get the package name from ROS
    #     package_name_ros = rospkg.get_package_name(package_name)
    #     # If package_name_ros is none it means the folder is not a ROS package
    #     if package_name and package_name_ros is None:
    #         display_error_message("Error message", "The selected directory is not ROS package", parent=self)
    #         return
    #     if package_name:
    #         self.package_path = package_name
    #         self.package_name = package_name_ros
    #         # If the provided package is valid then make the editors enabled
    #         self.move_group_xml_editor.setEnabled(True)
    #         self.rviz_xml_editor.setEnabled(True)
    #         # Display the skeleton helping the user to change options of some of the moveit launch files
    #         self.move_group_xml_editor.set_editor_content(self.get_moveit_config("move_group"))
    #         self.rviz_xml_editor.set_editor_content(self.get_moveit_config("moveit_rviz"))
    #
    # def update_xml_editors(self):
    #     """
    #         Method changing the editors' content according to provided MoveIt! package
    #     """
    #     path = self.sender().text()
    #     # If the path has been modified and is no longer correct then disabled the editors.
    #     if not os.path.isdir(path):
    #         self.move_group_xml_editor.code_editor.reset()
    #         self.rviz_xml_editor.code_editor.reset()
    #         self.move_group_xml_editor.setEnabled(False)
    #         self.rviz_xml_editor.setEnabled(False)
    #         self.package_name = None
    #         self.package_path = None
    #     # Otherwise change its content
    #     else:
    #         self.setup_editors()
    #
    # # --------------------------------------------------- Getters ------------------------------------------------------
    # def get_moveit_config(self, filename):
    #     """
    #         Returns the configuration for a given editor according to its filename
    #
    #         @param filename: String specifying which editor it refers to
    #
    #         @return: String corresponding to what will be added to the generated launch file
    #     """
    #     if filename == "move_group":
    #         arguments = self.move_group_xml_editor.get_formated_arguments()
    #     elif filename == "moveit_rviz":
    #         arguments = self.rviz_xml_editor.get_formated_arguments()
    #     else:
    #         return
    #
    #     if arguments is None:
    #         arguments = ""
    #     else:
    #         arguments = "\t" + arguments
    #
    #     return "<include file=\"$(eval find('{}') + '/launch/{}.launch')\">\n\t"\
    #            "<!-- You can add here any options you want to the file -->\n{}</include>".format(self.package_name,
    #                                                                                              filename, arguments)
    #
    # def get_parsed_info(self):
    #     """
    #         Returns information about MoveIt! controllers and planners
    #
    #         @return: Two dictionaries containing information about controllers and planners set in the MoveIt! package
    #     """
    #     # Just makes sure we can proceed
    #     if self.package_path is None:
    #         return
    #
    #     # Initialize a dictionary with an empty string as key. It is useful to let the user input another controller
    #     controllers_info = {"": ""}
    #     # Parse the name of the controllers used by MoveIt! (likely to be ROS controllers)
    #     with open(os.path.join(self.package_path, "config", "controllers.yaml"), "r") as f:
    #         moveit_controllers = yaml.safe_load(f)
    #     for controller in moveit_controllers["controller_list"]:
    #         controllers_info[controller["name"]] = controller["joints"]
    #
    #     planning_groups_info = dict()
    #     # Get the name of the planning groups as well as their list of possible planners
    #     with open(os.path.join(self.package_path, "config", "ompl_planning.yaml"), "r") as f:
    #         planning_groups = yaml.safe_load(f)
    #     for group_name, config in planning_groups.items():
    #         if group_name != "planner_configs":
    #             planning_groups_info[group_name] = config["planner_configs"]
    #
    #     return controllers_info, planning_groups
    #
    # # ---------------------------------- Methods related to configuration restoration ----------------------------------
    # def save_config(self, settings):
    #     """
    #         Store the state of this widget into settings
    #
    #         @settings: QSettings object in which widgets' information are stored
    #     """
    #     super(MoveitConfig, self).save_config(settings)
    #     settings.beginGroup(self.objectName())
    #     settings.setValue("package_name", self.package_name)
    #     settings.endGroup()
    #
    # def restore_config(self, settings):
    #     """
    #         Restore the children's widget from the configuration saved in settings
    #
    #         @settings: QSettings object that contains information of the widgets to restore
    #     """
    #     super(MoveitConfig, self).restore_config(settings)
    #     settings.beginGroup(self.objectName())
    #     self.package_name = settings.value("package_name")
    #     settings.endGroup()
    #     if self.package_name is not None:
    #         self.move_group_xml_editor.set_editor_content(self.get_moveit_config("move_group"))
    #         self.rviz_xml_editor.set_editor_content(self.get_moveit_config("moveit_rviz"))


class RobotInterfaceConfig(GenericInterfaceConfigWidget):

    """
        Widget allowing the user to interface robot
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different user entries

            @param parent: parent of the widget
        """
        super(RobotInterfaceConfig, self).__init__("Robot interface", parent=parent)
        self.initialize_content()
        # self.retrieve_editor()
        # self.findChild(QLineEdit, "entry_edit_line {}".format(
        #     "Custom launch file")).textChanged.connect(self.update_xml_editor)
        # self.launch_file_path = None
        # self.package_name = None

    # ----------------------------------------- Setup the content of the widget ----------------------------------------
    def initialize_content(self):
        """
            Create and set the different entries composing this widget
        """
        # self.robot_urdf_entry_widget = GenericUserEntryWidget("Robot's URDF file", parent=self)
        self.robot_urdf_entry_widget = UrdfEntryWidget(parent=self)
        self.urdf_args_entry_widget = UrdfArgumentsEntryWidget(browser_button=False, parent=self)
        self.launch_file_entry_widget = LaunchFileEntryWidget(browser_button=True, parent=self)
        self.launch_file_editor = XMLEditorWidget("Launch file arguments (optional)", parent=self)
        self.collision_scene_entry_widget = CollisionFileEntryWidget("Collision scene", enabled=False, parent=self)
        self.layout.addWidget(self.robot_urdf_entry_widget)
        self.layout.addWidget(self.urdf_args_entry_widget)
        self.layout.addWidget(self.launch_file_entry_widget)
        self.layout.addWidget(self.launch_file_editor)
        self.layout.addWidget(self.collision_scene_entry_widget)
        self.add_robot_composition()

    def add_robot_composition(self):
        """
            Add three custom made spin boxes allowing to specify what the robot is composed of
        """
        horizontal_layout = QHBoxLayout()
        horizontal_layout.addWidget(QLabel("The robot is composed of: "))
        self.arm_spin_box = HardwareSpinBox("arm", self)
        self.hand_spin_box = HardwareSpinBox("hand", self)
        self.sensor_spin_box = HardwareSpinBox("sensor", self)
        horizontal_layout.addWidget(self.arm_spin_box)
        horizontal_layout.addWidget(self.hand_spin_box)
        horizontal_layout.addWidget(self.sensor_spin_box)
        self.layout.addLayout(horizontal_layout, 5, 0, 1, 1)

    # ------------------------------------------- Methods related to editor -------------------------------------------
    # def retrieve_editor(self):
    #     """
    #         Store the xml editor used for the launch file into the attribute of the class
    #     """
    #     self.launch_xml_editor = self.findChild(XMLEditorWidget, "Editor {}".format("Launch file arguments (optional)"))
    #
    # def setup_editor(self):
    #     """
    #         Setup the editor allowing to modify the provided launch file
    #     """
    #     launch_file_path = self.get_entry_value("Custom launch file")
    #     package_name = rospkg.get_package_name(launch_file_path)
    #     if launch_file_path and package_name is None:
    #         display_error_message("Error message", "The selected file is not part of a ROS package", parent=self)
    #         return
    #     if launch_file_path:
    #         self.launch_file_path = launch_file_path.rsplit(package_name)[-1]
    #         self.package_name = package_name
    #         self.launch_xml_editor.setEnabled(True)
    #         self.launch_xml_editor.set_editor_content(self.get_launch_config())
    #
    # # ------------------------------------------ Methods triggered by signals ------------------------------------------
    # def update_xml_editor(self):
    #     """
    #         Method changing the editor's content according to provided launch file path
    #     """
    #     path = self.sender().text()
    #     if not os.path.isdir(path):
    #         self.launch_xml_editor.code_editor.reset()
    #         self.launch_xml_editor.setEnabled(False)
    #         self.launch_file_path = None
    #     else:
    #         self.setup_editor()
    #
    # def fill_line_with_browsing(self):
    #     """
    #         In addition to fill the edit line of the user entry, setup the XML editors
    #     """
    #     super(RobotInterfaceConfig, self).fill_line_with_browsing()
    #     # Automatically sets up the launch file editor
    #     if self.sender().objectName() == "browser Custom launch file":
    #         self.setup_editor()
    #
    # # ------------------------------------------ Parsers and getters ---------------------------------------------------
    # def get_robot_name(self):
    #     """
    #         Extract the robot name out of the provided URDF file
    #
    #         @return: String containing the name of the robot
    #     """
    #     robot_urdf_path = self.get_entry_value("Robot's URDF file")
    #     with open(robot_urdf_path, "r") as f:
    #         urdf_file_content = "".join(f.readlines())
    #     robot_name = re.search("<robot .*? name=\"(.*?)\">", urdf_file_content, re.DOTALL).group(1)
    #     return robot_name
    #
    # def get_launch_config(self):
    #     """
    #         Returns the configuration according to the editor's content
    #
    #         @return: String corresponding to what will be added to the generated launch file
    #     """
    #     arguments = self.launch_xml_editor.get_formated_arguments()
    #
    #     if arguments is None:
    #         arguments = ""
    #     else:
    #         arguments = "\t" + arguments
    #
    #     return "<include file=\"$(find {}){}\">\n\t"\
    #            "<!-- You can add any options you want to the file -->\n{}\n</include>".format(self.package_name,
    #                                                                                           self.launch_file_path,
    #                                                                                           arguments)
    #
    # def is_urdf_file(self):
    #     """
    #         Returns either the content of the user entry "Robot's URDF file" is a valid urdf file or not
    #
    #         @return: True if the provided file is a valid urdf file, False otherwise
    #     """
    #     path_file = self.get_entry_value("Robot's URDF file")
    #     return path_file.endswith(".urdf") or path_file.endswith(".urdf.xacro")
    #
    # # ---------------------------------- Methods related to configuration restoration ----------------------------------
    # def save_config(self, settings):
    #     """
    #         Store the state of this widget and its children into settings
    #
    #         @settings: QSettings object in which widgets' information are stored
    #     """
    #     super(RobotInterfaceConfig, self).save_config(settings)
    #     settings.beginGroup(self.objectName())
    #     settings.setValue("package_name", self.package_name)
    #     settings.setValue("launch_path", self.launch_file_path)
    #     settings.setValue("number_of_arm", self.arm_spin_box.get_value())
    #     settings.setValue("number_of_manipulator", self.hand_spin_box.get_value())
    #     settings.setValue("number_of_sensor", self.sensor_spin_box.get_value())
    #     settings.endGroup()
    #
    # def restore_config(self, settings):
    #     """
    #         Restore the children's widget from the configuration saved in settings
    #
    #         @settings: QSettings object that contains information of the widgets to restore
    #     """
    #     super(RobotInterfaceConfig, self).restore_config(settings)
    #     settings.beginGroup(self.objectName())
    #     self.package_name = settings.value("package_name")
    #     self.launch_file_path = settings.value("launch_path")
    #     self.arm_spin_box.set_value(settings.value("number_of_arm", type=int))
    #     self.hand_spin_box.set_value(settings.value("number_of_manipulator", type=int))
    #     self.sensor_spin_box.set_value(settings.value("number_of_sensor", type=int))
    #     settings.endGroup()
    #     if self.package_name is not None:
    #         self.launch_xml_editor.set_editor_content(self.get_launch_config())


class HardwareSpinBox(QWidget):

    """
        Widget containing a label and a spin box
    """

    def __init__(self, name, parent=None):
        """
            Initialize the class by creating the layout and wisgets

            @param name: text to write after the spin box
            @param parent: parent of the widget
        """
        super(HardwareSpinBox, self).__init__(parent=parent)
        self.init_ui()
        self.original_text = name
        self.create_widgets()

    def init_ui(self):
        """
            Set up the layout
        """
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def create_widgets(self):
        """
            Creates and adds to the layout both the spin box and label
        """
        self.spin_box = QSpinBox(self, objectName=self.original_text)
        self.spin_box.setMaximumWidth(45)
        self.label_text = QLabel(self.original_text)
        self.spin_box.valueChanged.connect(self.make_plural)
        self.layout.addWidget(self.spin_box)
        self.layout.addWidget(self.label_text)

    def make_plural(self):
        """
            Make the label plural if the value of the spin box is greater than 1
        """
        if self.spin_box.value() > 1:
            self.label_text.setText(self.original_text + "s")
        else:
            self.label_text.setText(self.original_text)

    def get_value(self):
        """
            Returns the value of the spin boxes

            @return: Value (int) of the spin box
        """
        return self.spin_box.value()

    def set_value(self, value):
        """
            Sets the vlaue of the spin box

            @param value: Value to set to the spin box (int)
        """
        self.spin_box.setValue(value)
