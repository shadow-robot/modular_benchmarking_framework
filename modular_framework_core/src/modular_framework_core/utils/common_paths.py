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

import rospkg
import os

# Framework packages
API_PATH = rospkg.RosPack().get_path("modular_framework_api")
CORE_PATH = rospkg.RosPack().get_path("modular_framework_core")

# Built in template folders
BUILT_IN_TEMPLATES_FOLDER = os.path.join(CORE_PATH, "templates")
FOLDER_TEMPLATE_LAUNCH_FILE = os.path.join(BUILT_IN_TEMPLATES_FOLDER, "launch_file")

# GUI configuration
GUI_CONFIGS_FOLDER = os.path.join(API_PATH, "gui_configs")
ROBOT_INTEGRATION_MAIN_CONFIG_FILE = os.path.join(GUI_CONFIGS_FOLDER, "gui.ini")
ROBOT_INTEGRATION_DEFAULT_CONFIG_FILE = os.path.join(GUI_CONFIGS_FOLDER, "gui_config.ini")

# Catkin workspace
CATKIN_WS = "/home/user/projects/shadow_robot/base/src"

# Robot launch
LAUNCH_CORE_FOLDER = os.path.join(CORE_PATH, "launch")
