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

API_PATH = rospkg.RosPack().get_path("modular_framework_api")

# Robot integration
ROBOT_INTEGRATION_CONFIGS_FOLDER = os.path.join(API_PATH, "robot_integration_configs")
ROBOT_INTEGRATION_MAIN_CONFIG_FILE = os.path.join(ROBOT_INTEGRATION_CONFIGS_FOLDER, "gui.ini")
ROBOT_INTEGRATION_DEFAULT_CONFIG_FILE = os.path.join(ROBOT_INTEGRATION_CONFIGS_FOLDER, "gui_config.ini")
