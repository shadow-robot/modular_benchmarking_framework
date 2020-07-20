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

from collections import OrderedDict


def is_pose_valid(dictionary, add_frame_id=False, add_reference_frame=True):
    """
        Method allowing to check that a dictionary is properly formatted i.e contains the proper keys and
        have appropriate values type to represent a pose

        @param dictionary: Dictionary containing the pose to check
        @param add_frame_id: Boolean stating whether the pose should contain the field "frame_id" or not
        @param add_reference_frame: Boolean stating whether the pose should contain the field "reference_frame" or not
        @return: True of the dictionary is properly formatted, False otherwise
    """
    keys_to_check = ["position", "orientation"]
    if add_frame_id:
        keys_to_check.append("frame_id")
    if add_reference_frame:
        keys_to_check.append("reference_frame")

    if not set(keys_to_check) == set(dictionary):
        return False
    # If the position does not have the proper keys
    if set(dictionary["position"].keys()) != set(["x", "y", "z"]):
        return False
    # If the orientation doen't have appropriate keys
    is_quaternion_orientation = set(dictionary["orientation"].keys()) == set(["x", "y", "z", "w"])
    is_rpy_orientation = set(dictionary["orientation"].keys()) == set(["r", "p", "y"])
    if not (is_quaternion_orientation or is_rpy_orientation):
        return False
    # Check that values of both orientation and position are either int or float
    valid_position_value = all(isinstance(x, float) or isinstance(x, int) for x in dictionary["position"].values())
    valid_orient_value = all(isinstance(x, float) or isinstance(x, int) for x in dictionary["orientation"].values())
    # Check values provided for the frames are strings
    valid_reference_frame = True if not add_reference_frame else isinstance(dictionary["reference_frame"], str)
    valid_frame_id = True if not add_frame_id else isinstance(dictionary["frame_id"], str)
    # If all are good return True
    if valid_orient_value and valid_position_value and valid_frame_id and valid_reference_frame:
        return True

    return False


def is_topic_valid(topic_info):
    """
        Check that the information provided as topic info is valid

        @param topic_info: Value provided by the user
        @return: True if the value is properly formatted, False otherwise
    """
    if not isinstance(topic_info, OrderedDict):
        return False

    if any(not isinstance(topic_name, str) for topic_type, topic_name in topic_info.items()):
        return False
    return True


def is_moveit_planner_valid(moveit_planner):
    """
        Check that the information provided to use a moveit planner is correct

        @param moveit_planner: Dictionary provided by the user
        @return: True if the dictionary is properly formatted, False otherwise
    """
    planner_name_value = moveit_planner["planner_name"]
    robot_speed_value = moveit_planner["robot_speed_factor"]
    number_plan_value = moveit_planner["number_plan_attempt"]
    planning_time_value = moveit_planner["planning_max_time"]
    if not isinstance(planner_name_value, str):
        return False
    if (not (isinstance(robot_speed_value, float) or isinstance(robot_speed_value, int)) or
       (robot_speed_value <= 0 or robot_speed_value > 1)):
        return False
    if not isinstance(number_plan_value, int) or number_plan_value <= 0:
        return False
    if not (isinstance(planning_time_value, float) or isinstance(planning_time_value, int)) or planning_time_value <= 0:
        return False

    return all(x for x in moveit_planner.values())
