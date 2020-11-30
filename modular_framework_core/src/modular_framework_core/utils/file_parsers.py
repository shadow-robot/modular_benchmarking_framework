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

import os
import re
from collections import OrderedDict

AVAILABLE_STATES = OrderedDict()
AVAILABLE_STATEMACHINES = OrderedDict()


def extract_init_from_file(file_path):
    """
        Extract and format the content of the __init__ function contained in a file

        @param file_path: Path to the file
        @return: List of strings
    """
    # Get a single string with the whole file inside
    with open(file_path, "r") as file_:
        file_content = "\n".join(file_.readlines())
    init_command = re.search("def __init__\((.*?)\):", file_content, re.DOTALL).group(1)
    init_command = re.sub("\n", "", init_command)
    init_command.replace(",", ", ")
    init_command = re.sub("\s{2}", "", init_command)
    init_command = re.split('\,\s*(?![^\[)]*\])', init_command)
    # Remove the self from the arguments
    return init_command[1:]


def extract_state_parameters_from_file(file_path):
    """
        Extract the parameters required to initialize a state

        @param file_path: Path to the file to parse
        @return: OrderedDict containing parameters name and potential default values
    """
    parameters = OrderedDict()
    split_parameters = extract_init_from_file(file_path)

    for parameter in split_parameters:
        # if parameter != "self":
        if "=" not in parameter:
            parameters[parameter] = ""
        else:
            name, value = parameter.split("=")
            parameters[name] = value
    return parameters


def extract_state_machine_parameters_from_file(file_path):
    """
        Extract the parameters required to initialize a state machine

        @param file_path: Path to the file to parse
        @return: OrderedDict containing parameters name and potential default values
    """
    parameters = OrderedDict()
    split_parameters = extract_init_from_file(file_path)
    # For each argument takes care of the jinja commands
    for parameter in split_parameters:
        parameter = re.sub("{%(.*?)%}", "", parameter)
        if "=" not in parameter:
            parameters[parameter] = ""
        else:
            name, value = parameter.split("=")
            if "{{" not in value and "%" not in value:
                parameters[name] = value
            else:
                value = re.sub("{{(.*?)}}", "", value)
                value = re.findall("[\'\"](.*?)[\'\"]", value)
                parameters[name] = value[0] if len(value) == 1 else value
    return parameters


def extract_description_from_file(file_path):
    """
        Extract the description (docstring) from the class contianed in the file

        @param file_path: Path to the file the description should be extracted
        @return: Description (string) if defined otherwise an empty string
    """
    with open(file_path, "r") as file_:
        file_content = "\n".join(file_.readlines())
    description = re.findall("class(?:[^\:]*)\:(?:[^\"]*)\"{3}(.*?)\"{3}", file_content, re.DOTALL)
    if not description:
        return "No description available"
    description = re.sub("\n", "", description[0])
    description = re.sub("\s{2}", "", description)
    return description


def fill_available_state_machines(path_folders):
    """
        Load all the state machine templates from the different paths contained in path_folders

        @param path_folders: List of paths pointing to directories containing templates to load
        @return: True if one of the path is not correct
    """
    not_a_path = False
    for path_folder in path_folders:
        if not os.path.isdir(path_folder):
            not_a_path = True
            continue
        for root, dirs, files in os.walk(path_folder):
            for file in files:
                if file.endswith(".template"):
                    file_path = os.path.join(root, file)
                    name = "".join([word.capitalize() for word in file.replace(".template", "").split("_")])
                    description = extract_description_from_file(file_path)
                    parameters = extract_state_machine_parameters_from_file(file_path)
                    if name not in AVAILABLE_STATEMACHINES.keys():
                        AVAILABLE_STATEMACHINES[name] = {"source": file_path, "parameters": parameters,
                                                         "description": description}
                    else:
                        print("Multiple file with the same name have been found. Ignoring the others.")
    if not_a_path:
        return True


def fill_available_states(path_folders):
    """
        Load all the states from the different paths contained in path_folders

        @param path_folders: List of paths pointing to directories containing the states to load
        @return: True if one of the path is not correct
    """
    not_a_path = False
    for path_folder in path_folders:
        if not os.path.isdir(path_folder):
            not_a_path = True
            continue

        for root, dirs, files in os.walk(path_folder):
            for file in files:
                if file.endswith(".py") and file != "__init__.py":
                    file_path = os.path.join(root, file)
                    name = "".join([word.capitalize() for word in file.replace(".py", "").split("_")])
                    description = extract_description_from_file(file_path)
                    parameters = extract_state_parameters_from_file(file_path)
                    if name not in AVAILABLE_STATES.keys():
                        AVAILABLE_STATES[name] = {"source": file_path, "parameters": parameters,
                                                  "description": description}
                    else:
                        print("The state named {} already exists. Ignoring the others.".format(name))
    if not_a_path:
        return True