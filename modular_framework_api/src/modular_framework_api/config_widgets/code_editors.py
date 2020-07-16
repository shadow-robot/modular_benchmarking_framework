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

from PyQt5 import Qsci
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtSignal, QTimer
import re
from collections import OrderedDict
import copy


class GenericCodeEditor(Qsci.QsciScintilla):

    """
        QScintilla-based widget allowing to create a generic code editor
    """
    # Signal sent when the the parsed content changes
    contentIsModified = pyqtSignal(bool)

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super(GenericCodeEditor, self).__init__(parent)
        self.init_ui()
        self.init_backround_markers()
        self.lexer_ = None
        self.initial_content = OrderedDict()
        # Will contain the index of the lines wrongly formatted
        self.wrong_format_lines = list()
        # Timer used to check content format and that will be handled by a different thread
        self.timer = QTimer()
        # Set it to single shot (basically can be run only one at a time)
        self.timer.setSingleShot(True)
        # Once the timer is timing out then starts the check
        # We do this to avoid having stuff signaled as wrong while editing
        self.timer.timeout.connect(self.parse_and_format_editor)
        # Each time a character is typed in the editor starts again the timer
        self.textChanged.connect(self.start_timer)
        self.add_marker_margin = False

    def init_ui(self):
        """
            Initialize the editor
        """
        # By default no lexer is set
        self.setLexer(None)
        # Set a grayish colour
        self.empty_color = QColor("#cccccc")
        self.setPaper(self.empty_color)
        # Set the tab width to 2 to save space
        self.setTabWidth(2)
        # Change tabs to spaces
        self.setIndentationsUseTabs(False)
        # Help to visualize the indentation
        self.setIndentationGuides(True)
        # Set auto indentation
        self.setAutoIndent(True)
        # Cannot be edited by the user
        self.setReadOnly(True)
        self.is_lexed = False

    def init_backround_markers(self):
        """
            Define markers to make lines not properly formatted into a red-ish colour
        """
        self.markerDefine(Qsci.QsciScintilla.Background, 0)
        self.setMarkerBackgroundColor(QColor("#40FF0000"), 0)

    def parse_and_format_editor(self):
        """
            Run the parser on the editor's content and signal which lines are not well formatted
        """
        # Parse the content of the editor
        self.parse_content()
        # Make the background of wrongly formatted lines red-ish
        self.update_background()

    def start_timer(self):
        """
            Start the timer that triggers the content's format checking
        """
        # The timer would timeout after 750ms meaning that the check would happend 750ms after the last text edit
        self.timer.start(750)

    def update_background(self):
        """
            Update the markers based on which lines are detected as wrong
        """
        self.markerDeleteAll()
        lines = self.wrong_format_lines
        for line in lines:
            self.markerAdd(line, 0)

    def parse_content(self):
        """
            Parse the content of the editor (will be overridden by children classes)
        """
        return

    def set_lexer(self):
        """
            Allow the user to edit the object
        """
        self.setLexer(self.lexer_)
        self.setReadOnly(False)
        self.is_lexed = True
        if self.add_marker_margin:
            self.markerAdd(0, 1)

    def reinitialize(self):
        """
            Set the editor to its initial state (uneditable with empty background)
        """
        self.clear()
        self.setLexer(None)
        self.is_lexed = False
        self.setReadOnly(True)
        self.setPaper(self.empty_color)
        self.initial_content = OrderedDict()

    def reset(self):
        """
            Clean the editor (i.e. remove content and reset attributes) but keep is editable
        """
        self.clear()
        self.initial_content = OrderedDict()


class YamlCodeEditor(GenericCodeEditor):

    """
        QScintilla-based widget allowing to create a YAML code editor
    """

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super(YamlCodeEditor, self).__init__(parent)
        self.init_symbol_margin()
        self.lexer_ = Qsci.QsciLexerYAML(self)
        # Will contain the parsed content
        self.parsed_content = OrderedDict()
        # By default no symbol should be present in the margin
        self.add_marker_margin = False

    def init_symbol_margin(self):
        """
            Initialize the margin with a symbol allowing to help integrating a given component
        """
        # Give the ability to set symbols in the margin
        self.setMarginType(1, Qsci.QsciScintilla.SymbolMargin)
        # Make sure the margin does not become too large
        self.setMarginWidth(1, "00")
        # Define a plus marker (index 0)
        self.markerDefine(Qsci.QsciScintilla.Plus, 1)
        # Make the margin clickable
        self.setMarginSensitivity(1, True)

    def set_autocompletion(self, items):
        """
            Allow the strings contained in items to be autocompleted after two characters

            @param items: List of strings containing the words to propose for autocompletion
        """
        self.setAutoCompletionSource(Qsci.QsciScintilla.AcsAPIs)
        self.setAutoCompletionThreshold(2)
        self.api = Qsci.QsciAPIs(self.lexer_)
        for item in items:
            self.api.add(item)
        self.api.prepare()

    def turn_off_autocompletion(self):
        """
            Turn off autocompletion
        """
        self.setAutoCompletionSource(Qsci.QsciScintilla.AcsNone)

    def parse_content(self):
        """
            Parse the current editor's content and store the valid content in the parsed_content attribute
        """
        self.wrong_format_lines = list()
        editor_content = self.text()

        if not editor_content:
            self.parsed_content = OrderedDict()
            self.contentIsModified.emit(self.initial_content != self.parsed_content)
            return

        split_content = editor_content.split("\n")
        indices = [i for i, x in enumerate(split_content) if re.search("^(\w+)", x) is not None]

        if not len(indices):
            self.slices = [split_content]
        else:
            self.slices = [split_content[:indices[0]]] if indices[0] else []
            self.slices += [split_content[indices[i]:indices[i + 1]]
                            for i in range(len(indices) - 1)] + [split_content[indices[-1]:]]

        line_number = 0
        parsed = OrderedDict()
        level_dict = OrderedDict([(-1, parsed)])

        for slice_index, slice in enumerate(self.slices):
            expected_depth = 0
            for j in slice:
                a = re.search("(^\s{1,})", j)
                number_space = len(a.group(1)) if a else 0
                # print(number_space)
                depth = number_space / 2 if number_space % 2 == 0 else expected_depth + 1

                # split_line = re.search("(\-\s*)?(\S[^:]*)(\:\s?)?(\-\s*)?(\w[^\-]*)?", j)
                # print("depth is: {}".format(depth))
                # print("expected is {}".format(expected_depth))
                # if split_line:
                #     print(split_line.groups())
                if depth > expected_depth and j:
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # else:
                    # split_line = split_line.groups()
                clean = j.strip()
                split_line = re.search("([^\#\:\s\-]*)(\s?\:\s?)?(?(2)([^\[\{\:\s\#]*)|)?(\-\s?)?(?(4)([^\{\[\#]*)|)?"
                                       "(\{[^\#\[\{\(\]\}\)]*\})?(\[[^\#\:\[\{\]\}\(\)]*\])?(\s*\#.*)?", clean).groups()
                # print("split is {}".format(split_line))

                if all(not x for x in split_line):
                    line_number += 1
                    continue

                if split_line[-1] and all(not x for x in split_line[:-1]):
                    line_number += 1
                    continue

                if split_line[0] and all(not x for x in split_line[1:-1]):
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue

                if split_line[0] and split_line[1] and all(not x for x in split_line[2:-1]):
                    self.wrong_format_lines.append(line_number)
                    expected_depth += 1

                if not depth and (split_line[3] or split_line[5] or split_line[6]):
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue

                if split_line[3] and not (split_line[4] or split_line[5] or split_line[6]):
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue

                if split_line[3] and split_line[4]:
                    a = re.search("([^\:\s]*)\s?:\s?([^\:\s]*)", split_line[4].strip())
                    # print("a is not None: {}".format(a is not None))
                    # if a is not None:
                    #     print("value: {}".format(a.groups()))
                    #     print("not all in groups: {}".format(all(a.groups())))
                    if split_line[4].startswith(" ") or (a is not None and not all(a.groups())):
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue

                if split_line[5]:
                    content = re.search("\{(.*)\}", split_line[5]).group(1)
                    args = re.findall("([^\:\s\{\,]*)\s?:\s?([^\:\s\}\,]*)", split_line[5])
                    if not args and content:
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    if content:
                        estimated_args = re.split(",", content)
                        if len(estimated_args) != len(args):
                            self.wrong_format_lines.append(line_number)
                            line_number += 1
                            continue
                    if any(not x or not y for x, y in args):
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue

                if not split_line[3] and not split_line[2]:
                    if depth - 1 not in level_dict:
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    test = OrderedDict()
                    level_dict[depth - 1][split_line[0]] = test
                    level_dict[depth] = test
                    if depth >= 1:
                        if line_number - len(level_dict[depth - 1]) in self.wrong_format_lines:
                            self.wrong_format_lines.remove(line_number - len(level_dict[depth - 1]))

                elif not split_line[3] and split_line[2]:
                    if depth - 1 not in level_dict:
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    level_dict[depth - 1][split_line[0]] = split_line[2]
                    if depth >= 1:
                        if line_number - len(level_dict[depth - 1]) in self.wrong_format_lines:
                            self.wrong_format_lines.remove(line_number - len(level_dict[depth - 1]))
                elif split_line[3] and split_line[4]:
                    object_to_fill = level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]
                    val = split_line[4].strip()
                    t = re.search("([^\:\s]*)\s?:\s?([^\:\s]*)", val)
                    if t is not None:
                        val = OrderedDict([t.groups()])
                    if isinstance(object_to_fill, list):
                        object_to_fill.append(val)
                    else:
                        level_dict[depth - 2][level_dict[depth - 2].keys()[-1]] = [val]
                        del level_dict[depth - 1]
                    if depth >= 1:
                        if line_number - len(level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]) in self.wrong_format_lines:
                            self.wrong_format_lines.remove(
                                line_number - len(level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]))
                elif split_line[3] and split_line[6]:
                    values = split_line[6].split(",")
                    object_to_fill = level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]
                    if isinstance(object_to_fill, list):
                        object_to_fill.append([v.strip("[").strip("]").strip() for v in values])
                    else:
                        level_dict[depth - 2][level_dict[depth - 2].keys()[-1]] = [
                            v.strip("[").strip("]").strip() for v in values]
                    if depth >= 1:
                        if line_number - len(level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]) in self.wrong_format_lines:
                            self.wrong_format_lines.remove(
                                line_number - len(level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]))
                elif split_line[3] and split_line[5]:
                    content = re.search("\{(.*)\}", split_line[5]).group(1)
                    args = re.findall("([^\:\s\{\,]*)\s?:\s?([^\:\s\}\,]*)", split_line[5])
                    val = OrderedDict(args)
                    object_to_fill = level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]
                    if isinstance(object_to_fill, list):
                        object_to_fill.append(val)
                    else:
                        level_dict[depth - 2][level_dict[depth - 2].keys()[-1]] = [val]
                    if depth >= 1:
                        if line_number - len(level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]) in self.wrong_format_lines:
                            self.wrong_format_lines.remove(
                                line_number - len(level_dict[depth - 2][level_dict[depth - 2].keys()[-1]]))
                # print("Level dict is {}".format(level_dict))
                line_number += 1

        self.parsed_content = parsed
        self.contentIsModified.emit(self.initial_content != self.parsed_content)

    def update_background(self):
        """
            Update the markers based on which lines are detected as wrong
        """
        super(YamlCodeEditor, self).update_background()
        if self.add_marker_margin and not self.isReadOnly():
            self.markerAdd(0, 1)

    def reset_init_content(self):
        """
            Reset the initial content
        """
        self.initial_content = copy.deepcopy(self.parsed_content) if self.parsed_content else OrderedDict()

    def mark_component(self, component_name):
        """
            Mark all lines belonging to dictionary named component_name as wrongly formatted
        """
        slice_index = 0
        for slice in self.slices:
            if slice[0].startswith(component_name):
                break
            slice_index += 1

        beginning = 0
        for i in range(slice_index):
            beginning += len(self.slices[i])
        end = beginning + len(self.slices[slice_index]) - list(map(lambda x: x.strip(), self.slices[slice_index])).count("")

        lines_indices = range(beginning, end)
        for line_index in lines_indices:
            if line_index not in self.wrong_format_lines:
                self.wrong_format_lines.append(line_index)
        self.update_background()

    def set_margin_marker(self):
        """
            Make a symbol apperas in the margin of the editor
        """
        self.add_marker_margin = True

    def reset(self):
        """
            Clean the editor (i.e. remove content and reset attributes) but keep is editable
        """
        super(YamlCodeEditor, self).reset()
        self.parsed_content = OrderedDict()


class XmlCodeEditor(GenericCodeEditor):

    """
        QScintilla-based widget allowing to create a XML code editor
    """

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super(XmlCodeEditor, self).__init__(parent)
        self.lexer_ = Qsci.QsciLexerXML(self)
        self.initial_content = list()
        self.parsed_content = list()

    def parse_content(self):
        """
            Parse the XML file to capture correctly formatted arguments
        """
        self.wrong_format_lines = list()
        editor_content = self.text()

        if not editor_content:
            self.parsed_content = None
            self.contentIsModified.emit(self.initial_content != self.parsed_content)
            return

        raw_arguments = re.search("\<include file=.*?\>(.*?)\<\/include\>", editor_content, re.DOTALL)
        if raw_arguments is None:
            self.parsed_content = None
            self.contentIsModified.emit(self.initial_content != self.parsed_content)
            return

        raw_arguments = re.sub("<!-- You can add any options you want to the file -->", "", raw_arguments.group(1))
        # Strip is used to remove possible spaces at the head and tail of the string
        arguments_list = re.split("\n", raw_arguments.strip())
        filtered_arguments = [x.strip() for x in arguments_list if x]

        editor_list = re.split("\n", editor_content.strip())
        filtered_editor = [x.strip() for x in editor_list if x]

        self.parsed_content = list()
        if not filtered_arguments:
            return

        for argument in filtered_arguments:
            template_search = re.search("\<arg name\s?=\s?(.*?) value\s?=\s?(.*?)\s?\/\>", argument)
            if template_search is None:
                self.wrong_format_lines.append(filtered_editor.index(argument))
            else:
                self.parsed_content.append(argument)

        self.contentIsModified.emit(self.initial_content != self.parsed_content)

    def update_background(self):
        """
            Update the markers based on which lines are detected as wrong
        """
        super(XmlCodeEditor, self).update_background()
        # In case the editor's content in empty notifies that something is wrong
        if self.parsed_content is None and self.isEnabled():
            for line in self.lines():
                self.markerAdd(line, 0)
