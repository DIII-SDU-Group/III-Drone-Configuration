#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent, SetParametersResult
from rclpy.parameter import Parameter, ParameterValue

import os
import yaml
from datetime import datetime
from threading import Thread, Lock, Semaphore
import time
import yaml

from iii_drone_interfaces.srv import GetParameterYaml, GetDeclaredParameters, SaveParameters, GetParameterFiles, LoadParameters, SetParameterFromGC, GetCurrentParameterFile, SetCurrentParameterFileAsDefault

from iii_drone_configuration.parameter_handler import ParameterHandler

import npyscreen
import curses

###############################################################################
# Classes
###############################################################################

class ConfigurationClient(Node):
    def __init__(self):
        super().__init__(
            node_name="configuration_client",
            namespace="/configuration/configuration_client",
        )
        
        self.cb_group_1 = rclpy.callback_groups.ReentrantCallbackGroup()
        self.cb_group_2 = rclpy.callback_groups.ReentrantCallbackGroup()
        self.cb_group_3 = rclpy.callback_groups.ReentrantCallbackGroup()
        
        self.parameter_yaml_client = self.create_client(
            GetParameterYaml, 
            "/configuration/configuration_server/get_parameter_yaml",
            callback_group=self.cb_group_1
        )
        
        self.get_declared_parameters_client = self.create_client(
            GetDeclaredParameters,
            "/configuration/configuration_server/get_declared_parameters",
            callback_group=self.cb_group_3
        )
        
        self.get_current_parameter_file_client = self.create_client(
            GetCurrentParameterFile,
            "/configuration/configuration_server/get_current_parameter_file"
        )

        self.save_parameters_client = self.create_client(
            SaveParameters,
            "/configuration/configuration_server/save_parameters",
            callback_group=self.cb_group_3
        )
        
        self.set_current_parameter_file_as_default_client = self.create_client(
            SetCurrentParameterFileAsDefault,
            "/configuration/configuration_server/set_current_parameter_file_as_default",
            callback_group=self.cb_group_3
        )
        
        self.get_parameter_files_client = self.create_client(
            GetParameterFiles,
            "/configuration/configuration_server/get_parameter_files",
            callback_group=self.cb_group_3
        )
        
        self.load_parameter_file_client = self.create_client(
            LoadParameters,
            "/configuration/configuration_server/load_parameters",
            callback_group=self.cb_group_3
        )

        self.set_parameter_from_gc_client = self.create_client(
            SetParameterFromGC,
            "/configuration/configuration_server/set_parameter_from_gc",
            callback_group=self.cb_group_3
        )
        
        # self.cnt = 0
        
        self.parameter_handler = ParameterHandler()
        self.parameter_handler_updated = False
        self.parameter_handler_lock = Lock()

        self.current_parameter_file, self.default_parameter_file = None, None
        self.parameter_files_lock = Lock()
        self.parameter_files_updated = False
        
        self.updater_members_timer = self.create_timer(1.0, self.update_members, callback_group=self.cb_group_2)

        # self.test_timer = self.create_timer(1.0, lambda: print("Timer"))

    def call_save_parameters(self, file, set_as_default, overwrite):
        request = SaveParameters.Request()
        request.file = file
        request.set_as_default = set_as_default
        request.overwrite = overwrite
        
        while not self.save_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        result = self.save_parameters_client.call(request)

        return result.success, result.message, result.file
        
        # raise RuntimeError("Failed to save parameters")

    def call_set_current_parameter_file_as_default(self):
        request = SetCurrentParameterFileAsDefault.Request()
        
        while not self.set_current_parameter_file_as_default_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        result = self.set_current_parameter_file_as_default_client.call(request)

        return result.success, result.message
    
    def call_get_parameter_files(self):
        request = GetParameterFiles.Request()
        
        while not self.get_parameter_files_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        result = self.get_parameter_files_client.call(request)

        return result.parameter_files
    
    def call_load_parameter_file(self, file, set_as_default):
        request = LoadParameters.Request()
        request.file = file
        request.set_as_default = set_as_default
        
        while not self.load_parameter_file_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        result = self.load_parameter_file_client.call(request)

        return result.success, result.message

    def call_set_parameter_from_gc(self, name, value):
        request = SetParameterFromGC.Request()
        request.parameter_name = name
        request.parameter_string_value = str(value)
        
        while not self.set_parameter_from_gc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        result = self.set_parameter_from_gc_client.call(request)

        return result.success, result.message

    def call_get_declared_parameters(self):
        request = GetDeclaredParameters.Request()
        
        while not self.get_declared_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        result = self.get_declared_parameters_client.call(request)

        return yaml.safe_load(result.declared_parameters_yaml)

    def update_members(self):
        # print("Updating members")
        # with self.parameter_handler_lock:
        self.start_fetch_parameter_handler()
        self.start_get_current_parameter_file()

        # self.cnt += 1
        
        # if self.cnt > 10:
        #     exit(1)

    def start_get_current_parameter_file(self):
        request = GetCurrentParameterFile.Request()
        
        while not self.get_current_parameter_file_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        future = self.get_current_parameter_file_client.call_async(request)
        future.add_done_callback(self.on_get_current_parameter_file_done)
        
    def on_get_current_parameter_file_done(self, future):
        if future.result() is not None:
            result = future.result()
            with self.parameter_files_lock:
                self.current_parameter_file = str(result.current_parameter_file)
                self.default_parameter_file = str(result.default_parameter_file)

            self.parameter_files_updated = True
            
            return
            
        raise RuntimeError("Failed to get current parameter file")
        
    def start_fetch_parameter_handler(self):
        # Call service to get parameter yaml
        request = GetParameterYaml.Request()
        
        while not self.parameter_yaml_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        future = self.parameter_yaml_client.call_async(request)

        future.add_done_callback(self.on_fetch_parameter_handler_done)
        
    def on_fetch_parameter_handler_done(self, future):
        if future.result() is not None:
            self.get_logger().debug("Received parameter yaml")
            parameter_yaml = str(future.result().yaml)
            
            # Create parameter handler
            with self.parameter_handler_lock:
                self.parameter_handler = ParameterHandler.from_raw_yaml_string(parameter_yaml)

            self.parameter_handler_updated = True
                
            return
        
        raise RuntimeError("Failed to fetch parameter handler")

    def get_all_parameter_entries(self, only_declared=True):
        with self.parameter_handler_lock:
            params_dict = self.parameter_handler.get_all_params()
        
        params = []
        
        declared_parameters = self.call_get_declared_parameters()
        
        for param_name, param_entry in params_dict.items():
            if only_declared and param_name not in declared_parameters:
                continue
            
            entry = {
                "name": param_name,
                "type": str(param_entry["type"]),
                "constant": str(param_entry["constant"] if "constant" in param_entry else False),
                "value": param_entry["value"] if param_name not in declared_parameters else declared_parameters[param_name]
            }
            
            params.append(entry)
            
        return params, params_dict

class ConfigurationClientApp(npyscreen.NPSAppManaged):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.client_node = node
        
    #     self.last_spin_time = self.client_node.get_clock().now()
        
    # def onInMainLoop(self):
    #     super().onInMainLoop()
        
    #     # If the time since the last spin is greater than 0.1 seconds, spin the node:
    #     current_time = self.client_node.get_clock().now()
    #     time_diff = current_time - self.last_spin_time
        
    #     if time_diff.nanoseconds > 100000000:
    #         self.last_spin_time = current_time
    #         rclpy.spin_once(self.client_node, timeout_sec=0.1)
    
    def onStart(self):
        self.addForm("MAIN", ConfigurationClientForm, name="Configuration Client", client_node=self.client_node)
        self.addForm("PARAMETER_FILES", ParameterFilesForm, name="Parameter Files", client_node=self.client_node)
        self.addForm("SAVE_PARAMETERS", SaveParametersForm, name="Save Parameters", client_node=self.client_node)
        self.addForm("LOAD_FILE", LoadFileForm, name="Load File", client_node=self.client_node)
        self.addForm("STRING_PARAMETER_EDIT", StringParameterEditForm, name="String Parameter Edit", client_node=self.client_node)
        self.addForm("STRING_PARAMETER_EDIT_WITH_OPTIONS", StringParameterEditWithOptionsForm, name="String Parameter Edit With Options", client_node=self.client_node)
        self.addForm("INT_PARAMETER_EDIT", IntParameterEditForm, name="Int Parameter Edit", client_node=self.client_node)
        self.addForm("FLOAT_PARAMETER_EDIT", FloatParameterEditForm, name="Float Parameter Edit", client_node=self.client_node)
        self.addForm("BOOL_PARAMETER_EDIT", BoolParameterEditForm, name="Bool Parameter Edit", client_node=self.client_node)

class SaveParametersForm(npyscreen.Form):
    def create(self):
        self._name = self.add(npyscreen.TitleText, name="Save parameters", max_height=2, editable=False)
        
        self.file_name = self.add(npyscreen.TitleText, name="File name", max_height=2, 
                                  value=self.parentApp.client_node.current_parameter_file.split("/")[-1])
        self.set_as_default = self.add(npyscreen.TitleSelectOne, name="Set as default", values=["True", "False"], max_height=4,
                                       value=[1])
        self.overwrite = self.add(npyscreen.TitleSelectOne, name="Overwrite", values=["True", "False"], max_height=4,
                                  value=[1])
        
        self.save_button = self.add(npyscreen.ButtonPress, name="Save", when_pressed_function=self.save, max_height=2)
        self.cancel_button = self.add(npyscreen.ButtonPress, name="Cancel", when_pressed_function=self.cancel, max_height=2)

    # def on_save_done(self, future):
    #     exit(1)
    #     result = future.result()
    #     success, message, file = result.success, result.message, result.file
            
    #     if success:
    #         npyscreen.notify(message + "\nSaved to: " + file)
    #     else:
    #         npyscreen.notify("Could not save file: " + message)
            
    #     time.sleep(1)
        
    #     self.parentApp.switchForm("PARAMETER_FILES")

    def cancel(self):
        self.parentApp.getForm("PARAMETER_FILES").update_files()
        self.parentApp.switchForm("PARAMETER_FILES")
        
    def save(self):
        file_name = str(self.file_name.value)
        set_as_default = True if str(self.set_as_default.values[self.set_as_default.value[0]]) == "True" else False
        overwrite = True if str(self.overwrite.values[self.overwrite.value[0]]) == "True" else False

        # self.file_name.editable = False
        # self.set_as_default.editable = False
        # self.overwrite.editable = False
        
        # self.save_button.editable = False
        # self.cancel_button.editable = False

        success, message, file = self.parentApp.client_node.call_save_parameters(file_name, set_as_default, overwrite)

        if success:
            npyscreen.notify(message + "\nSaved to: " + file)
        else:
            npyscreen.notify("Could not save file: " + message)
            
        time.sleep(2)

        self.parentApp: ConfigurationClientApp
        
        self.parentApp.getForm("PARAMETER_FILES").update_files()
        
        self.parentApp.switchForm("PARAMETER_FILES")

class LoadFileForm(npyscreen.FormBaseNew):
    def create(self):
        self._name = self.add(npyscreen.TitleText, name="Load file", editable=False, max_height=2)
        
        self.table = self.add(npyscreen.GridColTitles,
            col_titles=["File"],
            select_whole_line=True,
            editable=True,
            column_width=30,
        )
        
        self.table.add_handlers({
            ord("\n"): self.load_file,
            ord("<"): self.cancel
        })
        
        self.set_editing(self.table)
        
    def load_file(self, keypress):
        file = self.table.values[self.table.edit_cell[0]][0]
        
        success, message = self.parentApp.client_node.call_load_parameter_file(file, False)
        
        npyscreen.notify(message)
        
        time.sleep(2)
        
        self.parentApp.getForm("PARAMETER_FILES").update_files()
        self.parentApp.switchForm("PARAMETER_FILES")
    
    def cancel(self, keypress):
        self.parentApp.getForm("PARAMETER_FILES").update_files()
        self.parentApp.switchForm("PARAMETER_FILES")

    def update_table(self):
        files = self.parentApp.client_node.call_get_parameter_files()
        
        self.table.values = [[file] for file in files]
        
        self.table.display()

class ParameterFilesForm(npyscreen.FormBaseNew):
    def create(self):
        self.add_handlers({
            ord('<'): self.show_main_form
        })
        
        self._name = self.add(npyscreen.TitleText, name="Parameter files", editable=False, max_height=2)
        
        self.current_file = self.add(npyscreen.TitleText, name="Current file", value=self.parentApp.client_node.current_parameter_file, editable=False, max_height=2)
        self.default_file = self.add(npyscreen.TitleText, name="Default file", value=self.parentApp.client_node.default_parameter_file, editable=False, max_height=2)
        
        self.save_file_button = self.add(npyscreen.ButtonPress, name="Save file", when_pressed_function=self.save_file, max_height=2)
        self.load_file_button = self.add(npyscreen.ButtonPress, name="Load file", when_pressed_function=self.load_file, max_height=2)
        self.set_current_file_as_default_button = self.add(npyscreen.ButtonPress, name="Set current parameter file as default", when_pressed_function=self.set_current_file_as_default, max_height=2)

        self.update_files()

    def h_display(self, keypress):
        self.update_file()

    def show_main_form(self, keypress):
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")

    def update_files(self):
        # Get the current parameter file
        current_file = self.parentApp.client_node.current_parameter_file
        default_file = self.parentApp.client_node.default_parameter_file
        
        self.current_file.value = current_file
        self.current_file.display()
        
        self.default_file.value = default_file
        self.default_file.display()
        
    def save_file(self):
        self.parentApp.switchForm("SAVE_PARAMETERS")
            
    def load_file(self):
        self.parentApp.getForm("LOAD_FILE").update_table()
        self.parentApp.switchForm("LOAD_FILE")
    
    def set_current_file_as_default(self):
        success, message = self.parentApp.client_node.call_set_current_parameter_file_as_default()

        npyscreen.notify(message)
            
        time.sleep(2)
        
        self.update_files()

class ConfigurationClientForm(npyscreen.FormBaseNew):
    def create(self):
        self._name = self.add(npyscreen.TitleText, name="Parameters", editable=False)
        self.search_word = ""
        self.search_box = self.add(npyscreen.TitleText, name="Search", value=self.search_word, editable=False)

        # Create a table layout:
        self.table = self.add(npyscreen.GridColTitles,
            col_titles=["Name", "Namespace", "Type", "Constant", "Value"],
            select_whole_line=True,
            editable=True,
            column_width=30,
            # max_height=self.useable_space()[0],  # Leave space for the search box
        )

        self.table.add_handlers({
            # Enter key
            ord('\n'): self.h_edit_cell,
            # Every character, number, / and backspace:
        })

        for char in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/_.":
            self.table.add_handlers({
                ord(char): self.h_search,
            })
            
        self.table.add_handlers({
            curses.KEY_BACKSPACE: self.h_search,
        })
        
        self.table.add_handlers({
            ord(">"): self.show_parameter_files_form,
        })
        
        self.parentApp: ConfigurationClientApp
        
        # Populate the table with parameter entries
        # self.update_entries_timer = self.parentApp.client_node.create_timer(1.0, self.update_table)

        self.update_table()

        # self.update_table_timer = self.parentApp.client_node.create_timer(1.0, self.update_table)

        self.set_editing(self.table)

    def show_parameter_files_form(self, keypress):
        self.parentApp.getForm("PARAMETER_FILES").update_files()
        self.parentApp.switchForm("PARAMETER_FILES")

    def update_table(self):
        # Get the latest parameter entries
        self.parameter_entries, self.parameters_dict = self.parentApp.client_node.get_all_parameter_entries()

        self.filtered_parameter_entries = []
        
        if self.search_word == "":
            self.filtered_parameter_entries = self.parameter_entries
            
        else:
            for entry in self.parameter_entries:
                search_word_lower = self.search_word.lower()
                if search_word_lower in entry["name"].lower() or search_word_lower in entry["type"].lower() or search_word_lower in str(entry["value"]).lower() or search_word_lower in str(entry["constant"]).lower():
                    self.filtered_parameter_entries.append(entry)

        # Update the table values
        self.table.values = [[entry["name"].split("/")[-1], "/".join(entry["name"].split("/")[0:-1]), entry["type"], entry["constant"] if "constant" in entry else False, entry["value"]] for entry in self.filtered_parameter_entries]

        self.table.display()
        
    def h_edit_cell(self, keypress):
        # Get the current selected row
        selected_row = self.table.edit_cell[0]
        
        # Get the parameter name
        param_name = self.filtered_parameter_entries[selected_row]["name"]
        
        # Get the parameter entry
        param_dict_entry = self.parameters_dict[param_name]
        
        # Get the parameter constant
        param_constant = param_dict_entry["constant"] if "constant" in param_dict_entry else False
        
        if param_constant:
            message = "Cannot edit constant parameter"
            npyscreen.notify(message)
            time.sleep(1)
            return

        param_dict_entry["name"] = param_name
        
        self.launch_parameter_edit_form(param_dict_entry)

    def launch_parameter_edit_form(self, dict_entry):

        param_type = dict_entry["type"]
        
        if param_type == "string":
            if "options" in dict_entry:
                self.parentApp.getForm("STRING_PARAMETER_EDIT_WITH_OPTIONS").update_param(dict_entry)
                self.parentApp.switchForm("STRING_PARAMETER_EDIT_WITH_OPTIONS")
            else:
                self.parentApp.getForm("STRING_PARAMETER_EDIT").update_param(dict_entry)
                self.parentApp.switchForm("STRING_PARAMETER_EDIT")
        elif param_type == "int":
            self.parentApp.getForm("INT_PARAMETER_EDIT").update_param(dict_entry)
            self.parentApp.switchForm("INT_PARAMETER_EDIT")
        elif param_type == "float":
            self.parentApp.getForm("FLOAT_PARAMETER_EDIT").update_param(dict_entry)
            self.parentApp.switchForm("FLOAT_PARAMETER_EDIT")
        elif param_type == "bool":
            self.parentApp.getForm("BOOL_PARAMETER_EDIT").update_param(dict_entry)
            self.parentApp.switchForm("BOOL_PARAMETER_EDIT")
        
    def h_search(self, keypress):
        # If backspace:
        if keypress == curses.KEY_BACKSPACE:
            self.search_word = self.search_word[:-1] if len(self.search_word) > 0 else ""
        else:
            self.search_word += chr(keypress)
            
        self.search_box.value = self.search_word
        
        self.search_box.display()
        self.update_table()
        
class StringParameterEditForm(npyscreen.FormBaseNew):
    def create(self):
        self.param_dict_entry = {
            "name": "name",
            "value": "value"
        }
        self._name = self.add(npyscreen.TitleText, name="Edit string parameter", editable=False)
        
        self.param_name = self.add(npyscreen.TitleText, name="Name", editable=False, max_height=2)
        self.param_value = self.add(npyscreen.TitleText, name="Value", max_height=2)
        
        self.save_button = self.add(npyscreen.ButtonPress, name="Save", when_pressed_function=self.save, max_height=2)
        self.cancel_button = self.add(npyscreen.ButtonPress, name="Cancel", when_pressed_function=self.cancel, max_height=2)

        self.param_dict_entry = None
        
    def save(self):
        param_value = str(self.param_value.value)
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        try:
            parameter_handler.can_set_param(self.param_dict_entry["name"],param_value)
        except KeyError:
            npyscreen.notify("Parameter not found")
            time.sleep(2)
            return
        except TypeError:
            npyscreen.notify("Parameter value is not of the specified type")
            time.sleep(2)
            return
        except ValueError:
            npyscreen.notify("Parameter value is invalid")
            time.sleep(2)
            return
        
        success, message = self.parentApp.client_node.call_set_parameter_from_gc(self.param_dict_entry["name"], param_value)
        
        npyscreen.notify(message)
        
        time.sleep(2)
        
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")
        
    def cancel(self):
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")

    def update_param(self, param_dict_entry):
        self.param_dict_entry = param_dict_entry
        
        self.param_name.value = param_dict_entry["name"]
        self.param_value.value = param_dict_entry["value"]
        
        self.param_name.display()
        self.param_value.display()
        
class StringParameterEditWithOptionsForm(npyscreen.FormBaseNew):
    def create(self):
        self.param_dict_entry = {
            "name": "name",
            "value": "option1",
            "options": ["option1", "option2", "option3"]
        }
        self._name = self.add(npyscreen.TitleText, name="Edit string parameter", editable=False)
        
        self.param_name = self.add(npyscreen.TitleText, name="Name", editable=False, max_height=2)
        self.param_value = self.add(npyscreen.TitleSelectOne, name="Value", max_height=2*len(self.param_dict_entry["options"]))
        
        self.save_button = self.add(npyscreen.ButtonPress, name="Save", when_pressed_function=self.save, max_height=2)
        self.cancel_button = self.add(npyscreen.ButtonPress, name="Cancel", when_pressed_function=self.cancel, max_height=2)

        self.param_dict_entry = None
        
    def save(self):
        param_value = str(self.param_value.values[self.param_value.value[0]])
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        try:
            parameter_handler.can_set_param(self.param_dict_entry["name"],param_value)
        except KeyError:
            npyscreen.notify("Parameter not found")
            time.sleep(2)
            return
        except TypeError:
            npyscreen.notify("Parameter value is not of the specified type")
            time.sleep(2)
            return
        except ValueError:
            npyscreen.notify("Parameter value is invalid")
            time.sleep(2)
            return
        
        success, message = self.parentApp.client_node.call_set_parameter_from_gc(self.param_dict_entry["name"], param_value)
        
        npyscreen.notify(message)
        
        time.sleep(2)
        
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")
        
    def cancel(self):
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")

    def update_param(self, param_dict_entry):
        self.param_dict_entry = param_dict_entry
        
        self.param_name.value = param_dict_entry["name"]
        self.param_value.values = param_dict_entry["options"]
        self.param_value.value = [param_dict_entry["options"].index(param_dict_entry["value"])]
        
        self.param_name.display()
        self.param_value.display()
        
class IntParameterEditForm(npyscreen.FormBaseNew):
    def create(self):
        self._name = self.add(npyscreen.TitleText, name="Edit int parameter", editable=False)
        
        self.param_name = self.add(npyscreen.TitleText, name="Name", editable=False, max_height=2)
        self.minimum = self.add(npyscreen.TitleText, name="Minimum", max_height=2, editable=False)
        self.maximum = self.add(npyscreen.TitleText, name="Maximum", max_height=2, editable=False)
        self.param_value = self.add(npyscreen.TitleText, name="Value", max_height=2)
        
        self.save_button = self.add(npyscreen.ButtonPress, name="Save", when_pressed_function=self.save, max_height=2)
        self.cancel_button = self.add(npyscreen.ButtonPress, name="Cancel", when_pressed_function=self.cancel, max_height=2)

        self.param_dict_entry = None

        self.add_handlers({
            ord('<'): lambda keypress: self.cancel()
        })
        
        self.param_value.add_handlers({
            ord('<'): lambda keypress: self.cancel()
        })

    def save(self):

        try:
            param_value = int(self.param_value.value)
        except ValueError:
            npyscreen.notify("Parameter value is not an integer")
            time.sleep(2)
            return
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        try:
            parameter_handler.can_set_param(self.param_dict_entry["name"],param_value)
        except KeyError:
            npyscreen.notify("Parameter not found")
            time.sleep(2)
            return
        except TypeError:
            npyscreen.notify("Parameter value is not of the specified type")
            time.sleep(2)
            return
        except ValueError:
            npyscreen.notify("Parameter value is invalid")
            time.sleep(2)
            return
        
        success, message = self.parentApp.client_node.call_set_parameter_from_gc(self.param_dict_entry["name"], param_value)
        
        npyscreen.notify(message)
        
        time.sleep(2)
        
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")
        
    def cancel(self):
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")

    def update_param(self, param_dict_entry):
        self.param_dict_entry = param_dict_entry
        
        self.param_name.value = param_dict_entry["name"]
        self.param_value.value = str(param_dict_entry["value"])
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        if "min" in param_dict_entry:
            min_expr = str(param_dict_entry["min"])
            min_eval = str(parameter_handler._evaluate_expression(min_expr, parameter_handler.params_dict))
            
            if min_eval != min_expr:
                self.minimum.value = min_expr + ": " + str(min_eval)
            else:
                self.minimum.value = min_expr
                
        else:
            self.minimum.value = "No minimum"
            
        if "max" in param_dict_entry:
            max_expr = str(param_dict_entry["max"])
            max_eval = parameter_handler._evaluate_expression(max_expr, parameter_handler.params_dict)
            
            if max_eval != max_expr:
                self.maximum.value = max_expr + ": " + str(max_eval)
            else:
                self.maximum.value = max_expr
        
        else:
            self.maximum.value = "No maximum"
                
        self.param_name.display()
        self.param_value.display()
        self.minimum.display()
        self.maximum.display()
        
class FloatParameterEditForm(npyscreen.FormBaseNew):
    def create(self):
        self._name = self.add(npyscreen.TitleText, name="Edit float parameter", editable=False)
        
        self.param_name = self.add(npyscreen.TitleText, name="Name", editable=False, max_height=2)
        self.minimum = self.add(npyscreen.TitleText, name="Minimum", max_height=2, editable=False)
        self.maximum = self.add(npyscreen.TitleText, name="Maximum", max_height=2, editable=False)
        self.param_value = self.add(npyscreen.TitleText, name="Value", max_height=2)
        
        self.save_button = self.add(npyscreen.ButtonPress, name="Save", when_pressed_function=self.save, max_height=2)
        self.cancel_button = self.add(npyscreen.ButtonPress, name="Cancel", when_pressed_function=self.cancel, max_height=2)

        self.param_dict_entry = None

        self.add_handlers({
            ord('<'): lambda keypress: self.cancel()
        })
        
        self.param_value.add_handlers({
            ord('<'): lambda keypress: self.cancel()
        })

    def save(self):

        try:
            param_value = float(self.param_value.value)
        except ValueError:
            npyscreen.notify("Parameter value is not a float")
            time.sleep(2)
            return
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        try:
            parameter_handler.can_set_param(self.param_dict_entry["name"],param_value)
        except KeyError:
            npyscreen.notify("Parameter not found")
            time.sleep(2)
            return
        except TypeError:
            npyscreen.notify("Parameter value is not of the specified type")
            time.sleep(2)
            return
        except ValueError:
            npyscreen.notify("Parameter value is invalid")
            time.sleep(2)
            return
        
        success, message = self.parentApp.client_node.call_set_parameter_from_gc(self.param_dict_entry["name"], param_value)
        
        npyscreen.notify(message)
        
        time.sleep(2)
        
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")
        
    def cancel(self):
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")

    def update_param(self, param_dict_entry):
        self.param_dict_entry = param_dict_entry
        
        self.param_name.value = param_dict_entry["name"]
        self.param_value.value = str(param_dict_entry["value"])
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        if "min" in param_dict_entry:
            min_expr = str(param_dict_entry["min"])
            min_eval = str(parameter_handler._evaluate_expression(min_expr, parameter_handler.params_dict))
            
            if min_eval != min_expr:
                self.minimum.value = min_expr + ": " + str(min_eval)
            else:
                self.minimum.value = min_expr
                
        else:
            self.minimum.value = "No minimum"
            
        if "max" in param_dict_entry:
            max_expr = str(param_dict_entry["max"])
            max_eval = parameter_handler._evaluate_expression(max_expr, parameter_handler.params_dict)
            
            if max_eval != max_expr:
                self.maximum.value = max_expr + ": " + str(max_eval)
            else:
                self.maximum.value = max_expr
        
        else:
            self.maximum.value = "No maximum"
                
        self.param_name.display()
        self.param_value.display()
        self.minimum.display()
        self.maximum.display()
        
class BoolParameterEditForm(npyscreen.FormBaseNew):
    def create(self):
        self._name = self.add(npyscreen.TitleText, name="Edit bool parameter", editable=False)
        
        self.param_name = self.add(npyscreen.TitleText, name="Name", editable=False, max_height=2)
        self.param_value = self.add(npyscreen.TitleSelectOne, name="Value", values=["True", "False"], max_height=4)
        
        self.save_button = self.add(npyscreen.ButtonPress, name="Save", when_pressed_function=self.save, max_height=2)
        self.cancel_button = self.add(npyscreen.ButtonPress, name="Cancel", when_pressed_function=self.cancel, max_height=2)

        self.param_dict_entry = None

    def save(self):
        param_value = True if self.param_value.value[0] == 0 else False
        
        parameter_handler: ParameterHandler = self.parentApp.client_node.parameter_handler
        
        try:
            parameter_handler.can_set_param(self.param_dict_entry["name"],param_value)
        except KeyError:
            npyscreen.notify("Parameter not found")
            time.sleep(2)
            return
        except TypeError:
            npyscreen.notify("Parameter value is not of the specified type")
            time.sleep(2)
            return
        except ValueError:
            npyscreen.notify("Parameter value is invalid")
            time.sleep(2)
            return
        
        success, message = self.parentApp.client_node.call_set_parameter_from_gc(self.param_dict_entry["name"], param_value)
        
        npyscreen.notify(message)
        
        time.sleep(2)
        
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")
        
    def cancel(self):
        self.parentApp.getForm("MAIN").update_table()
        self.parentApp.switchForm("MAIN")

    def update_param(self, param_dict_entry):
        self.param_dict_entry = param_dict_entry
        
        self.param_name.value = param_dict_entry["name"]
        self.param_value.value = [0 if param_dict_entry["value"] else 1]
        
        self.param_name.display()
        self.param_value.display()
        
###############################################################################
# Main
###############################################################################

def main(args=None):
    rclpy.init(args=args)
    
    node = ConfigurationClient()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    thread = Thread(target=executor.spin)
    thread.start()
    
    while not node.parameter_handler_updated and not node.parameter_files_updated:
        time.sleep(0.1)
        
    print("Parameter handler updated")
    
    try:
        app = ConfigurationClientApp(node)
        app.run()
        
    except KeyboardInterrupt:
        pass
    
    if rclpy.ok():
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()