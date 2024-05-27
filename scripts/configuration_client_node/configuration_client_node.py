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
from threading import Thread

from iii_drone_interfaces.srv import GetParameterYaml, SaveParameters, GetParameterFiles, LoadParameters, SetParameterFromGC, GetCurrentParameterFile

from iii_drone_configuration.parameter_handler import ParameterHandler

import npyscreen

###############################################################################
# Classes
###############################################################################

class ConfigurationClient(Node):
    def __init__(self):
        super().__init__(
            node_name="configuration_client",
            namespace="/configuration/configuration_client",
        )
        
        self.parameter_yaml_client = self.create_client(
            GetParameterYaml, 
            "/configuration/configuration_server/get_parameter_yaml"
        )
        
        self.parameter_handler = self.fetch_parameter_handler()
        
    def fetch_parameter_handler(self):
        self.get_logger().debug("Fetching parameter handler")
        
        # Call service to get parameter yaml
        request = GetParameterYaml.Request()
        
        while not self.parameter_yaml_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
            
        future = self.parameter_yaml_client.call_async(request)
        
        # Wait for service to be available
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().debug("Received parameter yaml")
            parameter_yaml = str(future.result().yaml)
            
            # Create parameter handler
            return ParameterHandler.from_raw_yaml_string(parameter_yaml)
        
        raise RuntimeError("Failed to fetch parameter handler")

    def get_all_parameter_entries(self):
        params_dict = self.parameter_handler.get_all_params()
        
        params = []
        
        for param_name, param_entry in params_dict.items():
            entry = {
                "name": param_name,
                "type": str(param_entry["type"]),
                "constant": str(param_entry["constant"] if "constant" in param_entry else False),
                "value": str(param_entry["value"]),
            }
            
            params.append(entry)
            
        return params

class ConfigurationClientApp(npyscreen.NPSAppManaged):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.client_node = node
    
    def onStart(self):
        self.addForm("MAIN", ConfigurationClientForm, name="Configuration Client", client_node=self.client_node)

class ConfigurationClientForm(npyscreen.FormBaseNew):
    def create(self):
        # Create a table layout:
        self.table = self.add(npyscreen.GridColTitles,
            col_titles=["Parameter Long Name", "Type", "Is Constant", "Value"],
            select_whole_line=True,
            editable=True,
            max_height=5
        )
        
        self.parentApp: ConfigurationClientApp
        
        # Populate the table with parameter entries
        # self.update_entries_timer = self.parentApp.client_node.create_timer(1.0, self.update_table)

        self.update_table()

    def update_table(self):
        # Get the latest parameter entries
        # parameter_entries = self.parentApp.client_node.get_all_parameter_entries()

        parameter_entries = [
            # Make a few test entries:
            {"name": "param1", "type": "int", "constant": "False", "value": "1"},
            {"name": "param2", "type": "float", "constant": "True", "value": "3.14"},
            {"name": "param3", "type": "string", "constant": "False", "value": "hello"},
        ]
        
        # Update the table values
        self.table.values = [[entry["name"], entry["type"], entry["constant"], entry["value"]] for entry in parameter_entries]
        
###############################################################################
# Main
###############################################################################

def main(args=None):
    rclpy.init(args=args)
    
    node = ConfigurationClient()

    # Create another thread to spin the node
    client_node_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    client_node_thread.start()

    app = ConfigurationClientApp(node)

    app.run()
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()