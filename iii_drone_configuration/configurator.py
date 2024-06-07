###############################################################################
# Imports
###############################################################################

from .parameter_handler import ParameterHandler

from iii_drone_interfaces.srv import DeclareParameter
from iii_drone_interfaces.srv import UndeclareParameter

import rclpy
from rclpy import lifecycle

from rcl_interfaces.msg import ParameterEvent, SetParametersResult

from typing import Any, Callable, Dict, List, Optional
from threading import Lock
import yaml
import os

###############################################################################
# Class
###############################################################################

class Configurator:
    def __init__(
        self, 
        node: lifecycle.Node|rclpy.node.Node, 
        after_parameter_change_callback: Optional[Callable[[Parameter], None]] = None, 
        qos: QoSProfile = QoSProfile(depth=10)
    ):
        self.node = node
        
        self.configurator_node = rclpy.create_node(
            'configurator_node',
            self.node.get_namespace()
        )
        
        self.after_parameter_change_callback = after_parameter_change_callback
        self.qos = qos
        self.parameter_bundles = []
        self.parameters = []
        self.parameter_name_map = {}
        self.parameters_mutex = Lock()

        self.service_client_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        
        self.parameter_events_subscriber = self.node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            self.qos
        )
        
        self.declare_parameter_client = self.node.create_client(
            SetParameters, 
            '/configuration/configuration_server/declare_parameter',
            callback_group=self.service_client_cb_group
        )
        self.undeclare_parameter_client = self.node.create_client(
            SetParameters, 
            '/configuration/configuration_server/undeclare_parameter',
            callback_group=self.service_client_cb_group
        )
        
        self.get_parameters_client = self.node.create_client(
            GetParameters,
            '/configuration/configuration_server/get_parameters',
            callback_group=self.service_client_cb_group
        )
        
        if (not self.node.has_parameter("node_parameters_path_postfix")):
            self.node.declare_parameter("node_parameters_path_postfix", "node_parameters/")
            
        self.config_base_dir = os.getenv("CONFIG_BASE_DIR")
        
        if (self.config_base_dir is None):
            self.config_base_dir = os.path.join(os.path.expanduser("~"), ".config")
            
        self.parameter_yaml_path = os.path.join(
            self.config_base_dir,
            "iii_drone",
            str(self.node.get_parameter("node_parameters_path_postfix").value)
        )
        
        # Expand ~ in path
        self.parameter_yaml_path = os.path.expanduser(self.parameter_yaml_path)
        
        self.parameter_yaml_path = os.path.join(
            self.parameter_yaml_path,
            self.node.get_name() + ".yaml"
        )
        
        self.initialize()

    def get_parameter(self, simple_name: str) -> Parameter:
        with self.parameters_mutex:
            for param in self.parameters:
                if param.name == simple_name:
                    return param
            raise RuntimeError(f"Parameter {simple_name} does not exist.")

    def get_parameters(self, simple_names: List[str]) -> List[Parameter]:
        with self.parameters_mutex:
            return [param for param in self.parameters if param.name in simple_names]

    def sync_parameters(self, simple_names: List[str] = []):
        # This method would require interaction with a parameter server or similar mechanism in ROS 2
        pass

    @staticmethod
    def get_parameter_type_string(T: Any) -> str:
        return str(T)

    def print_parameters(self):
        with self.parameters_mutex:
            for param in self.parameters:
                self.node.get_logger().info(f"Parameter: {param.name} = {param.value}")

    def print_parameter_bundles(self):
        for bundle in self.parameter_bundles:
            self.node.get_logger().info(f"Parameter Bundle: {bundle.name}")

    def initialize(
        self,
        parameter_yaml_path: str
    ):
        pass
    
    def initialize_parameters(
        self,
        parameters: dict[str,str]
    ):
        pass
    
    def initialize_parameter_bundles(
        self,
        parameter_bundles: dict[str,dict[str,str]]
    ):
        pass
    
    def declare_parameter(
        self,
        parameter_full_name: str,
    ):
        pass
    
    def undeclare_parameter(
        self,
        parameter_full_name: str,
    ):
        pass
    
    def getParameterFullName(
        self,
        parameter_simple_name: str
    ) -> str:
        pass
    
    def get_parameter_simple_name(
        self,
        parameter_full_name: str
    ) -> str:
        pass

    def parameter_event_callback(self, parameter_event: ParameterEvent):
        # Handle parameter event
        pass

    def send_declare_parameter_request(self, name: str, type: str, message: str) -> bool:
        # This method would send a request to declare a parameter
        pass

    def send_undeclare_parameter_request(self, name: str, message: str) -> bool:
        # This method would send a request to undeclare a parameter
        pass

    def send_get_parameter_request(self, name: str, parameter: Parameter) -> bool:
        # This method would send a request to get a parameter
        pass

    def send_get_parameters_request(self, names: List[str], parameters: List[Parameter]) -> bool:
        # This method would send a request to get multiple parameters
        pass