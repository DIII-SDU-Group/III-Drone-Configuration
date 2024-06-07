###############################################################################
# Imports
###############################################################################

from .parameter_bundle import ParameterBundle, ParameterBundleEntry

from iii_drone_interfaces.srv import DeclareParameter
from iii_drone_interfaces.srv import UndeclareParameter

import rclpy
from rclpy import lifecycle
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

from rcl_interfaces.msg import ParameterEvent, SetParametersResult, ParameterValue
import rcl_interfaces
from rcl_interfaces.srv import GetParameters

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
            namespace=self.node.get_namespace()
        )
        
        self.after_parameter_change_callback = after_parameter_change_callback
        self.qos = qos
        self.parameter_bundles: List[ParameterBundle] = []
        self.parameters: List[Parameter] = []
        self.parameter_name_map = {}
        self.parameters_mutex = Lock()

        self.service_client_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        
        self.parameter_events_subscriber = self.node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            self.qos
        )
        
        self.declare_parameter_client = self.configurator_node.create_client(
            DeclareParameter, 
            '/configuration/configuration_server/declare_parameter',
            callback_group=self.service_client_cb_group
        )
        self.undeclare_parameter_client = self.configurator_node.create_client(
            UndeclareParameter, 
            '/configuration/configuration_server/undeclare_parameter',
            callback_group=self.service_client_cb_group
        )
        
        self.get_parameters_client = self.configurator_node.create_client(
            GetParameters,
            '/configuration/configuration_server/configuration_server/get_parameters',
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
        
        self.initialize(self.parameter_yaml_path)

    # Destructor
    def __del__(self):
        if rclpy.ok():
            self.node.get_logger().debug("Configurator.__del__")
        
        self.parameter_events_subscriber.destroy()
        
        if rclpy.ok():
            parameter_names = []
            
            with self.parameters_mutex:
                for param in self.parameters:
                    parameter_names.append(param.name)
                    
            for name in parameter_names:
                self.undeclare_parameter(name)
            
            self.node.get_logger().debug("Configurator.__del__: Parameters undeclared")

    def get_parameter(
        self, 
        simple_name: str
    ) -> Parameter:
        return self.get_parameters([simple_name])[0]

    def get_parameters(
        self, 
        simple_names: List[str]
    ) -> List[Parameter]:
        with self.parameters_mutex:
            parameters = []
            
            for simple_name in simple_names:
                full_name = self.getParameterFullName(simple_name)
                
                for param in self.parameters:
                    if (param.name == full_name):
                        parameters.append(param)
                        break
                    
            if len(parameters) != len(simple_names):
                fatal_msg = "Configurator.get_parameters: Some parameters were not found"
                
                self.node.get_logger().fatal(fatal_msg)
                
                raise RuntimeError(fatal_msg)
            
            return parameters
        
    def get_parameter_bundle(
        self,
        bundle_name: str
    ) -> ParameterBundle:
        
        for bundle in self.parameter_bundles:
            if (bundle.name == bundle_name):
                return bundle
            
        fatal_msg = f"Configurator.get_parameter_bundle: Parameter bundle '{bundle_name}' not found"
        
        self.node.get_logger().fatal(fatal_msg)
        
        raise RuntimeError(fatal_msg)

    def sync_parameters(
        self, 
        simple_names: List[str] = []
    ):
        with self.parameters_mutex:
            names_to_sync = []
            
            if (len(simple_names) == 0):
                for param in self.parameters:
                    param: Parameter
                    names_to_sync.append(param.name)
                    
            else:
                for simple_name in simple_names:
                    names_to_sync.append(self.getParameterFullName(simple_name))
                    
            success, parameters = self.send_get_parameters_request(names_to_sync)
            
            if not success:
                fatal_msg = "Configurator.sync_parameters: Failed to sync parameters"
                
                self.node.get_logger().fatal(fatal_msg)
                
                raise RuntimeError(fatal_msg)

            for param in parameters:
                simple_name = self.get_parameter_simple_name(param.name)
                
                for bundle in self.parameter_bundles:
                    if bundle.has_updatable_parameter(simple_name):
                        bundle.set_parameter(
                            simple_name, 
                            param
                        )
                        
                for i in range(len(self.parameters)):
                    if (self.parameters[i].name == param.name):
                        self.parameters[i] = param
                        break
            
    @staticmethod
    def get_parameter_type_string(T: Any) -> str:
        if T == bool:
            return "bool"
        
        if T == int:
            return "int"
        
        if T == float:
            return "float"
        
        if T == str:
            return "str"
        
        if T == List[bool]:
            return "bool_array"
        
        if T == List[int]:
            return "int_array"
        
        if T == List[float]:
            return "float_array"
        
        if T == List[str]:
            return "str_array"
        
        fatal_msg = f"Configurator.get_parameter_type_string: Unsupported type '{T}'"
        
        self.node.get_logger().fatal(fatal_msg)
        
        raise RuntimeError(fatal_msg)

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
        self.node.get_logger().debug(f"Configurator.initialize(): Declaring parameters from '{parameter_yaml_path}'")
        
        with open(parameter_yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            
        parameters = config.get("parameters")
        
        if parameters is None:
            fatal_msg = "Configurator.initialize: No parameters found in config file"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
            
        self.initialize_parameters(parameters)
        
        parameters_bundles = config.get("parameter_bundles")
        
        if parameters_bundles is not None:
            self.initialize_parameter_bundles(parameters_bundles)
    
    def initialize_parameters(
        self,
        parameters: dict[str,str]
    ):
        for simple_name, param in parameters.items():
            if simple_name in self.parameter_name_map:
                self.node.get_logger().warn(f"Configurator.initialize_parameters: Parameter '{simple_name}' already declared")
                
                continue
            
            name = param["name"]
            type_str = param["type"]
            
            self.declare_parameter(name, type_str)
            
            self.parameter_name_map[simple_name] = name
    
    def initialize_parameter_bundles(
        self,
        parameter_bundles: dict[str,dict[str,str]]
    ):
        for bundle_name, bundle in parameter_bundles.items():
            bundle_entries = []
            
            for simple_name, param in bundle.items():
                # data has "remap_name" and "update"
                
                remap_name = param["remap_name"]
                updatable = param["update"]
                
                bundle_entries.append(
                    ParameterBundleEntry(
                        simple_name=simple_name,
                        update=updatable,
                        remap_name=remap_name,
                        parameter=self.get_parameter(simple_name)
                    )
                )
                
            self.parameter_bundles.append(
                ParameterBundle(
                    name=bundle_name,
                    parameter_bundle_entries=bundle_entries
                )
            )
    
    def declare_parameter(
        self,
        parameter_full_name: str,
        type_str: str,
    ):
        self.node.get_logger().debug(f"Configurator.declare_parameter: Declaring parameter '{parameter_full_name}'")
        
        with self.parameters_mutex:
            for param in self.parameters:
                if (param.name == parameter_full_name):
                    self.node.get_logger().warn(f"Configurator.declare_parameter: Parameter '{parameter_full_name}' already declared")
                    
                    return
                
        success, message = self.send_declare_parameter_request(
            parameter_full_name,
            type_str
        )
        
        if not success:
            fatal_msg = f"Configurator.declare_parameter: Failed to declare parameter '{parameter_full_name}'"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        success, parameter = self.send_get_parameter_request(parameter_full_name)
        
        if not success:
            fatal_msg = f"Configurator.declare_parameter: Failed to get parameter '{parameter_full_name}'"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        with self.parameters_mutex:
            self.parameters.append(parameter)
    
    def undeclare_parameter(
        self,
        parameter_full_name: str,
    ):
        self.node.get_logger().debug(f"Configurator.undeclare_parameter: Undeclaring parameter '{parameter_full_name}'")
        
        success, message = self.send_undeclare_parameter_request(parameter_full_name)
        
        if not success:
            fatal_msg = f"Configurator.undeclare_parameter: Failed to undeclare parameter '{parameter_full_name}'"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        with self.parameters_mutex:
            for i in range(len(self.parameters)):
                if (self.parameters[i].name == parameter_full_name):
                    self.node.get_logger().debug(f"Configurator.undeclare_parameter: Removing parameter '{parameter_full_name}'")
                    del self.parameters[i]
                    break
                
        self.node.get_logger().debug(f"Configurator.undeclare_parameter: Parameter '{parameter_full_name}' undeclared")
    
    def getParameterFullName(
        self,
        parameter_simple_name: str
    ) -> str:

        full_name = self.parameter_name_map[parameter_simple_name]
        
        return full_name
    
    def get_parameter_simple_name(
        self,
        parameter_full_name: str
    ) -> str:
        for simple_name, full_name in self.parameter_name_map.items():
            if (full_name == parameter_full_name):
                return simple_name
            
        fatal_msg = f"Configurator.get_parameter_simple_name: Parameter '{parameter_full_name}' not found"
        
        self.node.get_logger().fatal(fatal_msg)
        
        raise RuntimeError(fatal_msg)

    def parameter_event_callback(self, parameter_event: ParameterEvent):
        # Handle parameter event
        with self.parameters_mutex:
            for param in parameter_event.changed_parameters:
                for i in range(len(self.parameters)):
                    if (self.parameters[i].name == param.name):
                        self.parameters[i] = param
                        simple_name = self.get_parameter_simple_name(param.name)
                        
                        for bundle in self.parameter_bundles:
                            if bundle.has_updatable_parameter(simple_name):
                                bundle.set_parameter(
                                    simple_name, 
                                    param
                                )
                                break
                            
                        if self.after_parameter_change_callback is not None:
                            self.after_parameter_change_callback(param)
                            
                        break

    def send_declare_parameter_request(
        self, 
        name: str, 
        type_str: str
    ) -> tuple[bool, str]:
        request = DeclareParameter.Request()
        
        request.name = name
        request.type = type_str
        request.node_name = self.node.get_name()

        if not self.declare_parameter_client.wait_for_service(timeout_sec=5.0):
            fatal_msg = "Configurator.send_declare_parameter_request: Service not available"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        if not self.declare_parameter_client.service_is_ready():
            fatal_msg = "Configurator.send_declare_parameter_request: Service not ready"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        future = self.declare_parameter_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.configurator_node, future)
        
        result: DeclareParameter.Response = future.result()

        if result is None:
            fatal_msg = "Configurator.send_declare_parameter_request: Service call failed"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        return result.succeeded, result.message
        
    def send_undeclare_parameter_request(
        self, 
        full_name: str
    ) -> tuple[bool, str]:
        request = UndeclareParameter.Request()
        
        request.name = full_name
        request.node_name = self.node.get_name()
        
        if not self.undeclare_parameter_client.wait_for_service(timeout_sec=5.0):
            fatal_msg = "Configurator.send_undeclare_parameter_request: Service not available"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        if not self.undeclare_parameter_client.service_is_ready():
            fatal_msg = "Configurator.send_undeclare_parameter_request: Service not ready"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        future = self.undeclare_parameter_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.configurator_node, future)
        
        result: UndeclareParameter.Response = future.result()
        
        if result is None:
            fatal_msg = "Configurator.send_undeclare_parameter_request: Service call failed"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        return result.succeeded, result.message
        

    def send_get_parameter_request(
        self, 
        full_name: str
    ) -> tuple[bool, Parameter]:
        
        names = [full_name]
        
        success, parameters = self.send_get_parameters_request(names)
        
        return success, parameters[0]

    def send_get_parameters_request(
        self, 
        names: List[str]
    ) -> tuple[bool, List[Parameter]]:

        request = GetParameters.Request()
        
        request.names = names

        if not self.get_parameters_client.wait_for_service(timeout_sec=5.0):
            fatal_msg = "Configurator.send_get_parameters_request: Service not available"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        if not self.get_parameters_client.service_is_ready():
            fatal_msg = "Configurator.send_get_parameters_request: Service not ready"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        future = self.get_parameters_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.configurator_node, future)
        
        result: GetParameters.Response = future.result()
        
        if result is None:
            fatal_msg = "Configurator.send_get_parameters_request: Service call failed"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        parameters = []
        
        for i, value in enumerate(result.values):
            value: ParameterValue
            
            param_msg = rcl_interfaces.msg.Parameter()
            param_msg.name = names[i]
            param_msg.value = value
            
            param = Parameter.from_parameter_msg(param_msg)
            
            parameters.append(
                param
            )
        
        return True, parameters