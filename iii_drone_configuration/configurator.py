
###############################################################################
# Imports
###############################################################################

import rcl_interfaces.msg
import rclpy.parameter
from .parameter_bundle import ParameterBundle, ParameterBundleEntry

from iii_drone_interfaces.srv import DeclareParameters
from iii_drone_interfaces.srv import UndeclareParameters

import rclpy
from rclpy import lifecycle
# from rclpy.parameter import Parameter, ParameterType
from rclpy.qos import QoSProfile

from rcl_interfaces.msg import ParameterEvent, SetParametersResult, ParameterValue, Parameter
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
        after_parameter_change_callback: Optional[Callable[[rclpy.parameter.Parameter], None]] = None, 
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
        self.parameters: List[rclpy.parameter.Parameter] = []
        self.parameter_name_map = {}
        self.parameters_mutex = Lock()

        self._initialized: bool = False

        self.parameter_events_subscriber = self.node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            self.qos
        )
        
        self.declare_parameters_client = self.configurator_node.create_client(
            DeclareParameters, 
            '/configuration/configuration_server/declare_parameters',
        )
        self.undeclare_parameters_client = self.configurator_node.create_client(
            UndeclareParameters, 
            '/configuration/configuration_server/undeclare_parameters',
        )
        
        self.get_parameters_client = self.configurator_node.create_client(
            GetParameters,
            '/configuration/configuration_server/configuration_server/get_parameters',
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

        self.is_cleaned_up = False

    # Destructor
    def cleanup(self):
        if self.is_cleaned_up:
            return
        
        if rclpy.ok():
            self.node.get_logger().debug("Configurator.cleanup")
        
        # self.parameter_events_subscriber.destroy()
        self.node.destroy_publisher(self.parameter_events_subscriber)
        self.parameter_events_subscriber = None
        
        self.parameter_bundles.clear()
        
        if rclpy.ok():
            self.undeclare_parameters()
            
            self.node.get_logger().debug("Configurator.cleanup: Parameters undeclared")

        self.configurator_node.destroy_client(self.declare_parameters_client)
        self.declare_parameters_client = None
        self.configurator_node.destroy_client(self.undeclare_parameters_client)
        self.undeclare_parameters_client = None
        self.configurator_node.destroy_client(self.get_parameters_client)
        self.get_parameters_client = None
        
        self.configurator_node.destroy_node()
        self.configurator_node = None
        
        self.node.get_logger().debug("Configurator.cleanup: Done")
        
        self.node = None

        self.is_cleaned_up = True

    def get_parameter(
        self, 
        simple_name: str
    ) -> rclpy.parameter.Parameter:
        return self.get_parameters([simple_name])[0]

    def get_parameters(
        self, 
        simple_names: List[str]
    ) -> List[rclpy.parameter.Parameter]:
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
    def get_parameter_type_string(T: rclpy.parameter.ParameterType) -> str:
        if T == rclpy.parameter.ParameterType.PARAMETER_BOOL:
            return "bool"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_INTEGER:
            return "int"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_DOUBLE:
            return "float"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_STRING:
            return "string"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_BOOL_ARRAY:
            return "bool_array"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_INTEGER_ARRAY:
            return "int_array"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_DOUBLE_ARRAY:
            return "float_array"
        
        if T == rclpy.parameter.ParameterType.PARAMETER_STRING_ARRAY:
            return "string_array"
        
        fatal_msg = f"Configurator.get_parameter_type_string: Unsupported type '{T}'"
        
        raise RuntimeError(fatal_msg)
    
    @staticmethod
    def get_parameter_type_from_string(parameter_type_string: str) -> rclpy.parameter.ParameterType:
        if parameter_type_string == "bool":
            return rclpy.parameter.ParameterType.PARAMETER_BOOL
        
        if parameter_type_string == "int":
            return rclpy.parameter.ParameterType.PARAMETER_INTEGER
        
        if parameter_type_string == "float":
            return rclpy.parameter.ParameterType.PARAMETER_DOUBLE
        
        if parameter_type_string == "string":
            return rclpy.parameter.ParameterType.PARAMETER_STRING
        
        if parameter_type_string == "bool_array":
            return rclpy.parameter.ParameterType.PARAMETER_BOOL_ARRAY
        
        if parameter_type_string == "int_array":
            return rclpy.parameter.ParameterType.PARAMETER_INTEGER_ARRAY
        
        if parameter_type_string == "float_array":
            return rclpy.parameter.ParameterType.PARAMETER_DOUBLE_ARRAY
        
        if parameter_type_string == "string_array":
            return rclpy.parameter.ParameterType.PARAMETER_STRING_ARRAY
        
        fatal_msg = f"Configurator.get_parameter_type_from_string: Unsupported type '{parameter_type_string}'"
        
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
        if self._initialized:
            fatal_msg = "Configurator.initialize: Already initialized"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
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

        self._initialized = True
    
    def initialize_parameters(
        self,
        parameters: dict[str,str]
    ):
        parameter_name_map_temp: Dict[str, str] = {}
        
        names: List[str] = []
        types: List[rclpy.parameter.ParameterType] = []
        
        for simple_name, param in parameters.items():
            name = param["name"]
            type_str = param["type"]
            
            param_type: rclpy.parameter.ParameterType = self.get_parameter_type_from_string(type_str)
            
            names.append(name)
            types.append(param_type)
            
            parameter_name_map_temp[simple_name] = name
            
        self.declare_parameters(
            names,
            types
        )
        
        self.parameter_name_map = parameter_name_map_temp
    
    def initialize_parameter_bundles(
        self,
        parameter_bundles: dict[str,dict[str,str]]
    ):
        for bundle_name, bundle in parameter_bundles.items():
            bundle_entries = []
            
            for simple_name, param in bundle.items():
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
    
    def declare_parameters(
        self,
        parameter_full_names: List[str],
        parameter_types: List[rclpy.parameter.ParameterType]
    ):
        self.node.get_logger().debug(f"Configurator.declare_parameter: Declaring parameters")
        
        types: List[str] = []
        
        for param_type in parameter_types:
            types.append(
                self.get_parameter_type_string(param_type)
            )
            
        success: bool = True
        parameters: List[rclpy.parameter.Parameter] = []
        message: str = ""
            
        success, parameters, message = self.send_declare_parameters_request(
            parameter_full_names,
            types
        )
        
        if not success:
            fatal_msg = f"Configurator.declare_parameter: Failed to declare parameters with error message: {message}"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        with self.parameters_mutex:
            for param in parameters:
                self.parameters.append(param)
    
    def undeclare_parameters(self) -> bool:
        self.node.get_logger().debug(f"Configurator.undeclare_parameters: Undeclaring parameters.")
        
        success, message = self.send_undeclare_parameters_request()
        
        if not success:
            warn_msg = f"Configurator.undeclare_parameters: Failed to undeclare parameters with server."
            
            self.node.get_logger().warn(warn_msg)
        
        with self.parameters_mutex:
            # for i in range(len(self.parameters)):
            #     if (self.parameters[i].name == parameter_full_name):
            #         self.node.get_logger().debug(f"Configurator.undeclare_parameters: Removing parameter '{parameter_full_name}'")
            #         del self.parameters[i]
            #         break
            self.parameters.clear()
                
        self.node.get_logger().debug(f"Configurator.undeclare_parameters: Parameters undeclared")
    
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

    def send_declare_parameters_request(
        self, 
        names: List[str],
        types: List[str]
    ) -> tuple[bool, List[rclpy.parameter.Parameter], str]:
        request = DeclareParameters.Request()
        
        request.names = names
        request.types = types
        request.node_name = self.node.get_name()

        if not self.declare_parameters_client.wait_for_service(timeout_sec=5.0):
            fatal_msg = "Configurator.send_declare_parameters_request: Service not available"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        if not self.declare_parameters_client.service_is_ready():
            fatal_msg = "Configurator.send_declare_parameters_request: Service not ready"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        future = self.declare_parameters_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.configurator_node, future)
        
        result: DeclareParameters.Response = future.result()

        if result is None:
            fatal_msg = "Configurator.send_declare_parameters_request: Service call failed"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)

        if len(result.values) != len(names) and result.succeeded:
            fatal_msg = "Configurator.send_declare_parameters_request: Number of received parameters does not match request"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        parameters: List[rclpy.parameter.Parameter] = []
        
        if result.succeeded:
            for i, param_msg in enumerate(result.values):
                param_msg = rcl_interfaces.msg.Parameter()
                param_msg.name = names[i]
                param_msg.value = result.values[i]
                
                param = rclpy.parameter.Parameter.from_parameter_msg(param_msg)
                
                parameters.append(param)
        
        return result.succeeded, parameters, result.message
        
    def send_undeclare_parameters_request(self) -> tuple[bool, str]:
        request = UndeclareParameters.Request()
        
        request.node_name = self.node.get_name()
        
        if not self.undeclare_parameters_client.wait_for_service(timeout_sec=5.0):
            warn_msg = "Configurator.send_undeclare_parameters_request: Service not available"
            
            self.node.get_logger().fatal(warn_msg)
            
            return False, warn_msg
        
        if not self.undeclare_parameters_client.service_is_ready():
            fatal_msg = "Configurator.send_undeclare_parameters_request: Service not ready"
            
            self.node.get_logger().fatal(fatal_msg)
            
            raise RuntimeError(fatal_msg)
        
        future = self.undeclare_parameters_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.configurator_node, future)
        
        result: UndeclareParameters.Response = future.result()
        
        if result is None:
            fatal_msg = "Configurator.send_undeclare_parameters_request: Service call failed"
            
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