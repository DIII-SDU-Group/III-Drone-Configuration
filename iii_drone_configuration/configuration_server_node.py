#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

import rclpy
import rclpy.parameter
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.exceptions import ParameterNotDeclaredException
import rcl_interfaces.msg as rcl_msg

import os
import sys
import yaml
from datetime import datetime
from typing import Optional, Callable, List
import time
import threading
import gc

from iii_drone_interfaces.srv import DeclareParameters, UndeclareParameters, GetParameterYaml, GetDeclaredParameters, SaveParameters, GetParameterFiles, LoadParameters, SetParameterFromGC, GetCurrentParameterFile, SetCurrentParameterFileAsDefault

from iii_drone_configuration.parameter_handler import ParameterHandler

#########################################################################
# Debugging:
#########################################################################

SIMULATION = os.environ.get('SIMULATION', 'false').lower() == 'true'

if SIMULATION:
    import debugpy

###############################################################################
# Class
###############################################################################

class ConfigurationServer(Node):
    """
    Configuration server node, handles parameter declaration and parameter events, while loading configuration from file.
    """
    def __init__(
        self,
        node_name="configuration_server",
        namespace="/configuration/configuration_server"
    ):
        """
        Constructor, initializes node and loads configuration from file.
        
        Parameters:
            node_name (str): Node name.
            namespace (str): Node namespace.
        """

        super().__init__(
            node_name=node_name, 
            namespace=namespace
        )

        log_level = os.environ.get('CONFIGURATION_SERVER_LOG_LEVEL')
        
        if log_level is not None:
            log_level = log_level.upper()
            
            if log_level == 'DEBUG':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            elif log_level == 'INFO':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
            elif log_level == 'WARN':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
            elif log_level == 'ERROR':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
            elif log_level == 'FATAL':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        
        self.get_logger().info("ConfigurationServer.__init__(): Initializing node " + node_name + " in namespace " + namespace + ".")

        self.declare_parameter("parameters_path_postfix", "parameters")
        self.declare_parameter("default_parameter_file", "parameters_real.yaml")
        self.declare_parameter("sim_parameter_file", "parameters_sim.yaml")

        if not SIMULATION:
            self.params_file = str(self.get_parameter("default_parameter_file").value)
        else:
            self.params_file = str(self.get_parameter("sim_parameter_file").value)

        self.get_logger().info("ConfigurationServer.__init__(): Using parameter file: " + self.params_file + ".")

        self.iii_config_dir: Optional[str] = None

        self.params_dir: Optional[str] = None
        
        self.params_dir: Optional[str] = None
        
        self.params_file: Optional[str] = None
        
        self.ros_params_file: Optional[str] = None

        self.parameter_handler: Optional[ParameterHandler] = None

        self.declared_params: Optional[dict[str, str|int|float|bool|list[str|int|float|bool]]] = None
        self.declared_params_nodes: Optional[dict[str, list[str]]] = None
        self.parameters_initialized: Optional[dict[str, bool]] = None

        self.param_success: Optional[bool] = None

        self.declare_parameters_service: Optional[Service] = None
        
        self.undeclare_parameters_service: Optional[Service] = None
        
        self.get_parameter_yaml_service: Optional[Service] = None
        
        self.get_declared_parameters_service: Optional[Service] = None
        
        self.save_parameters_service: Optional[Service] = None
        
        self.get_parameter_files_service: Optional[Service] = None
        
        self.load_parameters_service: Optional[Service] = None
        
        self.set_parameter_from_gc_service: Optional[Service] = None
        
        self.get_current_parameter_file_service: Optional[Service] = None
        
        self.set_current_parameter_file_as_default_service: Optional[Service] = None
        
        self.set_parameter_event_callback_handler: Optional[Callable[[List[rclpy.parameter.Parameter]], rcl_msg.SetParametersResult]] = None

        #/parameter_events topic subscriber:
        self.is_active = False
        
        self.parameter_events_subscription = self.create_subscription(
            rcl_msg.ParameterEvent,
            "/parameter_events",
            self.parameter_events_callback,
            10
        )

        self.get_logger().info("ConfigurationServer.__init__(): Node " + node_name + " initialized successfully, ready for configuration.")

        self.get_logger().info("ConfigurationServer.__init__(): test ")

    def __del__(self):
        if rclpy.ok():
            self.get_logger().info("ConfigurationServer.__del__(): Deleting ConfigurationServer object.")
        else:
            # Print to console if rclpy is not ok
            print("ConfigurationServer.__del__(): Deleting ConfigurationServer object.")

        self.on_delete()

    def on_configure(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ConfigurationServer.on_configure()")
        
        ret = super().on_configure(state)

        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error("ConfigurationServer.on_configure(): Base class configuration failed.")
            return ret

        # Get user:
        self.iii_config_dir = os.path.join(os.getenv("CONFIG_BASE_DIR", default="~/.config"), "iii_drone")


        parameters_path_postfix = str(self.get_parameter("parameters_path_postfix").value)
        
        self.params_dir = os.path.join(self.iii_config_dir, parameters_path_postfix)

        
        # Replace "~" with "/home/<user>" in the path
        self.params_dir = self.params_dir.replace("~", os.getenv("HOME"))
        
        if not SIMULATION:
            self.params_file = str(self.get_parameter("default_parameter_file").value)
        else:
            self.params_file = str(self.get_parameter("sim_parameter_file").value)

        params_file = self.params_file
        
        if not self.validate_parameter_file_name(params_file):
            msg = "ConfigurationServer.on_configure(): Default parameter file name " + params_file + " is not valid."
            self.get_logger().error(msg)
            return TransitionCallbackReturn.FAILURE
        
        self.params_file = os.path.join(
            self.params_dir,
            params_file
        )
        
        self.ros_params_file = os.path.join(self.iii_config_dir, "ros_params.yaml")

        self.parameter_handler = ParameterHandler.from_parameter_file(self.params_file)

        # Declared params empty dict:
        self.declared_params = {}
        self.declared_params_nodes = {}
        self.parameters_initialized = {}

        self.param_success = True

        self.cnt = 1

        self.get_logger().info("ConfigurationServer.on_configure(): ConfigurationServer configured successfully.")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ConfigurationServer.on_cleanup()")

        ret = super().on_cleanup(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error("ConfigurationServer.on_cleanup(): Base class cleanup failed.")
            return ret
        
        self._cleanup()

        gc.collect()
        
        self.get_logger().info("ConfigurationServer.on_cleanup(): ConfigurationServer cleaned up successfully.")
        
        return TransitionCallbackReturn.SUCCESS
        
    def _cleanup(self):
        self.get_logger().debug("ConfigurationServer._cleanup(): Cleaning up ConfigurationServer object.")

        self.on_delete()

        if self.declared_params is not None:
            for key, value in self.declared_params.items():
                try:
                    self.undeclare_parameter(key)
                except ParameterNotDeclaredException as e:
                    pass
        
        self.iii_config_dir: Optional[str] = None

        self.params_dir: Optional[str] = None
        
        self.params_dir: Optional[str] = None
        
        self.params_file: Optional[str] = None
        
        self.ros_params_file: Optional[str] = None

        self.parameter_handler: Optional[ParameterHandler] = None

        self.declared_params: Optional[dict[str, str|int|float|bool|list[str|int|float|bool]]] = None
        self.declared_params_nodes: Optional[dict[str, list[str]]] = None
        self.parameters_initialized: Optional[dict[str, bool]] = None

        self.param_success: Optional[bool] = None
        
    def on_activate(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ConfigurationServer.on_activate()")

        ret = super().on_activate(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error("ConfigurationServer.on_activate(): Base class activation failed.")
            return ret
        
        # Initialize services:
        self.declare_parameters_service = self.create_service(
            DeclareParameters,
            "declare_parameters",
            self.declare_parameters_callback
        )
        
        self.undeclare_parameters_service = self.create_service(
            UndeclareParameters,
            "undeclare_parameters",
            self.undeclare_parameters_callback
        )
        
        self.get_parameter_yaml_service = self.create_service(
            GetParameterYaml,
            "get_parameter_yaml",
            self.get_parameter_yaml_callback
        )
        
        self.get_declared_parameters_service = self.create_service(
            GetDeclaredParameters,
            "get_declared_parameters",
            self.get_declared_parameters_callback
        )
        
        self.save_parameters_service = self.create_service(
            SaveParameters,
            "save_parameters",
            self.save_parameters_callback
        )
        
        self.get_parameter_files_service = self.create_service(
            GetParameterFiles,
            "get_parameter_files",
            self.get_parameter_files_callback
        )
        
        self.load_parameters_service = self.create_service(
            LoadParameters,
            "load_parameters",
            self.load_parameters_callback
        )
        
        self.set_parameter_from_gc_service = self.create_service(
            SetParameterFromGC,
            "set_parameter_from_gc",
            self.set_parameter_from_gc_callback
        )
        
        self.get_current_parameter_file_service = self.create_service(
            GetCurrentParameterFile,
            "get_current_parameter_file",
            self.get_current_parameter_file_callback
        )
        
        self.set_current_parameter_file_as_default_service = self.create_service(
            SetCurrentParameterFileAsDefault,
            "set_current_parameter_file_as_default",
            self.set_current_parameter_file_as_default_callback
        )
        
        # Service callback that gets called before a parameter is set:
        self.set_parameter_event_callback_handler = self.add_on_set_parameters_callback(self.set_parameter_event_callback)

        self.is_active = True

        self.get_logger().info("ConfigurationServer.on_activate(): ConfigurationServer activated successfully.")
        
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ConfigurationServer.on_deactivate()")
        
        ret = super().on_deactivate(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error("ConfigurationServer.on_deactivate(): Base class deactivation failed.")
            return ret
        
        self._deactivate()

        gc.collect()
        
        self.get_logger().info("ConfigurationServer.on_deactivate(): ConfigurationServer deactivated successfully.")
        
        return TransitionCallbackReturn.SUCCESS
        
    def _deactivate(self):
        self.get_logger().debug("ConfigurationServer._deactivate(): Deactivating ConfigurationServer object.")

        if self.declare_parameters_service is not None:
            self.declare_parameters_service.destroy()
            del self.declare_parameters_service
            self.declare_parameters_service = None

        if self.undeclare_parameters_service is not None:
            self.undeclare_parameters_service.destroy()
            del self.undeclare_parameters_service
            self.undeclare_parameters_service = None

        if self.get_parameter_yaml_service is not None:
            self.get_parameter_yaml_service.destroy()
            del self.get_parameter_yaml_service
            self.get_parameter_yaml_service = None

        if self.get_declared_parameters_service is not None:
            self.get_declared_parameters_service.destroy()
            del self.get_declared_parameters_service
            self.get_declared_parameters_service = None

        if self.save_parameters_service is not None:
            self.save_parameters_service.destroy()
            del self.save_parameters_service
            self.save_parameters_service = None

        if self.get_parameter_files_service is not None:
            self.get_parameter_files_service.destroy()
            del self.get_parameter_files_service
            self.get_parameter_files_service = None

        if self.load_parameters_service is not None:
            self.load_parameters_service.destroy()
            del self.load_parameters_service
            self.load_parameters_service = None

        if self.set_parameter_from_gc_service is not None:
            self.set_parameter_from_gc_service.destroy()
            del self.set_parameter_from_gc_service
            self.set_parameter_from_gc_service = None

        if self.get_current_parameter_file_service is not None:
            self.get_current_parameter_file_service.destroy()
            del self.get_current_parameter_file_service
            self.get_current_parameter_file_service = None

        if self.set_current_parameter_file_as_default_service is not None:
            self.set_current_parameter_file_as_default_service.destroy()
            del self.set_current_parameter_file_as_default_service
            self.set_current_parameter_file_as_default_service = None

        if self.set_parameter_event_callback_handler is not None:
            self.remove_on_set_parameters_callback(self.set_parameter_event_callback_handler)
            del self.set_parameter_event_callback_handler
            self.set_parameter_event_callback_handler = None

        self.is_active = False
        
    def on_shutdown(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ConfigurationServer.on_shutdown()")
        
        ret = super().on_shutdown(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error("ConfigurationServer.on_shutdown(): Base class shutdown failed.")
            return ret
        
        self._deactivate()
        self._cleanup()
        
        self.get_logger().info("ConfigurationServer.on_shutdown(): ConfigurationServer shut down successfully.")

        def shutdown_rclpy():
            time.sleep(1)
            rclpy.shutdown()

        thread = threading.Thread(target=shutdown_rclpy)
        thread.start()
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ConfigurationServer.on_error()")
        
        ret = super().on_error(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error("ConfigurationServer.on_error(): Base class error failed.")
            return ret
        
        self._deactivate()
        self._cleanup()
        
    def on_delete(self):
        if rclpy.ok():
            self.get_logger().info("ConfigurationServer.on_delete(): Deleting ConfigurationServer object.")
        else:
            # Print to console if rclpy is not ok
            print("ConfigurationServer.on_delete(): Deleting ConfigurationServer object.")

        if self.parameter_handler is None:
            return
        
        if self.parameter_handler.any_params_changed:
            if rclpy.ok():
                self.get_logger().info("ConfigurationServer.on_delete(): Parameters changed, saving...")
            else:
                # Print to console if rclpy is not ok
                print("ConfigurationServer.on_delete(): Parameters changed, saving...")

            request = SaveParameters.Request()
            request.file = ""
            request.overwrite = True
            request.set_as_default = True

            response = self.save_parameters_callback(request, SaveParameters.Response())
            
            if rclpy.ok():
                if not response.success:
                    self.get_logger().fatal("ConfigurationServer.on_delete(): Failed to save parameters: " + response.message + ".")
                else:
                    self.get_logger().info("ConfigurationServer.on_delete(): Parameters saved successfully to file " + response.file + ".")
            else:
                # Print to console if rclpy is not ok
                if not response.success:
                    print("ConfigurationServer.on_delete(): Failed to save parameters: " + response.message + ".")
                else:
                    print("ConfigurationServer.on_delete(): Parameters saved successfully to file " + response.file + ".")
        
    def set_parameter_event_callback(
        self,
        parameters: "list[rcl_msg.Parameter]"
    ) -> rcl_msg.SetParametersResult:
        """
        Callback for parameter events, gets called before a parameter is set. 
        Rejects if the parameter is constant, if the type or value do not match the loaded parameters,
        if the parameter is not listed in the loaded parameters, or if the validation function returns false.

        Parameters:
            parameters (list[Parameter]): List of parameters to be set.

        Returns:
            SetParametersResult: Result of the callback.
        """

        self.get_logger().debug("ConfigurationServer.set_parameter_event_callback()")

        result = rcl_msg.SetParametersResult()

        for parameter in parameters:
            if parameter.name in self.declared_params:
                try:
                    self.parameter_handler.can_set_param(
                        parameter.name,
                        parameter.value,
                        self.parameters_initialized[parameter.name]
                    )
                    
                except KeyError as e:
                    return_string = "Parameter " + parameter.name + " not listed in parameters file, rejecting."
                    self.get_logger().fatal("ConfigurationServer.set_parameter_event_callback(): " + return_string + ": " + str(e))
                    result.successful = False
                    result.reason = return_string

                    return result

                except AttributeError as e:
                    return_string = "Parameter " + parameter.name + " is constant, rejecting."
                    self.get_logger().error("ConfigurationServer.set_parameter_event_callback(): " + return_string + ": " + str(e))
                    result.successful = False
                    result.reason = return_string

                    return result

                except TypeError as e:
                    return_string = "Parameter " + parameter.name + " has different type, rejecting."
                    self.get_logger().error("ConfigurationServer.set_parameter_event_callback(): " + return_string + ": " + str(e))
                    result.successful = False
                    result.reason = return_string

                    return result
                
                except ValueError as e:
                    return_string = "Parameter " + parameter.name + " failed validation, rejecting."
                    self.get_logger().error("ConfigurationServer.set_parameter_event_callback(): " + return_string + ": " + str(e))
                    result.successful = False
                    result.reason = return_string

                    return result

        self.get_logger().info("ConfigurationServer.set_parameter_event_callback(): Request to set parameters accepted.")

        result.successful = True

        return result
    
    def parameter_events_callback(
        self,
        parameter_event: rcl_msg.ParameterEvent
    ) -> None:
        """
        Callback for parameter events, gets called after a parameter is set.
        Updates the declared parameters dict with the new parameter values
        and updates the parameter_handler object.
        
        Parameters:
            parameter_event (ParameterEvent): Parameter event.

        Raises:
            RuntimeError: If parameters are deleted, since this is not supported.
            RuntimeError: If the param can not be set for whatever reason, since this should not happen as it is verified in the pre-set parameter callback.
        """

        self.get_logger().debug("ConfigurationServer.parameter_events_callback()")

        if not self.is_active:
            return

        if parameter_event.node != self.get_fully_qualified_name():
            return
        

        if len(parameter_event.deleted_parameters) > 0:
            self.get_logger().fatal("ConfigurationServer.parameter_events_callback(): Deleted parameters not supported.")
            raise RuntimeError("Deleted parameters not supported.")
        
        for parameter in parameter_event.changed_parameters + parameter_event.new_parameters:
            if parameter.name == self.params_file or "use_sim_time" in parameter.name:
                continue
            try:
                param_value = self._get_parameter_value_from_msg(
                    parameter.name, 
                    parameter.value
                )

                self.parameter_handler.set_param(
                    parameter.name, 
                    param_value,
                    self.parameters_initialized[parameter.name]
                )

                self.declared_params[parameter.name] = param_value
                self.parameters_initialized[parameter.name] = True

                self.get_logger().info("ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " set to " + str(param_value) + ".")

            except KeyError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " not listed in parameters file, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e

            except AttributeError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " is constant, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e
            
            except TypeError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " has different type, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e
            
            except ValueError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " failed validation, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e


    def _get_parameter_value_from_msg(
        self,
        parameter_name: str,
        parameter_value: rcl_msg.ParameterValue
    ) -> str|int|float|bool|list[str|int|float|bool]:
        """
        Gets the parameter value from the ParameterValue object 
        based on the type.

        Parameters:
            parameter_name (str): Parameter name.
            parameter_value (ParameterValue): Parameter value.

        Returns:
            str|int|float|bool|list[str|int|float|bool]: Parameter value.

        Raises:
            KeyError: If the parameter name is not listed in the loaded parameters.
            TypeError: If the parameter type is not recognized.
        """

        self.get_logger().debug("ConfigurationServer._get_parameter_value_from_msg()")

        param_dict = self.parameter_handler.get_param(parameter_name)

        if param_dict["type"] == "string":
            return parameter_value.string_value
        
        elif param_dict["type"] == "int":
            return parameter_value.integer_value
        
        elif param_dict["type"] == "float":
            return parameter_value.double_value
        
        elif param_dict["type"] == "bool":
            return parameter_value.bool_value
        
        elif param_dict["type"] == "string_array":
            return parameter_value.string_array_value
        
        elif param_dict["type"] == "int_array" or param_dict["type"] == "integer_array":
            return parameter_value.integer_array_value
        
        elif param_dict["type"] == "float_array":
            return parameter_value.double_array_value
        
        elif param_dict["type"] == "bool_array":
            return parameter_value.bool_array_value
        
        else:
            raise TypeError("Type " + str(param_dict["type"]) + " not recognized.")

    def declare_parameters_callback(
        self,
        request: DeclareParameters.Request,
        response: DeclareParameters.Response
    ) -> DeclareParameters.Response:
        """
        Callback for declare_parameters service. Checks that the parameters has been loaded,
        and that the types matches the loaded types.

        Parameters:
            request (DeclareParameters.Request): Service request.
            response (DeclareParameters.Response): Service response.

        Returns:
            DeclareParameters.Response: Service response.
        """

        self.get_logger().debug("ConfigurationServer.declare_parameters_callback()")

        if len(request.names) == 0:
            response.succeeded = False
            response.message = "No parameters to declare."

            return response
        
        if len(request.names) != len(request.types):
            response.succeeded = False
            response.message = "Number of names and types do not match."

            return response
        
        param_dicts = {}

        try:
            for name in request.names:
                param_dicts[name] = self.parameter_handler.get_param(name)
        
        except KeyError as e:
            response.succeeded = False
            response.message = "Parameter not listed in parameters file: " + str(e)

            return response

        if request.node_name == "":
            response.succeeded = False
            response.message = "Node name is empty."

            self.get_logger().error("ConfigurationServer.declare_parameters_callback(): " + response.message)

            return response

        failed_parameters = []
        already_declared_parameters = []

        for i in range(len(request.names)):
            name = request.names[i]
            param_type = request.types[i]
            
            param_dict = param_dicts[name]

            if name in self.declared_params:
                already_declared_parameters.append(name)
                continue

            if param_type != param_dict["type"]:
                failed_parameters.append(name)
                continue

        if len(failed_parameters) > 0:
            response.succeeded = False
            response.message = "Failed to declare parameters. Types do not match loaded parameters. Conflicting parameters: " + str(failed_parameters) + "."
            
            return response

        values: List[rcl_msg.ParameterValue] = []

        def get_parameter_value_msg(
            _type: str,
            value: str|int|float|bool|list[str|int|float|bool]
        ) -> rcl_msg.ParameterValue:
            parameter_value = rcl_msg.ParameterValue()
            parameter_value.type = self._string_to_parameter_type(_type)
            parameter_value = self._set_parameter_value(
                parameter_value,
                value
            )
            
            return parameter_value

        for name in request.names:
            param_dict = param_dicts[name]
            value = param_dict["value"]
            
            if name in already_declared_parameters:
                self.declared_params_nodes[name].append(request.node_name)
                
                parameter_value = get_parameter_value_msg(
                    param_dict["type"], 
                    value
                )
                values.append(parameter_value)
                
                continue
            
            self.declare_parameter(
                name,
                value
            )
            
            parameter_value = get_parameter_value_msg(
                param_dict["type"], 
                value
            )
            values.append(parameter_value)

            self.declared_params[name] = value
            self.declared_params_nodes[name] = [request.node_name]
            self.parameters_initialized[name] = False

        response.succeeded = True
        response.message = "Parameter declared successfully."
        response.values = values
        
        if len(already_declared_parameters) > 0:
            response.message += " Already declared parameters: " + str(already_declared_parameters) + "."

        return response

    def undeclare_parameters_callback(
        self,
        request: UndeclareParameters.Request,
        response: UndeclareParameters.Response
    ) -> UndeclareParameters.Response:
        """
        Callback for undeclare_parameters service. Removes the node from list of nodes that declared the parameter for all parameters.
        If the node name is the last one, undeclares the parameter.

        Parameters:
            request (UndeclareParameters.Request): Service request.
            response (UndeclareParameters.Response): Service response.

        Returns:
            UndeclareParameters.Response: Service response.
        """

        self.get_logger().debug("ConfigurationServer.undeclare_parameters_callback()")

        if self.declared_params is None:
            response.succeeded = False
            response.message = "Parameters not declared."

            return response

        has_declared_parameter = False

        self.get_logger().info("ConfigurationServer.undeclare_parameters_callback(): Undeclaring parameters for node " + request.node_name + ".")

        declared_params_nodes_copy = self.declared_params_nodes.copy()

        for name, nodes in declared_params_nodes_copy.items():
            if request.node_name in nodes:
                has_declared_parameter = True
                
                self.declared_params_nodes[name].remove(request.node_name)
                
                if self.declared_params_nodes[name] == []:
                    self.undeclare_parameter(name)

                    self.declared_params.pop(name)
                    self.declared_params_nodes.pop(name)

        if not has_declared_parameter:
            response.succeeded = False
            response.message = "Node has not declared any parameters."

            return response

        response.succeeded = True
        response.message = "Parameters fully undeclared for node."

        return response
    
    def get_parameter_yaml_callback(
        self,
        request: GetParameterYaml.Request,
        response: GetParameterYaml.Response
    ) -> GetParameterYaml.Response:
        """
        Callback for get_parameter_yaml service. Returns the loaded parameters as a yaml string.

        Parameters:
            request (GetParameterYaml.Request): Service request.
            response (GetParameterYaml.Response): Service response.

        Returns:
            GetParameterYaml.Response: Service response.
        """

        self.get_logger().debug("ConfigurationServer.get_parameter_yaml_callback()")

        response.yaml = self.parameter_handler.get_parameters_yaml_string()

        return response
    
    def get_declared_parameters_callback(
        self,
        request: GetDeclaredParameters.Request,
        response: GetDeclaredParameters.Response
    ) -> GetDeclaredParameters.Response:
        """
        Callback for get_declared_parameters service. Returns the declared parameters as a yaml string.

        Parameters:
            request (GetDeclaredParameters.Request): Service request.
            response (GetDeclaredParameters.Response): Service response.

        Returns:
            GetDeclaredParameters.Response: Service response.
        """

        self.get_logger().debug("ConfigurationServer.get_declared_parameters_callback()")

        response.declared_parameters_yaml = yaml.dump(self.declared_params)

        return response
    
    def save_parameters_callback(
        self,
        request: SaveParameters.Request,
        response: SaveParameters.Response
    ) -> SaveParameters.Response:
        """
        Callback for save_parameters service. Saves the parameters to a yaml file.

        Parameters:
            request (SaveParameters.Request): Service request.
            response (SaveParameters.Response): Service response.

        Returns:
            SaveParameters.Response: Service response.
        """

        self.get_logger().debug("ConfigurationServer.save_parameters_callback()")
        
        if request.file == "":
            # Get the current date and time
            now = datetime.now()

            # Format the date and time as a string in the format 'YYYYMMDD_HHMM'
            date_time_string = now.strftime('%Y%m%d_%H%M')
            
            if not SIMULATION:
                request.file = "parameters_real_" + date_time_string + ".yaml"
            else:
                request.file = "parameters_sim_" + date_time_string + ".yaml"

        self.get_logger().info("ConfigurationServer.save_parameters_callback(): Request to save parameters to file " + request.file + ".")
        
        if not self.validate_parameter_file_name(request.file):
            response.success = False
            response.message = "Invalid file name."

            return response
        
        file_name = os.path.join(
            self.params_dir,
            request.file
        )

        try:
            self.parameter_handler.save_parameters(
                file_name,
                request.overwrite
            )
            
            if request.set_as_default:
                self.get_logger().info("ConfigurationServer.save_parameters_callback(): Setting default parameter file to " + request.file + ".")
                self.set_default_parameter_file(request.file)

            self.params_file = os.path.join(
                self.params_dir,
                request.file
            )
            
            response.success = True
            response.message = "Parameters saved successfully."
            response.file = file_name
            
        except FileExistsError as e:
            self.get_logger().error("ConfigurationServer.save_parameters_callback(): File already exists: " + str(e))
            response.success = False
            response.message = "File already exists and overwrite is false."

        return response
    
    def get_parameter_files_callback(
        self,
        request: GetParameterFiles.Request,
        response: GetParameterFiles.Response
    ) -> GetParameterFiles.Response:
        """
        Callback for get_parameter_files service. Returns a list of all parameter files in the parameters directory.

        Parameters:
            request (GetParameterFiles.Request): Service request.
            response (GetParameterFiles.Response): Service response.

        Returns:
            GetParameterFiles.Response: Service response.
        """

        self.get_logger().debug("ConfigurationServer.get_parameter_files_callback()")

        parameter_files = os.listdir(self.params_dir)

        response.parameter_files = []
        
        for parameter_file in parameter_files:
            if self.validate_parameter_file_name(parameter_file):
                response.parameter_files.append(parameter_file)

        return response
    
    def load_parameters_callback(
        self,
        request: LoadParameters.Request,
        response: LoadParameters.Response
    ) -> LoadParameters.Response:
        """
        Callback for load_parameters service. Loads the parameters from a yaml file.

        Parameters:
            request (LoadParameters.Request): Service request.
            response (LoadParameters.Response): Service response.

        Returns:
            LoadParameters.Response: Service response.
        """

        self.get_logger().info("ConfigurationServer.load_parameters_callback(): Request to load parameters from file " + request.file + ".")

        if self.parameter_handler.any_params_changed:
            self.get_logger().info("ConfigurationServer.load_parameters_callback(): Parameters changed, saving...")

            tmp_request = SaveParameters.Request()
            tmp_request.file = ""
            tmp_request.overwrite = True
            tmp_request.set_as_default = True

            tmp_response = self.save_parameters_callback(tmp_request, SaveParameters.Response())
            
            if not response.success:
                self.get_logger().fatal("ConfigurationServer.load_parameters_callback(): Failed to save parameters: " + tmp_response.message + ".")
            else:
                self.get_logger().info("ConfigurationServer.load_parameters_callback(): Parameters saved successfully to file " + tmp_response.file + ".")
        
        if not self.validate_parameter_file_name(request.file):
            response.success = False
            response.message = "Invalid file name."

            return response
        
        file_name = os.path.join(
            self.params_dir,
            request.file
        )

        try:
            changed_parameter_names = self.parameter_handler.load_new_parameter_file(
                file_name,
                declared_parameters=list(self.declared_params.keys())
            )
            
            if request.set_as_default:
                self.set_default_parameter_file(request.file)
                
            if changed_parameter_names != []:
                changed_ros_parameters = [
                    rclpy.parameter.Parameter(
                        name=param_name,
                        value=self.parameter_handler.get_param_value(param_name)
                    ) for param_name in changed_parameter_names
                ]
                
                set_parameter_results = self.set_parameters(changed_ros_parameters)
                
                for result in set_parameter_results:
                    if not result.successful:
                        response.success = False
                        response.message = "Failed to set parameter: " + result.reason + "."
                        
                        return response
            
            self.params_file = request.file
            
            # Schedule reset changed parameters:
            self.parameter_handler.reset_changed_parameters(changed_parameter_names=changed_parameter_names)
            
            response.success = True
            response.message = "Parameters loaded successfully."
            
        except FileNotFoundError as e:
            response.success = False
            response.message = "File not found."
            
        except ValueError as e:
            response.success = False
            response.message = "Invalid parameter."
            
        except TypeError as e:
            response.success = False
            response.message = "Invalid parameter type."
            
        except Exception as e:
            response.success = False
            response.message = "Unknown error: " + str(e)

        return response
    
    def set_parameter_from_gc_callback(
        self,
        request: SetParameterFromGC.Request,
        response: SetParameterFromGC.Response
    ) -> SetParameterFromGC.Response:
        self.get_logger().debug("ConfigurationServer.set_parameter_from_gc_callback()")
        
        parameter_name = request.parameter_name

        try:
            parameter_dict = self.parameter_handler.get_param(parameter_name)
        except KeyError as e:
            response.success = False
            response.message = "Parameter not listed in parameters file."
            
            return response
        
        if not (parameter_dict["type"] == "bool" or parameter_dict["type"] == "int" or parameter_dict["type"] == "float" or parameter_dict["type"] == "string"):
            response.success = False
            response.message = "Parameter type " + parameter_dict["type"] + " not supported."
            
            return response
        
        parameter_value = request.parameter_string_value
        
        if parameter_dict["type"] == "bool":
            
            if not str(parameter_value).lower() in ["true", "false"]:
                response.success = False
                response.message = "Invalid bool value."
                
                return response
            
            parameter_value = str(parameter_value).lower() == "true"
            
        elif parameter_dict["type"] == "int":
            try:
                parameter_value = int(parameter_value)
            except ValueError as e:
                response.success = False
                response.message = "Invalid int value."
                
                return response
            
        elif parameter_dict["type"] == "float":
            try:
                parameter_value = float(parameter_value)
            except ValueError as e:
                response.success = False
                response.message = "Invalid float value."
                
                return response
            
        elif parameter_dict["type"] == "string":
            try:
                parameter_value = str(parameter_value)
            except ValueError as e:
                response.success = False
                response.message = "Invalid string value."
                
                return response
            
        try:
            if "constant" in parameter_dict and parameter_dict["constant"]:
                response.message = "Parameter " + parameter_name + " successfully set to " + str(parameter_value) + ", but change will only take place after restarting system."
            else:
                set_result: "list[rcl_msg.SetParametersResult]" = self.set_parameters([
                    rclpy.parameter.Parameter(
                        name=parameter_name,
                        value=parameter_value
                    )
                ])

                if not set_result[0].successful:
                    response.success = False
                    response.message = "Failed to set parameter: " + set_result[0].reason + "."
                    
                    return response

                response.message = "Parameter " + parameter_name + " successfully set to " + str(parameter_value) + "."
                
            self.parameter_handler.set_param(
                parameter_name,
                parameter_value,
                True,
                force_constant=True
            )
            
            response.success = True
            
            return response
                
        except Exception as e:
            response.success = False
            response.message = "Unknown error: " + str(e)
            
            return response
        
    def get_current_parameter_file_callback(
        self,
        request: GetCurrentParameterFile.Request,
        response: GetCurrentParameterFile.Response
    ) -> GetCurrentParameterFile.Response:
        self.get_logger().debug("ConfigurationServer.get_current_parameter_file_callback()")
        response.default_parameter_file = self.get_parameter("default_parameter_file").value
        response.current_parameter_file = os.path.basename(self.params_file)
        
        return response
    
    def set_current_parameter_file_as_default_callback(
        self,
        request: SetCurrentParameterFileAsDefault.Request,
        response: SetCurrentParameterFileAsDefault.Response
    ) -> SetCurrentParameterFileAsDefault.Response:
        self.get_logger().debug("ConfigurationServer.set_current_parameter_file_as_default_callback()")
        try:
            self.set_default_parameter_file(os.path.basename(self.params_file))
            
            response.success = True
            response.message = "Default parameter file set to " + os.path.basename(self.params_file) + "."
            
        except Exception as e:
            response.success = False
            response.message = "Failed to set default parameter file: " + str(e) + "."
            
        return response
    
    def validate_parameter_file_name(
        self,
        file_name: str
    ) -> bool:
        """
        Validates the parameter file name, checks that it is a yaml file.

        Parameters:
            file_name (str): File name.

        Returns:
            bool: True if the file name is valid, False otherwise.
        """

        self.get_logger().debug("ConfigurationServer.validate_parameter_file_name()")

        if file_name[-5:] != ".yaml":
            return False
        
        if file_name[0] == ".":
            return False
        
        if "/" in file_name:
            return False
        
        if "\\" in file_name:
            return False
        
        if file_name == "":
            return False
        
        return True
    
    def set_default_parameter_file(
        self,
        file: str
    ) -> None:
        """
        Sets the default parameter file.

        Parameters:
            file (str): File name.
        """

        self.get_logger().debug("ConfigurationServer.set_default_parameter_file()")
        
        ros_params_lines = []

        with open(self.ros_params_file, "r") as ros_params_file:
            ros_params_lines = ros_params_file.readlines()
            
        for i in range(len(ros_params_lines)):
            if not SIMULATION and "default_parameter_file" in ros_params_lines[i] or SIMULATION and "sim_parameter_file" in ros_params_lines[i]:
                splitted_line = ros_params_lines[i].split(":")
                splitted_line[1] = " " + f'"{file}"' + "\n"
                ros_params_lines[i] = ":".join(splitted_line)
                
                break
            
        with open(self.ros_params_file, "w") as ros_params_file:
            ros_params_file.writelines(ros_params_lines)
            
        self.set_parameters([
            rclpy.parameter.Parameter(
                name="default_parameter_file" if not SIMULATION else "sim_parameter_file",
                value=file
            )
        ])

    def _string_to_parameter_type(
        self,
        param_type: str
    ) -> rcl_msg.ParameterType:
        """
        Converts a string to a ParameterType.
        
        Parameters:
            param_type (str): Parameter type as a string.
            
        Returns:
            ParameterType: Parameter type.
            
        Raises:
            ValueError: If the parameter type is not recognized.
            
        """
        
        if param_type == "string":
            return rcl_msg.ParameterType.PARAMETER_STRING
        
        elif param_type == "int":
            return rcl_msg.ParameterType.PARAMETER_INTEGER
        
        elif param_type == "float":
            return rcl_msg.ParameterType.PARAMETER_DOUBLE
        
        elif param_type == "bool":
            return rcl_msg.ParameterType.PARAMETER_BOOL
        
        elif param_type == "string_array":
            return rcl_msg.ParameterType.PARAMETER_STRING_ARRAY
        
        elif param_type == "int_array":
            return rcl_msg.ParameterType.PARAMETER_INTEGER_ARRAY
        
        elif param_type == "float_array":
            return rcl_msg.ParameterType.PARAMETER_DOUBLE_ARRAY
        
        elif param_type == "bool_array":
            return rcl_msg.ParameterType.PARAMETER_BOOL_ARRAY
        
        else:
            raise ValueError("Type " + param_type + " not recognized.")

    def _set_parameter_value(
        self,
        parameter_value: rcl_msg.ParameterValue,
        value: str|int|float|bool|list[str|int|float|bool]
    ) -> rcl_msg.ParameterValue:
        """
        Sets the parameter value based on the type.

        Parameters:
            parameter_value (ParameterValue): Parameter value.
            value (str|int|float|bool|list[str|int|float|bool]): Value to set.

        Returns:
            ParameterValue: Parameter value.
            
        Raises:
            ValueError: If the parameter type is not recognized
        """

        if parameter_value.type == rcl_msg.ParameterType.PARAMETER_STRING:
            parameter_value.string_value = str(value)
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_INTEGER:
            parameter_value.integer_value = int(value)
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_DOUBLE:
            parameter_value.double_value = float(value)
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_BOOL:
            parameter_value.bool_value = bool(value)
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_STRING_ARRAY:
            parameter_value.string_array_value = value
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_INTEGER_ARRAY:
            parameter_value.integer_array_value = value
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_DOUBLE_ARRAY:
            parameter_value.double_array_value = value
            
        elif parameter_value.type == rcl_msg.ParameterType.PARAMETER_BOOL_ARRAY:
            parameter_value.bool_array_value = value
        
        else:
            raise ValueError("Type " + str(parameter_value.type) + " not recognized.")

        return parameter_value

###############################################################################
# Main
###############################################################################

def main():
    if SIMULATION:
        DEBUG_PORT = int(os.environ.get('CONFIGURATION_SERVER_DEBUG_PORT', 0))
        
        if DEBUG_PORT > 0:
            debugpy.listen(
                (
                    'localhost',
                    DEBUG_PORT
                )
            )
            
            print("Listening for debugger on port " + str(DEBUG_PORT))

    rclpy.init(args=sys.argv)

    print("Starting ConfigurationServer node...")
    node = ConfigurationServer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, rclpy.exceptions.ROSInterruptException):
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()