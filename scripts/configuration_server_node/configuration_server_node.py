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

from iii_drone_interfaces.srv import DeclareParameter, GetParameterYaml, GetDeclaredParameters, SaveParameters, GetParameterFiles, LoadParameters, SetParameterFromGC, GetCurrentParameterFile

from iii_drone_core.configuration.parameter_handler import ParameterHandler

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
        
        self.get_logger().info("ConfigurationServer.__init__(): Initializing node " + node_name + " in namespace " + namespace + ".")

        # Get user:
        self.user = os.environ["USER"]

        self.declare_parameter("parameters_dir", "/home/" + self.user + "/.config/iii_drone/parameters/")
        self.params_dir = str(self.get_parameter("parameters_dir").value)
        
        # Replace "~" with "/home/<user>" in the path
        if self.params_dir[0] == "~":
            self.params_dir = "/home/" + self.user + self.params_dir[1:]
        
        self.declare_parameter("default_parameter_file", "parameters.yaml")
        
        params_file = str(self.get_parameter("default_parameter_file").value)
        
        if not self.validate_parameter_file_name(params_file):
            raise ValueError("ConfigurationServer.__init__(): Default parameter file name " + params_file + " is not valid.")
        
        self.params_file = os.path.join(
            self.params_dir,
            params_file
        )
        
        self.ros_params_file = "/home/" + self.user + "/.config/iii_drone/ros_params.yaml"

        self.parameter_handler = ParameterHandler.from_parameter_file(self.params_file)

        # Declared params empty dict:
        self.declared_params = {}
        self.parameters_initialized = {}

        # Initialize services:
        self.declare_parameter_service = self.create_service(
            DeclareParameter,
            "declare_parameter",
            self.declare_parameter_callback
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
        
        # Service callback that gets called before a parameter is set:
        self.set_parameter_event_callback_handler = self.add_on_set_parameters_callback(self.set_parameter_event_callback)

        #/parameter_events topic subscriber:
        self.parameter_events_subscription = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_events_callback,
            10
        )

        self.param_success = True
        
    def on_delete(self):
        self.get_logger().info("ConfigurationServer.on_delete(): Deleting ConfigurationServer object.")

        if self.parameter_handler.any_params_changed:
            self.get_logger().info("ConfigurationServer.on_delete(): Parameters changed, saving...")

            request = SaveParameters.Request()
            request.file = ""
            request.overwrite = True
            request.set_as_default = True

            response = self.save_parameters_callback(request, SaveParameters.Response())
            
            if not response.success:
                self.get_logger().fatal("ConfigurationServer.on_delete(): Failed to save parameters: " + response.message + ".")
            else:
                self.get_logger().info("ConfigurationServer.on_delete(): Parameters saved successfully to file " + response.file + ".")
        
    def set_parameter_event_callback(
        self,
        parameters: "list[Parameter]"
    ) -> SetParametersResult:
        """
        Callback for parameter events, gets called before a parameter is set. 
        Rejects if the parameter is constant, if the type or value do not match the loaded parameters,
        if the parameter is not listed in the loaded parameters, or if the validation function returns false.

        Parameters:
            parameters (list[Parameter]): List of parameters to be set.

        Returns:
            SetParametersResult: Result of the callback.
        """

        result = SetParametersResult()

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
        parameter_event: ParameterEvent
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

        if parameter_event.node != self.get_fully_qualified_name():
            return
        

        if len(parameter_event.deleted_parameters) > 0:
            self.get_logger().fatal("ConfigurationServer.parameter_events_callback(): Deleted parameters not supported.")
            raise RuntimeError("Deleted parameters not supported.")
        
        for parameter in parameter_event.changed_parameters + parameter_event.new_parameters:
            if parameter.name == "default_parameter_file" or "use_sim_time" in parameter.name:
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
        parameter_value: ParameterValue
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

    def declare_parameter_callback(
        self,
        request: DeclareParameter.Request,
        response: DeclareParameter.Response
    ) -> DeclareParameter.Response:
        """
        Callback for declare_parameter service. Checks that the the parameter has been loaded,
        and that the type matches the loaded type.

        Parameters:
            request (DeclareParameter.Request): Service request.
            response (DeclareParameter.Response): Service response.

        Returns:
            DeclareParameter.Response: Service response.
        """

        try:
            param_dict = self.parameter_handler.get_param(request.name)
        
        except KeyError as e:
            response.succeeded = False
            response.message = "Parameter not listed in parameters file: " + str(e)

            return response

        already_declared = False

        if request.name in self.declared_params:
            already_declared = True

        if request.type != param_dict["type"]:
            response.succeeded = False
            response.message = "Type " + str(request.type) + " does not match type " + str(param_dict["type"]) + " in loaded parameters for parameter " + request.name + "."

            return response

        self.get_logger().info("ConfigurationServer.declare_parameter_callback(): Declaring parameter " + request.name + " with value " + str(param_dict["value"]) + " of type " + str(param_dict["type"]) + ".")
        
        if already_declared:
            response.succeeded = True
            response.message = "Parameter already declared."

            return response
        
        value = param_dict["value"]

        self.declare_parameter(
            request.name, 
            value
        )

        self.declared_params[request.name] = value
        self.parameters_initialized[request.name] = False

        response.succeeded = True
        response.message = "Parameter declared successfully."

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
        
        if request.file == "":
            # Get the current date and time
            now = datetime.now()

            # Format the date and time as a string in the format 'YYYYMMDD_HHMM'
            date_time_string = now.strftime('%Y%m%d_%H%M')
            
            request.file = "parameters_" + date_time_string + ".yaml"

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
                self.set_default_parameter_file(request.file)
            
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
            self.get_logger().info("ConfigurationServer.on_delete(): Parameters changed, saving...")

            tmp_request = SaveParameters.Request()
            tmp_request.file = ""
            tmp_request.overwrite = True
            tmp_request.set_as_default = True

            tmp_response = self.save_parameters_callback(tmp_request, SaveParameters.Response())
            
            if not response.success:
                self.get_logger().fatal("ConfigurationServer.on_delete(): Failed to save parameters: " + tmp_response.message + ".")
            else:
                self.get_logger().info("ConfigurationServer.on_delete(): Parameters saved successfully to file " + tmp_response.file + ".")
        
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
                    Parameter(
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
            try:
                parameter_value = bool(parameter_value)
            except ValueError as e:
                response.success = False
                response.message = "Invalid bool value."
                
                return response
            
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
                set_result: "list[SetParametersResult]" = self.set_parameters([
                    Parameter(
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
        response.default_parameter_file = self.get_parameter("default_parameter_file").value
        response.current_parameter_file = os.path.basename(self.params_file)
        
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
        
        ros_params_lines = []

        with open(self.ros_params_file, "r") as ros_params_file:
            ros_params_lines = ros_params_file.readlines()
            
        for i in range(len(ros_params_lines)):
            if "default_parameter_file" in ros_params_lines[i]:
                splitted_line = ros_params_lines[i].split(":")
                splitted_line[1] = " " + f'"{file}"' + "\n"
                ros_params_lines[i] = ":".join(splitted_line)
                
                break
            
        with open(self.ros_params_file, "w") as ros_params_file:
            ros_params_file.writelines(ros_params_lines)
            
        self.set_parameters([
            Parameter(
                name="default_parameter_file",
                value=file
            )
        ])

###############################################################################
# Main
###############################################################################

def main():
    rclpy.init()

    print("Starting ConfigurationServer node...")
    node = ConfigurationServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_delete()

if __name__ == "__main__":
    main()