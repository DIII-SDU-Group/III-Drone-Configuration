#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

import yaml
import os
from copy import deepcopy

###############################################################################
# Class
###############################################################################

class ParameterHandler:
    """
    ParameterHandler class that loads parameters from a yaml file and provides methods for getting, setting, and validating parameters.
    """

    def __init__(self):
        """
        Constructor for ParameterHandler class.
        """

        self.file_path = ""
        self.raw_yaml_dict = {}
        self.params_dict = {}
        self._any_params_changed = False
        self._ignore_changed_parameter_names = []
        
    @staticmethod
    def from_parameter_file(file_path: str) -> "ParameterHandler":
        """
        Creates a ParameterHandler object from a yaml file.

        Parameters:
            file_path (str): Path to the yaml file containing the parameters.

        Returns:
            ParameterHandler: ParameterHandler object.
            
        Raises:
            ValueError: If a parameter name is not a string or contains other characters than letters, numbers, and underscores.
            ValueError: If a parameter name already exists.
            ValueError: If a parameter is missing the 'type' or 'value' key.
            TypeError: If a parameter name is not a string.
            TypeError: If a parameter 'type' is not a string.
            TypeError: If a parameter 'constant' is not a bool.
            TypeError: If a parameter 'options' is not a list of strings.
            TypeError: If a parameter value is not of the specified type.
            TypeError: If a parameter 'min' or 'max' is not of the specified type.
            ValueError: If a parameter 'min' or 'max' is not of type int or float.
            ValueError: If a parameter 'options' is not a list of strings.
        """

        ph = ParameterHandler()
        ph.file_path = file_path

        ph.load_parameter_file()
        
        return ph
    
    @staticmethod
    def from_raw_yaml_string(raw_yaml_string: str) -> "ParameterHandler":
        """
        Creates a ParameterHandler object from a yaml string.

        Parameters:
            raw_yaml_string (str): String containing the parameters in yaml format.

        Returns:
            ParameterHandler: ParameterHandler object.
        
        Raises:
            ValueError: If a parameter name is not a string or contains other characters than letters, numbers, and underscores.
            ValueError: If a parameter name already exists.
            ValueError: If a parameter is missing the 'type' or 'value' key.
            TypeError: If a parameter name is not a string.
            TypeError: If a parameter 'type' is not a string.
            TypeError: If a parameter 'constant' is not a bool.
            TypeError: If a parameter 'options' is not a list of strings.
            TypeError: If a parameter value is not of the specified type.
            TypeError: If a parameter 'min' or 'max' is not of the specified type.
            ValueError: If a parameter 'min' or 'max' is not of type int or float.
            ValueError: If a parameter 'options' is not a list of strings.
        """

        ph = ParameterHandler()
        ph.raw_yaml_dict = yaml.safe_load(raw_yaml_string)

        ph.load_from_raw_yaml_dict()
        
        return ph
    
    @property
    def any_params_changed(self) -> bool:
        """
        Returns whether any parameters have changed.

        Returns:
            bool: True if any parameters have changed, False otherwise.
        """

        return self._any_params_changed
        
    def get_parameters_yaml_dict(self) -> dict:
        """
        Returns the parameters yaml dict.

        Returns:
            str: Parameters yaml dict.
        """

        return self.raw_yaml_dict
    
    def get_parameters_yaml_string(self) -> str:
        """
        Returns the parameters yaml string.

        Returns:
            str: Parameters yaml string.
        """

        return yaml.dump(self.raw_yaml_dict, sort_keys=False)
    
    def reset_changed_parameters(
        self,
        changed_parameter_names: "list[str]" = []
    ):
        """
        Resets the changed parameters flag. 
        If changed_parameter_names is specified, the next change to these parameters will not be counted as a change.
        
        Parameters:
            changed_parameter_names (list[str]): List of parameter names that were changed, default [].
        """
        self._any_params_changed = False
        self._ignore_changed_parameter_names = changed_parameter_names
    
    def load_new_parameter_file(
        self,
        file_path: str,
        declared_parameters: "list[str]"
    ) -> "list[str]":
        """
        Loads a new parameter file. Returns a list of parameter names that were changed.
        All previously declared parameters must be present in the new parameter file.

        Parameters:
            file_path (str): Path to the yaml file containing the parameters.
            declared_parameters (list[str]): List of parameter names that are currently declared.
            
        Returns:
            list[str]: List of parameter names that were changed.
            
        Raises:
            FileNotFoundError: If the parameter file is not found.

            ValueError: If a parameter name is not a string or contains other characters than letters, numbers, and underscores.
            ValueError: If a parameter name already exists.
            ValueError: If a parameter is missing the 'type' or 'value' key.
            TypeError: If a parameter name is not a string.
            TypeError: If a parameter 'type' is not a string.
            TypeError: If a parameter 'constant' is not a bool.
            TypeError: If a parameter 'options' is not a list of strings.
            TypeError: If a parameter value is not of the specified type.
            TypeError: If a parameter 'min' or 'max' is not of the specified type.
            ValueError: If a parameter 'min' or 'max' is not of type int or float.
            ValueError: If a parameter 'options' is not a list of strings.

            ValueError: If a parameter name is not found.
            ValueError: If a parameter value is less than the minimum value.
            ValueError: If a parameter value is greater than the maximum value.
            ValueError: If a parameter value is not one of the allowed options.
            
        """

        ph = ParameterHandler()
        ph.file_path = file_path
        
        ph.load_parameter_file()
        
        changed_parameter_names = self._evaluate_parameters_from_new_handler(
            ph,
            declared_parameters=declared_parameters
        )
        
        self.file_path = file_path
        self.params_dict = ph.params_dict
        
        return changed_parameter_names
    
    def save_parameters(
        self,
        file_path: str,
        overwrite: bool = False
    ):
        """
        Saves the parameters to a yaml file.

        Parameters:
            file_path (str): Path to the yaml file to save the parameters to.
            overwrite (bool): Whether to overwrite the file if it already exists, default False.

        Raises:
            FileExistsError: If the file already exists and overwrite is False.
        """

        if not overwrite and os.path.exists(file_path):
            raise FileExistsError(f"File {file_path} already exists")

        with open(file_path, 'w') as f:
            f.write(self.get_parameters_yaml_string())

    def _evaluate_parameters_from_new_handler(
        self,
        new_handler: "ParameterHandler",
        declared_parameters: "list[str]" = None
    ) -> "list[str]":
        """
        Evaluates the parameters from a new ParameterHandler object and returns a list of parameter names that were changed.

        Parameters:
            new_handler (ParameterHandler): ParameterHandler object containing the new parameters.
            declared_parameters (list[str]): List of parameter names that are currently declared, default [].

        Returns:
            list[str]: List of parameter names that were changed.

        Raises:
            ValueError: If a parameter name is not found.
            TypeError: If a parameter value is not of the specified type.
            ValueError: If a parameter value is less than the minimum value.
            ValueError: If a parameter value is greater than the maximum value.
            ValueError: If a parameter value is not one of the allowed options.
        """

        changed_parameter_names = []
        parameters_to_be_evaluated = declared_parameters if declared_parameters is not None else self.params_dict.keys()

        for param_name in parameters_to_be_evaluated:
            if param_name not in new_handler.params_dict:
                raise ValueError(f"Parameter {param_name} not found in new parameter file.")

            param_value = new_handler.params_dict[param_name]['value']

            self.validate_param(
                param_name,
                param_value
            )

            if param_value != self.params_dict[param_name]['value']:
                changed_parameter_names.append(param_name)

        return changed_parameter_names
        

    def load_parameter_file(self) -> None:
        """
        Loads the parameter file and validates the parameters.

        Raises:
            ValueError: If a parameter name is not a string or contains other characters than letters, numbers, and underscores.
            ValueError: If a parameter name already exists.
            ValueError: If a parameter is missing the 'type' or 'value' key.
            TypeError: If a parameter name is not a string.
            TypeError: If a parameter 'type' is not a string.
            TypeError: If a parameter 'constant' is not a bool.
            TypeError: If a parameter 'options' is not a list of strings.
            TypeError: If a parameter value is not of the specified type.
            TypeError: If a parameter 'min' or 'max' is not of the specified type.
            ValueError: If a parameter 'min' or 'max' is not of type int or float.
            ValueError: If a parameter 'options' is not a list of strings.
        """

        with open(self.file_path, 'r') as f:
            self.raw_yaml_dict = yaml.safe_load(f)
            
        self.load_from_raw_yaml_dict()

    def load_from_raw_yaml_dict(self) -> None:
        params_dict, params_with_expressions = self._load_params(
            self.raw_yaml_dict,
            {},
            {}
        )

        self._validate_expressions(
            params_dict,
            params_with_expressions
        )

        self.params_dict = params_dict

    def validate_param(
        self, 
        param_name: str,
        param_value: "str|int|float|bool|list[str|int|float|bool]"
    ) -> None:
        """
        Validates a parameter value.

        Parameters:
            param_name (str): Name of the parameter.
            param_value (str|int|float|bool|list[str|int|float|bool]): Value of the parameter.

        Raises:
            KeyError: If the parameter name is not found.
            TypeError: If the parameter value is not of the specified type.
            ValueError: If the parameter value is less than the minimum value.
            ValueError: If the parameter value is greater than the maximum value.
            ValueError: If the parameter value is not one of the allowed options.
        """
        if param_name not in self.params_dict:      
            raise KeyError(f"Parameter {param_name} not found")

        param_type = self.params_dict[param_name]['type']

        self._validate_parameter_type(
            param_value,
            param_type
        )

        if 'min' in self.params_dict[param_name]:
            if isinstance(self.params_dict[param_name]['min'], str):
                min_value = self._evaluate_expression(
                    self.params_dict[param_name]['min'],
                    self.params_dict
                )

            else:
                min_value = self.params_dict[param_name]['min']

            if param_value < min_value:
                if isinstance(self.params_dict[param_name]['min'], str):
                    raise ValueError(f"Parameter {param_name} is less than minimum value {self.params_dict[param_name]['min']} = {min_value}")

                else:
                    raise ValueError(f"Parameter {param_name} is less than minimum value {min_value}")

        if 'max' in self.params_dict[param_name]:
            if isinstance(self.params_dict[param_name]['max'], str):
                max_value = self._evaluate_expression(
                    self.params_dict[param_name]['max'],
                    self.params_dict
                )

            else:
                max_value = self.params_dict[param_name]['max']

            if param_value > max_value:
                if isinstance(self.params_dict[param_name]['max'], str):
                    raise ValueError(f"Parameter {param_name} is greater than maximum value {self.params_dict[param_name]['max']} = {max_value}")

                else:
                    raise ValueError(f"Parameter {param_name} is greater than maximum value {max_value}")

        if 'options' in self.params_dict[param_name] and param_value not in self.params_dict[param_name]['options']:
            raise ValueError(f"Parameter {param_name} is not one of the allowed options {self.params_dict[param_name]['options']}")

    def get_param_value(
        self, 
        param_name: str
    ) -> "str|int|float|bool|list[str|int|float|bool]":
        """
        Returns the value of a parameter.

        Parameters:
            param_name (str): Name of the parameter.

        Returns:
            str|int|float|bool|list[str|int|float|bool]: Value of the parameter.

        Raises:
            ValueError: If the parameter name is not found.
        """

        if param_name not in self.params_dict:
            raise ValueError(f"Parameter {param_name} not found")

        return self.params_dict[param_name]['value']

    def get_param(
        self, 
        param_name: str
    ) -> dict:
        """
        Returns a parameter data structure.

        Parameters:
            param_name (str): Name of the parameter.

        Returns:
            dict: Dictionary containing the parameter information.

        Raises:
            KeyError: If the parameter name is not found.
        """

        if param_name not in self.params_dict:
            raise KeyError(f"Parameter {param_name} not found")

        return self.params_dict[param_name]

    def set_param(
        self, 
        param_name: str, 
        param_value: "str|int|float|bool|list[str|int|float|bool]",
        parameter_initialized: bool,
        force_constant: bool = False
    ) -> None:
        """
        Sets the value of a parameter.

        Parameters:
            param_name (str): Name of the parameter.
            param_value (str|int|float|bool|list[str|int|float|bool]): Value of the parameter.
            parameter_initialized (bool): Whether the parameter has been initialized.
            force_constant (bool): Whether to force setting the parameter even if it is constant, default False.

        Raises:
            KeyError: If the parameter name is not found.
            AttributeError: If the parameter is constant and cannot be changed.
            TypeError: If the parameter value is not of the specified type.
            ValueError: If the parameter value is less than the minimum value.
            ValueError: If the parameter value is greater than the maximum value.
            ValueError: If the parameter value is not one of the allowed options.
        """
        
        param_value = self.cast_param_value(
            param_name, 
            param_value
        )

        self.can_set_param(
            param_name, 
            param_value,
            (not parameter_initialized) or force_constant
        )

        dict_to_update = self._get_raw_yaml_param_dict(param_name)

        self.params_dict[param_name]['value'] = param_value
        
        previous_value = dict_to_update['value']
        dict_to_update['value'] = param_value
        
        if previous_value != param_value:
            if param_name not in self._ignore_changed_parameter_names:
                self._any_params_changed = True
            else:
                self._ignore_changed_parameter_names.remove(param_name)

    def cast_param_value(
        self,
        param_name: str,
        param_value: "str|int|float|bool|list[str|int|float|bool]"
    ) -> "str|int|float|bool|list[str|int|float|bool]":
        """
        Attempts to cast the value of a parameter to the correct type.

        Parameters:
            param_name (str): Name of the parameter.
            param_value (str|int|float|bool|list[str|int|float|bool]): Value of the parameter.

        Returns:
            str|int|float|bool|list[str|int|float|bool]: Value of the parameter.

        Raises:
            KeyError: If the parameter name is not found.
            TypeError: If the parameter value is not of the specified type.
        """

        if param_name not in self.params_dict:
            raise KeyError(f"Parameter {param_name} not found")

        param_type = self.params_dict[param_name]['type']
        
        if param_type == "float":
            param_value = float(param_value)
        elif param_type == "int":
            param_value = int(param_value)
        elif param_type == "bool":
            param_value = bool(param_value)
        elif param_type == "string":
            param_value = str(param_value)
        elif param_type == "string_array":
            param_value = list(param_value)
            for i, value in enumerate(param_value):
                param_value[i] = str(value)
        elif param_type == "int_array" or param_type == "integer_array":
            param_value = list(param_value)
            for i, value in enumerate(param_value):
                param_value[i] = int(value)
        elif param_type == "float_array":
            param_value = list(param_value)
            for i, value in enumerate(param_value):
                param_value[i] = float(value)
        elif param_type == "bool_array":
            param_value = list(param_value)
            for i, value in enumerate(param_value):
                param_value[i] = bool(value)

        return param_value
                
    def _get_raw_yaml_param_dict(
        self,
        param_name: str,
        create_missing_namespaces: bool = False,
        get_parent_dict: bool = False
    ) -> "dict|tuple[dict,str]":
        """
        Returns the raw yaml dict of a parameter.

        Parameters:
            param_name (str): Name of the parameter.
            create_missing_namespaces (bool): Whether to create missing namespaces in case of adding a new parameter, default False.
            get_parent_dict (bool): Whether to return the parent dict of the parameter instead of the parameter dict, default False.

        Returns:
            dict: Dictionary containing the parameter information.

        Raises:
            KeyError: If the parameter name is not found.
        """

        param_namespaces = param_name.split("/")[1:]
        
        dict_to_update = self.raw_yaml_dict
        
        for i, param_namespace in enumerate(param_namespaces):
            if i == len(param_namespaces) - 1 and get_parent_dict:
                break
            
            if param_namespace not in dict_to_update and not create_missing_namespaces:
                raise KeyError(f"Parameter namespace {param_namespace} not found")
            elif param_namespace not in dict_to_update and create_missing_namespaces:
                dict_to_update[param_namespace] = {}
            dict_to_update = dict_to_update[param_namespace]
            
        if 'value' not in dict_to_update and not create_missing_namespaces and not get_parent_dict:
            raise KeyError(f"Parameter {param_name} not found")

        if not get_parent_dict:
            return dict_to_update
        else:
            return dict_to_update, param_namespaces[-1]

    def can_set_param(
        self,
        param_name: str,
        param_value: "str|int|float|bool|list[str|int|float|bool]",
        force_constant: bool = False,
    ) -> None:
        """
        Checks if a parameter can be set.

        Parameters:
            param_name (str): Name of the parameter.
            param_value (str|int|float|bool|list[str|int|float|bool]): Value of the parameter.
            force_constant (bool): Whether to force setting the parameter even if it is constant, default False.

        Raises:
            KeyError: If the parameter name is not found.
            TypeError: If the parameter value is not of the specified type.
            ValueError: If the parameter value is less than the minimum value.
            ValueError: If the parameter value is greater than the maximum value.
            ValueError: If the parameter value is not one of the allowed options.
        """

        self.validate_param(param_name, param_value)

        if "constant" in self.params_dict[param_name] and self.params_dict[param_name]["constant"] and not force_constant:
            raise AttributeError(f"Parameter {param_name} is constant and cannot be changed")

    def get_all_params(self) -> dict:
        """
        Returns all parameter data structures as a dictionary.

        Returns:
            dict: Dictionary containing all parameters.
        """

        return self.params_dict
    
    def update_param(
        self,
        param_name: str,
        param_dict: dict,
        keep_value: bool = False
    ):
        """
        Updates a parameter dict.
        Attention: Updates the full parameter configuration, not just the value.
        If only updating the value, use set_param() instead.

        Parameters:
            param_name (str): Name of the parameter.
            param_dict (dict): Dictionary containing the parameter configuration.
            keep_value (bool): Whether to keep the current value of the parameter, default False.
            
        Raises:
            KeyError: If the parameter name is not found.
            KeyError: If the parameter is missing the 'type' or 'value' key.
            TypeError: If the parameter 'type' is not a string.
            TypeError: If the parameter 'constant' is not a bool.
            TypeError: If the parameter 'options' is not a list of strings.
            TypeError: If the parameter value is not of the specified type.
            TypeError: If the parameter 'min' or 'max' is not of the specified type.
            ValueError: If the parameter 'min' or 'max' is not of type int or float.
            ValueError: If the parameter 'options' is not a list of strings.
        """

        if param_name not in self.params_dict:
            raise KeyError(f"Parameter {param_name} not found")
        
        dict_to_update = self._get_raw_yaml_param_dict(param_name)
        
        old_keys = list(dict_to_update.keys())
        new_keys = list(param_dict.keys())
        new_keys_copy = deepcopy(new_keys)

        for key in new_keys_copy:
            old_keys.remove(key)
            new_keys.remove(key)

            if key == "value" and keep_value:
                continue
            
            if key == "type" and dict_to_update[key] != param_dict[key]:
                raise ValueError(f"Attempted change of parameter type from {dict_to_update[key]} to {param_dict[key]} for parameter {param_name}")
            
            dict_to_update[key] = param_dict[key]

        for key in old_keys:
            del dict_to_update[key]

        self.params_dict[param_name] = param_dict
        
        self.validate_param(
            param_name,
            self.get_param_value(param_name)
        )
        
    def add_param(
        self,
        param_name: str,
        param_dict: dict
    ):
        """
        Adds a parameter dict.

        Parameters:
            param_name (str): Name of the parameter.
            param_dict (dict): Dictionary containing the parameter configuration.
            
        Raises:
            KeyError: If the parameter name already exists.
            KeyError: If the parameter is missing the 'type' or 'value' key.
            TypeError: If the parameter 'type' is not a string.
            TypeError: If the parameter 'constant' is not a bool.
            TypeError: If the parameter 'options' is not a list of strings.
            TypeError: If the parameter value is not of the specified type.
            TypeError: If the parameter 'min' or 'max' is not of the specified type.
            ValueError: If the parameter 'min' or 'max' is not of type int or float.
            ValueError: If the parameter 'options' is not a list of strings.
        """

        if param_name in self.params_dict:
            raise KeyError(f"Parameter {param_name} already exists")

        dict_to_update = self._get_raw_yaml_param_dict(
            param_name, 
            create_missing_namespaces=True
        )

        for key in param_dict.keys():
            dict_to_update[key] = param_dict[key]

            self.params_dict[param_name] = param_dict
        
    def remove_param(
        self,
        param_name: str
    ):
        """
        Removes a parameter.

        Parameters:
            param_name (str): Name of the parameter.
            
        Raises:
            KeyError: If the parameter name is not found.
        """

        if param_name not in self.params_dict:
            raise KeyError(f"Parameter {param_name} not found")
        
        parent_dict, param_final_key = self._get_raw_yaml_param_dict(
            param_name,
            get_parent_dict=True
        )

        del parent_dict[param_final_key]
        
        del self.params_dict[param_name]
        
    def validate(self):
        """
        Validates all parameters.

        Raises:
            ValueError: If a parameter name is not found.
            TypeError: If a parameter value is not of the specified type.
            ValueError: If a parameter value is less than the minimum value.
            ValueError: If a parameter value is greater than the maximum value.
            ValueError: If a parameter value is not one of the allowed options.
        """

        for param_name, param in self.params_dict.items():
            self.validate_param(
                param_name,
                param['value']
            )

    def _load_params(
        self,
        params_dict: dict,
        params_out_dict: dict,
        params_with_expressions: dict,
        namespace=""
    ) -> "tuple(dict[str, Any],dict[str, Any])":
        """
        Loads parameters from a dictionary recursively, parsing nested namespaces.

        Parameters:
            params_dict (dict): Dictionary containing the parameters.
            params_out_dict (dict): Dictionary containing the output parameters.
            params_with_expressions (dict): Dictionary containing the parameters with expressions as min/max values.
            namespace (str): Namespace of the parameters, default "".

        Returns:
            dict[str, Any]: Dictionary containing the output parameters.
            dict[str, Any]: Dictionary containing the parameters with expressions as min/max values.

        Raises:
            TypeError: If a parameter name is not a string.
            ValueError: If a parameter name contains other characters than letters, numbers, and underscores.
            ValueError: If a parameter name already exists.
            ValueError: If a parameter is missing the 'type' or 'value' key.
            TypeError: If a parameter 'type' is not a string.
            TypeError: If a parameter 'constant' is not a bool.
            TypeError: If a parameter 'options' is not a list of strings.
            TypeError: If a parameter value is not of the specified type.
            TypeError: If a parameter 'min' or 'max' is not of the specified type.
            ValueError: If a parameter 'min' or 'max' is not of type int or float.
            ValueError: If a parameter 'options' is not a list of strings.
        """

        for key, value in params_dict.items():
            # If key is not a string, raise error:
            if not isinstance(key, str):
                raise TypeError(f"Parameter name must be a string, not {type(key)}")
            
            # If key has other values than letters, numbers, and underscores, raise error:
            if not all(char.isalnum() or char == '_' for char in key):
                raise ValueError(f"Parameter name must only contain letters, numbers, and underscores, not {key}")

            # Check if there is nested namespaces:
            if isinstance(value, dict) and all(isinstance(value2, dict) for value2 in value.values()):
                params_out_dict, params_with_expressions = self._load_params(
                    value, 
                    params_out_dict, 
                    params_with_expressions,
                    namespace + "/" + key
                )

                continue

            # for _, param in value.items():
            param_name = f"{namespace}/{key}"

            if param_name in params_out_dict:
                raise ValueError(f"Parameter {param_name} already exists")

            params_out_dict[param_name], has_expression = self._load_param(
                param_name, 
                # param
                value
            )

            if has_expression:
                if param_name in params_with_expressions:
                    raise ValueError(f"Parameter {param_name} already has an expression")

                params_with_expressions[param_name] = params_out_dict[param_name]

        return params_out_dict, params_with_expressions
    
    def _load_param(
        self, 
        param_name: str,
        param: dict
    ) -> "tuple(dict, bool)":
        """
        Loads a parameter from a dictionary.

        Parameters:
            param_name (str): Name of the parameter.
            param (dict): Dictionary containing the parameter.

        Returns:
            dict: Dictionary containing the parameter.
            bool: True if the parameter has an expression as min/max value, False otherwise.

        Raises:
            ValueError: If a parameter is missing the 'type' or 'value' key.
            TypeError: If a parameter 'type' is not a string.
            TypeError: If a parameter 'constant' is not a bool.
            TypeError: If a parameter 'options' is not a list of strings.
            TypeError: If a parameter value is not of the specified type.
            TypeError: If a parameter 'min' or 'max' is not of the specified type.
            ValueError: If a parameter 'min' or 'max' is not of type int or float.
            ValueError: If a parameter 'options' is not a list of strings.
        """

        if 'type' not in param:
            raise ValueError(f"Parameter {param_name} is missing 'type' key")
        if 'value' not in param:
            raise ValueError(f"Parameter {param_name} is missing 'type' or 'value' keys")

        if not isinstance(param['type'], str):
            raise TypeError(f"Parameter {param_name} 'type' must be a string specifying the type")

        param_type = param['type']
        param_value = param['value']

        try:
            self._validate_parameter_type(
                param_value, 
                param_type
            )
        except TypeError as e:
            raise TypeError(f"Parameter {param_name} has invalid type specification or value is not of specified type.") from e

        if param_type == "float":
            param_value = float(param_value)

        param_out = {
            'type': param_type,
            'value': param_value
        }

        if 'constant' in param:
            if not isinstance(param['constant'], bool):
                raise TypeError(f"Parameter {param_name} 'constant' must be a bool")

            param_out['constant'] = param['constant']

        has_expression = False

        if 'min' in param:
            if param['type'] not in ['int', 'float']:
                raise ValueError(f"Parameter {param_name} 'min' is only valid for int and float types")

            if isinstance(param['min'], str):
                has_expression = True
            else:
                try:
                    self._validate_parameter_type(
                        param['min'], 
                        param['type']
                    )
                except TypeError as e:
                    raise TypeError(f"Parameter {param_name} 'min' is not of type {param['type']}")


            param_out['min'] = param['min']

        if 'max' in param:
            if param['type'] not in ['int', 'float']:
                raise ValueError(f"Parameter {param_name} 'max' is only valid for int and float types")

            if isinstance(param['max'], str):
                has_expression = True
            else:
                try:
                    self._validate_parameter_type(
                        param['max'], 
                        param['type']
                    )
                except TypeError as e:
                    raise TypeError(f"Parameter {param_name} 'max' is not of type {param['type']}")

            param_out['max'] = param['max']

        if 'options' in param:
            if param['type'] != 'string':
                raise ValueError(f"Parameter {param_name} 'options' is only valid for string type")

            if not isinstance(param['options'], list) or not all(isinstance(option, str) for option in param['options']):
                raise TypeError(f"Parameter {param_name} 'options' must be a list of strings")

            param_out['options'] = param['options']

        return param_out, has_expression

    def _validate_parameter_type(
        self,
        param_value: "str|int|float|bool|list[str|int|float|bool]",
        param_type: str
    ) -> None:
        """
        Validates a parameter type.

        Parameters:
            param_value (str|int|float|bool|list[str|int|float|bool]): Value of the parameter.
            param_type (str): Type of the parameter.

        Raises:
            TypeError: If the parameter value is not of the specified type.
        """

        if param_type == "string":
            if not isinstance(param_value, str):
                raise TypeError(f"Parameter value must be a string, not {type(param_value)}")

        elif param_type == "int":
            if not isinstance(param_value, int):
                raise TypeError(f"Parameter value must be an int, not {type(param_value)}")
            
        elif param_type == "float":
            if not isinstance(param_value, float) and not isinstance(param_value, int):
                raise TypeError(f"Parameter value must be a float, not {type(param_value)}")
            
        elif param_type == "bool":
            if not isinstance(param_value, bool):
                raise TypeError(f"Parameter value must be a bool, not {type(param_value)}")
            
        elif param_type == "string_array":
            if not isinstance(param_value, list) or not all(isinstance(value, str) for value in param_value):
                raise TypeError(f"Parameter value must be a list of strings, not {type(param_value)}")
            
        elif param_type == "int_array":
            if not isinstance(param_value, list) or not all(isinstance(value, int) for value in param_value):
                raise TypeError(f"Parameter value must be a list of ints, not {type(param_value)}")
            
        elif param_type == "float_array":
            if not isinstance(param_value, list) or not all(isinstance(value, float) or isinstance(value, int) for value in param_value):
                raise TypeError(f"Parameter value must be a list of floats, not {type(param_value)}")

        elif param_type == "bool_array":
            if not isinstance(param_value, list) or not all(isinstance(value, bool) for value in param_value):
                raise TypeError(f"Parameter value must be a list of bools, not {type(param_value)}")
            
        else:
            raise ValueError(f"Invalid parameter type {param_type}")

    def _validate_expressions(
        self,
        params_dict: dict,
        params_with_expressions: dict
    ) -> None:
        """
        Validates expressions in parameters.

        Parameters:
            params_dict (dict): Dictionary containing the parameters.
            params_with_expressions (dict): Dictionary containing the parameters with expressions as min/max values.
            
        Raises:
            ValueError: If a parameter 'min' or 'max' expression is invalid.
            ValueError: If a parameter value is less than the minimum value.
            ValueError: If a parameter value is greater than the maximum value.
        """

        for param_name, param in params_with_expressions.items():
            if 'min' in param and isinstance(param['min'], str):
                try:
                    min_value = self._evaluate_expression(
                        param['min'],
                        params_dict
                    )

                except Exception as e:
                    raise ValueError(f"Parameter {param_name} 'min' expression is invalid") from e

                # Check if parameter value is less than minimum value:
                if param['value'] < min_value:
                    raise ValueError(f"Parameter {param_name} value is less than minimum value {param['min']} = {min_value}")

            if 'max' in param and isinstance(param['max'], str):
                try:
                    max_value = self._evaluate_expression(
                        param['max'],
                        params_dict
                    )

                except Exception as e:
                    raise ValueError(f"Parameter {param_name} 'max' expression is invalid") from e

                # Check if parameter value is greater than maximum value:
                if param['value'] > max_value:
                    raise ValueError(f"Parameter {param_name} value is greater than maximum value {param['max']} = {max_value}")

    def _evaluate_expression(
        self,
        expression: str,
        params_dict: dict
    ) -> "int|float":
        """
        Evaluates an expression.

        Parameters:
            expression (str): Expression to evaluate.
            params_dict (dict): Dictionary containing the parameters.

        Returns:
            int|float: Result of the expression.
        
        Raises:
            ValueError: If a parameter name is not found.
            ValueError: If a parameter value is less than the minimum value.
            ValueError: If a parameter value is greater than the maximum value.
            ValueError: If a parameter value is not one of the allowed options.
        """

        # Replace all parameter names with their values:
        for param_name, param in params_dict.items():
            expression = expression.replace(param_name, str(param['value']))

        # Evaluate the expression and return the result:
        try:
            val = eval(expression)

        except Exception as e:
            raise ValueError(f"Error evaluating expression {expression}") from e
        
        return val

####################################################################################################
# Main
####################################################################################################

def main():
    # Load parameter file:
    parameter_handler = ParameterHandler('/home/fn/Workspace/ros2_refactor_ws/src/III-Drone-Core/config/parameters.yaml')

    # # Get all parameters and print nicely formatted yaml style:
    # print(yaml.dump(parameter_handler.get_all_params(), sort_keys=False))

    # Get the value of the parameter "/perception"pl_mapper/alive_cnt_high_thresh" and print it nicely formatted yaml style:
    print(yaml.dump(parameter_handler.get_param("/perception/pl_mapper/alive_cnt_high_thresh"), sort_keys=False))

if __name__ == '__main__':
    main()
