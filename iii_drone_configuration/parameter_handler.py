#!/usr/bin/python3

import os
from copy import deepcopy
from typing import Any, Dict, List, Optional, Tuple

import yaml


class ParameterHandler:
    """
    YAML/state helper for the configuration server and TUI client.

    Schema parsing and validation are delegated to the shared native core so
    Python nodes, the server, and the client use the same validation logic.
    """

    def __init__(self):
        self.file_path = ""
        self.raw_yaml_dict: Dict[str, Any] = {}
        self.params_dict: Dict[str, Dict[str, Any]] = {}
        self._any_params_changed = False
        self._ignore_changed_parameter_names: List[str] = []
        self._native_core = None

    @staticmethod
    def _require_native_core():
        try:
            from ._native import NativeConfiguratorCore
        except ImportError as exc:
            raise ImportError(
                "iii_drone_configuration native extension is required for ParameterHandler. "
                "Build III-Drone-Configuration before using the Python configuration tools."
            ) from exc
        return NativeConfiguratorCore

    @classmethod
    def from_parameter_file(cls, file_path: str) -> "ParameterHandler":
        handler = cls()
        handler.file_path = file_path
        handler.load_parameter_file()
        return handler

    @classmethod
    def from_raw_yaml_string(cls, raw_yaml_string: str) -> "ParameterHandler":
        handler = cls()
        handler.raw_yaml_dict = yaml.safe_load(raw_yaml_string) or {}
        NativeConfiguratorCore = cls._require_native_core()
        handler._native_core = NativeConfiguratorCore(raw_yaml_string, True)
        handler.load_from_raw_yaml_dict()
        return handler

    @property
    def any_params_changed(self) -> bool:
        return self._any_params_changed

    def get_parameters_yaml_dict(self) -> dict:
        return self.raw_yaml_dict

    def get_parameters_yaml_string(self) -> str:
        return yaml.dump(self.raw_yaml_dict, sort_keys=False)

    def reset_changed_parameters(self, changed_parameter_names: List[str] = []):
        self._any_params_changed = False
        self._ignore_changed_parameter_names = list(changed_parameter_names)

    def load_new_parameter_file(self, file_path: str, declared_parameters: List[str]) -> List[str]:
        new_handler = ParameterHandler.from_parameter_file(file_path)
        changed_parameter_names = self._evaluate_parameters_from_new_handler(
            new_handler,
            declared_parameters=declared_parameters,
        )

        self.file_path = file_path
        self.raw_yaml_dict = deepcopy(new_handler.raw_yaml_dict)
        self.params_dict = deepcopy(new_handler.params_dict)
        self._native_core = new_handler._native_core
        return changed_parameter_names

    def save_parameters(self, file_path: str, overwrite: bool = False):
        if not overwrite and os.path.exists(file_path):
            raise FileExistsError(f"File {file_path} already exists")

        with open(file_path, "w") as file:
            file.write(self.get_parameters_yaml_string())

    def _evaluate_parameters_from_new_handler(
        self,
        new_handler: "ParameterHandler",
        declared_parameters: Optional[List[str]] = None,
    ) -> List[str]:
        changed_parameter_names = []
        parameters_to_be_evaluated = declared_parameters if declared_parameters is not None else list(self.params_dict.keys())

        for param_name in parameters_to_be_evaluated:
            if param_name not in new_handler.params_dict:
                raise ValueError(f"Parameter {param_name} not found in new parameter file.")

            param_value = new_handler.params_dict[param_name]["value"]
            self.validate_param(param_name, param_value)

            if param_value != self.params_dict[param_name]["value"]:
                changed_parameter_names.append(param_name)

        return changed_parameter_names

    def load_parameter_file(self) -> None:
        with open(self.file_path, "r") as file:
            raw_yaml_string = file.read()

        self.raw_yaml_dict = yaml.safe_load(raw_yaml_string) or {}
        NativeConfiguratorCore = self._require_native_core()
        self._native_core = NativeConfiguratorCore(self.file_path)
        self.load_from_raw_yaml_dict()

    def load_from_raw_yaml_dict(self) -> None:
        self._require_native_core()
        if self._native_core is None:
            self._native_core = self._require_native_core()(self.get_parameters_yaml_string(), True)

        self.params_dict = self._flatten_params(self.raw_yaml_dict)
        self.validate()

    def validate_param(self, param_name: str, param_value: Any) -> None:
        self._ensure_parameter_exists(param_name)
        candidate_values = self._candidate_values_with_override(param_name, param_value)
        self._native_core.validate_parameter_value(
            param_name,
            candidate_values[param_name]["value"],
            candidate_values[param_name]["type"],
            candidate_values,
            True,
        )

    def get_param_value(self, param_name: str) -> Any:
        self._ensure_parameter_exists(param_name)
        return self.params_dict[param_name]["value"]

    def get_param(self, param_name: str) -> Dict[str, Any]:
        self._ensure_parameter_exists(param_name)
        return self.params_dict[param_name]

    def set_param(
        self,
        param_name: str,
        param_value: Any,
        parameter_initialized: bool,
        force_constant: bool = False,
    ) -> None:
        param_value = self.cast_param_value(param_name, param_value)
        self.can_set_param(param_name, param_value, (not parameter_initialized) or force_constant)

        dict_to_update = self._get_raw_yaml_param_dict(param_name)
        previous_value = dict_to_update["value"]
        dict_to_update["value"] = param_value
        self.params_dict[param_name]["value"] = param_value

        if previous_value != param_value:
            if param_name not in self._ignore_changed_parameter_names:
                self._any_params_changed = True
            else:
                self._ignore_changed_parameter_names.remove(param_name)

    def cast_param_value(self, param_name: str, param_value: Any) -> Any:
        self._ensure_parameter_exists(param_name)
        param_type = self.params_dict[param_name]["type"]

        if param_type == "float":
            return float(param_value)
        if param_type == "int":
            return int(param_value)
        if param_type == "bool":
            if isinstance(param_value, str):
                lowered = param_value.strip().lower()
                if lowered in ("true", "1", "yes", "on"):
                    return True
                if lowered in ("false", "0", "no", "off"):
                    return False
                raise TypeError(f"Cannot cast '{param_value}' to bool for parameter {param_name}")
            return bool(param_value)
        if param_type == "string":
            return str(param_value)
        if param_type == "string_array":
            return [str(value) for value in list(param_value)]
        if param_type in ("int_array", "integer_array"):
            return [int(value) for value in list(param_value)]
        if param_type == "float_array":
            return [float(value) for value in list(param_value)]
        if param_type == "bool_array":
            values = []
            for value in list(param_value):
                if isinstance(value, str):
                    lowered = value.strip().lower()
                    if lowered in ("true", "1", "yes", "on"):
                        values.append(True)
                        continue
                    if lowered in ("false", "0", "no", "off"):
                        values.append(False)
                        continue
                    raise TypeError(f"Cannot cast '{value}' to bool for parameter {param_name}")
                values.append(bool(value))
            return values

        return param_value

    def _get_raw_yaml_param_dict(
        self,
        param_name: str,
        create_missing_namespaces: bool = False,
        get_parent_dict: bool = False,
    ) -> Any:
        param_namespaces = [name for name in param_name.split("/") if name]
        dict_to_update = self.raw_yaml_dict

        for i, param_namespace in enumerate(param_namespaces):
            if i == len(param_namespaces) - 1 and get_parent_dict:
                break

            if param_namespace not in dict_to_update and not create_missing_namespaces:
                raise KeyError(f"Parameter namespace {param_namespace} not found")
            if param_namespace not in dict_to_update and create_missing_namespaces:
                dict_to_update[param_namespace] = {}
            dict_to_update = dict_to_update[param_namespace]

        if "value" not in dict_to_update and not create_missing_namespaces and not get_parent_dict:
            raise KeyError(f"Parameter {param_name} not found")

        if not get_parent_dict:
            return dict_to_update
        return dict_to_update, param_namespaces[-1]

    def can_set_param(self, param_name: str, param_value: Any, force_constant: bool = False) -> None:
        self._ensure_parameter_exists(param_name)
        candidate_values = self._candidate_values_with_override(param_name, param_value)
        try:
            self._native_core.validate_parameter_value(
                param_name,
                candidate_values[param_name]["value"],
                candidate_values[param_name]["type"],
                candidate_values,
                force_constant,
            )
        except RuntimeError as exc:
            if "constant" in str(exc):
                raise AttributeError(str(exc)) from exc
            raise

    def get_all_params(self) -> Dict[str, Dict[str, Any]]:
        return self.params_dict

    def update_param(self, param_name: str, param_dict: Dict[str, Any], keep_value: bool = False):
        self._ensure_parameter_exists(param_name)
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
                raise ValueError(
                    f"Attempted change of parameter type from {dict_to_update[key]} to {param_dict[key]} for parameter {param_name}"
                )

            dict_to_update[key] = param_dict[key]

        for key in old_keys:
            del dict_to_update[key]

        self.params_dict[param_name] = deepcopy(dict_to_update)
        self.validate()

    def add_param(self, param_name: str, param_dict: Dict[str, Any]):
        if param_name in self.params_dict:
            raise KeyError(f"Parameter {param_name} already exists")

        dict_to_update = self._get_raw_yaml_param_dict(param_name, create_missing_namespaces=True)
        dict_to_update.update(deepcopy(param_dict))
        self._native_core = self._require_native_core()(self.get_parameters_yaml_string(), True)
        self.load_from_raw_yaml_dict()

    def remove_param(self, param_name: str):
        self._ensure_parameter_exists(param_name)
        parent_dict, param_final_key = self._get_raw_yaml_param_dict(param_name, get_parent_dict=True)
        del parent_dict[param_final_key]
        self.load_from_raw_yaml_dict()

    def validate(self):
        if not self.params_dict:
            return
        self._native_core.validate_parameter_map(self._candidate_values(), True)

    def _flatten_params(self, params_dict: Dict[str, Any], namespace: str = "") -> Dict[str, Dict[str, Any]]:
        params_out_dict: Dict[str, Dict[str, Any]] = {}

        for key, value in params_dict.items():
            if not isinstance(key, str):
                raise TypeError(f"Parameter name must be a string, not {type(key)}")

            if not all(char.isalnum() or char == "_" for char in key):
                raise ValueError(f"Parameter name must only contain letters, numbers, and underscores, not {key}")

            param_name = f"{namespace}/{key}"

            if isinstance(value, dict) and "type" in value and "value" in value:
                if param_name in params_out_dict:
                    raise ValueError(f"Parameter {param_name} already exists")

                param_entry = deepcopy(value)
                param_entry["value"] = self.cast_value_for_type(param_entry["type"], param_entry["value"], param_name)
                params_out_dict[param_name] = param_entry
                continue

            if not isinstance(value, dict):
                raise ValueError(f"Invalid schema node at {param_name}")

            params_out_dict.update(self._flatten_params(value, param_name))

        return params_out_dict

    @staticmethod
    def cast_value_for_type(param_type: str, param_value: Any, param_name: str = "") -> Any:
        try:
            if param_type == "float":
                return float(param_value)
            if param_type == "int":
                return int(param_value)
            if param_type == "bool":
                if isinstance(param_value, bool):
                    return param_value
                if isinstance(param_value, str):
                    lowered = param_value.strip().lower()
                    if lowered in ("true", "1", "yes", "on"):
                        return True
                    if lowered in ("false", "0", "no", "off"):
                        return False
                return bool(param_value)
            if param_type == "string":
                return str(param_value)
            if param_type == "string_array":
                return [str(value) for value in list(param_value)]
            if param_type in ("int_array", "integer_array"):
                return [int(value) for value in list(param_value)]
            if param_type == "float_array":
                return [float(value) for value in list(param_value)]
            if param_type == "bool_array":
                result = []
                for value in list(param_value):
                    if isinstance(value, bool):
                        result.append(value)
                    elif isinstance(value, str):
                        lowered = value.strip().lower()
                        if lowered in ("true", "1", "yes", "on"):
                            result.append(True)
                        elif lowered in ("false", "0", "no", "off"):
                            result.append(False)
                        else:
                            raise TypeError
                    else:
                        result.append(bool(value))
                return result
        except (TypeError, ValueError) as exc:
            suffix = f" for parameter {param_name}" if param_name else ""
            raise TypeError(f"Could not cast value of type {param_type}{suffix}") from exc

        return param_value

    def _candidate_values(self) -> Dict[str, Dict[str, Any]]:
        return {
            name: {
                "type": self._native_core.get_parameter_type_from_string(param["type"]),
                "value": param["value"],
            }
            for name, param in self.params_dict.items()
        }

    def _candidate_values_with_override(self, param_name: str, param_value: Any) -> Dict[str, Dict[str, Any]]:
        candidate_values = self._candidate_values()
        candidate_values[param_name] = {
            "type": self._native_core.get_parameter_type_from_string(self.params_dict[param_name]["type"]),
            "value": param_value,
        }
        return candidate_values

    def _ensure_parameter_exists(self, param_name: str):
        if param_name not in self.params_dict:
            raise KeyError(f"Parameter {param_name} not found")

    def _evaluate_expression(self, expression: str, params_dict: Dict[str, Dict[str, Any]]) -> float:
        replaced_expression = expression
        for param_name, param in params_dict.items():
            replaced_expression = replaced_expression.replace(param_name, str(param["value"]))

        try:
            return eval(replaced_expression)
        except Exception as exc:
            raise ValueError(f"Error evaluating expression {replaced_expression}") from exc
