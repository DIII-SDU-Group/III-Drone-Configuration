###############################################################################
# Imports
###############################################################################

from __future__ import annotations

from threading import Lock
from typing import Callable, Dict, List, Optional

import rclpy
from rclpy import lifecycle
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import ParameterEvent, SetParametersResult
from std_msgs.msg import String

from .configuration import Configuration, ConfigurationEntry
from .schema_utils import resolve_schema_file


###############################################################################
# Class
###############################################################################


class Configurator:
    @staticmethod
    def _parameter_type_value(parameter_type: rclpy.parameter.Parameter.Type | int) -> int:
        return parameter_type.value if hasattr(parameter_type, "value") else int(parameter_type)

    def __init__(
        self,
        node: lifecycle.Node | rclpy.node.Node,
        after_parameter_change_callback: Optional[Callable[[rclpy.parameter.Parameter], None]] = None,
        qos: QoSProfile = QoSProfile(depth=10),
    ):
        self.node = node
        self.after_parameter_change_callback = after_parameter_change_callback
        self.qos = qos
        self.configurations: List[Configuration] = []
        self.parameters_mutex = Lock()
        self._managed_parameter_names: List[str] = []
        self.is_cleaned_up = False
        self._managed_node_announced = False

        self._declare_support_parameter_if_missing("parameters_path_postfix", "parameters/")
        self._declare_support_parameter_if_missing("default_parameter_file", "parameter_manifest.yaml")
        self._declare_support_parameter_if_missing("sim_parameter_file", "parameter_manifest.yaml")

        self.schema_file_path = resolve_schema_file(
            parameters_path_postfix=str(self.node.get_parameter("parameters_path_postfix").value),
            default_parameter_file=str(self.node.get_parameter("default_parameter_file").value),
            sim_parameter_file=str(self.node.get_parameter("sim_parameter_file").value),
        )
        try:
            from ._native import NativeConfiguratorCore
        except ImportError as exc:
            raise ImportError(
                "iii_drone_configuration native bindings are not available. "
                "Build the iii_drone_configuration package before using the Python Configurator."
            ) from exc
        self._native_core = NativeConfiguratorCore(str(self.schema_file_path))

        self.parameter_events_subscriber = self.node.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            self.qos,
        )
        self._on_set_callback_handle = self.node.add_on_set_parameters_callback(self.on_set_parameters_callback)
        self._announcement_publisher = self.node.create_publisher(
            String,
            "/configuration/configuration_server/managed_node_available",
            10,
        )

    def _declare_support_parameter_if_missing(self, name: str, default_value: str) -> None:
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default_value)

    def _announce_managed_node(self) -> None:
        if not self._managed_parameter_names:
            return
        msg = String()
        msg.data = self.node.get_fully_qualified_name()
        self._announcement_publisher.publish(msg)
        self._managed_node_announced = True

    def _current_managed_parameter_values(self) -> Dict[str, object]:
        values: Dict[str, object] = {}
        for parameter_name in self._managed_parameter_names:
            values[parameter_name] = self.node.get_parameter(parameter_name).value
        return values

    def _candidate_parameter_map(
        self,
        candidate_values: Dict[str, object],
        candidate_types: Dict[str, int],
    ) -> Dict[str, Dict[str, object]]:
        parameter_map = {}
        for name in self._native_core.schema_parameter_names():
            entry = self._native_core.get_schema_entry(name)
            parameter_map[name] = {
                "type": entry["parameter_type"],
                "value": entry["default_value"],
            }

        for name, value in candidate_values.items():
            parameter_map[name] = {
                "type": candidate_types[name],
                "value": value,
            }

        return parameter_map

    def cleanup(self):
        if self.is_cleaned_up:
            return

        if self.parameter_events_subscriber is not None:
            self.node.destroy_subscription(self.parameter_events_subscriber)
            self.parameter_events_subscriber = None

        if self._announcement_publisher is not None:
            self.node.destroy_publisher(self._announcement_publisher)
            self._announcement_publisher = None

        self.configurations.clear()
        self._managed_parameter_names.clear()
        self._native_core = None
        self.node = None
        self.is_cleaned_up = True

    def get_parameter(self, parameter_full_name: str) -> rclpy.parameter.Parameter:
        return self.get_parameters([parameter_full_name])[0]

    def get_parameters(self, parameter_full_names: List[str]) -> List[rclpy.parameter.Parameter]:
        with self.parameters_mutex:
            return [self.node.get_parameter(parameter_full_name) for parameter_full_name in parameter_full_names]

    def get_configuration(self, configuration_name: str) -> Configuration:
        for configuration in self.configurations:
            if configuration.name == configuration_name:
                return configuration
        raise RuntimeError(f"Configurator.get_configuration: Configuration '{configuration_name}' not found")

    def create_configuration(self, configuration_name: str, entries: List[ConfigurationEntry]) -> Configuration:
        configuration = Configuration(
            native_configuration=self._native_core.create_configuration(
                configuration_name,
                [(entry.full_name, self._parameter_type_value(entry.parameter_type)) for entry in entries],
            ),
            parameter_getter=lambda full_name: self.node.get_parameter(full_name),
        )
        self.configurations.append(configuration)
        return configuration

    @staticmethod
    def get_parameter_type_string(T: rclpy.parameter.Parameter.Type) -> str:
        from ._native import NativeConfiguratorCore
        return NativeConfiguratorCore.get_parameter_type_string(Configurator._parameter_type_value(T))

    @staticmethod
    def get_parameter_type_from_string(parameter_type_string: str) -> rclpy.parameter.Parameter.Type:
        from ._native import NativeConfiguratorCore
        return rclpy.parameter.Parameter.Type(NativeConfiguratorCore.get_parameter_type_from_string(parameter_type_string))

    def print_parameters(self):
        for param_name in self._managed_parameter_names:
            param = self.node.get_parameter(param_name)
            self.node.get_logger().info(f"Parameter: {param.name} = {param.value}")

    def print_configurations(self):
        for configuration in self.configurations:
            self.node.get_logger().info(f"Configuration: {configuration.name}")

    def validate(self):
        current_values = self._current_managed_parameter_values()
        if not current_values:
            return
        current_types = {
            name: self._parameter_type_value(self.node.get_parameter(name).type_)
            for name in self._managed_parameter_names
            if self.node.has_parameter(name)
        }
        self._native_core.validate_parameter_map(
            self._candidate_parameter_map(current_values, current_types),
            True,
        )
        if not self._managed_node_announced:
            self._announce_managed_node()

    def declare_parameter(
        self,
        parameter_full_name: str,
        parameter_type: rclpy.parameter.Parameter.Type,
    ) -> None:
        self.declare_parameters([parameter_full_name], [parameter_type])

    def declare_parameters(
        self,
        parameter_full_names: List[str],
        parameter_types: List[rclpy.parameter.Parameter.Type],
    ) -> None:
        if len(parameter_full_names) != len(parameter_types):
            raise RuntimeError("Configurator.declare_parameters: names/types size mismatch")

        with self.parameters_mutex:
            default_values = self._native_core.declare_parameters(
                parameter_full_names,
                [self._parameter_type_value(parameter_type) for parameter_type in parameter_types],
            )

            for name, default_value in zip(parameter_full_names, default_values):
                if not self.node.has_parameter(name):
                    self.node.declare_parameter(name, default_value)

                if name not in self._managed_parameter_names:
                    self._managed_parameter_names.append(name)

    def parameter_event_callback(self, parameter_event: ParameterEvent):
        if parameter_event.node != self.node.get_fully_qualified_name():
            return

        if self.after_parameter_change_callback is None:
            return

        for parameter_msg in parameter_event.changed_parameters:
            if parameter_msg.name in self._managed_parameter_names:
                self.after_parameter_change_callback(rclpy.parameter.Parameter.from_parameter_msg(parameter_msg))

    def on_set_parameters_callback(
        self,
        parameters: List[rclpy.parameter.Parameter],
    ) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        candidate_values = self._current_managed_parameter_values()
        candidate_types = {
            name: self._parameter_type_value(self.node.get_parameter(name).type_)
            for name in self._managed_parameter_names
            if self.node.has_parameter(name)
        }

        for parameter in parameters:
            if parameter.name not in self._managed_parameter_names:
                continue

            try:
                candidate_values[parameter.name] = parameter.value
                candidate_types[parameter.name] = self._parameter_type_value(parameter.type_)
                self._native_core.validate_parameter_value(
                    parameter.name,
                    parameter.value,
                    self._parameter_type_value(parameter.type_),
                    self._candidate_parameter_map(candidate_values, candidate_types),
                    False,
                )
            except Exception as exc:
                result.successful = False
                result.reason = str(exc)
                return result

        try:
            self._native_core.validate_parameter_map(
                self._candidate_parameter_map(candidate_values, candidate_types),
                True,
            )
        except Exception as exc:
            result.successful = False
            result.reason = str(exc)
            return result

        return result
