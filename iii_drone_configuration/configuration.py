from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Any, Callable
import rclpy

@dataclass
class ConfigurationEntry:
    full_name: str
    parameter_type: rclpy.parameter.Parameter.Type


class Configuration:
    def __init__(
        self,
        native_configuration: Any,
        parameter_getter: Callable[[str], rclpy.parameter.Parameter],
    ):
        self._native_configuration = native_configuration
        self._parameter_getter = parameter_getter
        self._lock = Lock()

    def get_parameter(self, parameter_full_name: str) -> rclpy.parameter.Parameter:
        with self._lock:
            if self._native_configuration.has_parameter(parameter_full_name):
                return self._parameter_getter(parameter_full_name)

        raise RuntimeError(f"Parameter {parameter_full_name} does not exist.")

    def has_parameter(self, parameter_full_name: str) -> bool:
        with self._lock:
            return self._native_configuration.has_parameter(parameter_full_name)

    @property
    def name(self) -> str:
        return self._native_configuration.name
