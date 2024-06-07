###############################################################################
# Imports
###############################################################################

from threading import Lock
from dataclasses import dataclass

import rclpy

###############################################################################
# Class
###############################################################################

@dataclass
class ParameterBundleEntry:
    parameter: rclpy.parameter.Parameter
    simple_nmae: str
    remap_name: str
    update: bool

class ParameterBundle:
    def __init__(
        self, 
        name: str, 
        parameter_bundle_entries: list[ParameterBundleEntry]):
        """
        Constructor. Takes a list of parameters bundle entries.

        :param name: Name of the parameter bundle
        :param parameter_bundle_entries: List of parameters bundle entries
        """
        self._name = name
        self._parameter_bundle_entries = parameter_bundle_entries
        self._lock = Lock()

    def get_parameter(
        self, 
        parameter_remap_name: str
    ) -> rclpy.parameter.Parameter:
        """
        Get a parameter.

        :param parameter_remap_name: Name of the parameter

        :return: The parameter

        :raises RuntimeError: if the parameter does not exist.
        """
        with self._lock:
            for entry in self._parameter_bundle_entries:
                if entry.remap_name == parameter_remap_name:
                    return entry.parameter
                
            raise RuntimeError(f"Parameter {parameter_remap_name} does not exist.")

    def set_parameter(
        self, 
        parameter_simple_name: str, 
        parameter: rclpy.parameter.Parameter
    ):
        """
        Sets a parameter.

        :param parameter_simple_name: Name of the parameter
        :param parameter: The parameter

        :raises RuntimeError: if the parameter does not exist or if it is not marked for updates.
        """
        with self._lock:
            found = False
            for entry in self._parameter_bundle_entries:
                if entry.simple_name == parameter_simple_name:
                    if not entry.get("updatable", False):
                        raise RuntimeError(f"Parameter {parameter_simple_name} is not marked for updates.")
                    if not entry.parameter.type_ == parameter.type_:
                        raise RuntimeError(f"Parameter {parameter_simple_name} has a different type.")
                    entry.parameter = parameter
                    return
                
        raise RuntimeError(f"Parameter {parameter_simple_name} does not exist.")

    def has_parameter(
        self, 
        parameter_simple_name: str
    ) -> bool:
        """
        Whether a parameter is contained in the bundle.

        :param parameter_simple_name: Name of the parameter

        :return: True if the parameter is contained in the bundle, false otherwise
        """
        with self._lock:
            return any(entry.simple_name == parameter_simple_name for entry in self._parameter_bundle_entries)

    def has_updatable_parameter(
        self, 
        parameter_simple_name: str
    ) -> bool:
        """
        Whether a parameter is marked for updates.

        :param parameter_simple_name: Name of the parameter

        :return: True if the parameter is contained and is marked for updates, false otherwise
        """
        with self._lock:
            for entry in self._parameter_bundle_entries:
                if entry.simple_name == parameter_simple_name:
                    return entry.update
                
        return False

    @property
    def name(self) -> str:
        """
        Name getter.
        """
        return self._name