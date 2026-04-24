#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import threading
import time
import traceback
import yaml

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String

from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters

from iii_drone_interfaces.srv import (
    DeclareParameters,
    GetCurrentParameterFile,
    GetDeclaredParameters,
    GetParameterFiles,
    GetParameterYaml,
    LoadParameters,
    SaveParameters,
    SetCurrentParameterFileAsDefault,
    SetParameterFromGC,
    UndeclareParameters,
)
from iii_drone_configuration.parameter_handler import ParameterHandler
from iii_drone_configuration.schema_utils import (
    build_parameter_file_data,
    normalize_parameter_set_reference,
    persist_active_parameter_set_reference,
    profile_name_from_environment,
    resolve_active_parameter_file,
    resolve_default_parameter_file_name,
    resolve_parameter_set_path,
    resolve_schema_file,
    seed_runtime_configuration,
)


###############################################################################
# Helpers
###############################################################################


@dataclass
class ManagedNodeRecord:
    fq_name: str
    parameter_names: list[str] = field(default_factory=list)
    values: dict[str, object] = field(default_factory=dict)


###############################################################################
# Class
###############################################################################


class ConfigurationServer(Node):
    _RUNTIME_SNAPSHOT_PREFIX = "runtime_parameters_"

    def __init__(
        self,
        node_name: str = "configuration_server",
        namespace: str = "/configuration/configuration_server",
    ):
        super().__init__(node_name=node_name, namespace=namespace)

        self.cb_group = ReentrantCallbackGroup()
        self._profile_name = profile_name_from_environment()

        self.schema_file_path = resolve_schema_file()
        self.get_logger().info(f"Configuration server initialized with schema file: {self.schema_file_path}")
        self.parameter_handler: Optional[ParameterHandler] = None
        self.native_core: Optional[NativeConfiguratorCore] = None
        self.managed_keys: set[str] = set()
        self.server_values: dict[str, object] = {}
        self.node_registry: dict[str, ManagedNodeRecord] = {}
        self._state_lock = threading.RLock()
        self.current_parameter_file: str = ""
        self._default_parameter_file_path: Optional[Path] = None
        self._boot_server_values: dict[str, object] = {}
        self._boot_current_parameter_file: str = ""
        self._boot_default_parameter_file_path: Optional[Path] = None
        self._boot_default_snapshot_file_name: str = ""
        self._runtime_snapshot_file_name: Optional[str] = None

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
        self.managed_node_notification_subscription = None
        self.reconcile_timer = None
        self.pending_node_notifications: set[str] = set()

    def _default_snapshot_file_name(self) -> str:
        return resolve_default_parameter_file_name(self._profile_name)

    def _normalize_parameter_file_reference(self, file_name: str, *, default_subdir: str) -> str:
        return normalize_parameter_set_reference(file_name, default_subdir=default_subdir)

    def _current_parameter_file_reference_for_boot_path(self, parameter_file_path: Path) -> str:
        active_reference = self._default_snapshot_file_name()
        try:
            if resolve_parameter_set_path(self._profile_name, active_reference) == parameter_file_path:
                return active_reference
        except Exception:
            pass
        return str(parameter_file_path)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        ret = super().on_configure(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        try:
            from iii_drone_configuration._native import NativeConfiguratorCore

            seed_runtime_configuration(self._profile_name)

            self.get_logger().info(f"Loading parameter schema: {self.schema_file_path}")
            self.native_core = NativeConfiguratorCore(str(self.schema_file_path))
            self.parameter_handler = ParameterHandler.from_parameter_file(str(self.schema_file_path))
            with self._state_lock:
                self.managed_keys = set(self.native_core.schema_parameter_names())
                self.server_values = {
                    key: self.native_core.get_schema_entry(key)["default_value"] for key in sorted(self.managed_keys)
                }
                self.node_registry.clear()
                self._load_boot_parameter_file_if_available()
                self._capture_boot_configuration()
            self.get_logger().info(f"Configuration server loaded {len(self.managed_keys)} managed parameters.")
            return TransitionCallbackReturn.SUCCESS
        except ImportError as exc:
            self.get_logger().error(
                "iii_drone_configuration native bindings are not available. "
                "Build the iii_drone_configuration package before using the configuration server."
            )
            self.get_logger().error(str(exc))
        except Exception as exc:
            self.get_logger().error(
                f"Failed to configure configuration server with schema '{self.schema_file_path}': {exc}"
            )
            self.get_logger().error(traceback.format_exc())
        return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        self.declare_parameters_service = self.create_service(
            DeclareParameters, "declare_parameters", self.declare_parameters_callback, callback_group=self.cb_group
        )
        self.undeclare_parameters_service = self.create_service(
            UndeclareParameters, "undeclare_parameters", self.undeclare_parameters_callback, callback_group=self.cb_group
        )
        self.get_parameter_yaml_service = self.create_service(
            GetParameterYaml, "get_parameter_yaml", self.get_parameter_yaml_callback, callback_group=self.cb_group
        )
        self.get_declared_parameters_service = self.create_service(
            GetDeclaredParameters,
            "get_declared_parameters",
            self.get_declared_parameters_callback,
            callback_group=self.cb_group,
        )
        self.save_parameters_service = self.create_service(
            SaveParameters, "save_parameters", self.save_parameters_callback, callback_group=self.cb_group
        )
        self.get_parameter_files_service = self.create_service(
            GetParameterFiles, "get_parameter_files", self.get_parameter_files_callback, callback_group=self.cb_group
        )
        self.load_parameters_service = self.create_service(
            LoadParameters, "load_parameters", self.load_parameters_callback, callback_group=self.cb_group
        )
        self.set_parameter_from_gc_service = self.create_service(
            SetParameterFromGC,
            "set_parameter_from_gc",
            self.set_parameter_from_gc_callback,
            callback_group=self.cb_group,
        )
        self.get_current_parameter_file_service = self.create_service(
            GetCurrentParameterFile,
            "get_current_parameter_file",
            self.get_current_parameter_file_callback,
            callback_group=self.cb_group,
        )
        self.set_current_parameter_file_as_default_service = self.create_service(
            SetCurrentParameterFileAsDefault,
            "set_current_parameter_file_as_default",
            self.set_current_parameter_file_as_default_callback,
            callback_group=self.cb_group,
        )

        self.managed_node_notification_subscription = self.create_subscription(
            String,
            "/configuration/configuration_server/managed_node_available",
            self.managed_node_notification_callback,
            10,
            callback_group=self.cb_group,
        )
        self.reconcile_timer = self.create_timer(1.0, self.reconcile_nodes, callback_group=self.cb_group)
        self.reconcile_nodes()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        ret = super().on_deactivate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        for service_name in (
            "declare_parameters_service",
            "undeclare_parameters_service",
            "get_parameter_yaml_service",
            "get_declared_parameters_service",
            "save_parameters_service",
            "get_parameter_files_service",
            "load_parameters_service",
            "set_parameter_from_gc_service",
            "get_current_parameter_file_service",
            "set_current_parameter_file_as_default_service",
        ):
            service = getattr(self, service_name)
            if service is not None:
                service.destroy()
                setattr(self, service_name, None)

        if self.managed_node_notification_subscription is not None:
            self.destroy_subscription(self.managed_node_notification_subscription)
            self.managed_node_notification_subscription = None

        if self.reconcile_timer is not None:
            self.destroy_timer(self.reconcile_timer)
            self.reconcile_timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        ret = super().on_cleanup(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self.native_core = None
        self.parameter_handler = None
        with self._state_lock:
            self.managed_keys.clear()
            self.server_values.clear()
            self.node_registry.clear()
            self.pending_node_notifications.clear()
            self._default_parameter_file_path = None
            self._boot_server_values.clear()
            self._boot_current_parameter_file = ""
            self._boot_default_parameter_file_path = None
            self._boot_default_snapshot_file_name = ""
            self._runtime_snapshot_file_name = None
        return TransitionCallbackReturn.SUCCESS

    def managed_node_notification_callback(self, msg: String) -> None:
        with self._state_lock:
            self.pending_node_notifications.add(msg.data)
        self.reconcile_nodes()

    def _get_node_fq_names(self) -> list[str]:
        names = []
        for name, namespace in self.get_node_names_and_namespaces():
            fq_name = (namespace.rstrip("/") + "/" + name).replace("//", "/")
            names.append(fq_name)
        return names

    def _service_path(self, node_fq_name: str, service_name: str) -> str:
        return f"{node_fq_name.rstrip('/')}/{service_name}"

    @staticmethod
    def _consume_future_result(future) -> None:
        try:
            future.result()
        except Exception:
            pass

    def _call_client(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        try:
            future._set_executor(None)
        except Exception:
            pass
        future.add_done_callback(self._consume_future_result)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(0.01)
        future.cancel()
        return None

    def _call_list_parameters(self, node_fq_name: str) -> Optional[list[str]]:
        client = self.create_client(ListParameters, self._service_path(node_fq_name, "list_parameters"), callback_group=self.cb_group)
        if not client.wait_for_service(timeout_sec=0.2):
            self.destroy_client(client)
            return None
        request = ListParameters.Request()
        request.prefixes = []
        request.depth = 1000
        try:
            response = self._call_client(client, request, timeout_sec=0.5)
            if response is None:
                return None
            return list(response.result.names)
        finally:
            self.destroy_client(client)

    def _call_get_parameters(self, node_fq_name: str, parameter_names: list[str]) -> Optional[dict[str, object]]:
        client = self.create_client(GetParameters, self._service_path(node_fq_name, "get_parameters"), callback_group=self.cb_group)
        if not client.wait_for_service(timeout_sec=0.2):
            self.destroy_client(client)
            return None
        request = GetParameters.Request()
        request.names = parameter_names
        try:
            response = self._call_client(client, request, timeout_sec=0.5)
            if response is None:
                return None
            if len(response.values) != len(parameter_names):
                return None
            values = {}
            for name, value in zip(parameter_names, response.values):
                values[name] = rclpy.parameter.parameter_value_to_python(value)
            return values
        finally:
            self.destroy_client(client)

    def _call_set_parameter(self, node_fq_name: str, parameter_name: str, value: object) -> tuple[bool, str]:
        client = self.create_client(SetParameters, self._service_path(node_fq_name, "set_parameters"), callback_group=self.cb_group)
        if not client.wait_for_service(timeout_sec=0.5):
            self.destroy_client(client)
            return False, f"Service not available for {node_fq_name}"

        request = SetParameters.Request()
        parameter = rclpy.parameter.Parameter(name=parameter_name, value=value)
        request.parameters = [parameter.to_parameter_msg()]

        try:
            response = self._call_client(client, request, timeout_sec=0.5)
            if response is None:
                return False, f"Timed out waiting for {node_fq_name}"
            if not response.results:
                return False, f"No result returned by {node_fq_name}"
            result = response.results[0]
            return bool(result.successful), str(result.reason)
        finally:
            self.destroy_client(client)

    def reconcile_nodes(self) -> None:
        with self._state_lock:
            if self.parameter_handler is None:
                return

            discovered_fq_names = set(self._get_node_fq_names())
            discovered_fq_names.discard(self.get_fully_qualified_name())

            for node_fq_name in sorted(discovered_fq_names):
                parameter_names = self._call_list_parameters(node_fq_name)
                if parameter_names is None:
                    continue

                managed_parameter_names = sorted(set(parameter_names).intersection(self.managed_keys))
                if not managed_parameter_names:
                    continue

                values = self._call_get_parameters(node_fq_name, managed_parameter_names)
                if values is None:
                    continue

                authoritative_values = {
                    parameter_name: self.server_values[parameter_name]
                    for parameter_name in managed_parameter_names
                }

                for parameter_name in managed_parameter_names:
                    authoritative_value = authoritative_values[parameter_name]
                    if values.get(parameter_name) == authoritative_value:
                        continue

                    success, message = self._call_set_parameter(
                        node_fq_name,
                        parameter_name,
                        authoritative_value,
                    )
                    if not success:
                        self.get_logger().warn(
                            f"Failed to synchronize newly discovered node '{node_fq_name}' "
                            f"parameter '{parameter_name}': {message}"
                        )
                        continue

                    values[parameter_name] = authoritative_value

                self.node_registry[node_fq_name] = ManagedNodeRecord(
                    fq_name=node_fq_name,
                    parameter_names=managed_parameter_names,
                    values=dict(authoritative_values),
                )

            offline_nodes = set(self.node_registry.keys()) - discovered_fq_names
            for node_fq_name in offline_nodes:
                del self.node_registry[node_fq_name]

            self.pending_node_notifications.clear()

    def _group_nodes_for_parameter(self, parameter_name: str) -> list[str]:
        with self._state_lock:
            return [
                node_fq_name
                for node_fq_name, record in self.node_registry.items()
                if parameter_name in record.parameter_names
            ]

    def _parameter_type(self, parameter_name: str) -> int:
        if self.native_core is None:
            raise RuntimeError("Configuration server native core is not initialized")
        return int(self.native_core.get_schema_entry(parameter_name)["parameter_type"])

    def _parameter_is_constant(self, parameter_name: str) -> bool:
        if self.native_core is None:
            raise RuntimeError("Configuration server native core is not initialized")
        return bool(self.native_core.get_schema_entry(parameter_name).get("constant", False))

    def _candidate_parameter_map(self, values: dict[str, object]) -> dict[str, dict[str, object]]:
        return {
            name: {
                "type": self._parameter_type(name),
                "value": value,
            }
            for name, value in values.items()
        }

    def _rollback_parameter_updates(
        self,
        parameter_name: str,
        previous_values: dict[str, object],
        updated_nodes: list[str],
    ) -> None:
        with self._state_lock:
            for node_fq_name in reversed(updated_nodes):
                rollback_value = previous_values.get(node_fq_name)
                updated_record = self.node_registry.get(node_fq_name)
                if rollback_value is None or updated_record is None:
                    continue
                self._call_set_parameter(node_fq_name, parameter_name, rollback_value)
                updated_record.values[parameter_name] = rollback_value

    def _apply_shared_parameter_update(
        self,
        parameter_name: str,
        value: object,
        require_targets: bool = True,
        allow_constant_override: bool = False,
    ) -> tuple[bool, str]:
        with self._state_lock:
            if self.parameter_handler is None or self.native_core is None:
                return False, "Configuration server is not configured"

            if self.server_values.get(parameter_name) == value:
                return True, ""

            try:
                candidate_values = dict(self.server_values)
                candidate_values[parameter_name] = value
                self.native_core.validate_parameter_value(
                    parameter_name,
                    value,
                    self._parameter_type(parameter_name),
                    self._candidate_parameter_map(candidate_values),
                    allow_constant_override,
                )
                self.native_core.validate_parameter_map(
                    self._candidate_parameter_map(candidate_values),
                    True,
                )
            except Exception as exc:
                return False, str(exc)

            target_nodes = [
                node_fq_name
                for node_fq_name in self._group_nodes_for_parameter(parameter_name)
                if node_fq_name in self.node_registry
            ]
            if require_targets and not target_nodes:
                self.reconcile_nodes()
                target_nodes = [
                    node_fq_name
                    for node_fq_name in self._group_nodes_for_parameter(parameter_name)
                    if node_fq_name in self.node_registry
                ]
            if require_targets and not target_nodes:
                return False, f"No running nodes currently declare '{parameter_name}'"

            previous_values = {
                node_fq_name: self.node_registry[node_fq_name].values.get(parameter_name)
                for node_fq_name in target_nodes
            }
            successfully_updated_nodes: list[str] = []

            for node_fq_name in target_nodes:
                success, message = self._call_set_parameter(node_fq_name, parameter_name, value)
                if not success:
                    self._rollback_parameter_updates(parameter_name, previous_values, successfully_updated_nodes)
                    return False, f"{node_fq_name} rejected update: {message}"

                successfully_updated_nodes.append(node_fq_name)
                updated_record = self.node_registry.get(node_fq_name)
                if updated_record is not None:
                    updated_record.values[parameter_name] = value

            for node_fq_name in target_nodes:
                readback = self._call_get_parameters(node_fq_name, [parameter_name])
                if readback is None:
                    self._rollback_parameter_updates(parameter_name, previous_values, successfully_updated_nodes)
                    return False, f"{node_fq_name} did not confirm the applied value"

                applied_value = readback.get(parameter_name)
                if applied_value != value:
                    self._rollback_parameter_updates(parameter_name, previous_values, successfully_updated_nodes)
                    return False, (
                        f"{node_fq_name} reported {applied_value!r} after applying "
                        f"{parameter_name}={value!r}"
                    )

            self.parameter_handler.set_param(parameter_name, value, parameter_initialized=False, force_constant=True)
            self.server_values[parameter_name] = value
            return True, ""

    def _serialize_current_yaml(self) -> str:
        with self._state_lock:
            if self.parameter_handler is None:
                return ""
            return self.parameter_handler.get_parameters_yaml_string()

    def _serialize_declared_parameters(self) -> str:
        with self._state_lock:
            grouped = {
                parameter_name: sorted(self._group_nodes_for_parameter(parameter_name))
                for parameter_name in sorted(self.server_values.keys())
                if self._group_nodes_for_parameter(parameter_name)
            }
            return yaml.dump(grouped, sort_keys=False)

    def _load_snapshot_values(self, file_name: str) -> dict[str, object]:
        reference = self._normalize_parameter_file_reference(file_name, default_subdir="snapshots")
        path = resolve_parameter_set_path(self._profile_name, reference)
        with open(path, "r") as file:
            data = yaml.safe_load(file) or {}
        ros_parameters = data.get("/**", {}).get("ros__parameters", {})
        return {name: value for name, value in ros_parameters.items() if name in self.managed_keys}

    def _load_parameter_values_from_path(self, path: Path) -> dict[str, object]:
        with open(path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
        ros_parameters = data.get("/**", {}).get("ros__parameters", {})
        return {name: value for name, value in ros_parameters.items() if name in self.managed_keys}

    def _write_snapshot_file(self, file_name: str, overwrite: bool) -> Path:
        with self._state_lock:
            reference = self._normalize_parameter_file_reference(file_name, default_subdir="snapshots")
            path = resolve_parameter_set_path(self._profile_name, reference)
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists() and not overwrite:
                raise FileExistsError(f"Snapshot file '{file_name}' already exists")

            for parameter_name, node_fq_names in (
                (name, self._group_nodes_for_parameter(name)) for name in sorted(self.server_values.keys())
            ):
                for node_fq_name in node_fq_names:
                    node_value = self.node_registry[node_fq_name].values.get(parameter_name)
                    if node_value != self.server_values[parameter_name]:
                        raise RuntimeError(
                            f"Refusing to save inconsistent parameter '{parameter_name}': "
                            f"{node_fq_name} has {node_value!r}, server has {self.server_values[parameter_name]!r}"
                        )

            save_parameter_file = build_parameter_file_data(self.server_values)
            with open(path, "w", encoding="utf-8") as file:
                yaml.safe_dump(save_parameter_file, file, sort_keys=False)
            return path

    def _set_default_snapshot_file(self, file_name: str) -> None:
        normalized = self._normalize_parameter_file_reference(file_name, default_subdir="tracked")
        persist_active_parameter_set_reference(self._profile_name, normalized)
        self._default_parameter_file_path = resolve_parameter_set_path(self._profile_name, normalized)

    def _capture_boot_configuration(self) -> None:
        self._boot_server_values = dict(self.server_values)
        self._boot_current_parameter_file = self.current_parameter_file
        self._boot_default_parameter_file_path = self._default_parameter_file_path
        self._boot_default_snapshot_file_name = self._default_snapshot_file_name()
        self._runtime_snapshot_file_name = None

    def _new_runtime_snapshot_file_name(self) -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return f"{self._RUNTIME_SNAPSHOT_PREFIX}{timestamp}.yaml"

    def _delete_runtime_snapshot_file(self, file_name: Optional[str]) -> None:
        if not file_name or not Path(file_name).name.startswith(self._RUNTIME_SNAPSHOT_PREFIX):
            return

        try:
            resolve_parameter_set_path(
                self._profile_name,
                self._normalize_parameter_file_reference(file_name, default_subdir="snapshots"),
            ).unlink()
        except FileNotFoundError:
            pass

    def _restore_boot_default_parameter_file(self) -> None:
        if self._boot_default_snapshot_file_name:
            persist_active_parameter_set_reference(self._profile_name, self._boot_default_snapshot_file_name)
        self.current_parameter_file = self._boot_current_parameter_file
        self._default_parameter_file_path = self._boot_default_parameter_file_path

    def _mark_current_configuration_as_default_baseline(self, file_name: str, path: Path) -> None:
        self._boot_server_values = dict(self.server_values)
        self._boot_current_parameter_file = file_name
        self._boot_default_parameter_file_path = path
        self._boot_default_snapshot_file_name = file_name

    def _sync_runtime_parameter_file(self) -> str:
        if self.server_values == self._boot_server_values:
            removed_file_name = self._runtime_snapshot_file_name
            self._runtime_snapshot_file_name = None
            self._restore_boot_default_parameter_file()
            self._delete_runtime_snapshot_file(removed_file_name)
            if removed_file_name:
                return "Runtime parameter snapshot removed; restored boot default parameter file"
            return ""

        previous_runtime_file_name = self._runtime_snapshot_file_name
        file_name = self._new_runtime_snapshot_file_name()
        reference = self._normalize_parameter_file_reference(file_name, default_subdir="snapshots")
        while resolve_parameter_set_path(self._profile_name, reference).exists():
            file_name = self._new_runtime_snapshot_file_name()
            reference = self._normalize_parameter_file_reference(file_name, default_subdir="snapshots")

        self._write_snapshot_file(reference, overwrite=False)
        try:
            self._set_default_snapshot_file(reference)
        except Exception:
            self._delete_runtime_snapshot_file(reference)
            raise

        self.current_parameter_file = reference
        self._runtime_snapshot_file_name = reference

        if previous_runtime_file_name != reference:
            self._delete_runtime_snapshot_file(previous_runtime_file_name)

        return f"Runtime parameter snapshot updated: {reference}"

    def _load_boot_parameter_file_if_available(self) -> None:
        parameter_file_path = resolve_active_parameter_file(self._profile_name)
        self.current_parameter_file = self._current_parameter_file_reference_for_boot_path(parameter_file_path)
        self._default_parameter_file_path = parameter_file_path
        if not parameter_file_path.exists():
            return

        values = self._load_parameter_values_from_path(parameter_file_path)
        if self.native_core is None:
            raise RuntimeError("Configuration server native core is not initialized")

        self.native_core.validate_parameter_map(
            self._candidate_parameter_map(values),
            True,
        )

        for parameter_name, value in values.items():
            self.parameter_handler.set_param(parameter_name, value, parameter_initialized=False, force_constant=True)
            self.server_values[parameter_name] = value

    ############################################################################
    # Compatibility services
    ############################################################################

    def declare_parameters_callback(self, request, response):
        response.succeeded = False
        response.message = "Remote parameter declaration is no longer supported"
        response.values = []
        return response

    def undeclare_parameters_callback(self, request, response):
        response.succeeded = False
        response.message = "Remote parameter undeclaration is no longer supported"
        return response

    def get_parameter_yaml_callback(self, request, response):
        response.yaml = self._serialize_current_yaml()
        return response

    def get_declared_parameters_callback(self, request, response):
        response.declared_parameters_yaml = self._serialize_declared_parameters()
        return response

    def save_parameters_callback(self, request, response):
        with self._state_lock:
            file_name = request.file or datetime.now().strftime("snapshot_%Y%m%d_%H%M%S.yaml")
            file_reference = self._normalize_parameter_file_reference(file_name, default_subdir="snapshots")
            try:
                self._write_snapshot_file(file_reference, request.overwrite)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.file = ""
                return response

            self.current_parameter_file = file_reference
            if request.set_as_default:
                previous_runtime_file_name = self._runtime_snapshot_file_name
                saved_path = resolve_parameter_set_path(self._profile_name, file_reference)
                self._set_default_snapshot_file(file_reference)
                self._mark_current_configuration_as_default_baseline(file_reference, saved_path)
                self._runtime_snapshot_file_name = None
                if previous_runtime_file_name != file_reference:
                    self._delete_runtime_snapshot_file(previous_runtime_file_name)

            response.success = True
            response.message = ""
            response.file = file_reference
            return response

    def get_parameter_files_callback(self, request, response):
        parameter_set_root = resolve_parameter_set_path(self._profile_name, "tracked/default.yaml").parents[1]
        response.parameter_files = sorted(
            str(file.relative_to(parameter_set_root))
            for file in parameter_set_root.rglob("*.yaml")
            if file.is_file()
        )
        return response

    def load_parameters_callback(self, request, response):
        with self._state_lock:
            try:
                file_reference = self._normalize_parameter_file_reference(request.file, default_subdir="snapshots")
                values = self._load_snapshot_values(file_reference)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            applied: list[tuple[str, object]] = []
            for parameter_name, value in values.items():
                old_value = self.server_values.get(parameter_name)
                success, message = self._apply_shared_parameter_update(
                    parameter_name,
                    value,
                    require_targets=False,
                )
                if not success:
                    for applied_name, old_value in reversed(applied):
                        if old_value is not None:
                            self._apply_shared_parameter_update(
                                applied_name,
                                old_value,
                                require_targets=False,
                            )
                    response.success = False
                    response.message = message
                    return response
                applied.append((parameter_name, old_value))

            self.current_parameter_file = file_reference
            if request.set_as_default:
                previous_runtime_file_name = self._runtime_snapshot_file_name
                saved_path = self._write_snapshot_file(file_reference, overwrite=True)
                self._set_default_snapshot_file(file_reference)
                self._mark_current_configuration_as_default_baseline(file_reference, saved_path)
                self._runtime_snapshot_file_name = None
                if previous_runtime_file_name != file_reference:
                    self._delete_runtime_snapshot_file(previous_runtime_file_name)

            response.success = True
            response.message = ""
            return response

    def set_parameter_from_gc_callback(self, request, response):
        with self._state_lock:
            if self.parameter_handler is None or self.native_core is None:
                response.success = False
                response.message = "Configuration server is not configured"
                return response

            try:
                cast_value = self.parameter_handler.cast_param_value(request.parameter_name, request.parameter_string_value)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            if self._parameter_is_constant(request.parameter_name):
                response.success = False
                response.message = (
                    f"'{request.parameter_name}' is boot-only and cannot be changed live. "
                    "Stop the system, update the parameter set, then start again."
                )
                return response

            previous_values = dict(self.server_values)
            success, message = self._apply_shared_parameter_update(request.parameter_name, cast_value, require_targets=True)
            if not success:
                response.success = False
                response.message = message
                return response

            if self.server_values != previous_values:
                try:
                    message = self._sync_runtime_parameter_file()
                except Exception as exc:
                    rollback_success, rollback_message = self._apply_shared_parameter_update(
                        request.parameter_name,
                        previous_values[request.parameter_name],
                        require_targets=True,
                        allow_constant_override=True,
                    )
                    response.success = False
                    if rollback_success:
                        response.message = (
                            "Live parameter update was rolled back because persistence failed: "
                            f"{exc}"
                        )
                    else:
                        response.message = (
                            "Persistence failed after the live parameter update and rollback also failed: "
                            f"{exc}; rollback error: {rollback_message}"
                        )
                    return response

            response.success = True
            response.message = message
            return response

    def get_current_parameter_file_callback(self, request, response):
        with self._state_lock:
            response.current_parameter_file = self.current_parameter_file
            response.default_parameter_file = self._default_snapshot_file_name()
            return response

    def set_current_parameter_file_as_default_callback(self, request, response):
        with self._state_lock:
            if not self.current_parameter_file:
                response.success = False
                response.message = "No current parameter file selected"
                return response

            try:
                previous_runtime_file_name = self._runtime_snapshot_file_name
                saved_path = self._write_snapshot_file(self.current_parameter_file, overwrite=True)
                self._set_default_snapshot_file(self.current_parameter_file)
                self._mark_current_configuration_as_default_baseline(self.current_parameter_file, saved_path)
                self._runtime_snapshot_file_name = None
                if previous_runtime_file_name != self.current_parameter_file:
                    self._delete_runtime_snapshot_file(previous_runtime_file_name)
                response.success = True
                response.message = ""
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurationServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Configuration server received shutdown signal.")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
