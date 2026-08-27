#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
from typing import Mapping, Optional, Sequence, TYPE_CHECKING

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
    ActivatePendingBootParameters,
    ApplyConfigurationTransaction,
    DeleteParameterFile,
    DeclareParameters,
    EnsureConfigurationSession,
    GetConfigurationJournal,
    GetCurrentParameterFile,
    GetConfigurationSession,
    GetDeclaredParameters,
    GetParameterFiles,
    GetParameterFile,
    GetParameterYaml,
    GetPendingBootParameters,
    LoadParameters,
    SaveParameters,
    SetCurrentParameterFileAsDefault,
    SetBootParameter,
    SetParameterFromGC,
    UndeclareParameters,
)
from iii_drone_configuration.installed_contracts import (
    load_installed_contract,
    resolve_installed_contract_root,
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
    save_parameter_file,
)
from iii_drone_configuration.tuning import (
    TransactionPlan,
    TuningError,
    TuningSessionStore,
)

if TYPE_CHECKING:
    from iii_drone_configuration._native import NativeConfiguratorCore

###############################################################################
# Helpers
###############################################################################


@dataclass
class ManagedNodeRecord:
    fq_name: str
    parameter_names: list[str] = field(default_factory=list)
    values: dict[str, object] = field(default_factory=dict)
    last_seen_monotonic: float = 0.0
    offline_since_monotonic: Optional[float] = None


###############################################################################
# Class
###############################################################################


class ConfigurationServer(Node):
    _RUNTIME_SNAPSHOT_PREFIX = "runtime_parameters_"
    _OFFLINE_PRUNE_GRACE_SEC = 10.0
    _PARAMETER_SERVICE_TIMEOUT_SEC = 2.0
    _PARAMETER_READBACK_ATTEMPTS = 3
    _PARAMETER_READBACK_RETRY_SEC = 0.1

    def __init__(
        self,
        node_name: str = "configuration_server",
        namespace: str = "/configuration/configuration_server",
    ):
        super().__init__(node_name=node_name, namespace=namespace)

        self.cb_group = ReentrantCallbackGroup()
        self._profile_name = profile_name_from_environment()

        self.schema_file_path = resolve_schema_file()
        self.get_logger().info(
            f"Configuration server initialized with schema file: {self.schema_file_path}"
        )
        self.parameter_handler: Optional[ParameterHandler] = None
        self.native_core: Optional[NativeConfiguratorCore] = None
        self.managed_keys: set[str] = set()
        self.server_values: dict[str, object] = {}
        self.node_registry: dict[str, ManagedNodeRecord] = {}
        self._state_lock = threading.RLock()
        self._reconcile_lock = threading.Lock()
        self.current_parameter_file: str = ""
        self._default_parameter_file_path: Optional[Path] = None
        self._boot_server_values: dict[str, object] = {}
        self._boot_current_parameter_file: str = ""
        self._boot_default_parameter_file_path: Optional[Path] = None
        self._boot_default_snapshot_file_name: str = ""
        self._runtime_snapshot_file_name: Optional[str] = None
        self._pending_boot_values: dict[str, object] = {}
        self._tuning_store: Optional[TuningSessionStore] = None

        self.declare_parameters_service: Optional[Service] = None
        self.undeclare_parameters_service: Optional[Service] = None
        self.get_parameter_yaml_service: Optional[Service] = None
        self.get_declared_parameters_service: Optional[Service] = None
        self.save_parameters_service: Optional[Service] = None
        self.get_parameter_files_service: Optional[Service] = None
        self.load_parameters_service: Optional[Service] = None
        self.set_parameter_from_gc_service: Optional[Service] = None
        self.apply_configuration_transaction_service: Optional[Service] = None
        self.get_configuration_session_service: Optional[Service] = None
        self.ensure_configuration_session_service: Optional[Service] = None
        self.get_configuration_journal_service: Optional[Service] = None
        self.get_parameter_file_service: Optional[Service] = None
        self.delete_parameter_file_service: Optional[Service] = None
        self.set_boot_parameter_service: Optional[Service] = None
        self.get_pending_boot_parameters_service: Optional[Service] = None
        self.activate_pending_boot_parameters_service: Optional[Service] = None
        self.get_current_parameter_file_service: Optional[Service] = None
        self.set_current_parameter_file_as_default_service: Optional[Service] = None
        self.managed_node_notification_subscription = None
        self.reconcile_timer = None
        self.pending_node_notifications: set[str] = set()

    def _default_snapshot_file_name(self) -> str:
        return resolve_default_parameter_file_name(self._profile_name)

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    def _tuning_state_root(self) -> Path:
        explicit = os.environ.get("III_TUNING_STATE_ROOT")
        if explicit:
            return Path(os.path.expanduser(explicit)).absolute()
        workspace = os.environ.get("WORKSPACE_DIR")
        if self._profile_name == "sim" and workspace:
            return Path(workspace).expanduser().absolute() / ".iii/tuning"
        config_base = Path(
            os.path.expanduser(os.environ.get("CONFIG_BASE_DIR", "~/.config"))
        ).absolute()
        return config_base / ".iii/tuning"

    def _configuration_manifest_id(self) -> str:
        explicit_schema = os.environ.get("III_DRONE_SCHEMA_FILE")
        if explicit_schema:
            return hashlib.sha256(Path(explicit_schema).read_bytes()).hexdigest()
        return load_installed_contract(
            resolve_installed_contract_root()
        ).contract.manifest_id

    def _initialize_tuning_store(self) -> None:
        manifest_id = self._configuration_manifest_id()
        release_id = (
            os.environ.get("III_ACTIVE_RELEASE_ID")
            or os.environ.get("III_WORKSPACE_RELEASE_ID")
            or manifest_id
        )
        workspace_id = os.environ.get("III_WORKSPACE_RELEASE_ID") or release_id
        target_id = os.environ.get("III_LOGICAL_TARGET") or (
            "sim" if self._profile_name == "sim" else "drone"
        )
        self._tuning_store = TuningSessionStore(
            root=self._tuning_state_root(),
            target_id=target_id,
            runtime_profile=self._profile_name,
            release_id=release_id,
            workspace_id=workspace_id,
            manifest_id=manifest_id,
            now=self._utc_now,
        )
        status = self._tuning_store.status()
        if status["session_id"] is None:
            return
        self._pending_boot_values = dict(status["pending_boot_values"])
        if status["divergent"]:
            # A failed compensation may have left both the living YAML and this
            # process at mixed values. Re-establish the prior durable authority;
            # a later full-graph pass must still prove exact fresh readbacks
            # before the fault is cleared.
            for name, value in status["active_values"].items():
                if name not in self.managed_keys:
                    continue
                self.server_values[name] = value
                if self.parameter_handler is not None:
                    self.parameter_handler.set_param(
                        name,
                        value,
                        parameter_initialized=False,
                        force_constant=True,
                    )
        # Persisted boot values may already be present in the active YAML while
        # the running graph still reports the prior active value.  The journal,
        # not the file alone, distinguishes those states.
        for name in self._pending_boot_values:
            if name in status["active_values"]:
                value = status["active_values"][name]
                self.server_values[name] = value
                if self.parameter_handler is not None:
                    self.parameter_handler.set_param(
                        name,
                        value,
                        parameter_initialized=False,
                        force_constant=True,
                    )

    def _normalize_parameter_file_reference(
        self, file_name: str, *, default_subdir: str
    ) -> str:
        return normalize_parameter_set_reference(
            file_name, default_subdir=default_subdir
        )

    def _current_parameter_file_reference_for_boot_path(
        self, parameter_file_path: Path
    ) -> str:
        active_reference = self._default_snapshot_file_name()
        try:
            if (
                resolve_parameter_set_path(self._profile_name, active_reference)
                == parameter_file_path
            ):
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
            self.parameter_handler = ParameterHandler.from_parameter_file(
                str(self.schema_file_path)
            )
            with self._state_lock:
                self.managed_keys = set(self.native_core.schema_parameter_names())
                self.server_values = {
                    key: self.native_core.get_schema_entry(key)["default_value"]
                    for key in sorted(self.managed_keys)
                }
                self.node_registry.clear()
                self._pending_boot_values.clear()
                self._load_boot_parameter_file_if_available()
                self._capture_boot_configuration()
                self._initialize_tuning_store()
            self.get_logger().info(
                f"Configuration server loaded {len(self.managed_keys)} managed parameters."
            )
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
            DeclareParameters,
            "declare_parameters",
            self.declare_parameters_callback,
            callback_group=self.cb_group,
        )
        self.undeclare_parameters_service = self.create_service(
            UndeclareParameters,
            "undeclare_parameters",
            self.undeclare_parameters_callback,
            callback_group=self.cb_group,
        )
        self.get_parameter_yaml_service = self.create_service(
            GetParameterYaml,
            "get_parameter_yaml",
            self.get_parameter_yaml_callback,
            callback_group=self.cb_group,
        )
        self.get_declared_parameters_service = self.create_service(
            GetDeclaredParameters,
            "get_declared_parameters",
            self.get_declared_parameters_callback,
            callback_group=self.cb_group,
        )
        self.save_parameters_service = self.create_service(
            SaveParameters,
            "save_parameters",
            self.save_parameters_callback,
            callback_group=self.cb_group,
        )
        self.get_parameter_files_service = self.create_service(
            GetParameterFiles,
            "get_parameter_files",
            self.get_parameter_files_callback,
            callback_group=self.cb_group,
        )
        self.load_parameters_service = self.create_service(
            LoadParameters,
            "load_parameters",
            self.load_parameters_callback,
            callback_group=self.cb_group,
        )
        self.set_parameter_from_gc_service = self.create_service(
            SetParameterFromGC,
            "set_parameter_from_gc",
            self.set_parameter_from_gc_callback,
            callback_group=self.cb_group,
        )
        self.apply_configuration_transaction_service = self.create_service(
            ApplyConfigurationTransaction,
            "apply_configuration_transaction",
            self.apply_configuration_transaction_callback,
            callback_group=self.cb_group,
        )
        self.get_configuration_session_service = self.create_service(
            GetConfigurationSession,
            "get_configuration_session",
            self.get_configuration_session_callback,
            callback_group=self.cb_group,
        )
        self.ensure_configuration_session_service = self.create_service(
            EnsureConfigurationSession,
            "ensure_configuration_session",
            self.ensure_configuration_session_callback,
            callback_group=self.cb_group,
        )
        self.get_configuration_journal_service = self.create_service(
            GetConfigurationJournal,
            "get_configuration_journal",
            self.get_configuration_journal_callback,
            callback_group=self.cb_group,
        )
        self.get_parameter_file_service = self.create_service(
            GetParameterFile,
            "get_parameter_file",
            self.get_parameter_file_callback,
            callback_group=self.cb_group,
        )
        self.delete_parameter_file_service = self.create_service(
            DeleteParameterFile,
            "delete_parameter_file",
            self.delete_parameter_file_callback,
            callback_group=self.cb_group,
        )
        self.set_boot_parameter_service = self.create_service(
            SetBootParameter,
            "set_boot_parameter",
            self.set_boot_parameter_callback,
            callback_group=self.cb_group,
        )
        self.get_pending_boot_parameters_service = self.create_service(
            GetPendingBootParameters,
            "get_pending_boot_parameters",
            self.get_pending_boot_parameters_callback,
            callback_group=self.cb_group,
        )
        self.activate_pending_boot_parameters_service = self.create_service(
            ActivatePendingBootParameters,
            "activate_pending_boot_parameters",
            self.activate_pending_boot_parameters_callback,
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
        with self._state_lock:
            self.pending_node_notifications.update(self._get_node_fq_names())
        self.reconcile_timer = self.create_timer(
            2.0, self.reconcile_nodes, callback_group=self.cb_group
        )
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
            "apply_configuration_transaction_service",
            "get_configuration_session_service",
            "ensure_configuration_session_service",
            "get_configuration_journal_service",
            "get_parameter_file_service",
            "delete_parameter_file_service",
            "set_boot_parameter_service",
            "get_pending_boot_parameters_service",
            "activate_pending_boot_parameters_service",
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
            self._pending_boot_values.clear()
            self._tuning_store = None
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
        client = self.create_client(
            ListParameters,
            self._service_path(node_fq_name, "list_parameters"),
            callback_group=self.cb_group,
        )
        if not client.wait_for_service(timeout_sec=0.2):
            self.destroy_client(client)
            return None
        request = ListParameters.Request()
        request.prefixes = []
        request.depth = 1000
        try:
            response = self._call_client(
                client,
                request,
                timeout_sec=self._PARAMETER_SERVICE_TIMEOUT_SEC,
            )
            if response is None:
                return None
            return list(response.result.names)
        finally:
            self.destroy_client(client)

    def _call_get_parameters(
        self, node_fq_name: str, parameter_names: list[str]
    ) -> Optional[dict[str, object]]:
        client = self.create_client(
            GetParameters,
            self._service_path(node_fq_name, "get_parameters"),
            callback_group=self.cb_group,
        )
        if not client.wait_for_service(timeout_sec=0.2):
            self.destroy_client(client)
            return None
        request = GetParameters.Request()
        request.names = parameter_names
        try:
            response = self._call_client(
                client,
                request,
                timeout_sec=self._PARAMETER_SERVICE_TIMEOUT_SEC,
            )
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

    def _call_set_parameter(
        self, node_fq_name: str, parameter_name: str, value: object
    ) -> tuple[bool, str]:
        client = self.create_client(
            SetParameters,
            self._service_path(node_fq_name, "set_parameters"),
            callback_group=self.cb_group,
        )
        if not client.wait_for_service(timeout_sec=0.5):
            self.destroy_client(client)
            return False, f"Service not available for {node_fq_name}"

        request = SetParameters.Request()
        parameter = rclpy.parameter.Parameter(name=parameter_name, value=value)
        request.parameters = [parameter.to_parameter_msg()]

        try:
            response = self._call_client(
                client,
                request,
                timeout_sec=self._PARAMETER_SERVICE_TIMEOUT_SEC,
            )
            if response is None:
                return False, f"Timed out waiting for {node_fq_name}"
            if not response.results:
                return False, f"No result returned by {node_fq_name}"
            result = response.results[0]
            return bool(result.successful), str(result.reason)
        finally:
            self.destroy_client(client)

    def reconcile_nodes(self) -> None:
        if not self._reconcile_lock.acquire(blocking=False):
            return

        full_graph_scan = False
        try:
            with self._state_lock:
                if self.parameter_handler is None:
                    return
                managed_keys = set(self.managed_keys)
                pending_node_notifications = set(self.pending_node_notifications)
                full_graph_scan = not pending_node_notifications
                target_fq_names = pending_node_notifications or set(
                    self._get_node_fq_names()
                )

            now_monotonic = time.monotonic()
            discovered_fq_names = set(target_fq_names)
            discovered_fq_names.discard(self.get_fully_qualified_name())
            discovered_records: dict[str, ManagedNodeRecord] = {}

            for node_fq_name in sorted(discovered_fq_names):
                parameter_names = self._call_list_parameters(node_fq_name)
                if parameter_names is None:
                    continue

                managed_parameter_names = sorted(
                    set(parameter_names).intersection(managed_keys)
                )
                if not managed_parameter_names:
                    continue

                values = self._call_get_parameters(
                    node_fq_name, managed_parameter_names
                )
                if values is None:
                    continue

                reconciled_values: dict[str, object] = {}

                for parameter_name in managed_parameter_names:
                    # A GC update may complete after the graph/readback work
                    # above. Serialize authority lookup and node synchronization
                    # with live updates so this pass cannot restore its stale
                    # initial snapshot over the newly accepted value.
                    with self._state_lock:
                        authoritative_value = self.server_values[parameter_name]
                        if values.get(parameter_name) != authoritative_value:
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
                            else:
                                values[parameter_name] = authoritative_value

                        reconciled_values[parameter_name] = values.get(parameter_name)

                discovered_records[node_fq_name] = ManagedNodeRecord(
                    fq_name=node_fq_name,
                    parameter_names=managed_parameter_names,
                    values=reconciled_values,
                    last_seen_monotonic=now_monotonic,
                )

            with self._state_lock:
                for node_fq_name, record in discovered_records.items():
                    record.offline_since_monotonic = None
                    self.node_registry[node_fq_name] = record

                if full_graph_scan:
                    for node_fq_name in list(self.node_registry.keys()):
                        record = self.node_registry[node_fq_name]
                        if node_fq_name in discovered_fq_names:
                            record.offline_since_monotonic = None
                            continue

                        if record.offline_since_monotonic is None:
                            record.offline_since_monotonic = now_monotonic
                            continue

                        if (
                            now_monotonic - record.offline_since_monotonic
                            >= self._OFFLINE_PRUNE_GRACE_SEC
                        ):
                            del self.node_registry[node_fq_name]

                self.pending_node_notifications.clear()
        finally:
            self._reconcile_lock.release()
        if full_graph_scan:
            self._recover_prepared_transaction()
            self._retry_divergent_compensation()

    def _recover_prepared_transaction(self) -> None:
        with self._state_lock:
            if self._tuning_store is None:
                return
            try:
                status = self._tuning_store.status()
                if status["session_id"] is None:
                    return
                active_truth = dict(self.server_values)
                for name in sorted(self.managed_keys):
                    values = {
                        node: record.values[name]
                        for node, record in self.node_registry.items()
                        if name in record.values
                    }
                    if values:
                        encoded = {
                            json.dumps(value, sort_keys=True)
                            for value in values.values()
                        }
                        active_truth[name] = (
                            next(iter(values.values())) if len(encoded) == 1 else values
                        )
                result = self._tuning_store.recover_prepared(
                    active_values=active_truth,
                    persisted_values=self._effective_boot_values(),
                    pending_boot_values=self._pending_boot_values,
                    persistence_reference=self.current_parameter_file,
                )
                if result is not None and result.get("status") == "divergent":
                    self.get_logger().error(
                        "Interrupted configuration transaction recovered as divergent; "
                        "further parameter writes are blocked."
                    )
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to recover interrupted configuration transaction: {exc}"
                )

    def _retry_divergent_compensation(self) -> None:
        with self._state_lock:
            if self._tuning_store is None:
                return
            try:
                status = self._tuning_store.status()
                if not status["divergent"]:
                    return

                active_values = dict(status["active_values"])
                affected_names = sorted(status["divergent_observations"])
                self._pending_boot_values = dict(status["pending_boot_values"])
                for name, value in active_values.items():
                    if name not in self.managed_keys:
                        raise TuningError(
                            f"durable divergent state names an unmanaged parameter: {name}"
                        )
                    self.server_values[name] = value
                    if self.parameter_handler is not None:
                        self.parameter_handler.set_param(
                            name,
                            value,
                            parameter_initialized=False,
                            force_constant=True,
                        )

                for name in affected_names:
                    if self._restart_semantics(name) != "none":
                        continue
                    target_nodes = self._group_nodes_for_parameter(name)
                    if not target_nodes:
                        raise TuningError(
                            f"no fresh managed node declares divergent parameter: {name}"
                        )
                    for node in target_nodes:
                        success, message = self._call_set_parameter(
                            node, name, active_values[name]
                        )
                        if not success:
                            raise TuningError(
                                f"{node} rejected divergent compensation for {name}: {message}"
                            )
                        readback = self._call_get_parameters(node, [name])
                        if (
                            readback is None
                            or readback.get(name) != active_values[name]
                        ):
                            raise TuningError(
                                f"{node} did not confirm divergent compensation for {name}"
                            )
                        record = self.node_registry.get(node)
                        if record is not None:
                            record.values[name] = active_values[name]

                self._sync_runtime_parameter_file()
                observations = self._transaction_observations(sorted(active_values))
                result = self._tuning_store.reconcile_divergence(
                    observed_values=observations,
                    persisted_values=self._effective_boot_values(),
                    pending_boot_values=self._pending_boot_values,
                    persistence_reference=self.current_parameter_file,
                )
                if result["reconciled"]:
                    self.get_logger().info(
                        "Configuration divergence reconciled to the exact prior durable state."
                    )
            except Exception as exc:
                self.get_logger().warn(
                    "Configuration remains divergent; exact compensation retry failed: "
                    f"{exc}"
                )

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
        return bool(
            self.native_core.get_schema_entry(parameter_name).get("constant", False)
        )

    def _candidate_parameter_map(
        self, values: dict[str, object]
    ) -> dict[str, dict[str, object]]:
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
                node_fq_name: self.node_registry[node_fq_name].values.get(
                    parameter_name
                )
                for node_fq_name in target_nodes
            }
            successfully_updated_nodes: list[str] = []

            for node_fq_name in target_nodes:
                success, message = self._call_set_parameter(
                    node_fq_name, parameter_name, value
                )
                if not success:
                    self._rollback_parameter_updates(
                        parameter_name, previous_values, successfully_updated_nodes
                    )
                    return False, f"{node_fq_name} rejected update: {message}"

                successfully_updated_nodes.append(node_fq_name)
                updated_record = self.node_registry.get(node_fq_name)
                if updated_record is not None:
                    updated_record.values[parameter_name] = value

            for node_fq_name in target_nodes:
                readback = self._call_get_parameters(node_fq_name, [parameter_name])
                if readback is None:
                    self._rollback_parameter_updates(
                        parameter_name, previous_values, successfully_updated_nodes
                    )
                    return False, f"{node_fq_name} did not confirm the applied value"

                applied_value = readback.get(parameter_name)
                if applied_value != value:
                    self._rollback_parameter_updates(
                        parameter_name, previous_values, successfully_updated_nodes
                    )
                    return False, (
                        f"{node_fq_name} reported {applied_value!r} after applying "
                        f"{parameter_name}={value!r}"
                    )

            self.parameter_handler.set_param(
                parameter_name, value, parameter_initialized=False, force_constant=True
            )
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
        reference = self._normalize_parameter_file_reference(
            file_name, default_subdir="snapshots"
        )
        path = resolve_parameter_set_path(self._profile_name, reference)
        with open(path, "r") as file:
            data = yaml.safe_load(file) or {}
        ros_parameters = data.get("/**", {}).get("ros__parameters", {})
        return {
            name: value
            for name, value in ros_parameters.items()
            if name in self.managed_keys
        }

    def _load_parameter_values_from_path(self, path: Path) -> dict[str, object]:
        with open(path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
        ros_parameters = data.get("/**", {}).get("ros__parameters", {})
        return {
            name: value
            for name, value in ros_parameters.items()
            if name in self.managed_keys
        }

    def _write_snapshot_file(
        self,
        file_name: str,
        overwrite: bool,
        *,
        validate_live_nodes: bool = True,
    ) -> Path:
        with self._state_lock:
            reference = self._normalize_parameter_file_reference(
                file_name, default_subdir="snapshots"
            )
            path = resolve_parameter_set_path(self._profile_name, reference)
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists() and not overwrite:
                raise FileExistsError(f"Snapshot file '{file_name}' already exists")

            if validate_live_nodes:
                # The registry is a discovery/reconciliation cache and may lag
                # a just-completed SetParameters response. Explicit saves audit
                # fresh live values; transactional runtime snapshots skip this
                # broader audit because the changed value was already read back
                # synchronously by _apply_shared_parameter_update().
                for node_fq_name, record in self.node_registry.items():
                    parameter_names = sorted(
                        set(record.parameter_names).intersection(self.server_values)
                    )
                    if not parameter_names:
                        continue
                    live_values = None
                    for attempt in range(self._PARAMETER_READBACK_ATTEMPTS):
                        live_values = self._call_get_parameters(
                            node_fq_name, parameter_names
                        )
                        if live_values is not None and all(
                            live_values.get(name) == self.server_values[name]
                            for name in parameter_names
                        ):
                            break
                        if attempt + 1 < self._PARAMETER_READBACK_ATTEMPTS:
                            time.sleep(self._PARAMETER_READBACK_RETRY_SEC)
                    if live_values is None:
                        raise RuntimeError(
                            f"Refusing to save because '{node_fq_name}' did not provide fresh parameter readback"
                        )
                    record.values.update(live_values)
                    for parameter_name in parameter_names:
                        node_value = live_values.get(parameter_name)
                        if node_value != self.server_values[parameter_name]:
                            raise RuntimeError(
                                f"Refusing to save inconsistent parameter '{parameter_name}': "
                                f"{node_fq_name} has {node_value!r}, server has {self.server_values[parameter_name]!r}"
                            )

            parameter_file_data = build_parameter_file_data(
                self._effective_boot_values()
            )
            save_parameter_file(path, parameter_file_data)
            return path

    def _set_default_snapshot_file(self, file_name: str) -> None:
        normalized = self._normalize_parameter_file_reference(
            file_name, default_subdir="tracked"
        )
        persist_active_parameter_set_reference(self._profile_name, normalized)
        self._default_parameter_file_path = resolve_parameter_set_path(
            self._profile_name, normalized
        )

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
        if not file_name or not Path(file_name).name.startswith(
            self._RUNTIME_SNAPSHOT_PREFIX
        ):
            return

        try:
            resolve_parameter_set_path(
                self._profile_name,
                self._normalize_parameter_file_reference(
                    file_name, default_subdir="snapshots"
                ),
            ).unlink()
        except FileNotFoundError:
            pass

    def _restore_boot_default_parameter_file(self) -> None:
        if self._boot_default_snapshot_file_name:
            persist_active_parameter_set_reference(
                self._profile_name, self._boot_default_snapshot_file_name
            )
        self.current_parameter_file = self._boot_current_parameter_file
        self._default_parameter_file_path = self._boot_default_parameter_file_path

    def _mark_current_configuration_as_default_baseline(
        self, file_name: str, path: Path
    ) -> None:
        self._boot_server_values = dict(self.server_values)
        self._boot_current_parameter_file = file_name
        self._boot_default_parameter_file_path = path
        self._boot_default_snapshot_file_name = file_name

    def _effective_boot_values(self) -> dict[str, object]:
        values = dict(self.server_values)
        values.update(self._pending_boot_values)
        return values

    def _sync_runtime_parameter_file(self) -> str:
        if self._effective_boot_values() == self._boot_server_values:
            removed_file_name = self._runtime_snapshot_file_name
            self._runtime_snapshot_file_name = None
            self._restore_boot_default_parameter_file()
            self._delete_runtime_snapshot_file(removed_file_name)
            if removed_file_name:
                return "Runtime parameter snapshot removed; restored boot default parameter file"
            return ""

        previous_runtime_file_name = self._runtime_snapshot_file_name
        file_name = self._new_runtime_snapshot_file_name()
        reference = self._normalize_parameter_file_reference(
            file_name, default_subdir="snapshots"
        )
        while resolve_parameter_set_path(self._profile_name, reference).exists():
            file_name = self._new_runtime_snapshot_file_name()
            reference = self._normalize_parameter_file_reference(
                file_name, default_subdir="snapshots"
            )

        self._write_snapshot_file(reference, overwrite=False, validate_live_nodes=False)
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
        self.current_parameter_file = (
            self._current_parameter_file_reference_for_boot_path(parameter_file_path)
        )
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
            self.parameter_handler.set_param(
                parameter_name, value, parameter_initialized=False, force_constant=True
            )
            self.server_values[parameter_name] = value

    def _require_tuning_store(self) -> TuningSessionStore:
        if self._tuning_store is None:
            raise TuningError("configuration tuning store is unavailable")
        return self._tuning_store

    def _restart_semantics(self, parameter_name: str) -> str:
        if self.native_core is None:
            raise TuningError("configuration server is not initialized")
        entry = self.native_core.get_schema_entry(parameter_name)
        if bool(entry.get("constant", False)):
            return "runtime"
        if bool(entry.get("static", False)):
            return "node"
        return "none"

    def _validate_transaction_candidate(
        self, candidate_values: Mapping[str, object]
    ) -> None:
        if self.native_core is None:
            raise TuningError("configuration server is not initialized")
        self.native_core.validate_parameter_map(
            self._candidate_parameter_map(dict(candidate_values)),
            True,
        )

    def _transaction_observations(
        self, parameter_names: Sequence[str]
    ) -> dict[str, object]:
        observed: dict[str, object] = {}
        for parameter_name in sorted(set(parameter_names)):
            nodes = self._group_nodes_for_parameter(parameter_name)
            if not nodes:
                observed[parameter_name] = self.server_values.get(parameter_name)
                continue
            by_node: dict[str, object] = {}
            for node in sorted(nodes):
                values = self._call_get_parameters(node, [parameter_name])
                by_node[node] = None if values is None else values.get(parameter_name)
            unique = {json.dumps(value, sort_keys=True) for value in by_node.values()}
            observed[parameter_name] = (
                next(iter(by_node.values())) if len(unique) == 1 else by_node
            )
        return observed

    @staticmethod
    def _canonical_json(value: Mapping[str, object]) -> str:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )

    def _parse_transaction_request(self, raw: str) -> dict[str, object]:
        try:
            value = json.loads(raw)
        except (TypeError, json.JSONDecodeError) as exc:
            raise TuningError(
                f"configuration transaction request is invalid JSON: {exc}"
            ) from exc
        if not isinstance(value, dict) or raw != self._canonical_json(value):
            raise TuningError(
                "configuration transaction request must be canonical JSON"
            )
        if set(value) != {
            "schema",
            "request_id",
            "expected_revision",
            "operator_id",
            "edits",
        }:
            raise TuningError(
                "configuration transaction request fields do not match the fixed contract"
            )
        if value["schema"] != "iii.configuration-transaction-request/v1":
            raise TuningError("configuration transaction request schema is unsupported")
        if not isinstance(value["edits"], list):
            raise TuningError("configuration transaction edits must be a list")
        return value

    def _prepare_transaction_edits(
        self, request_edits: Sequence[Mapping[str, object]]
    ) -> list[dict[str, object]]:
        prepared: list[dict[str, object]] = []
        for edit in request_edits:
            if not isinstance(edit, dict) or set(edit) != {"node_id", "name", "value"}:
                raise TuningError(
                    "configuration edit fields do not match the fixed request contract"
                )
            name = edit["name"]
            if not isinstance(name, str) or name not in self.managed_keys:
                raise TuningError("configuration edit names an unmanaged parameter")
            if self.native_core is None:
                raise TuningError("configuration server is not initialized")
            entry = self.native_core.get_schema_entry(name)
            if bool(entry.get("readonly", False) or entry.get("read_only", False)):
                raise TuningError(f"configuration parameter is read-only: {name}")
            prepared.append(
                {
                    "node_id": edit["node_id"],
                    "name": name,
                    "value": edit["value"],
                    "restart_required": self._restart_semantics(name),
                }
            )
        return prepared

    def _legacy_transaction(
        self, *, parameter_name: str, value: object
    ) -> dict[str, object]:
        status = self._require_tuning_store().status()
        request_seed = {
            "parameter": parameter_name,
            "value": value,
            "revision": status["revision"],
            "timestamp": self._utc_now(),
        }
        request_id = (
            "legacy-"
            + hashlib.sha256(
                self._canonical_json(request_seed).encode("utf-8")
            ).hexdigest()[:32]
        )
        return self._execute_configuration_transaction(
            {
                "schema": "iii.configuration-transaction-request/v1",
                "request_id": request_id,
                "expected_revision": status["revision"],
                "operator_id": "legacy-service",
                "edits": [
                    {
                        "node_id": "configuration",
                        "name": parameter_name,
                        "value": value,
                    }
                ],
            }
        )

    def _rollback_transaction(
        self,
        plan: TransactionPlan,
        *,
        updated_live_names: Sequence[str],
    ) -> tuple[bool, dict[str, object], str | None]:
        errors: list[str] = []
        self._pending_boot_values = dict(plan.previous_pending_boot_values)
        for name in reversed(list(updated_live_names)):
            success, message = self._apply_shared_parameter_update(
                name,
                plan.previous_active_values[name],
                require_targets=True,
                allow_constant_override=True,
            )
            if not success:
                errors.append(f"{name}: {message}")
        try:
            self._sync_runtime_parameter_file()
        except Exception as exc:
            errors.append(f"persistence: {exc}")
        observations = self._transaction_observations(
            [edit["name"] for edit in plan.edits]
        )
        for name in updated_live_names:
            if observations.get(name) != plan.previous_active_values.get(name):
                errors.append(
                    f"{name}: observed {observations.get(name)!r}, expected rollback value "
                    f"{plan.previous_active_values.get(name)!r}"
                )
        return not errors, observations, "; ".join(errors) or None

    def _execute_configuration_transaction(
        self, document: Mapping[str, object]
    ) -> dict[str, object]:
        store = self._require_tuning_store()
        request_edits = self._prepare_transaction_edits(document["edits"])
        current_status = store.status()
        active_values = (
            dict(current_status["active_values"])
            if current_status["session_id"] is not None
            else dict(self.server_values)
        )
        persisted_values = (
            dict(current_status["persisted_values"])
            if current_status["session_id"] is not None
            else self._effective_boot_values()
        )
        plan_or_result = store.prepare(
            baseline_values=active_values,
            persisted_values=persisted_values,
            pending_boot_values=self._pending_boot_values,
            request_id=document["request_id"],
            expected_revision=document["expected_revision"],
            operator_id=document["operator_id"],
            edits=request_edits,
            validate_candidate=self._validate_transaction_candidate,
        )
        if not isinstance(plan_or_result, TransactionPlan):
            return plan_or_result
        plan = plan_or_result
        updated_live_names: list[str] = []
        commit_started = False
        try:
            for edit in plan.edits:
                name = edit["name"]
                if edit["restart_required"] != "none":
                    self._pending_boot_values[name] = edit["value"]
                    continue
                success, message = self._apply_shared_parameter_update(
                    name,
                    edit["value"],
                    require_targets=True,
                )
                if not success:
                    raise TuningError(message)
                updated_live_names.append(name)
            self._sync_runtime_parameter_file()
            observations = self._transaction_observations(updated_live_names)
            commit_started = True
            result = store.commit(
                plan,
                observed_values=observations,
                persistence_reference=self.current_parameter_file,
            )
            return result
        except Exception as exc:
            # If a commit WAL record reached disk but its state replacement was
            # interrupted, replay it instead of compensating an accepted revision.
            if commit_started:
                try:
                    last_result = store.status().get("last_result")
                except Exception:
                    last_result = None
                if (
                    isinstance(last_result, dict)
                    and last_result.get("transaction_id") == plan.transaction_id
                    and last_result.get("ok") is True
                ):
                    return last_result
            compensated, observations, rollback_error = self._rollback_transaction(
                plan, updated_live_names=updated_live_names
            )
            reason = str(exc)
            if rollback_error:
                reason += f"; compensation failed: {rollback_error}"
            return store.abort(
                plan,
                reason=reason,
                observed_values=observations,
                compensation_succeeded=compensated,
            )

    def apply_configuration_transaction_callback(self, request, response):
        with self._state_lock:
            try:
                document = self._parse_transaction_request(request.request_json)
                result = self._execute_configuration_transaction(document)
                response.success = bool(result.get("ok"))
                response.message = str(result.get("reason") or "")
                response.result_json = self._canonical_json(result)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.result_json = ""
            return response

    def get_configuration_session_callback(self, request, response):
        del request
        with self._state_lock:
            try:
                status = self._require_tuning_store().status()
                response.success = True
                response.message = ""
                response.session_json = self._canonical_json(
                    {
                        "schema": "iii.configuration-session-status/v1",
                        **status,
                    }
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.session_json = ""
            return response

    def ensure_configuration_session_callback(self, request, response):
        del request
        with self._state_lock:
            try:
                store = self._require_tuning_store()
                existing = store.status()
                active_values = (
                    dict(existing["active_values"])
                    if existing["session_id"] is not None
                    else dict(self.server_values)
                )
                persisted_values = (
                    dict(existing["persisted_values"])
                    if existing["session_id"] is not None
                    else self._effective_boot_values()
                )
                status = store.ensure_session(
                    baseline_values=active_values,
                    persisted_values=persisted_values,
                )
                response.success = True
                response.message = ""
                response.session_json = self._canonical_json(
                    {
                        "schema": "iii.configuration-session-status/v1",
                        **status,
                    }
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.session_json = ""
            return response

    def get_configuration_journal_callback(self, request, response):
        with self._state_lock:
            try:
                document = self._parse_canonical_request(
                    request.request_json,
                    schema="iii.configuration-journal-request/v1",
                    fields={"schema", "session_id", "after_sequence", "limit"},
                    label="configuration journal request",
                )
                batch = self._require_tuning_store().journal_batch(
                    session_id=document["session_id"],
                    after_sequence=document["after_sequence"],
                    limit=document["limit"],
                )
                response.success = True
                response.message = ""
                response.journal_json = self._canonical_json(batch)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.journal_json = ""
            return response

    def get_parameter_file_callback(self, request, response):
        with self._state_lock:
            try:
                reference, path, content = self._read_parameter_file(request.file)
                response.success = True
                response.message = ""
                response.parameter_yaml = content.decode("utf-8")
                response.content_sha256 = hashlib.sha256(content).hexdigest()
                del reference
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.parameter_yaml = ""
                response.content_sha256 = ""
            return response

    def delete_parameter_file_callback(self, request, response):
        with self._state_lock:
            try:
                document = self._parse_canonical_request(
                    request.request_json,
                    schema="iii.configuration-snapshot-delete-request/v1",
                    fields={
                        "schema",
                        "snapshot_id",
                        "force",
                        "confirmation",
                        "capture_receipt",
                    },
                    label="configuration snapshot delete request",
                )
                result = self._delete_parameter_file(document)
                response.success = True
                response.message = ""
                response.result_json = self._canonical_json(result)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.result_json = ""
            return response

    def _parse_canonical_request(
        self,
        raw: str,
        *,
        schema: str,
        fields: set[str],
        label: str,
    ) -> dict[str, object]:
        try:
            value = json.loads(raw)
        except (TypeError, json.JSONDecodeError) as exc:
            raise TuningError(f"{label} is invalid JSON: {exc}") from exc
        if not isinstance(value, dict) or raw != self._canonical_json(value):
            raise TuningError(f"{label} must be canonical JSON")
        if set(value) != fields or value.get("schema") != schema:
            raise TuningError(f"{label} fields or schema are invalid")
        return value

    def _read_parameter_file(self, file_name: str) -> tuple[str, Path, bytes]:
        reference = self._normalize_parameter_file_reference(
            file_name, default_subdir="snapshots"
        )
        path = resolve_parameter_set_path(self._profile_name, reference)
        if path.is_symlink() or not path.is_file():
            raise TuningError("configuration snapshot is missing or linked")
        content = path.read_bytes()
        if len(content) > 8 * 1024 * 1024:
            raise TuningError("configuration snapshot exceeds the fixed size limit")
        try:
            values = self._load_parameter_values_from_path(path)
            if self.native_core is None:
                raise TuningError("configuration server is not initialized")
            self.native_core.validate_parameter_map(
                self._candidate_parameter_map(values), True
            )
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
            raise TuningError(f"configuration snapshot is invalid: {exc}") from exc
        return reference, path, content

    def _delete_parameter_file(
        self, document: Mapping[str, object]
    ) -> dict[str, object]:
        snapshot_id = document["snapshot_id"]
        force = document["force"]
        confirmation = document["confirmation"]
        receipt = document["capture_receipt"]
        if not isinstance(snapshot_id, str) or not snapshot_id.startswith("snapshots/"):
            raise TuningError("only named snapshot files may be deleted")
        if not isinstance(force, bool):
            raise TuningError("snapshot delete force flag is invalid")
        if confirmation is not None and not isinstance(confirmation, str):
            raise TuningError("snapshot delete confirmation is invalid")
        reference, path, content = self._read_parameter_file(snapshot_id)
        protected = {self.current_parameter_file, self._default_snapshot_file_name()}
        status = self._require_tuning_store().status()
        last_result = status.get("last_result")
        if isinstance(last_result, dict):
            persistence_reference = last_result.get("persistence_reference")
            if isinstance(persistence_reference, str):
                protected.add(persistence_reference)
        if reference in protected:
            raise TuningError(
                "active, default, or pending configuration sets cannot be deleted"
            )
        digest = hashlib.sha256(content).hexdigest()
        if force:
            if confirmation != f"delete:{reference}":
                raise TuningError(
                    "force deletion requires the exact snapshot-bound confirmation"
                )
        else:
            self._validate_capture_receipt(
                receipt,
                snapshot_id=reference,
                content_sha256=digest,
                status=status,
            )
        path.unlink()
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        return {
            "schema": "iii.configuration-snapshot-delete-result/v1",
            "snapshot_id": reference,
            "content_sha256": digest,
            "forced": force,
            "deleted": True,
        }

    def _validate_capture_receipt(
        self,
        value: object,
        *,
        snapshot_id: str,
        content_sha256: str,
        status: Mapping[str, object],
    ) -> None:
        fields = {
            "schema",
            "receipt_id",
            "capture_id",
            "snapshot_id",
            "content_sha256",
            "target_id",
            "runtime_profile",
            "release_id",
            "manifest_id",
        }
        if not isinstance(value, dict) or set(value) != fields:
            raise TuningError("a verified local capture receipt is required")
        identity_value = {
            key: item for key, item in value.items() if key != "receipt_id"
        }
        receipt_id = hashlib.sha256(
            self._canonical_json(identity_value).encode("utf-8")
        ).hexdigest()
        if (
            value["schema"] != "iii.configuration-capture-receipt/v1"
            or value["receipt_id"] != receipt_id
            or not isinstance(value["capture_id"], str)
            or len(value["capture_id"]) != 64
            or any(
                character not in "0123456789abcdef" for character in value["capture_id"]
            )
            or value["snapshot_id"] != snapshot_id
            or value["content_sha256"] != content_sha256
            or any(
                value[field] != status[field]
                for field in (
                    "target_id",
                    "runtime_profile",
                    "release_id",
                    "manifest_id",
                )
            )
        ):
            raise TuningError("local capture receipt does not match this snapshot")

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
            file_name = request.file or datetime.now().strftime(
                "snapshot_%Y%m%d_%H%M%S.yaml"
            )
            file_reference = self._normalize_parameter_file_reference(
                file_name, default_subdir="snapshots"
            )
            try:
                self._write_snapshot_file(file_reference, request.overwrite)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.file = ""
                return response

            if request.set_as_default:
                self.current_parameter_file = file_reference
                previous_runtime_file_name = self._runtime_snapshot_file_name
                saved_path = resolve_parameter_set_path(
                    self._profile_name, file_reference
                )
                self._set_default_snapshot_file(file_reference)
                self._mark_current_configuration_as_default_baseline(
                    file_reference, saved_path
                )
                self._runtime_snapshot_file_name = None
                if previous_runtime_file_name != file_reference:
                    self._delete_runtime_snapshot_file(previous_runtime_file_name)

            response.success = True
            response.message = ""
            response.file = file_reference
            return response

    def get_parameter_files_callback(self, request, response):
        parameter_set_root = resolve_parameter_set_path(
            self._profile_name, "tracked/default.yaml"
        ).parents[1]
        response.parameter_files = sorted(
            str(file.relative_to(parameter_set_root))
            for file in parameter_set_root.rglob("*.yaml")
            if file.is_file()
        )
        return response

    def load_parameters_callback(self, request, response):
        with self._state_lock:
            try:
                file_reference = self._normalize_parameter_file_reference(
                    request.file, default_subdir="snapshots"
                )
                values = self._load_snapshot_values(file_reference)
                status = self._require_tuning_store().status()
                active_values = (
                    dict(status["active_values"])
                    if status["session_id"] is not None
                    else dict(self.server_values)
                )
                persisted_values = (
                    dict(status["persisted_values"])
                    if status["session_id"] is not None
                    else self._effective_boot_values()
                )
                edits = []
                for name, value in sorted(values.items()):
                    restart_required = self._restart_semantics(name)
                    current = (
                        persisted_values.get(name)
                        if restart_required != "none"
                        else active_values.get(name)
                    )
                    if current != value:
                        edits.append(
                            {
                                "node_id": "configuration",
                                "name": name,
                                "value": value,
                            }
                        )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            if edits:
                request_seed = {
                    "snapshot": file_reference,
                    "revision": status["revision"],
                    "values": values,
                }
                request_id = (
                    "snapshot-load-"
                    + hashlib.sha256(
                        self._canonical_json(request_seed).encode("utf-8")
                    ).hexdigest()[:32]
                )
                result = self._execute_configuration_transaction(
                    {
                        "schema": "iii.configuration-transaction-request/v1",
                        "request_id": request_id,
                        "expected_revision": status["revision"],
                        "operator_id": "legacy-snapshot-load",
                        "edits": edits,
                    }
                )
                if not result.get("ok"):
                    response.success = False
                    response.message = str(
                        result.get("reason") or "snapshot transaction was rejected"
                    )
                    return response

            if request.set_as_default:
                previous_runtime_file_name = self._runtime_snapshot_file_name
                active_reference = self.current_parameter_file
                active_path = resolve_parameter_set_path(
                    self._profile_name, active_reference
                )
                self._set_default_snapshot_file(active_reference)
                self._mark_current_configuration_as_default_baseline(
                    active_reference, active_path
                )
                self._runtime_snapshot_file_name = None
                if previous_runtime_file_name != active_reference:
                    self._delete_runtime_snapshot_file(previous_runtime_file_name)

            response.success = True
            response.message = (
                f"Loaded {file_reference} through configuration transaction"
                if edits
                else f"{file_reference} already matches the durable configuration"
            )
            return response

    def set_parameter_from_gc_callback(self, request, response):
        with self._state_lock:
            if self.parameter_handler is None or self.native_core is None:
                response.success = False
                response.message = "Configuration server is not configured"
                return response

            try:
                cast_value = self.parameter_handler.cast_param_value(
                    request.parameter_name, request.parameter_string_value
                )
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

            result = self._legacy_transaction(
                parameter_name=request.parameter_name, value=cast_value
            )
            response.success = bool(result.get("ok"))
            if response.success:
                response.message = (
                    "Runtime parameter snapshot removed; restored boot default parameter file"
                    if self.current_parameter_file == self._boot_current_parameter_file
                    else f"Runtime parameter snapshot updated: {self.current_parameter_file}"
                )
            else:
                response.message = str(
                    result.get("reason") or "parameter update rejected"
                )
            return response

    def set_boot_parameter_callback(self, request, response):
        with self._state_lock:
            if self.parameter_handler is None or self.native_core is None:
                response.success = False
                response.message = "Configuration server is not configured"
                return response

            try:
                if not self._parameter_is_constant(request.parameter_name):
                    raise ValueError(
                        f"'{request.parameter_name}' is not a constant parameter"
                    )
                cast_value = self.parameter_handler.cast_param_value(
                    request.parameter_name,
                    request.parameter_string_value,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            result = self._legacy_transaction(
                parameter_name=request.parameter_name, value=cast_value
            )
            response.success = bool(result.get("ok"))
            response.message = (
                "Boot parameter persisted; only valid after system restart"
                if response.success
                else str(result.get("reason") or "boot parameter update rejected")
            )
            response.persisted_parameter_file = self.current_parameter_file
            return response

    def get_pending_boot_parameters_callback(self, request, response):
        del request
        with self._state_lock:
            response.pending_parameters_yaml = yaml.safe_dump(
                self._pending_boot_values,
                sort_keys=True,
            )
            response.persisted_parameter_file = self.current_parameter_file
            return response

    def activate_pending_boot_parameters_callback(self, request, response):
        del request
        with self._state_lock:
            if self.parameter_handler is None or self.native_core is None:
                response.success = False
                response.message = "Configuration server is not configured"
                return response
            if not self._pending_boot_values:
                response.success = True
                response.message = "No boot parameters are pending"
                response.activated_parameter_names = []
                return response

            activated_names = sorted(self._pending_boot_values)
            try:
                effective_values = self._effective_boot_values()
                self.native_core.validate_parameter_map(
                    self._candidate_parameter_map(effective_values),
                    True,
                )
                for parameter_name in activated_names:
                    value = self._pending_boot_values[parameter_name]
                    self.parameter_handler.set_param(
                        parameter_name,
                        value,
                        parameter_initialized=False,
                        force_constant=True,
                    )
                    self.server_values[parameter_name] = value
                self.node_registry.clear()
                self.pending_node_notifications.update(self._get_node_fq_names())
                self.reconcile_nodes()
                missing = [
                    name
                    for name in activated_names
                    if not self._group_nodes_for_parameter(name)
                ]
                if missing:
                    raise TuningError(
                        "fresh runtime did not declare pending parameters: "
                        + ", ".join(missing)
                    )
                observations = self._transaction_observations(activated_names)
                confirmation = self._require_tuning_store().confirm_pending_boot(
                    observed_values=observations
                )
                self._pending_boot_values.clear()
                active_path = resolve_parameter_set_path(
                    self._profile_name, self.current_parameter_file
                )
                self._mark_current_configuration_as_default_baseline(
                    self.current_parameter_file,
                    active_path,
                )
                self._runtime_snapshot_file_name = None
            except Exception as exc:
                try:
                    durable = self._require_tuning_store().status()
                    for parameter_name in activated_names:
                        if parameter_name in durable["active_values"]:
                            value = durable["active_values"][parameter_name]
                            self.server_values[parameter_name] = value
                            self.parameter_handler.set_param(
                                parameter_name,
                                value,
                                parameter_initialized=False,
                                force_constant=True,
                            )
                except Exception:
                    pass
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = (
                "Pending boot parameters activated after fresh whole-graph readback; "
                f"revision {confirmation['revision']}"
            )
            response.activated_parameter_names = activated_names
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
                saved_path = self._write_snapshot_file(
                    self.current_parameter_file, overwrite=True
                )
                self._set_default_snapshot_file(self.current_parameter_file)
                self._mark_current_configuration_as_default_baseline(
                    self.current_parameter_file, saved_path
                )
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
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.02)
    except KeyboardInterrupt:
        node.get_logger().info("Configuration server received shutdown signal.")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
