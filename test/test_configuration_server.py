from types import SimpleNamespace

import pytest
import yaml

from rclpy.lifecycle import TransitionCallbackReturn

from iii_drone_configuration.configuration_server_node import ConfigurationServer, ManagedNodeRecord

from conftest import TEST_SCHEMA_FILE, write_bootstrap_parameter_file


@pytest.fixture
def configured_server(monkeypatch, tmp_path):
    monkeypatch.setenv("III_DRONE_SCHEMA_FILE", str(TEST_SCHEMA_FILE))
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    write_bootstrap_parameter_file(tmp_path)

    server = ConfigurationServer(node_name="configuration_server_test", namespace="/configuration/configuration_server")
    assert server.trigger_configure() == TransitionCallbackReturn.SUCCESS

    yield server, tmp_path

    try:
        server.trigger_cleanup()
    except Exception:
        pass
    server.destroy_node()


def _bootstrap_default_snapshot_file(tmp_path):
    selector = yaml.safe_load((tmp_path / "iii_drone" / "profiles" / "sim.yaml").read_text(encoding="utf-8"))
    return selector["active_parameter_set"]


def _snapshot_path(tmp_path, file_name):
    return tmp_path / "iii_drone" / "parameter_sets" / "sim" / file_name


def test_server_configure_loads_schema_defaults(configured_server):
    server, _ = configured_server

    assert "/control/gains/p" in server.managed_keys
    assert server.server_values["/control/gains/p"] == pytest.approx(1.5)
    assert server.parameter_handler.get_param_value("/control/mode") == "auto"
    assert server.current_parameter_file == "tracked/default.yaml"


def test_server_reconcile_discovers_nodes_and_syncs_authoritative_values(configured_server, monkeypatch):
    server, _ = configured_server
    updates = []

    monkeypatch.setattr(
        server,
        "_get_node_fq_names",
        lambda: ["/node_a", "/node_b", server.get_fully_qualified_name()],
    )
    monkeypatch.setattr(
        server,
        "_call_list_parameters",
        lambda node_fq_name: ["/control/gains/p", "/control/mode"],
    )
    monkeypatch.setattr(
        server,
        "_call_get_parameters",
        lambda node_fq_name, parameter_names: {
            "/control/gains/p": 9.0 if node_fq_name == "/node_a" else server.server_values["/control/gains/p"],
            "/control/mode": server.server_values["/control/mode"],
        },
    )

    def fake_set_parameter(node_fq_name, parameter_name, value):
        updates.append((node_fq_name, parameter_name, value))
        return True, ""

    monkeypatch.setattr(server, "_call_set_parameter", fake_set_parameter)

    server.reconcile_nodes()

    assert set(server.node_registry.keys()) == {"/node_a", "/node_b"}
    assert updates == [("/node_a", "/control/gains/p", server.server_values["/control/gains/p"])]


def test_server_reconcile_does_not_restore_stale_value_during_live_update(configured_server, monkeypatch):
    server, _ = configured_server
    updates = []

    monkeypatch.setattr(
        server,
        "_get_node_fq_names",
        lambda: ["/node_a", server.get_fully_qualified_name()],
    )
    monkeypatch.setattr(server, "_call_list_parameters", lambda _: ["/control/gains/p"])

    def read_after_concurrent_update(node_fq_name, parameter_names):
        # Model set_parameter_from_gc completing after reconcile_nodes snapshots
        # the old 1.5 authority but before it compares the node readback.
        server.server_values["/control/gains/p"] = 2.75
        return {"/control/gains/p": 2.75}

    monkeypatch.setattr(server, "_call_get_parameters", read_after_concurrent_update)
    monkeypatch.setattr(
        server,
        "_call_set_parameter",
        lambda node_fq_name, parameter_name, value: (updates.append((node_fq_name, parameter_name, value)) or True, ""),
    )

    server.reconcile_nodes()

    assert updates == []
    assert server.node_registry["/node_a"].values["/control/gains/p"] == pytest.approx(2.75)


def test_server_live_parameter_update_allows_bounded_node_callback_latency(configured_server, monkeypatch):
    server, _ = configured_server
    observed_timeouts = []
    client = SimpleNamespace(wait_for_service=lambda timeout_sec: True)
    monkeypatch.setattr(server, "create_client", lambda *args, **kwargs: client)
    monkeypatch.setattr(server, "destroy_client", lambda _: None)

    def fake_call_client(actual_client, request, timeout_sec):
        observed_timeouts.append(timeout_sec)
        return SimpleNamespace(results=[SimpleNamespace(successful=True, reason="")])

    monkeypatch.setattr(server, "_call_client", fake_call_client)

    success, message = server._call_set_parameter("/node_a", "/control/gains/p", 2.75)

    assert success, message
    assert observed_timeouts == [pytest.approx(2.0)]


def test_server_serialization_callbacks_reflect_current_state(configured_server):
    server, _ = configured_server
    server.node_registry = {
        "/node_a": ManagedNodeRecord("/node_a", ["/control/gains/p"], {"/control/gains/p": 1.5}),
        "/node_b": ManagedNodeRecord("/node_b", ["/control/gains/p", "/control/mode"], {"/control/gains/p": 1.5, "/control/mode": "auto"}),
    }

    yaml_response = server.get_parameter_yaml_callback(SimpleNamespace(), SimpleNamespace(yaml=""))
    assert "/control/gains/p" in yaml_response.yaml

    declared_response = server.get_declared_parameters_callback(
        SimpleNamespace(),
        SimpleNamespace(declared_parameters_yaml=""),
    )
    declared = yaml.safe_load(declared_response.declared_parameters_yaml)
    assert declared["/control/gains/p"] == ["/node_a", "/node_b"]
    assert declared["/control/mode"] == ["/node_b"]


def test_server_shared_update_rolls_back_on_node_rejection(configured_server, monkeypatch):
    server, _ = configured_server
    server.node_registry = {
        "/node_a": ManagedNodeRecord("/node_a", ["/control/mode"], {"/control/mode": "auto"}),
        "/node_b": ManagedNodeRecord("/node_b", ["/control/mode"], {"/control/mode": "auto"}),
    }

    calls = []

    def fake_set_parameter(node_fq_name, parameter_name, value):
        calls.append((node_fq_name, parameter_name, value))
        if node_fq_name == "/node_b" and value == "manual":
            return False, "rejected"
        return True, ""

    monkeypatch.setattr(server, "_call_set_parameter", fake_set_parameter)

    success, message = server._apply_shared_parameter_update("/control/mode", "manual")

    assert not success
    assert "rejected" in message
    assert server.server_values["/control/mode"] == "auto"
    assert calls[-1] == ("/node_a", "/control/mode", "auto")


def test_server_save_and_load_snapshot_callbacks(configured_server):
    server, tmp_path = configured_server

    save_request = SimpleNamespace(file="snapshot.yaml", set_as_default=False, overwrite=True)
    save_response = SimpleNamespace(success=False, message="", file="")
    result = server.save_parameters_callback(save_request, save_response)

    assert result.success
    snapshot_path = tmp_path / "iii_drone" / "parameter_sets" / "sim" / "snapshots" / "snapshot.yaml"
    assert snapshot_path.exists()

    snapshot_data = yaml.safe_load(snapshot_path.read_text())
    snapshot_data["/**"]["ros__parameters"]["/control/mode"] = "manual"
    snapshot_path.write_text(yaml.safe_dump(snapshot_data, sort_keys=False))

    load_request = SimpleNamespace(file="snapshot.yaml", set_as_default=False)
    load_response = SimpleNamespace(success=False, message="")
    load_result = server.load_parameters_callback(load_request, load_response)

    assert load_result.success
    assert server.server_values["/control/mode"] == "manual"
    assert server.current_parameter_file == "snapshots/snapshot.yaml"


def test_server_runtime_updates_do_not_change_late_join_defaults(configured_server, monkeypatch):
    server, _ = configured_server

    success, message = server._apply_shared_parameter_update("/control/mode", "manual", require_targets=False)
    assert success, message
    assert server.server_values["/control/mode"] == "manual"

    monkeypatch.setattr(
        server,
        "_get_node_fq_names",
        lambda: ["/late_joiner", server.get_fully_qualified_name()],
    )
    monkeypatch.setattr(server, "_call_list_parameters", lambda _: ["/control/mode"])
    monkeypatch.setattr(server, "_call_get_parameters", lambda *_: {"/control/mode": "auto"})

    updates = []

    def fake_set_parameter(node_fq_name, parameter_name, value):
        updates.append((node_fq_name, parameter_name, value))
        return True, ""

    monkeypatch.setattr(server, "_call_set_parameter", fake_set_parameter)

    server.reconcile_nodes()

    assert updates == [("/late_joiner", "/control/mode", "manual")]
    assert server.node_registry["/late_joiner"].values["/control/mode"] == "manual"


def test_server_periodic_reconcile_marks_then_prunes_offline_nodes(configured_server, monkeypatch):
    server, _ = configured_server
    server._OFFLINE_PRUNE_GRACE_SEC = 10.0
    server.node_registry = {
        "/node_a": ManagedNodeRecord("/node_a", ["/control/mode"], {"/control/mode": "auto"}),
        "/node_b": ManagedNodeRecord("/node_b", ["/control/mode"], {"/control/mode": "auto"}),
    }

    now = 100.0
    monkeypatch.setattr("iii_drone_configuration.configuration_server_node.time.monotonic", lambda: now)
    monkeypatch.setattr(server, "_get_node_fq_names", lambda: ["/node_a", server.get_fully_qualified_name()])
    monkeypatch.setattr(server, "_call_list_parameters", lambda _: ["/control/mode"])
    monkeypatch.setattr(server, "_call_get_parameters", lambda *_: {"/control/mode": "auto"})

    server.reconcile_nodes()

    assert "/node_b" in server.node_registry
    assert server.node_registry["/node_b"].offline_since_monotonic == pytest.approx(100.0)

    now = 109.9
    server.reconcile_nodes()

    assert "/node_b" in server.node_registry

    now = 110.0
    server.reconcile_nodes()

    assert "/node_b" not in server.node_registry
    assert "/node_a" in server.node_registry


def test_server_notification_reconcile_does_not_prune_unmentioned_nodes(configured_server, monkeypatch):
    server, _ = configured_server
    server._OFFLINE_PRUNE_GRACE_SEC = 0.0
    server.node_registry = {
        "/node_a": ManagedNodeRecord("/node_a", ["/control/mode"], {"/control/mode": "auto"}),
        "/node_b": ManagedNodeRecord("/node_b", ["/control/mode"], {"/control/mode": "auto"}),
    }
    server.pending_node_notifications.add("/node_a")

    monkeypatch.setattr("iii_drone_configuration.configuration_server_node.time.monotonic", lambda: 100.0)
    monkeypatch.setattr(server, "_call_list_parameters", lambda _: ["/control/mode"])
    monkeypatch.setattr(server, "_call_get_parameters", lambda *_: {"/control/mode": "auto"})

    server.reconcile_nodes()

    assert set(server.node_registry.keys()) == {"/node_a", "/node_b"}
    assert server.node_registry["/node_b"].offline_since_monotonic is None


def test_server_save_writes_standalone_parameter_file(configured_server):
    server, tmp_path = configured_server
    success, message = server._apply_shared_parameter_update("/control/mode", "manual", require_targets=False)
    assert success, message

    save_request = SimpleNamespace(file="standalone.yaml", set_as_default=False, overwrite=True)
    save_response = SimpleNamespace(success=False, message="", file="")
    result = server.save_parameters_callback(save_request, save_response)

    assert result.success
    saved_yaml = yaml.safe_load(
        (tmp_path / "iii_drone" / "parameter_sets" / "sim" / "snapshots" / "standalone.yaml").read_text()
    )
    ros_parameters = saved_yaml["/**"]["ros__parameters"]
    assert ros_parameters["/control/mode"] == "manual"


def test_server_save_validates_fresh_node_values_instead_of_registry_cache(configured_server, monkeypatch):
    server, tmp_path = configured_server
    server.server_values["/control/gains/p"] = 2.75
    server.node_registry = {
        "/node_a": ManagedNodeRecord(
            "/node_a",
            ["/control/gains/p"],
            {"/control/gains/p": 1.5},
        ),
    }
    monkeypatch.setattr(
        server,
        "_call_get_parameters",
        lambda node_fq_name, parameter_names: {"/control/gains/p": 2.75},
    )

    save_request = SimpleNamespace(file="fresh-readback.yaml", set_as_default=False, overwrite=True)
    save_response = SimpleNamespace(success=False, message="", file="")
    result = server.save_parameters_callback(save_request, save_response)

    assert result.success, result.message
    assert server.node_registry["/node_a"].values["/control/gains/p"] == pytest.approx(2.75)
    saved_yaml = yaml.safe_load(
        (tmp_path / "iii_drone" / "parameter_sets" / "sim" / "snapshots" / "fresh-readback.yaml").read_text()
    )
    assert saved_yaml["/**"]["ros__parameters"]["/control/gains/p"] == pytest.approx(2.75)


def test_server_save_retries_transient_stale_parameter_readback(configured_server, monkeypatch):
    server, _ = configured_server
    server.server_values["/control/gains/p"] = 2.75
    server.node_registry = {
        "/node_a": ManagedNodeRecord(
            "/node_a",
            ["/control/gains/p"],
            {"/control/gains/p": 1.5},
        ),
    }
    readbacks = iter([
        {"/control/gains/p": 1.5},
        {"/control/gains/p": 2.75},
    ])
    monkeypatch.setattr(server, "_call_get_parameters", lambda *args: next(readbacks))
    monkeypatch.setattr("iii_drone_configuration.configuration_server_node.time.sleep", lambda _: None)

    save_request = SimpleNamespace(file="eventual-readback.yaml", set_as_default=False, overwrite=True)
    save_response = SimpleNamespace(success=False, message="", file="")
    result = server.save_parameters_callback(save_request, save_response)

    assert result.success, result.message


def test_server_set_parameter_from_gc_casts_applies_and_updates_runtime_snapshot(configured_server, monkeypatch):
    server, tmp_path = configured_server
    server.node_registry = {
        "/node_a": ManagedNodeRecord("/node_a", ["/control/gains/p"], {"/control/gains/p": 1.5}),
    }

    updates = []

    def fake_set_parameter(node_fq_name, parameter_name, value):
        updates.append((node_fq_name, parameter_name, value))
        return True, ""

    monkeypatch.setattr(server, "_call_set_parameter", fake_set_parameter)
    monkeypatch.setattr(
        server,
        "_call_get_parameters",
        lambda node_fq_name, parameter_names: {"/control/gains/p": 2.75},
    )

    request = SimpleNamespace(parameter_name="/control/gains/p", parameter_string_value="2.75")
    response = SimpleNamespace(success=False, message="")
    result = server.set_parameter_from_gc_callback(request, response)

    assert result.success
    assert server.server_values["/control/gains/p"] == pytest.approx(2.75)
    assert updates == [("/node_a", "/control/gains/p", 2.75)]
    assert result.message.startswith("Runtime parameter snapshot updated: ")

    runtime_snapshot_file = result.message.rsplit(" ", 1)[-1]
    assert runtime_snapshot_file.startswith("snapshots/runtime_parameters_")
    assert server.current_parameter_file == runtime_snapshot_file
    assert _bootstrap_default_snapshot_file(tmp_path) == runtime_snapshot_file

    snapshot = yaml.safe_load(_snapshot_path(tmp_path, runtime_snapshot_file).read_text(encoding="utf-8"))
    ros_parameters = snapshot["/**"]["ros__parameters"]
    assert ros_parameters["/control/gains/p"] == pytest.approx(2.75)


def test_server_runtime_snapshot_uses_transaction_state_not_unrelated_registry_cache(configured_server, monkeypatch):
    server, _ = configured_server
    server.node_registry = {
        "/node_a": ManagedNodeRecord(
            "/node_a",
            ["/control/gains/p", "/control/mode"],
            {"/control/gains/p": 1.5, "/control/mode": "stale"},
        ),
    }
    monkeypatch.setattr(server, "_call_set_parameter", lambda *args: (True, ""))
    monkeypatch.setattr(
        server,
        "_call_get_parameters",
        lambda node_fq_name, parameter_names: {
            parameter_name: 2.75 if parameter_name == "/control/gains/p" else "stale"
            for parameter_name in parameter_names
        },
    )

    request = SimpleNamespace(parameter_name="/control/gains/p", parameter_string_value="2.75")
    response = SimpleNamespace(success=False, message="")
    result = server.set_parameter_from_gc_callback(request, response)

    assert result.success, result.message
    assert result.message.startswith("Runtime parameter snapshot updated: ")


def test_server_runtime_snapshot_is_renamed_and_removed_when_values_match_boot(configured_server, monkeypatch):
    server, tmp_path = configured_server
    server.node_registry = {
        "/node_a": ManagedNodeRecord(
            "/node_a",
            ["/control/mode", "/control/gains/p"],
            {"/control/mode": "auto", "/control/gains/p": 1.5},
        ),
    }
    generated_names = iter(
        [
            "runtime_parameters_first.yaml",
            "runtime_parameters_second.yaml",
            "runtime_parameters_third.yaml",
        ]
    )
    monkeypatch.setattr(server, "_new_runtime_snapshot_file_name", lambda: next(generated_names))

    def fake_set_parameter(node_fq_name, parameter_name, value):
        server.node_registry[node_fq_name].values[parameter_name] = value
        return True, ""

    monkeypatch.setattr(server, "_call_set_parameter", fake_set_parameter)
    monkeypatch.setattr(
        server,
        "_call_get_parameters",
        lambda node_fq_name, parameter_names: {
            parameter_name: server.node_registry[node_fq_name].values[parameter_name]
            for parameter_name in parameter_names
        },
    )

    def set_from_gc(parameter_name, value):
        request = SimpleNamespace(parameter_name=parameter_name, parameter_string_value=value)
        response = SimpleNamespace(success=False, message="")
        result = server.set_parameter_from_gc_callback(request, response)
        assert result.success, result.message
        return result

    first = set_from_gc("/control/mode", "manual")
    assert first.message.endswith("snapshots/runtime_parameters_first.yaml")
    assert _snapshot_path(tmp_path, "snapshots/runtime_parameters_first.yaml").exists()
    assert _bootstrap_default_snapshot_file(tmp_path) == "snapshots/runtime_parameters_first.yaml"

    second = set_from_gc("/control/gains/p", "2.0")
    assert second.message.endswith("snapshots/runtime_parameters_second.yaml")
    assert not _snapshot_path(tmp_path, "snapshots/runtime_parameters_first.yaml").exists()
    assert _snapshot_path(tmp_path, "snapshots/runtime_parameters_second.yaml").exists()
    assert _bootstrap_default_snapshot_file(tmp_path) == "snapshots/runtime_parameters_second.yaml"

    third = set_from_gc("/control/gains/p", "1.5")
    assert third.message.endswith("snapshots/runtime_parameters_third.yaml")
    assert not _snapshot_path(tmp_path, "snapshots/runtime_parameters_second.yaml").exists()
    assert _snapshot_path(tmp_path, "snapshots/runtime_parameters_third.yaml").exists()
    assert _bootstrap_default_snapshot_file(tmp_path) == "snapshots/runtime_parameters_third.yaml"

    restored = set_from_gc("/control/mode", "auto")
    assert restored.message == "Runtime parameter snapshot removed; restored boot default parameter file"
    assert not _snapshot_path(tmp_path, "snapshots/runtime_parameters_third.yaml").exists()
    assert _bootstrap_default_snapshot_file(tmp_path) == "tracked/default.yaml"
    assert server.current_parameter_file == "tracked/default.yaml"


def test_server_set_parameter_from_gc_rejects_constant_parameters(configured_server):
    server, _ = configured_server

    request = SimpleNamespace(parameter_name="/control/immutable_name", parameter_string_value="beta")
    response = SimpleNamespace(success=False, message="")
    result = server.set_parameter_from_gc_callback(request, response)

    assert not result.success
    assert "boot-only" in result.message
    assert server.server_values["/control/immutable_name"] == "alpha"


def test_server_boot_parameter_is_persisted_without_live_application(configured_server):
    server, tmp_path = configured_server
    request = SimpleNamespace(parameter_name="/control/immutable_name", parameter_string_value="beta")
    response = SimpleNamespace(success=False, message="", persisted_parameter_file="")

    result = server.set_boot_parameter_callback(request, response)

    assert result.success, result.message
    assert server.server_values["/control/immutable_name"] == "alpha"
    assert server._pending_boot_values == {"/control/immutable_name": "beta"}
    assert result.persisted_parameter_file.startswith("snapshots/runtime_parameters_")
    assert _bootstrap_default_snapshot_file(tmp_path) == result.persisted_parameter_file
    snapshot = yaml.safe_load(
        _snapshot_path(tmp_path, result.persisted_parameter_file).read_text(encoding="utf-8")
    )
    assert snapshot["/**"]["ros__parameters"]["/control/immutable_name"] == "beta"

    pending = server.get_pending_boot_parameters_callback(
        SimpleNamespace(),
        SimpleNamespace(pending_parameters_yaml="", persisted_parameter_file=""),
    )
    assert yaml.safe_load(pending.pending_parameters_yaml) == {"/control/immutable_name": "beta"}


def test_server_pending_boot_activation_and_full_restart_consume_same_snapshot(configured_server, monkeypatch):
    server, tmp_path = configured_server
    request = SimpleNamespace(parameter_name="/control/immutable_name", parameter_string_value="beta")
    staged = server.set_boot_parameter_callback(
        request,
        SimpleNamespace(success=False, message="", persisted_parameter_file=""),
    )
    assert staged.success, staged.message

    activated = server.activate_pending_boot_parameters_callback(
        SimpleNamespace(),
        SimpleNamespace(success=False, message="", activated_parameter_names=[]),
    )

    assert activated.success, activated.message
    assert activated.activated_parameter_names == ["/control/immutable_name"]
    assert server.server_values["/control/immutable_name"] == "beta"
    assert server._pending_boot_values == {}

    monkeypatch.setenv("III_DRONE_SCHEMA_FILE", str(TEST_SCHEMA_FILE))
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    restarted = ConfigurationServer(
        node_name="configuration_server_restart_test",
        namespace="/configuration/configuration_server_restart_test",
    )
    try:
        assert restarted.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert restarted.server_values["/control/immutable_name"] == "beta"
    finally:
        try:
            restarted.trigger_cleanup()
        except Exception:
            pass
        restarted.destroy_node()


def test_server_boot_parameter_path_rejects_nonconstant_parameter(configured_server):
    server, _ = configured_server
    result = server.set_boot_parameter_callback(
        SimpleNamespace(parameter_name="/control/gains/p", parameter_string_value="2.0"),
        SimpleNamespace(success=False, message="", persisted_parameter_file=""),
    )

    assert result.success is False
    assert "not a constant parameter" in result.message


def test_server_set_parameter_from_gc_requires_live_targets(configured_server):
    server, _ = configured_server

    request = SimpleNamespace(parameter_name="/control/gains/p", parameter_string_value="2.75")
    response = SimpleNamespace(success=False, message="")
    result = server.set_parameter_from_gc_callback(request, response)

    assert not result.success
    assert "No running nodes currently declare" in result.message


def test_server_set_parameter_from_gc_reconciles_before_rejecting_live_update(configured_server, monkeypatch):
    server, _ = configured_server

    def fake_reconcile():
        server.node_registry["/mission/mission_executor/mission_executor"] = ManagedNodeRecord(
            "/mission/mission_executor/mission_executor",
            ["/control/mode"],
            {"/control/mode": "auto"},
        )

    monkeypatch.setattr(server, "reconcile_nodes", fake_reconcile)
    monkeypatch.setattr(server, "_call_set_parameter", lambda *args: (True, ""))
    monkeypatch.setattr(
        server,
        "_call_get_parameters",
        lambda node_fq_name, parameter_names: {
            "/control/mode": "manual",
        },
    )

    request = SimpleNamespace(parameter_name="/control/mode", parameter_string_value="manual")
    response = SimpleNamespace(success=False, message="")
    result = server.set_parameter_from_gc_callback(request, response)

    assert result.success
    assert server.server_values["/control/mode"] == "manual"
