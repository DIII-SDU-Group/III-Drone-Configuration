import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest
import yaml

from rcl_interfaces.srv import GetParameters
from rclpy import executors, lifecycle
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python

from iii_drone_interfaces.srv import GetDeclaredParameters, LoadParameters, SetParameterFromGC
from iii_drone_configuration.configuration_server_node import ConfigurationServer
from iii_drone_configuration.configurator import Configurator

from conftest import TEST_SCHEMA_FILE


def wait_until(predicate, timeout=5.0, period=0.05, message="condition not met"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(period)
    raise AssertionError(message)


class ExecutorHarness:
    def __init__(self, server):
        self.executor = executors.MultiThreadedExecutor()
        self.nodes = []
        self.server = server
        self.executor.add_node(server)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def add_node(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)

    def shutdown(self):
        for node in reversed(self.nodes):
            configurator = getattr(node, "_test_configurator", None)
            if configurator is not None:
                configurator.cleanup()
            self.executor.remove_node(node)
            node.destroy_node()
        self.nodes.clear()

        self.executor.remove_node(self.server)
        try:
            self.server.trigger_deactivate()
        except Exception:
            pass
        try:
            self.server.trigger_cleanup()
        except Exception:
            pass
        self.server.destroy_node()
        self.executor.shutdown()
        self._thread.join(timeout=2.0)


def make_managed_node(node_cls, name):
    node = node_cls(name)
    configurator = Configurator(node)
    configurator.declare_parameters(
        ["/control/gains/p", "/control/gains/i", "/control/mode", "/control/immutable_name"],
        [Parameter.Type.DOUBLE, Parameter.Type.DOUBLE, Parameter.Type.STRING, Parameter.Type.STRING],
    )
    configurator.validate()
    node._test_configurator = configurator
    return node


def call_service(client_node, srv_type, service_name, request):
    client = client_node.create_client(srv_type, service_name)
    try:
        wait_until(lambda: client.wait_for_service(timeout_sec=0.1), timeout=5.0, message=f"{service_name} unavailable")
        future = client.call_async(request)
        wait_until(lambda: future.done(), timeout=5.0, message=f"{service_name} call timed out")
        return future.result()
    finally:
        client_node.destroy_client(client)


def get_remote_parameter(client_node, node_fq_name, parameter_name):
    request = GetParameters.Request()
    request.names = [parameter_name]
    response = call_service(client_node, GetParameters, f"{node_fq_name}/get_parameters", request)
    return parameter_value_to_python(response.values[0])


@pytest.fixture
def running_graph(monkeypatch, tmp_path):
    monkeypatch.setenv("III_DRONE_SCHEMA_FILE", str(TEST_SCHEMA_FILE))
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))

    server = ConfigurationServer(node_name="configuration_server_e2e", namespace="/configuration/configuration_server")
    assert server.trigger_configure().name == "SUCCESS"
    assert server.trigger_activate().name == "SUCCESS"

    harness = ExecutorHarness(server)
    client_node = Node("configuration_e2e_client")
    harness.add_node(client_node)

    try:
        yield harness, server, client_node, tmp_path
    finally:
        harness.shutdown()


@pytest.mark.parametrize("node_cls", [Node, lifecycle.Node], ids=["node", "lifecycle_node"])
def test_python_managed_nodes_sync_and_late_join(running_graph, node_cls):
    harness, server, client_node, _ = running_graph

    node_a = make_managed_node(node_cls, f"py_managed_a_{node_cls.__name__.replace('.', '_')}")
    harness.add_node(node_a)
    wait_until(lambda: node_a.get_fully_qualified_name() in server.node_registry, message="node_a not discovered")

    set_request = SetParameterFromGC.Request()
    set_request.parameter_name = "/control/gains/p"
    set_request.parameter_string_value = "4.5"
    set_response = call_service(
        client_node,
        SetParameterFromGC,
        "/configuration/configuration_server/set_parameter_from_gc",
        set_request,
    )
    assert set_response.success
    wait_until(lambda: node_a.get_parameter("/control/gains/p").value == pytest.approx(4.5), message="node_a not synced")

    node_b = make_managed_node(node_cls, f"py_managed_b_{node_cls.__name__.replace('.', '_')}")
    harness.add_node(node_b)
    wait_until(lambda: node_b.get_fully_qualified_name() in server.node_registry, message="node_b not discovered")
    wait_until(lambda: node_b.get_parameter("/control/gains/p").value == pytest.approx(4.5), message="late join node not synced")

    declared_response = call_service(
        client_node,
        GetDeclaredParameters,
        "/configuration/configuration_server/get_declared_parameters",
        GetDeclaredParameters.Request(),
    )
    declared = yaml.safe_load(declared_response.declared_parameters_yaml)
    assert node_a.get_fully_qualified_name() in declared["/control/gains/p"]
    assert node_b.get_fully_qualified_name() in declared["/control/gains/p"]


@pytest.mark.parametrize("node_cls", [Node, lifecycle.Node], ids=["node", "lifecycle_node"])
def test_python_managed_nodes_reject_invalid_updates_and_accept_snapshot_load(running_graph, node_cls):
    harness, server, client_node, tmp_path = running_graph

    node = make_managed_node(node_cls, f"py_snapshot_{node_cls.__name__.replace('.', '_')}")
    harness.add_node(node)
    wait_until(lambda: node.get_fully_qualified_name() in server.node_registry, message="managed node not discovered")

    invalid_request = SetParameterFromGC.Request()
    invalid_request.parameter_name = "/control/gains/i"
    invalid_request.parameter_string_value = "9.0"
    invalid_response = call_service(
        client_node,
        SetParameterFromGC,
        "/configuration/configuration_server/set_parameter_from_gc",
        invalid_request,
    )
    assert not invalid_response.success
    assert node.get_parameter("/control/gains/i").value == pytest.approx(0.3)

    snapshot_path = tmp_path / "iii_drone" / "parameter_snapshots" / "e2e_snapshot.yaml"
    snapshot_path.parent.mkdir(parents=True, exist_ok=True)
    snapshot_path.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "/control/gains/p": 2.5,
                        "/control/gains/i": 1.0,
                        "/control/mode": "manual",
                    }
                }
            },
            sort_keys=False,
        )
    )

    load_request = LoadParameters.Request()
    load_request.file = "e2e_snapshot.yaml"
    load_request.set_as_default = False
    load_response = call_service(
        client_node,
        LoadParameters,
        "/configuration/configuration_server/load_parameters",
        load_request,
    )
    assert load_response.success
    wait_until(lambda: node.get_parameter("/control/gains/p").value == pytest.approx(2.5), message="snapshot p not loaded")
    wait_until(lambda: node.get_parameter("/control/mode").value == "manual", message="snapshot mode not loaded")
    assert node.get_parameter("/control/immutable_name").value == "alpha"


@pytest.mark.parametrize("cpp_node_type", ["node", "lifecycle"])
def test_cpp_managed_node_discovers_and_syncs_with_server(running_graph, cpp_node_type):
    harness, server, client_node, _ = running_graph

    executable = Path.cwd() / "configurator_managed_test_node"
    assert executable.exists(), f"Missing C++ test executable at {executable}"

    node_name = f"cpp_managed_{cpp_node_type}"
    process = subprocess.Popen([str(executable), cpp_node_type, node_name], cwd=str(Path.cwd()))
    node_fq_name = f"/{node_name}"

    try:
        wait_until(lambda: node_fq_name in server.node_registry, timeout=10.0, message="cpp node not discovered")

        request = SetParameterFromGC.Request()
        request.parameter_name = "/control/gains/p"
        request.parameter_string_value = "6.25"
        response = call_service(
            client_node,
            SetParameterFromGC,
            "/configuration/configuration_server/set_parameter_from_gc",
            request,
        )
        assert response.success

        wait_until(
            lambda: get_remote_parameter(client_node, node_fq_name, "/control/gains/p") == pytest.approx(6.25),
            timeout=10.0,
            message="cpp node parameter not updated",
        )
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)
