import pytest

from rcl_interfaces.msg import ParameterEvent
from rclpy import lifecycle
from rclpy.node import Node
from rclpy.parameter import Parameter

from iii_drone_configuration.configuration import ConfigurationEntry
from iii_drone_configuration.configurator import Configurator
from iii_drone_configuration._native import NativeConfiguratorCore

from conftest import TEST_SCHEMA_FILE


@pytest.fixture(params=[Node, lifecycle.Node], ids=["node", "lifecycle_node"])
def ros_node_factory(request):
    return request.param


@pytest.fixture
def managed_node(monkeypatch, ros_node_factory):
    monkeypatch.setenv("III_DRONE_SCHEMA_FILE", str(TEST_SCHEMA_FILE))
    node = ros_node_factory(f"python_configurator_{ros_node_factory.__name__.replace('.', '_')}")
    try:
        yield node
    finally:
        node.destroy_node()


def test_native_core_exposes_schema_defaults():
    core = NativeConfiguratorCore(str(TEST_SCHEMA_FILE))

    assert core.declare_parameter("/control/gains/p", Parameter.Type.DOUBLE.value) == pytest.approx(1.5)
    entry = core.get_schema_entry("/control/mode")
    assert entry["default_value"] == "auto"
    assert entry["type"] == "string"


def test_configurator_declares_schema_default_and_validates(managed_node):
    configurator = None

    try:
        configurator = Configurator(managed_node)
        configurator.declare_parameters(
            ["/control/gains/p", "/control/gains/i", "/control/immutable_name"],
            [Parameter.Type.DOUBLE, Parameter.Type.DOUBLE, Parameter.Type.STRING],
        )

        assert managed_node.get_parameter("/control/gains/p").value == pytest.approx(1.5)
        configurator.validate()

        failure = configurator.on_set_parameters_callback([Parameter("/control/gains/i", value=2.0)])
        assert not failure.successful

        failure = configurator.on_set_parameters_callback([Parameter("/control/immutable_name", value="beta")])
        assert not failure.successful

        success = configurator.on_set_parameters_callback([Parameter("/control/gains/p", value=3.0)])
        assert success.successful
    finally:
        if configurator is not None:
            configurator.cleanup()


def test_configurator_validates_partial_parameter_sets_with_schema_defaults(managed_node):
    configurator = None

    try:
        configurator = Configurator(managed_node)
        configurator.declare_parameter("/control/gains/i", Parameter.Type.DOUBLE)

        configurator.validate()
        success = configurator.on_set_parameters_callback([Parameter("/control/gains/i", value=0.4)])
        assert success.successful
    finally:
        if configurator is not None:
            configurator.cleanup()


def test_configuration_reads_latest_values(managed_node):
    configurator = None

    try:
        configurator = Configurator(managed_node)
        configurator.declare_parameters(
            ["/control/gains/p", "/control/gains/i"],
            [Parameter.Type.DOUBLE, Parameter.Type.DOUBLE],
        )
        configuration = configurator.create_configuration(
            "controller",
            [
                ConfigurationEntry("/control/gains/p", Parameter.Type.DOUBLE),
                ConfigurationEntry("/control/gains/i", Parameter.Type.DOUBLE),
            ],
        )

        assert configuration.get_parameter("/control/gains/p").value == pytest.approx(1.5)
        managed_node.set_parameters([Parameter("/control/gains/p", value=5.0)])
        assert configuration.get_parameter("/control/gains/p").value == pytest.approx(5.0)
        assert configuration.has_parameter("/control/gains/i")
    finally:
        if configurator is not None:
            configurator.cleanup()


def test_parameter_event_callback_only_for_managed_local_parameters(managed_node):
    seen = []
    configurator = None

    try:
        configurator = Configurator(managed_node, after_parameter_change_callback=seen.append)
        configurator.declare_parameter("/control/gains/p", Parameter.Type.DOUBLE)

        local_event = ParameterEvent()
        local_event.node = managed_node.get_fully_qualified_name()
        changed = Parameter("/control/gains/p", value=2.5).to_parameter_msg()
        local_event.changed_parameters = [changed]
        configurator.parameter_event_callback(local_event)

        foreign_event = ParameterEvent()
        foreign_event.node = "/other/node"
        foreign_event.changed_parameters = [changed]
        configurator.parameter_event_callback(foreign_event)

        assert len(seen) == 1
        assert seen[0].name == "/control/gains/p"
        assert seen[0].value == pytest.approx(2.5)
    finally:
        if configurator is not None:
            configurator.cleanup()
