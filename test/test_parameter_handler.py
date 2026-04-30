import os
from pathlib import Path

import pytest
import yaml

from iii_drone_configuration._native import NativeConfiguratorCore
from iii_drone_configuration.parameter_handler import ParameterHandler

from conftest import TEST_SCHEMA_FILE


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _production_schema_file() -> Path:
    configured = os.environ.get("III_DRONE_PRODUCTION_SCHEMA_FILE")
    if configured:
        return Path(configured)
    return _workspace_root() / "src" / "III-Drone-Configuration" / "config" / "parameters" / "parameter_manifest.yaml"


def _production_ros_params_file(env_var_name: str, fallback_name: str) -> Path:
    configured = os.environ.get(env_var_name)
    if configured:
        return Path(configured)
    return _workspace_root() / "src" / "III-Drone-Configuration" / "config" / fallback_name


def test_parameter_handler_loads_and_validates_from_file():
    handler = ParameterHandler.from_parameter_file(str(TEST_SCHEMA_FILE))

    assert handler.get_param_value("/control/mode") == "auto"
    assert handler.get_param("/control/gains/p")["value"] == pytest.approx(1.5)
    assert "/perception/pl_mapper/weights" in handler.get_all_params()


def test_parameter_handler_can_set_and_tracks_changes():
    handler = ParameterHandler.from_parameter_file(str(TEST_SCHEMA_FILE))

    handler.set_param("/control/gains/p", "2.5", parameter_initialized=True)
    assert handler.get_param_value("/control/gains/p") == pytest.approx(2.5)
    assert handler.any_params_changed

    handler.reset_changed_parameters(["/control/gains/p"])
    handler.set_param("/control/gains/p", 3.0, parameter_initialized=True)
    assert not handler.any_params_changed


def test_parameter_handler_rejects_invalid_expression_and_constant_updates():
    handler = ParameterHandler.from_parameter_file(str(TEST_SCHEMA_FILE))

    with pytest.raises(Exception):
        handler.can_set_param("/control/gains/i", 4.0)

    with pytest.raises(AttributeError):
        handler.can_set_param("/control/immutable_name", "beta")


def test_parameter_handler_accepts_negative_numeric_bounds(tmp_path):
    schema_path = tmp_path / "negative_bounds.yaml"
    schema_path.write_text(
        """
control:
  yaw_rate:
    type: float
    value: 0.0
    min: -3.1415
    max: 3.1415
"""
    )

    handler = ParameterHandler.from_parameter_file(str(schema_path))
    assert handler.get_param_value("/control/yaw_rate") == pytest.approx(0.0)


def test_parameter_handler_loads_from_yaml_string_and_serializes():
    raw_yaml = TEST_SCHEMA_FILE.read_text()
    handler = ParameterHandler.from_raw_yaml_string(raw_yaml)

    serialized = handler.get_parameters_yaml_string()
    assert "/control/gains/p" in serialized
    assert handler.cast_param_value("/control/enabled", "false") is False
    assert handler._evaluate_expression("/control/gains/p + /control/gains/i", handler.params_dict) == pytest.approx(1.8)


def test_parameter_handler_save_and_reload(tmp_path):
    handler = ParameterHandler.from_parameter_file(str(TEST_SCHEMA_FILE))
    handler.set_param("/control/mode", "manual", parameter_initialized=True)

    out_file = tmp_path / "saved.yaml"
    handler.save_parameters(str(out_file), overwrite=True)

    reloaded = ParameterHandler.from_parameter_file(str(out_file))
    assert reloaded.get_param_value("/control/mode") == "manual"


def test_production_schema_file_loads_via_parameter_handler():
    core = NativeConfiguratorCore(str(_production_schema_file()))

    managed_names = set(core.schema_parameter_names())
    assert "/control/maneuver_controller/landed_altitude_threshold" in managed_names
    assert "/payload/charger_gripper/gripper_command_interface" in managed_names


def test_production_schema_defaults_validate():
    core = NativeConfiguratorCore(str(_production_schema_file()))
    candidates = {}
    for name in core.schema_parameter_names():
        entry = core.get_schema_entry(name)
        candidates[name] = {"type": entry["parameter_type"], "value": entry["default_value"]}

    core.validate_parameter_map(candidates, True)


@pytest.mark.parametrize(
    ("env_var_name", "fallback_name"),
    [
        ("III_DRONE_PRODUCTION_ROS_PARAMS_REAL_FILE", "parameter_sets/real/tracked/default.yaml"),
        ("III_DRONE_PRODUCTION_ROS_PARAMS_SIM_FILE", "parameter_sets/sim/tracked/default.yaml"),
    ],
)
def test_production_ros_param_files_are_schema_compatible(env_var_name, fallback_name):
    production_schema_file = _production_schema_file()
    ros_params_file = _production_ros_params_file(env_var_name, fallback_name)

    core = NativeConfiguratorCore(str(production_schema_file))
    ros_params = yaml.safe_load(Path(ros_params_file).read_text())
    ros_parameters = ros_params["/**"]["ros__parameters"]

    for name, value in ros_parameters.items():
        schema_entry = core.get_schema_entry(name)
        assert schema_entry["name"] == name, f"Unexpected production ros param key: {name}"

        default_value = schema_entry["default_value"]
        if isinstance(default_value, bool):
            assert isinstance(value, bool)
        elif isinstance(default_value, int) and not isinstance(default_value, bool):
            assert isinstance(value, int) and not isinstance(value, bool)
        elif isinstance(default_value, float):
            assert isinstance(value, (int, float)) and not isinstance(value, bool)
        elif isinstance(default_value, str):
            assert isinstance(value, str)
            options = schema_entry.get("options", [])
            if options:
                assert value in options
        elif isinstance(default_value, list):
            assert isinstance(value, list)
            if default_value:
                sample = default_value[0]
                if isinstance(sample, bool):
                    assert all(isinstance(item, bool) for item in value)
                elif isinstance(sample, int) and not isinstance(sample, bool):
                    assert all(isinstance(item, int) and not isinstance(item, bool) for item in value)
                elif isinstance(sample, float):
                    assert all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
                elif isinstance(sample, str):
                    assert all(isinstance(item, str) for item in value)
