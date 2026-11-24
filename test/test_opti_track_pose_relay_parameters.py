from pathlib import Path

import pytest
import yaml

from iii_drone_configuration._native import NativeConfiguratorCore
from iii_drone_configuration.reconciliation import (
    _flatten_schema,
    _parameter_values_from_document,
    _validate_candidates,
)

CONFIG = Path(__file__).resolve().parents[1] / "config"
PREFIX = "/opti_track/pose_relay/"
# The pose relay (iii_drone_core opti_track_pose_relay) reads exactly these
# parameters from the opti_track profile parameter file.
EXPECTED = {
    "rigid_body_id": {"type": "int", "value": -1, "min": -1, "max": 999},
    "lab_ros_domain_id": {"type": "int", "value": 0, "min": 0, "max": 232},
    "output_rate_hz": {"type": "float", "value": 50.0, "min": 10.0, "max": 120.0},
    "stale_timeout_s": {"type": "float", "value": 0.15, "min": 0.05, "max": 1.0},
    "position_variance_m2": {"type": "float", "value": 0.0001, "min": 0.000001, "max": 1.0},
    "orientation_variance_rad2": {"type": "float", "value": 0.0004, "min": 0.000001, "max": 1.0},
    "send_origin": {"type": "bool", "value": True},
    "origin_latitude_deg": {"type": "float", "value": 55.3672, "min": -90.0, "max": 90.0},
    "origin_longitude_deg": {"type": "float", "value": 10.431, "min": -180.0, "max": 180.0},
    "origin_altitude_m": {"type": "float", "value": 20.0, "min": -500.0, "max": 9000.0},
}


def _schema(name: str) -> dict:
    return yaml.safe_load((CONFIG / "parameters" / name).read_text(encoding="utf-8"))


def _tracked_default(profile: str) -> dict:
    document = yaml.safe_load(
        (CONFIG / "parameter_sets" / profile / "tracked" / "default.yaml").read_text(
            encoding="utf-8"
        )
    )
    return _parameter_values_from_document(document, label=f"{profile} default")


def test_pose_relay_parameters_are_boot_only_schema_entries():
    manifest = _schema("parameter_manifest.yaml")["opti_track"]["pose_relay"]
    # The compatibility copy of the schema carries the same declarations.
    assert _schema("parameters.yaml")["opti_track"]["pose_relay"] == manifest

    assert set(manifest) == set(EXPECTED)
    for name, expected in EXPECTED.items():
        entry = manifest[name]
        assert entry["description"]
        assert entry["constant"] is True
        assert {key: entry[key] for key in expected} == expected
        assert set(entry) == {*expected, "constant", "description"}


@pytest.mark.parametrize("profile", ["real", "sim"])
def test_tracked_defaults_carry_the_schema_defaults_and_validate(profile):
    entries = _flatten_schema(_schema("parameter_manifest.yaml"))
    values = _tracked_default(profile)

    assert {
        name[len(PREFIX):]: value
        for name, value in values.items()
        if name.startswith(PREFIX)
    } == {name: expected["value"] for name, expected in EXPECTED.items()}
    assert _validate_candidates(values, entries) == (True, ())


@pytest.mark.parametrize(
    ("name", "value", "reason"),
    [
        ("rigid_body_id", -2, "below minimum"),
        ("rigid_body_id", 1000, "above maximum"),
        ("lab_ros_domain_id", 233, "above maximum"),
        ("output_rate_hz", 9.5, "below minimum"),
        ("output_rate_hz", 121.0, "above maximum"),
        ("stale_timeout_s", 0.04, "below minimum"),
        ("stale_timeout_s", 1.5, "above maximum"),
        ("position_variance_m2", 0.0, "below minimum"),
        ("orientation_variance_rad2", 0.0, "below minimum"),
        ("origin_latitude_deg", 90.5, "above maximum"),
        ("origin_longitude_deg", -180.5, "below minimum"),
    ],
)
def test_out_of_range_pose_relay_values_are_rejected_by_both_validators(
    name, value, reason
):
    parameter = PREFIX + name
    entries = _flatten_schema(_schema("parameter_manifest.yaml"))
    candidates = {**_tracked_default("real"), parameter: value}

    valid, reasons = _validate_candidates(candidates, entries)
    assert not valid
    assert reasons == (f"{parameter}: value is {reason}",)

    core = NativeConfiguratorCore(str(CONFIG / "parameters" / "parameter_manifest.yaml"))
    entry = core.get_schema_entry(parameter)
    with pytest.raises(RuntimeError, match="schema (minimum|maximum)"):
        core.validate_parameter_value(parameter, value, entry["parameter_type"], {}, True)
