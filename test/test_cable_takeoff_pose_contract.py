from pathlib import Path
import shutil

import yaml

from iii_drone_configuration.installed_contracts import load_installed_contract

CONFIG = Path(__file__).resolve().parents[1] / "config"
CORE = Path(__file__).resolve().parents[2] / "III-Drone-Core"
PARAMETER = "/control/maneuver_controller/cable_takeoff_reached_pose_norm_threshold"


def _cpp_function(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"unterminated C++ function: {signature}")


def test_cable_takeoff_pose_norm_threshold_is_profile_scoped_and_consistent():
    schema = yaml.safe_load((CONFIG / "parameters" / "parameters.yaml").read_text())
    manifest = yaml.safe_load((CONFIG / "parameters" / "parameter_manifest.yaml").read_text())
    declaration = schema["control"]["maneuver_controller"]["cable_takeoff_reached_pose_norm_threshold"]
    manifest_entry = manifest["control"]["maneuver_controller"]["cable_takeoff_reached_pose_norm_threshold"]
    assert declaration == manifest_entry
    assert declaration == {"type": "float", "value": 0.10, "min": 0.05, "max": 0.25}

    profile_values = {}
    for profile in ("sim", "real"):
        values = yaml.safe_load(
            (CONFIG / "parameter_sets" / profile / "tracked" / "default.yaml").read_text()
        )["/**"]["ros__parameters"]
        profile_values[profile] = values[PARAMETER]
    assert profile_values == {"sim": 0.15, "real": 0.10}

    controller = (CORE / "src/control/maneuver_controller_node/maneuver_controller_node.cpp").read_text()
    assert f'DeclareParameter("{PARAMETER}"' in controller
    takeoff_scope = controller.split(
        'CreateConfiguration("cable_takeoff_maneuver_server", {', 1
    )[1].split("});", 1)[0]
    assert f'ConfigurationEntry("{PARAMETER}"' in takeoff_scope
    assert "reached_position_euclidean_distance_threshold" not in takeoff_scope

    takeoff = (CORE / "src/control/maneuver/cable_takeoff_maneuver_server.cpp").read_text()
    metric = _cpp_function(takeoff, "double CableTakeoffPoseNorm(")
    success = _cpp_function(takeoff, "bool CableTakeoffManeuverServer::hasSucceeded(")
    rebase = _cpp_function(takeoff, "bool CableTakeoffManeuverServer::rebaseExecution(")
    failure = _cpp_function(takeoff, "bool CableTakeoffManeuverServer::hasFailed(")
    assert "state.position()[2]" in metric
    assert "state.yaw()" in metric
    assert "reference.position()[2]" in metric
    assert "reference.yaw()" in metric
    assert "CableTakeoffPoseNorm(state, target_reference)" in success
    assert PARAMETER in success
    assert "std::chrono::milliseconds(1500)" in success
    assert "CableTakeoffPoseNorm(stopped_state, *target_reference_)" in rebase
    assert "CableTakeoffPoseNorm(state, target_reference)" in failure
    assert PARAMETER in failure


def test_source_configuration_contract_authenticates_updated_cable_takeoff_values(tmp_path):
    source_contract = CONFIG / "configuration_contract"
    root = tmp_path / "captured-contract"
    (root / "schema").mkdir(parents=True)
    (root / "tracked_defaults" / "real").mkdir(parents=True)
    (root / "tracked_defaults" / "sim").mkdir(parents=True)
    for name in (
        "package-manifest.json",
        "configuration-package.schema.json",
        "profiles.json",
        "migrations.json",
    ):
        shutil.copyfile(source_contract / name, root / name)
    shutil.copyfile(
        CONFIG / "parameters" / "parameter_manifest.yaml",
        root / "schema" / "parameter_manifest.yaml",
    )
    for profile in ("real", "sim"):
        shutil.copyfile(
            CONFIG / "parameter_sets" / profile / "tracked" / "default.yaml",
            root / "tracked_defaults" / profile / "default.yaml",
        )

    result = load_installed_contract(root)

    assert result.contract.manifest_id
    assert set(result.verified_artifacts) == {
        "configuration-package.schema.json",
        "migrations.json",
        "profiles.json",
        "schema/parameter_manifest.yaml",
        "tracked_defaults/real/default.yaml",
        "tracked_defaults/sim/default.yaml",
    }
    for profile, expected in (("real", 0.10), ("sim", 0.15)):
        tracked_set = result.contract.default_set(profile)
        values = yaml.safe_load((root / tracked_set.path).read_text())["/**"]["ros__parameters"]
        assert values[PARAMETER] == expected
