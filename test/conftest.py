from pathlib import Path

import pytest
import yaml

import rclpy


TEST_SCHEMA_FILE = Path(__file__).resolve().parent / "resources" / "test_parameter_manifest.yaml"


@pytest.fixture(scope="session", autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def write_bootstrap_parameter_file(
    base_dir: Path,
    *,
    profile_name: str = "sim",
    managed_overrides: dict[str, object] | None = None,
    default_snapshot_file: str | None = None,
) -> Path:
    config_root = base_dir / "iii_drone"
    config_root.mkdir(parents=True, exist_ok=True)

    ros_parameters = {
        "parameters_path_postfix": "parameters/",
        "default_parameter_file": "parameter_manifest.yaml",
        "sim_parameter_file": "parameter_manifest.yaml",
        "parameter_snapshots_path_postfix": "parameter_snapshots/",
        "default_snapshot_file": "default_snapshot.yaml",
        "sim_snapshot_file": "sim_snapshot.yaml",
        "use_sim_time": profile_name == "sim",
    }
    if default_snapshot_file is not None:
        key = "sim_snapshot_file" if profile_name == "sim" else "default_snapshot_file"
        ros_parameters[key] = default_snapshot_file
    if managed_overrides:
        ros_parameters.update(managed_overrides)

    file_name = "ros_params_sim.yaml" if profile_name == "sim" else "ros_params_real.yaml"
    path = config_root / file_name
    path.write_text(
        yaml.safe_dump({"/**": {"ros__parameters": ros_parameters}}, sort_keys=False),
        encoding="utf-8",
    )
    return path
