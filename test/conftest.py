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
    active_parameter_file: str | None = None,
) -> Path:
    config_root = base_dir / "iii_drone"
    config_root.mkdir(parents=True, exist_ok=True)

    selector_dir = config_root / "profiles"
    selector_dir.mkdir(parents=True, exist_ok=True)

    parameter_set_root = config_root / "parameter_sets" / profile_name
    tracked_path = parameter_set_root / "tracked" / "default.yaml"
    tracked_path.parent.mkdir(parents=True, exist_ok=True)
    tracked_path.write_text(
        yaml.safe_dump(
            {"/**": {"ros__parameters": managed_overrides or {}}},
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    selector_path = selector_dir / f"{profile_name}.yaml"
    selector_path.write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "active_parameter_set": active_parameter_file or "tracked/default.yaml",
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    if active_parameter_file and active_parameter_file != "tracked/default.yaml":
        active_path = parameter_set_root / active_parameter_file
        active_path.parent.mkdir(parents=True, exist_ok=True)
        active_path.write_text(
            yaml.safe_dump(
                {"/**": {"ros__parameters": managed_overrides or {}}},
                sort_keys=False,
            ),
            encoding="utf-8",
        )

    return selector_path
