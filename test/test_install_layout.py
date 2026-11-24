from pathlib import Path
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def test_configuration_executables_are_installed_and_executable():
    libexec_dir = (
        Path(get_package_prefix("iii_drone_configuration"))
        / "lib"
        / "iii_drone_configuration"
    )

    expected = (
        "configuration_server",
        "configuration_client",
        "configuration_server_node.py",
        "configuration_client_node.py",
    )

    for executable in expected:
        path = libexec_dir / executable
        assert path.exists(), f"missing installed executable: {path}"
        assert os.access(
            path, os.X_OK
        ), f"installed executable is not executable: {path}"


def test_immutable_contract_is_installed_from_build_capture_not_source():
    root = (
        Path(get_package_share_directory("iii_drone_configuration"))
        / "configuration_contract"
    )
    assert root.is_dir() and not root.is_symlink()
    expected = {
        "configuration-package.schema.json",
        "migrations.json",
        "package-manifest.json",
        "profiles.json",
        "schema/parameter_manifest.yaml",
        "tracked_defaults/real/default.yaml",
        "tracked_defaults/sim/default.yaml",
    }
    observed = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() or path.is_symlink()
    }
    assert observed == expected
    source_root = Path(__file__).resolve().parents[1]
    for path in root.rglob("*"):
        if path.is_file() or path.is_symlink():
            assert not path.resolve().is_relative_to(source_root)
