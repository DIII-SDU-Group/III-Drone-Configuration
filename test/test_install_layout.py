from pathlib import Path
import os

from ament_index_python.packages import get_package_prefix


def test_configuration_executables_are_installed_and_executable():
    libexec_dir = Path(get_package_prefix("iii_drone_configuration")) / "lib" / "iii_drone_configuration"

    expected = (
        "configuration_server",
        "configuration_client",
        "configuration_server_node.py",
        "configuration_client_node.py",
    )

    for executable in expected:
        path = libexec_dir / executable
        assert path.exists(), f"missing installed executable: {path}"
        assert os.access(path, os.X_OK), f"installed executable is not executable: {path}"
