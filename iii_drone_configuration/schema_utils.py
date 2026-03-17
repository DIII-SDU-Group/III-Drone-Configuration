import os
from pathlib import Path


def is_simulation() -> bool:
    return os.environ.get("SIMULATION", "false").lower() == "true"


def resolve_config_base_dir() -> Path:
    return Path(os.path.expanduser(os.environ.get("CONFIG_BASE_DIR", "~/.config")))


def resolve_iii_config_dir() -> Path:
    return resolve_config_base_dir() / "iii_drone"


def resolve_schema_file(
    parameters_path_postfix: str = "parameters/",
    default_parameter_file: str = "parameter_manifest.yaml",
    sim_parameter_file: str = "parameter_manifest.yaml",
) -> Path:
    explicit_file = os.environ.get("III_DRONE_SCHEMA_FILE")
    if explicit_file:
        return Path(os.path.expanduser(explicit_file))

    file_name = sim_parameter_file if is_simulation() else default_parameter_file
    return resolve_iii_config_dir() / parameters_path_postfix / file_name

def resolve_snapshot_dir(snapshot_path_postfix: str = "parameter_snapshots/") -> Path:
    return resolve_iii_config_dir() / snapshot_path_postfix
