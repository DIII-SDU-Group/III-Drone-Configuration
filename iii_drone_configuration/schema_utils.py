import os
from pathlib import Path
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory


def is_simulation() -> bool:
    return os.environ.get("SIMULATION", "false").lower() == "true"


def profile_name_from_environment() -> str:
    return "sim" if is_simulation() else "real"


def resolve_config_base_dir() -> Path:
    return Path(os.path.expanduser(os.environ.get("CONFIG_BASE_DIR", "~/.config")))


def resolve_iii_config_dir() -> Path:
    return resolve_config_base_dir() / "iii_drone"


def _ros_params_filename(profile_name: str) -> str:
    return "ros_params_sim.yaml" if profile_name == "sim" else "ros_params_real.yaml"


def _default_snapshot_parameter_name(profile_name: str) -> str:
    return "sim_snapshot_file" if profile_name == "sim" else "default_snapshot_file"


def _workspace_root_from_env() -> Path | None:
    workspace_dir = os.environ.get("WORKSPACE_DIR")
    if workspace_dir:
        return Path(os.path.expanduser(workspace_dir))

    inferred = Path(__file__).resolve().parents[3]
    if (inferred / "src").exists() and (inferred / "setup").exists():
        return inferred
    return None


def _source_config_dir() -> Path | None:
    workspace_root = _workspace_root_from_env()
    if workspace_root is not None:
        source_copy = workspace_root / "src" / "III-Drone-Configuration" / "config"
        if source_copy.exists():
            return source_copy

    try:
        package_share = Path(get_package_share_directory("iii_drone_configuration"))
        installed = package_share / "config"
        if installed.exists():
            return installed
    except Exception:
        pass

    return None


def _copy_if_missing(source: Path, target: Path, *, overwrite: bool = False) -> bool:
    if not source.exists():
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and not overwrite:
        return False
    shutil.copyfile(source, target)
    return True


def resolve_bootstrap_parameter_file(profile_name: str) -> Path:
    filename = _ros_params_filename(profile_name)

    configured = resolve_iii_config_dir() / filename
    if configured.exists():
        return configured

    source_config_dir = _source_config_dir()
    if source_config_dir is not None:
        source_copy = source_config_dir / filename
        if source_copy.exists():
            return source_copy

    return configured


def ensure_writable_bootstrap_parameter_file(profile_name: str) -> Path:
    filename = _ros_params_filename(profile_name)
    target = resolve_iii_config_dir() / filename
    target.parent.mkdir(parents=True, exist_ok=True)

    if target.exists():
        return target

    source = resolve_bootstrap_parameter_file(profile_name)
    if source.exists() and source != target:
        shutil.copyfile(source, target)
        return target

    target.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
    return target


def load_parameter_file(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def save_parameter_file(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file, sort_keys=False)


def resolve_schema_file(
    parameters_path_postfix: str = "parameters/",
    default_parameter_file: str = "parameter_manifest.yaml",
    sim_parameter_file: str = "parameter_manifest.yaml",
) -> Path:
    explicit_file = os.environ.get("III_DRONE_SCHEMA_FILE")
    if explicit_file:
        return Path(os.path.expanduser(explicit_file))

    file_name = sim_parameter_file if is_simulation() else default_parameter_file
    configured = resolve_iii_config_dir() / parameters_path_postfix / file_name
    if configured.exists():
        return configured

    source_config_dir = _source_config_dir()
    if source_config_dir is not None:
        source_copy = source_config_dir / parameters_path_postfix / file_name
        if source_copy.exists():
            return source_copy

    return configured


def seed_runtime_configuration(profile_name: str, *, overwrite: bool = False) -> dict[str, Path]:
    """Seed the writable runtime config root from package defaults.

    The runtime config root remains authoritative after seeding. Existing files are
    preserved unless overwrite=True, so saved defaults survive devcontainer rebuilds.
    """
    source_config_dir = _source_config_dir()
    iii_config_dir = resolve_iii_config_dir()
    seeded: dict[str, Path] = {}

    iii_config_dir.mkdir(parents=True, exist_ok=True)
    (iii_config_dir / "parameter_snapshots").mkdir(parents=True, exist_ok=True)

    if source_config_dir is None:
        ensure_writable_bootstrap_parameter_file(profile_name)
        return seeded

    profile_bootstrap = _ros_params_filename(profile_name)
    if _copy_if_missing(source_config_dir / profile_bootstrap, iii_config_dir / profile_bootstrap, overwrite=overwrite):
        seeded["bootstrap"] = iii_config_dir / profile_bootstrap

    parameters_source_dir = source_config_dir / "parameters"
    parameters_target_dir = iii_config_dir / "parameters"
    if parameters_source_dir.exists():
        for source_file in parameters_source_dir.glob("*.yaml"):
            target_file = parameters_target_dir / source_file.name
            if _copy_if_missing(source_file, target_file, overwrite=overwrite):
                seeded[f"parameters/{source_file.name}"] = target_file

    ensure_writable_bootstrap_parameter_file(profile_name)
    return seeded


def resolve_snapshot_dir(snapshot_path_postfix: str = "parameter_snapshots/") -> Path:
    return resolve_iii_config_dir() / snapshot_path_postfix


def resolve_default_parameter_file_name(profile_name: str) -> str:
    bootstrap_file = resolve_bootstrap_parameter_file(profile_name)
    if bootstrap_file.exists():
        ros_params = load_parameter_file(bootstrap_file)
        name = (
            ros_params.get("/**", {})
            .get("ros__parameters", {})
            .get(_default_snapshot_parameter_name(profile_name))
        )
        if isinstance(name, str) and name:
            return name

    return "sim_snapshot.yaml" if profile_name == "sim" else "default_snapshot.yaml"


def persist_default_parameter_file_name(profile_name: str, file_name: str) -> Path:
    bootstrap_file = ensure_writable_bootstrap_parameter_file(profile_name)
    ros_params = load_parameter_file(bootstrap_file)
    ros_params.setdefault("/**", {}).setdefault("ros__parameters", {})[
        _default_snapshot_parameter_name(profile_name)
    ] = file_name
    save_parameter_file(bootstrap_file, ros_params)
    return bootstrap_file


def resolve_active_parameter_file(profile_name: str) -> Path:
    explicit_file = os.environ.get("III_SYSTEM_PARAMETER_FILE")
    if explicit_file:
        return Path(os.path.expanduser(explicit_file))

    default_file_name = resolve_default_parameter_file_name(profile_name)
    snapshot_file = resolve_snapshot_dir() / default_file_name
    if snapshot_file.exists():
        return snapshot_file

    return resolve_bootstrap_parameter_file(profile_name)
