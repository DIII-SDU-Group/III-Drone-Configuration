import os
from pathlib import Path, PurePosixPath
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory


LEGACY_SUPPORT_PARAMETER_NAMES = {
    "parameters_path_postfix",
    "default_parameter_file",
    "sim_parameter_file",
    "parameter_snapshots_path_postfix",
    "default_snapshot_file",
    "sim_snapshot_file",
    "use_sim_time",
}

DEFAULT_TRACKED_PARAMETER_SET_REFERENCE = "tracked/default.yaml"
PROFILE_SELECTOR_ACTIVE_FIELD = "active_parameter_set"


def is_simulation() -> bool:
    return os.environ.get("SIMULATION", "false").lower() == "true"


def profile_name_from_environment() -> str:
    return "sim" if is_simulation() else "real"


def resolve_config_base_dir() -> Path:
    return Path(os.path.expanduser(os.environ.get("CONFIG_BASE_DIR", "~/.config")))


def resolve_iii_config_dir() -> Path:
    return resolve_config_base_dir() / "iii_drone"


def resolve_profiles_dir() -> Path:
    return resolve_iii_config_dir() / "profiles"


def resolve_profile_dir() -> Path:
    return resolve_profiles_dir()


def resolve_profile_selector_file(profile_name: str) -> Path:
    return resolve_profiles_dir() / f"{profile_name}.yaml"


def resolve_profile_file(profile_name: str) -> Path:
    return resolve_profile_selector_file(profile_name)


def resolve_parameter_sets_dir(profile_name: str) -> Path:
    return resolve_iii_config_dir() / "parameter_sets" / profile_name


def resolve_parameter_set_dir(profile_name: str) -> Path:
    return resolve_parameter_sets_dir(profile_name)


def resolve_tracked_parameter_set_path(profile_name: str) -> Path:
    return resolve_parameter_set_path(profile_name, DEFAULT_TRACKED_PARAMETER_SET_REFERENCE)


def normalize_parameter_set_reference(reference: str, *, default_subdir: str = "snapshots") -> str:
    normalized = reference.strip()
    if not normalized:
        raise ValueError("Parameter set reference must not be empty")

    candidate = PurePosixPath(normalized)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise ValueError(f"Invalid parameter set reference '{reference}'")

    if len(candidate.parts) == 1:
        candidate = PurePosixPath(default_subdir) / candidate.name

    return candidate.as_posix()


def resolve_parameter_set_path(profile_name: str, reference: str) -> Path:
    normalized = normalize_parameter_set_reference(reference, default_subdir="snapshots")
    return resolve_parameter_sets_dir(profile_name) / PurePosixPath(normalized)


def resolve_schema_parameters_dir() -> Path:
    return resolve_iii_config_dir() / "parameters"


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


def _source_profile_selector_file(profile_name: str) -> Path | None:
    source_config_dir = _source_config_dir()
    if source_config_dir is None:
        return None
    return source_config_dir / "profiles" / f"{profile_name}.yaml"


def _source_parameter_sets_dir(profile_name: str) -> Path | None:
    source_config_dir = _source_config_dir()
    if source_config_dir is None:
        return None
    return source_config_dir / "parameter_sets" / profile_name


def _copy_if_missing(source: Path, target: Path, *, overwrite: bool = False) -> bool:
    if not source.exists():
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and not overwrite:
        return False
    shutil.copyfile(source, target)
    return True


def _copy_tree_if_missing(source_dir: Path, target_dir: Path, *, overwrite: bool = False) -> dict[str, Path]:
    copied: dict[str, Path] = {}
    if not source_dir.exists():
        return copied

    for source_file in sorted(path for path in source_dir.rglob("*") if path.is_file()):
        relative_path = source_file.relative_to(source_dir)
        target_file = target_dir / relative_path
        if _copy_if_missing(source_file, target_file, overwrite=overwrite):
            copied[str(relative_path)] = target_file

    return copied


def load_parameter_file(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def save_parameter_file(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file, sort_keys=False)


def build_parameter_file_data(parameter_values: dict[str, object]) -> dict:
    return {
        "/**": {
            "ros__parameters": dict(sorted(parameter_values.items())),
        }
    }


def resolve_schema_file(
    parameters_path_postfix: str = "parameters/",
    default_parameter_file: str = "parameter_manifest.yaml",
    sim_parameter_file: str = "parameter_manifest.yaml",
) -> Path:
    explicit_file = os.environ.get("III_DRONE_SCHEMA_FILE")
    if explicit_file:
        return Path(os.path.expanduser(explicit_file))

    file_name = sim_parameter_file if is_simulation() else default_parameter_file
    configured = resolve_iii_config_dir() / Path(parameters_path_postfix) / file_name
    if configured.exists():
        return configured

    source_config_dir = _source_config_dir()
    if source_config_dir is not None:
        source_copy = source_config_dir / Path(parameters_path_postfix) / file_name
        if source_copy.exists():
            return source_copy

    return configured


def _legacy_bootstrap_filename(profile_name: str) -> str:
    return "ros_params_sim.yaml" if profile_name == "sim" else "ros_params_real.yaml"


def _legacy_bootstrap_path(profile_name: str) -> Path:
    return resolve_iii_config_dir() / _legacy_bootstrap_filename(profile_name)


def _legacy_snapshot_dir() -> Path:
    return resolve_iii_config_dir() / "parameter_snapshots"


def _legacy_snapshot_reference(profile_name: str, bootstrap_data: dict) -> str | None:
    parameter_name = "sim_snapshot_file" if profile_name == "sim" else "default_snapshot_file"
    ros_parameters = bootstrap_data.get("/**", {}).get("ros__parameters", {})
    snapshot_name = ros_parameters.get(parameter_name)
    if not isinstance(snapshot_name, str) or not snapshot_name:
        return None
    return normalize_parameter_set_reference(snapshot_name, default_subdir="snapshots")


def _extract_managed_parameter_values(parameter_file_data: dict) -> dict[str, object]:
    ros_parameters = parameter_file_data.get("/**", {}).get("ros__parameters", {})
    if not isinstance(ros_parameters, dict):
        return {}
    return {
        name: value
        for name, value in ros_parameters.items()
        if name not in LEGACY_SUPPORT_PARAMETER_NAMES
    }


def load_active_parameter_set_reference(profile_name: str) -> str:
    selector_file = resolve_profile_selector_file(profile_name)
    if selector_file.exists():
        selector_data = load_parameter_file(selector_file)
        reference = selector_data.get(PROFILE_SELECTOR_ACTIVE_FIELD)
        if isinstance(reference, str) and reference:
            return normalize_parameter_set_reference(reference, default_subdir="tracked")

    return DEFAULT_TRACKED_PARAMETER_SET_REFERENCE


def persist_active_parameter_set_reference(profile_name: str, reference: str) -> Path:
    selector_file = resolve_profile_selector_file(profile_name)
    normalized = normalize_parameter_set_reference(reference, default_subdir="tracked")
    save_parameter_file(
        selector_file,
        {
            "version": 1,
            PROFILE_SELECTOR_ACTIVE_FIELD: normalized,
        },
    )
    return selector_file


def migrate_legacy_runtime_configuration(profile_name: str) -> dict[str, Path]:
    migrated: dict[str, Path] = {}
    legacy_bootstrap = _legacy_bootstrap_path(profile_name)
    if not legacy_bootstrap.exists():
        return migrated

    legacy_bootstrap_data = load_parameter_file(legacy_bootstrap)
    managed_values = _extract_managed_parameter_values(legacy_bootstrap_data)
    legacy_reference = _legacy_snapshot_reference(profile_name, legacy_bootstrap_data)

    tracked_target = resolve_tracked_parameter_set_path(profile_name)
    if managed_values and not tracked_target.exists():
        save_parameter_file(tracked_target, build_parameter_file_data(managed_values))
        migrated["tracked/default.yaml"] = tracked_target

    if legacy_reference is not None:
        legacy_snapshot_file = _legacy_snapshot_dir() / PurePosixPath(legacy_reference).name
        snapshot_target = resolve_parameter_set_path(profile_name, legacy_reference)
        if legacy_snapshot_file.exists() and not snapshot_target.exists():
            snapshot_data = load_parameter_file(legacy_snapshot_file)
            managed_snapshot_values = _extract_managed_parameter_values(snapshot_data)
            save_parameter_file(snapshot_target, build_parameter_file_data(managed_snapshot_values))
            migrated[legacy_reference] = snapshot_target

    selector_file = resolve_profile_selector_file(profile_name)
    if not selector_file.exists():
        active_reference = DEFAULT_TRACKED_PARAMETER_SET_REFERENCE
        if legacy_reference is not None:
            candidate_path = resolve_parameter_set_path(profile_name, legacy_reference)
            if candidate_path.exists():
                active_reference = legacy_reference
        persist_active_parameter_set_reference(profile_name, active_reference)
        migrated[f"profiles/{profile_name}.yaml"] = selector_file

    return migrated


def seed_runtime_configuration(profile_name: str, *, overwrite: bool = False) -> dict[str, Path]:
    """Seed the writable runtime config root from package defaults.

    Runtime config remains authoritative after seeding. Existing files are preserved
    unless overwrite=True, so operator-selected defaults survive rebuilds.
    """
    seeded: dict[str, Path] = {}
    iii_config_dir = resolve_iii_config_dir()
    iii_config_dir.mkdir(parents=True, exist_ok=True)

    seeded.update(migrate_legacy_runtime_configuration(profile_name))

    source_selector = _source_profile_selector_file(profile_name)
    if source_selector is not None:
        target_selector = resolve_profile_selector_file(profile_name)
        if _copy_if_missing(source_selector, target_selector, overwrite=overwrite):
            seeded[f"profiles/{profile_name}.yaml"] = target_selector

    source_parameter_sets_dir = _source_parameter_sets_dir(profile_name)
    if source_parameter_sets_dir is not None:
        copied = _copy_tree_if_missing(
            source_parameter_sets_dir,
            resolve_parameter_sets_dir(profile_name),
            overwrite=overwrite,
        )
        seeded.update({f"parameter_sets/{profile_name}/{key}": value for key, value in copied.items()})

    source_config_dir = _source_config_dir()
    if source_config_dir is not None:
        parameters_source_dir = source_config_dir / "parameters"
        parameters_target_dir = resolve_schema_parameters_dir()
        copied = _copy_tree_if_missing(parameters_source_dir, parameters_target_dir, overwrite=True)
        seeded.update({f"parameters/{key}": value for key, value in copied.items()})

    selector_file = resolve_profile_selector_file(profile_name)
    if not selector_file.exists():
        persist_active_parameter_set_reference(profile_name, DEFAULT_TRACKED_PARAMETER_SET_REFERENCE)
        seeded[f"profiles/{profile_name}.yaml"] = selector_file

    tracked_target = resolve_tracked_parameter_set_path(profile_name)
    if not tracked_target.exists():
        save_parameter_file(tracked_target, build_parameter_file_data({}))
        seeded[f"parameter_sets/{profile_name}/{DEFAULT_TRACKED_PARAMETER_SET_REFERENCE}"] = tracked_target

    resolve_parameter_snapshot_dir(profile_name).mkdir(parents=True, exist_ok=True)

    return seeded


def resolve_parameter_snapshot_dir(profile_name: str) -> Path:
    return resolve_parameter_sets_dir(profile_name) / "snapshots"


def resolve_snapshot_dir(snapshot_path_postfix: str = "parameter_snapshots/") -> Path:
    return resolve_iii_config_dir() / snapshot_path_postfix


def resolve_default_parameter_file_name(profile_name: str) -> str:
    return load_active_parameter_set_reference(profile_name)


def persist_default_parameter_file_name(profile_name: str, file_name: str) -> Path:
    return persist_active_parameter_set_reference(profile_name, file_name)


def resolve_active_parameter_file(profile_name: str) -> Path:
    explicit_file = os.environ.get("III_SYSTEM_PARAMETER_FILE")
    if explicit_file:
        return Path(os.path.expanduser(explicit_file))

    active_reference = load_active_parameter_set_reference(profile_name)
    active_path = resolve_parameter_set_path(profile_name, active_reference)
    if active_path.exists():
        return active_path

    tracked_path = resolve_tracked_parameter_set_path(profile_name)
    if tracked_path.exists():
        return tracked_path

    return active_path
