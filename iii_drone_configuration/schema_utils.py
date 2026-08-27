import os
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import yaml

from iii_drone_configuration.installed_contracts import (
    load_installed_contract,
    resolve_installed_contract_root,
)
from iii_drone_configuration.reconciliation import (
    ReconciliationError,
    reconcile_simulation_startup,
)

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
    return resolve_parameter_set_path(
        profile_name, DEFAULT_TRACKED_PARAMETER_SET_REFERENCE
    )


def normalize_parameter_set_reference(
    reference: str, *, default_subdir: str = "snapshots"
) -> str:
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
    normalized = normalize_parameter_set_reference(
        reference, default_subdir="snapshots"
    )
    return resolve_parameter_sets_dir(profile_name) / PurePosixPath(normalized)


def resolve_schema_parameters_dir() -> Path:
    return resolve_iii_config_dir() / "parameters"


def _copy_if_missing(source: Path, target: Path, *, overwrite: bool = False) -> bool:
    if not source.exists():
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and not overwrite:
        return False
    shutil.copyfile(source, target)
    return True


def load_parameter_file(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def save_parameter_file(path: Path, data: dict) -> None:
    if path.exists() and (path.is_symlink() or not path.is_file()):
        raise ValueError(f"Refusing to replace unsafe parameter file: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.parent.is_symlink() or not path.parent.is_dir():
        raise ValueError(f"Parameter file parent is unsafe: {path.parent}")
    content = yaml.safe_dump(data, sort_keys=False).encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        os.fchmod(descriptor, 0o640)
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


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
    if (
        parameters_path_postfix != "parameters/"
        or default_parameter_file != "parameter_manifest.yaml"
        or sim_parameter_file != "parameter_manifest.yaml"
    ):
        raise ValueError(
            "legacy schema filename selection is unsupported; use III_DRONE_SCHEMA_FILE for an explicit debug override"
        )
    contract = load_installed_contract(resolve_installed_contract_root()).contract
    return contract.root / "schema" / "parameter_manifest.yaml"


def _legacy_bootstrap_filename(profile_name: str) -> str:
    return "ros_params_sim.yaml" if profile_name == "sim" else "ros_params_real.yaml"


def _legacy_bootstrap_path(profile_name: str) -> Path:
    return resolve_iii_config_dir() / _legacy_bootstrap_filename(profile_name)


def _legacy_snapshot_dir() -> Path:
    return resolve_iii_config_dir() / "parameter_snapshots"


def _legacy_snapshot_reference(profile_name: str, bootstrap_data: dict) -> str | None:
    parameter_name = (
        "sim_snapshot_file" if profile_name == "sim" else "default_snapshot_file"
    )
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
            return normalize_parameter_set_reference(
                reference, default_subdir="tracked"
            )

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
        legacy_snapshot_file = (
            _legacy_snapshot_dir() / PurePosixPath(legacy_reference).name
        )
        snapshot_target = resolve_parameter_set_path(profile_name, legacy_reference)
        if legacy_snapshot_file.exists() and not snapshot_target.exists():
            snapshot_data = load_parameter_file(legacy_snapshot_file)
            managed_snapshot_values = _extract_managed_parameter_values(snapshot_data)
            save_parameter_file(
                snapshot_target, build_parameter_file_data(managed_snapshot_values)
            )
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


def seed_runtime_configuration(
    profile_name: str, *, overwrite: bool = False
) -> dict[str, Path]:
    """Compatibility wrapper for the canonical reconciliation/verification gate.

    Simulation is reconciled transactionally. Aircraft profiles are read-only at
    runtime and must already have been staged and reconciled by the receiver.
    ``overwrite`` is rejected because runtime callers may never reset living state
    implicitly.
    """
    if overwrite:
        raise ReconciliationError(
            "implicit configuration overwrite is retired; use 'iii config sim reset --confirm'"
        )
    immutable_root = resolve_installed_contract_root()
    contract = load_installed_contract(immutable_root).contract
    profile = contract.profile(profile_name)
    if not profile.bootable:
        raise ReconciliationError(
            f"runtime profile is reserved and non-bootable: {profile_name}"
        )
    selector_scope = profile.selector_scope
    iii_config_dir = resolve_iii_config_dir()
    if profile.parameter_profile == "sim":
        result = reconcile_simulation_startup(
            immutable_root=immutable_root,
            writable_state_root=iii_config_dir,
            operations_root=resolve_configuration_operations_root(),
            runtime_profile=profile_name,
            target_id=os.environ.get("III_LOGICAL_TARGET", "sim"),
            release_id=(
                os.environ.get("III_ACTIVE_RELEASE_ID")
                or os.environ.get("III_WORKSPACE_RELEASE_ID")
                or contract.manifest_id
            ),
        )
        return {
            relative: iii_config_dir / PurePosixPath(relative)
            for relative in result.changed_paths
        }

    selector_file = resolve_profile_selector_file(selector_scope)
    active_file = resolve_active_parameter_file(selector_scope)
    state_file = iii_config_dir / "state" / selector_scope / "contract.json"
    missing = [
        str(path)
        for path in (selector_file, active_file, state_file)
        if not path.is_file()
    ]
    if missing:
        raise ReconciliationError(
            "aircraft configuration is not receiver-reconciled; runtime mutation is forbidden; missing: "
            + ", ".join(missing)
        )
    return {}


def resolve_configuration_operations_root() -> Path:
    explicit = os.environ.get("III_OPERATIONS_ROOT")
    if explicit:
        return Path(explicit).expanduser().absolute()
    workspace = os.environ.get("WORKSPACE_DIR")
    if workspace:
        return (Path(workspace).expanduser() / ".iii" / "operations").absolute()
    return (Path.home() / ".local" / "state" / "iii" / "operations").absolute()


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
