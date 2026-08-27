"""Immutable installed configuration contracts and compatibility planning.

This module deliberately has no ROS or writable-state side effects.  Callers must
provide immutable and writable roots explicitly; the latter is recorded in plans
but is never opened by compatibility evaluation.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
from typing import Any, Mapping

PACKAGE_NAME = "iii_drone_configuration"
CONTRACT_DIRECTORY = "configuration_contract"
PACKAGE_SCHEMA = "iii.configuration-package/v1"
PROFILE_SCHEMA = "iii.configuration-runtime-profiles/v1"
MIGRATION_SCHEMA = "iii.configuration-migrations/v1"
HASH_LENGTH = 64
EXPECTED_PROFILE_MAP = {
    "hil": ("sim", "hil", False),
    "opti_track": ("real", "opti_track", True),
    "real": ("real", "real", True),
    "sim": ("sim", "sim", True),
}
SET_ID = re.compile(r"^[a-z][a-z0-9_-]*$")


class ConfigurationContractError(RuntimeError):
    """An installed contract is absent, malformed, untrusted, or incompatible."""


@dataclass(frozen=True)
class VersionRange:
    minimum: int
    maximum: int

    def contains(self, version: int) -> bool:
        return self.minimum <= version <= self.maximum


@dataclass(frozen=True)
class RuntimeProfileDescriptor:
    runtime_profile: str
    parameter_profile: str
    selector_scope: str
    bootable: bool


@dataclass(frozen=True)
class TrackedSetDescriptor:
    profile: str
    set_id: str
    default: bool
    path: PurePosixPath
    sha256: str


@dataclass(frozen=True)
class InstalledConfigurationContract:
    root: Path
    manifest_id: str
    package_version: str
    schema_version: int
    readable: VersionRange
    upgrade_from: VersionRange
    downgrade_from: VersionRange
    profiles: tuple[RuntimeProfileDescriptor, ...]
    tracked_sets: tuple[TrackedSetDescriptor, ...]
    artifact_hashes: tuple[tuple[str, str], ...]

    def profile(self, runtime_profile: str) -> RuntimeProfileDescriptor:
        matches = [
            descriptor
            for descriptor in self.profiles
            if descriptor.runtime_profile == runtime_profile
        ]
        if len(matches) != 1:
            raise ConfigurationContractError(
                f"runtime profile is not declared exactly once: {runtime_profile}"
            )
        return matches[0]

    def default_set(self, parameter_profile: str) -> TrackedSetDescriptor:
        matches = [
            descriptor
            for descriptor in self.tracked_sets
            if descriptor.profile == parameter_profile and descriptor.default
        ]
        if len(matches) != 1:
            raise ConfigurationContractError(
                f"parameter profile does not have exactly one default: {parameter_profile}"
            )
        return matches[0]


@dataclass(frozen=True)
class ContractLoadResult:
    contract: InstalledConfigurationContract
    verified_artifacts: tuple[str, ...]


@dataclass(frozen=True)
class ConfigurationCompatibilityPlan:
    plan_id: str
    old_manifest_id: str
    new_manifest_id: str
    old_schema_version: int
    new_schema_version: int
    direction: str
    compatible: bool
    reason: str
    runtime_profile: str
    parameter_profile: str
    selector_scope: str
    bootable: bool
    old_immutable_root: Path
    new_immutable_root: Path
    writable_state_root: Path
    mutations: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": "iii.configuration-compatibility-plan/v1",
            "plan_id": self.plan_id,
            "old_manifest_id": self.old_manifest_id,
            "new_manifest_id": self.new_manifest_id,
            "old_schema_version": self.old_schema_version,
            "new_schema_version": self.new_schema_version,
            "direction": self.direction,
            "compatible": self.compatible,
            "reason": self.reason,
            "runtime_profile": self.runtime_profile,
            "parameter_profile": self.parameter_profile,
            "selector_scope": self.selector_scope,
            "bootable": self.bootable,
            "old_immutable_root": str(self.old_immutable_root),
            "new_immutable_root": str(self.new_immutable_root),
            "writable_state_root": str(self.writable_state_root),
            "mutations": list(self.mutations),
        }


def _canonical(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def _identity(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _is_hash(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == HASH_LENGTH
        and all(character in "0123456789abcdef" for character in value)
    )


def _require_absolute(path: Path, *, label: str) -> Path:
    if not path.is_absolute():
        raise ConfigurationContractError(f"{label} must be an explicit absolute path")
    return path


def _read_regular(path: Path, *, root: Path, label: str) -> bytes:
    try:
        resolved = path.resolve(strict=True)
        root_resolved = root.resolve(strict=True)
        capture_root = (root / "package-manifest.json").resolve(strict=True).parent
    except OSError as exc:
        raise ConfigurationContractError(f"cannot resolve {label}: {exc}") from exc
    _reject_source_tree(capture_root)
    if not (
        resolved.is_relative_to(root_resolved) or resolved.is_relative_to(capture_root)
    ):
        raise ConfigurationContractError(f"{label} escapes the immutable contract root")
    descriptor = os.open(resolved, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        observed = os.fstat(descriptor)
        if not stat.S_ISREG(observed.st_mode):
            raise ConfigurationContractError(f"{label} is not a regular file")
        data = bytearray()
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            data.extend(block)
        if len(data) != observed.st_size:
            raise ConfigurationContractError(f"{label} changed while being read")
        return bytes(data)
    finally:
        os.close(descriptor)


def _json_object(path: Path, *, root: Path, label: str) -> dict[str, Any]:
    raw = _read_regular(path, root=root, label=label)
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ConfigurationContractError(f"{label} is invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigurationContractError(f"{label} must contain one JSON object")
    return value


def _range(value: Any, *, label: str) -> VersionRange:
    if (
        not isinstance(value, dict)
        or set(value) != {"minimum", "maximum"}
        or not all(
            isinstance(item, int) and not isinstance(item, bool) and item >= 1
            for item in value.values()
        )
        or value["minimum"] > value["maximum"]
    ):
        raise ConfigurationContractError(f"{label} is not a valid version range")
    return VersionRange(value["minimum"], value["maximum"])


def _reject_source_tree(root: Path) -> None:
    resolved = root.resolve(strict=True)
    for candidate in (resolved, *resolved.parents):
        if (
            candidate.name == "III-Drone-Configuration"
            and (candidate / ".git").exists()
            and (candidate / "CMakeLists.txt").is_file()
            and (candidate / "package.xml").is_file()
        ):
            raise ConfigurationContractError(
                "immutable configuration contracts must resolve through an installed build, not the workspace source tree"
            )


def resolve_installed_contract_root() -> Path:
    """Resolve only through the ament index; no workspace/source fallback exists."""

    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory(PACKAGE_NAME))
    except Exception as exc:
        raise ConfigurationContractError(
            "iii_drone_configuration is not available through the ament index"
        ) from exc
    root = (share / CONTRACT_DIRECTORY).absolute()
    _reject_source_tree(root)
    if root.is_symlink() or not root.is_dir():
        raise ConfigurationContractError(
            "installed immutable configuration contract directory is missing or linked"
        )
    return root


def load_installed_contract(immutable_root: Path) -> ContractLoadResult:
    """Authenticate a build-captured contract bundle without writable-state access."""

    root = _require_absolute(immutable_root, label="immutable input root")
    _reject_source_tree(root)
    if root.is_symlink() or not root.is_dir():
        raise ConfigurationContractError(
            "immutable input root is missing, linked, or not a directory"
        )
    manifest = _json_object(
        root / "package-manifest.json",
        root=root,
        label="configuration package manifest",
    )
    required = {
        "schema",
        "manifest_id",
        "package_version",
        "configuration_schema_version",
        "compatibility",
        "artifacts",
        "tracked_sets",
    }
    if set(manifest) != required or manifest.get("schema") != PACKAGE_SCHEMA:
        raise ConfigurationContractError(
            "configuration package manifest schema/fields are unsupported"
        )
    expected_identity = _identity(
        {key: value for key, value in manifest.items() if key != "manifest_id"}
    )
    if manifest.get("manifest_id") != expected_identity:
        raise ConfigurationContractError(
            "configuration package manifest identity mismatch"
        )
    version = manifest["configuration_schema_version"]
    if not isinstance(version, int) or isinstance(version, bool) or version < 1:
        raise ConfigurationContractError("configuration schema version is invalid")
    if (
        not isinstance(manifest["package_version"], str)
        or not manifest["package_version"]
    ):
        raise ConfigurationContractError("configuration package version is invalid")
    compatibility = manifest["compatibility"]
    if not isinstance(compatibility, dict) or set(compatibility) != {
        "readable",
        "upgrade_from",
        "downgrade_from",
    }:
        raise ConfigurationContractError(
            "configuration compatibility metadata is invalid"
        )
    readable = _range(compatibility["readable"], label="readable range")
    upgrade_from = _range(compatibility["upgrade_from"], label="upgrade range")
    downgrade_from = _range(compatibility["downgrade_from"], label="downgrade range")
    if not readable.contains(version):
        raise ConfigurationContractError(
            "configuration package cannot read its own schema version"
        )

    artifact_rows = manifest["artifacts"]
    if not isinstance(artifact_rows, list) or not artifact_rows:
        raise ConfigurationContractError("configuration artifact inventory is empty")
    artifact_hashes: list[tuple[str, str]] = []
    for row in artifact_rows:
        if not isinstance(row, dict) or set(row) != {"path", "sha256"}:
            raise ConfigurationContractError("configuration artifact row is malformed")
        relative = PurePosixPath(str(row["path"]))
        if (
            relative.is_absolute()
            or not relative.parts
            or any(part in {"", ".", ".."} for part in relative.parts)
            or not _is_hash(row["sha256"])
        ):
            raise ConfigurationContractError(
                "configuration artifact locator/hash is unsafe"
            )
        raw = _read_regular(
            root / Path(*relative.parts), root=root, label=f"artifact {relative}"
        )
        if hashlib.sha256(raw).hexdigest() != row["sha256"]:
            raise ConfigurationContractError(
                f"configuration artifact hash mismatch: {relative}"
            )
        artifact_hashes.append((relative.as_posix(), row["sha256"]))
    if artifact_hashes != sorted(set(artifact_hashes)):
        raise ConfigurationContractError(
            "configuration artifacts must be sorted and unique"
        )
    declared_files = {path for path, _digest in artifact_hashes} | {
        "package-manifest.json"
    }
    observed_files = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() or path.is_symlink()
    }
    if observed_files != declared_files:
        raise ConfigurationContractError(
            "immutable configuration bundle contains undeclared or missing files"
        )

    profile_document = _json_object(
        root / "profiles.json", root=root, label="runtime profile descriptors"
    )
    profiles = _profiles(profile_document)
    migration_document = _json_object(
        root / "migrations.json", root=root, label="migration metadata"
    )
    _validate_migrations(
        migration_document,
        version=version,
        upgrade_from=upgrade_from,
        downgrade_from=downgrade_from,
    )
    tracked_sets = _tracked_sets(manifest["tracked_sets"], artifact_hashes)
    contract = InstalledConfigurationContract(
        root=root,
        manifest_id=manifest["manifest_id"],
        package_version=manifest["package_version"],
        schema_version=version,
        readable=readable,
        upgrade_from=upgrade_from,
        downgrade_from=downgrade_from,
        profiles=profiles,
        tracked_sets=tracked_sets,
        artifact_hashes=tuple(artifact_hashes),
    )
    for parameter_profile in ("real", "sim"):
        contract.default_set(parameter_profile)
    return ContractLoadResult(
        contract=contract,
        verified_artifacts=tuple(path for path, _digest in artifact_hashes),
    )


def _profiles(document: Mapping[str, Any]) -> tuple[RuntimeProfileDescriptor, ...]:
    if (
        set(document) != {"schema", "profiles"}
        or document.get("schema") != PROFILE_SCHEMA
    ):
        raise ConfigurationContractError("runtime profile descriptor schema is invalid")
    rows = document["profiles"]
    if not isinstance(rows, list):
        raise ConfigurationContractError("runtime profile descriptors must be a list")
    profiles: list[RuntimeProfileDescriptor] = []
    for row in rows:
        if not isinstance(row, dict) or set(row) != {
            "runtime_profile",
            "parameter_profile",
            "selector_scope",
            "bootable",
        }:
            raise ConfigurationContractError("runtime profile descriptor is malformed")
        if not all(
            isinstance(row[field], str) and row[field]
            for field in (
                "runtime_profile",
                "parameter_profile",
                "selector_scope",
            )
        ) or not isinstance(row["bootable"], bool):
            raise ConfigurationContractError(
                "runtime profile descriptor values are invalid"
            )
        profiles.append(RuntimeProfileDescriptor(**row))
    observed = {
        item.runtime_profile: (
            item.parameter_profile,
            item.selector_scope,
            item.bootable,
        )
        for item in profiles
    }
    if observed != EXPECTED_PROFILE_MAP or len(profiles) != len(observed):
        raise ConfigurationContractError(
            "runtime profile aliases/selectors do not match the canonical mapping"
        )
    if [item.runtime_profile for item in profiles] != sorted(observed):
        raise ConfigurationContractError("runtime profile descriptors are not sorted")
    return tuple(profiles)


def _validate_migrations(
    document: Mapping[str, Any],
    *,
    version: int,
    upgrade_from: VersionRange,
    downgrade_from: VersionRange,
) -> None:
    if (
        set(document)
        != {
            "schema",
            "current_schema_version",
            "upgrade_from",
            "downgrade_from",
            "steps",
        }
        or document.get("schema") != MIGRATION_SCHEMA
        or document.get("current_schema_version") != version
        or _range(document.get("upgrade_from"), label="migration upgrade range")
        != upgrade_from
        or _range(document.get("downgrade_from"), label="migration downgrade range")
        != downgrade_from
        or not isinstance(document.get("steps"), list)
    ):
        raise ConfigurationContractError(
            "migration metadata is not bound to the package compatibility contract"
        )


def _tracked_sets(
    rows: Any, artifact_hashes: list[tuple[str, str]]
) -> tuple[TrackedSetDescriptor, ...]:
    if not isinstance(rows, list):
        raise ConfigurationContractError("tracked set inventory must be a list")
    known_artifacts = dict(artifact_hashes)
    result: list[TrackedSetDescriptor] = []
    keys: set[tuple[str, str]] = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {
            "profile",
            "set_id",
            "default",
            "path",
            "sha256",
        }:
            raise ConfigurationContractError("tracked set descriptor is malformed")
        if (
            row["profile"] not in {"real", "sim"}
            or not isinstance(row["set_id"], str)
            or SET_ID.fullmatch(row["set_id"]) is None
            or not isinstance(row["default"], bool)
            or not _is_hash(row["sha256"])
        ):
            raise ConfigurationContractError(
                "tracked set descriptor values are invalid"
            )
        relative = PurePosixPath(str(row["path"]))
        if known_artifacts.get(relative.as_posix()) != row["sha256"]:
            raise ConfigurationContractError(
                "tracked set is not bound to an authenticated artifact"
            )
        key = (row["profile"], row["set_id"])
        if key in keys:
            raise ConfigurationContractError("tracked set identity is duplicated")
        keys.add(key)
        result.append(
            TrackedSetDescriptor(
                profile=row["profile"],
                set_id=row["set_id"],
                default=row["default"],
                path=relative,
                sha256=row["sha256"],
            )
        )
    if [(item.profile, item.set_id) for item in result] != sorted(keys):
        raise ConfigurationContractError("tracked set inventory is not sorted")
    return tuple(result)


def plan_compatibility(
    *,
    old_immutable_root: Path,
    new_immutable_root: Path,
    writable_state_root: Path,
    runtime_profile: str,
) -> ConfigurationCompatibilityPlan:
    """Return a pure, content-identified plan; never open writable state."""

    old_root = _require_absolute(old_immutable_root, label="old immutable input root")
    new_root = _require_absolute(new_immutable_root, label="new immutable input root")
    writable_root = _require_absolute(writable_state_root, label="writable state root")
    old = load_installed_contract(old_root).contract
    new = load_installed_contract(new_root).contract
    profile = new.profile(runtime_profile)
    if new.schema_version == old.schema_version:
        direction = "same-schema"
        compatible = new.readable.contains(old.schema_version)
    elif new.schema_version > old.schema_version:
        direction = "upgrade"
        compatible = new.upgrade_from.contains(old.schema_version)
    else:
        direction = "downgrade"
        compatible = new.downgrade_from.contains(old.schema_version)
    reason = (
        f"schema {old.schema_version} is supported for {direction} by schema {new.schema_version}"
        if compatible
        else f"schema {old.schema_version} is outside the new package {direction} range"
    )
    content = {
        "old_manifest_id": old.manifest_id,
        "new_manifest_id": new.manifest_id,
        "old_schema_version": old.schema_version,
        "new_schema_version": new.schema_version,
        "direction": direction,
        "compatible": compatible,
        "reason": reason,
        "runtime_profile": runtime_profile,
        "parameter_profile": profile.parameter_profile,
        "selector_scope": profile.selector_scope,
        "bootable": profile.bootable,
        "old_immutable_root": str(old_root),
        "new_immutable_root": str(new_root),
        "writable_state_root": str(writable_root),
        "mutations": [],
    }
    return ConfigurationCompatibilityPlan(
        plan_id=_identity(content),
        old_manifest_id=old.manifest_id,
        new_manifest_id=new.manifest_id,
        old_schema_version=old.schema_version,
        new_schema_version=new.schema_version,
        direction=direction,
        compatible=compatible,
        reason=reason,
        runtime_profile=runtime_profile,
        parameter_profile=profile.parameter_profile,
        selector_scope=profile.selector_scope,
        bootable=profile.bootable,
        old_immutable_root=old_root,
        new_immutable_root=new_root,
        writable_state_root=writable_root,
    )


__all__ = [
    "ConfigurationCompatibilityPlan",
    "ConfigurationContractError",
    "ContractLoadResult",
    "InstalledConfigurationContract",
    "RuntimeProfileDescriptor",
    "TrackedSetDescriptor",
    "VersionRange",
    "load_installed_contract",
    "plan_compatibility",
    "resolve_installed_contract_root",
]
