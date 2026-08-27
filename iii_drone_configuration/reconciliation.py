"""Transactional reconciliation of writable parameter sets.

The immutable package contract describes what may run.  This module is the only
place that translates an existing writable configuration tree to a new contract.
It deliberately has no ROS dependency so simulation startup, the receiver, and
the CLI can share exactly the same policy.
"""

from __future__ import annotations

import ast
from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
from typing import Any, Mapping

import yaml

from iii_drone_configuration.installed_contracts import (
    ConfigurationCompatibilityPlan,
    InstalledConfigurationContract,
    load_installed_contract,
    plan_compatibility,
)

PLAN_SCHEMA = "iii.configuration-reconciliation-plan/v1"
RESULT_SCHEMA = "iii.configuration-reconciliation-result/v1"
REVIEW_SCHEMA = "iii.configuration-reintroduction-review/v1"
DECISIONS_SCHEMA = "iii.configuration-reintroduction-decisions/v1"
SHADOW_SCHEMA = "iii.configuration-legacy-shadow/v1"
STATE_SCHEMA = "iii.configuration-state/v1"
CHECKPOINT_SCHEMA = "iii.configuration-checkpoint/v1"
JOURNAL_SCHEMA = "iii.configuration-reconciliation-journal/v1"
HASH = re.compile(r"^[0-9a-f]{64}$")
OPERATION_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{7,127}$")
PARAMETER_NAME = re.compile(r"^/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*$")
BOUND_REFERENCE = re.compile(r"/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*")
ALLOWED_MODES = {"simulation", "receiver-staged"}


class ReconciliationError(RuntimeError):
    """A reconciliation input, review, state, or transaction is unsafe."""


@dataclass(frozen=True)
class ParameterSetPlan:
    reference: str
    selected: bool
    source_sha256: str | None
    result_sha256: str
    preserved: tuple[str, ...]
    added: tuple[str, ...]
    removed: tuple[str, ...]
    reintroduced: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "reference": self.reference,
            "selected": self.selected,
            "source_sha256": self.source_sha256,
            "result_sha256": self.result_sha256,
            "preserved": list(self.preserved),
            "added": list(self.added),
            "removed": list(self.removed),
            "reintroduced": list(self.reintroduced),
        }


@dataclass(frozen=True)
class ReconciliationPlan:
    plan_id: str
    operation_id: str
    mode: str
    purpose: str
    target_id: str
    old_release_id: str
    new_release_id: str
    runtime_profile: str
    parameter_profile: str
    selector_scope: str
    writable_state_root: Path
    operations_root: Path
    old_manifest_id: str
    new_manifest_id: str
    old_schema_version: int
    new_schema_version: int
    initial_state_id: str
    compatibility: ConfigurationCompatibilityPlan
    sets: tuple[ParameterSetPlan, ...]
    review_required: bool
    review_items: tuple[Mapping[str, Any], ...]
    mutations: tuple[str, ...]
    _documents: Mapping[str, bytes] = field(repr=False, compare=False)
    _shadow_documents: Mapping[str, bytes] = field(repr=False, compare=False)

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": PLAN_SCHEMA,
            "plan_id": self.plan_id,
            "operation_id": self.operation_id,
            "mode": self.mode,
            "purpose": self.purpose,
            "target_id": self.target_id,
            "old_release_id": self.old_release_id,
            "new_release_id": self.new_release_id,
            "runtime_profile": self.runtime_profile,
            "parameter_profile": self.parameter_profile,
            "selector_scope": self.selector_scope,
            "writable_state_root": str(self.writable_state_root),
            "operations_root": str(self.operations_root),
            "old_manifest_id": self.old_manifest_id,
            "new_manifest_id": self.new_manifest_id,
            "old_schema_version": self.old_schema_version,
            "new_schema_version": self.new_schema_version,
            "initial_state_id": self.initial_state_id,
            "compatibility": self.compatibility.as_dict(),
            "sets": [item.as_dict() for item in self.sets],
            "review_required": self.review_required,
            "review_items": [dict(item) for item in self.review_items],
            "mutations": list(self.mutations),
        }


@dataclass(frozen=True)
class ReconciliationResult:
    operation_id: str
    plan_id: str
    status: str
    state_id: str
    journal_path: Path
    review_path: Path | None
    sets: tuple[ParameterSetPlan, ...]
    changed_paths: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": RESULT_SCHEMA,
            "operation_id": self.operation_id,
            "plan_id": self.plan_id,
            "status": self.status,
            "state_id": self.state_id,
            "journal_path": str(self.journal_path),
            "review_path": str(self.review_path) if self.review_path else None,
            "sets": [item.as_dict() for item in self.sets],
            "changed_paths": list(self.changed_paths),
        }


def _canonical(value: Any) -> bytes:
    return (
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        + "\n"
    ).encode("utf-8")


def _identity(value: Mapping[str, Any], identity_field: str | None = None) -> str:
    document = dict(value)
    if identity_field is not None:
        document.pop(identity_field, None)
    return hashlib.sha256(_canonical(document)).hexdigest()


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _require_absolute(path: Path, *, label: str) -> Path:
    path = Path(path)
    if not path.is_absolute():
        raise ReconciliationError(f"{label} must be an explicit absolute path")
    return path


def _require_operation_id(value: str) -> str:
    if not OPERATION_ID.fullmatch(value):
        raise ReconciliationError("operation ID is malformed")
    return value


def _require_release_id(value: str, *, label: str) -> str:
    if not value or len(value) > 160 or any(character.isspace() for character in value):
        raise ReconciliationError(f"{label} is malformed")
    return value


def _safe_relative(value: str, *, label: str) -> PurePosixPath:
    candidate = PurePosixPath(value)
    if (
        candidate.is_absolute()
        or not candidate.parts
        or any(part in {"", ".", ".."} for part in candidate.parts)
    ):
        raise ReconciliationError(f"{label} is unsafe")
    return candidate


def _regular_bytes(path: Path, *, label: str) -> bytes:
    if path.is_symlink():
        raise ReconciliationError(f"{label} is linked")
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    except OSError as exc:
        raise ReconciliationError(f"cannot open {label}: {exc}") from exc
    try:
        observed = os.fstat(descriptor)
        if not stat.S_ISREG(observed.st_mode):
            raise ReconciliationError(f"{label} is not a regular file")
        data = bytearray()
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            data.extend(block)
        if len(data) != observed.st_size:
            raise ReconciliationError(f"{label} changed while being read")
        return bytes(data)
    finally:
        os.close(descriptor)


def _yaml(path: Path, *, label: str) -> Any:
    try:
        return yaml.safe_load(_regular_bytes(path, label=label))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ReconciliationError(f"{label} is invalid YAML: {exc}") from exc


def _contract_bytes(
    contract: InstalledConfigurationContract, relative: str, *, label: str
) -> bytes:
    """Read an already-authenticated contract artifact, including safe symlink installs."""

    safe = _safe_relative(relative, label=label)
    path = contract.root / Path(*safe.parts)
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise ReconciliationError(f"cannot resolve {label}: {exc}") from exc
    try:
        descriptor = os.open(resolved, os.O_RDONLY | os.O_NOFOLLOW)
    except OSError as exc:
        raise ReconciliationError(f"cannot open {label}: {exc}") from exc
    try:
        observed = os.fstat(descriptor)
        if not stat.S_ISREG(observed.st_mode):
            raise ReconciliationError(f"{label} is not a regular file")
        data = bytearray()
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            data.extend(block)
    finally:
        os.close(descriptor)
    payload = bytes(data)
    expected = dict(contract.artifact_hashes).get(relative)
    if expected is not None and _sha256(payload) != expected:
        raise ReconciliationError(f"authenticated {label} changed after contract load")
    return payload


def _contract_yaml(
    contract: InstalledConfigurationContract, relative: str, *, label: str
) -> Any:
    try:
        return yaml.safe_load(_contract_bytes(contract, relative, label=label))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ReconciliationError(f"{label} is invalid YAML: {exc}") from exc


def _json(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(_regular_bytes(path, label=label))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReconciliationError(f"{label} is invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ReconciliationError(f"{label} must contain one JSON object")
    return value


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_bytes(path: Path, data: bytes, *, mode: int = 0o640) -> None:
    if path.exists() and path.is_symlink():
        raise ReconciliationError(f"refusing to replace linked path: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        os.fchmod(descriptor, mode)
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _atomic_document(
    path: Path, value: Mapping[str, Any], *, mode: int = 0o640
) -> None:
    _atomic_bytes(path, _canonical(value), mode=mode)


def _flatten_schema(value: Any, namespace: str = "") -> dict[str, dict[str, Any]]:
    if not isinstance(value, dict):
        raise ReconciliationError("parameter schema root must be a mapping")
    result: dict[str, dict[str, Any]] = {}
    for name, child in value.items():
        if (
            not isinstance(name, str)
            or not name
            or not all(character.isalnum() or character == "_" for character in name)
        ):
            raise ReconciliationError(f"parameter schema name is malformed: {name!r}")
        full_name = f"{namespace}/{name}"
        if isinstance(child, dict) and "type" in child and "value" in child:
            if full_name in result:
                raise ReconciliationError(f"parameter is duplicated: {full_name}")
            result[full_name] = dict(child)
        else:
            result.update(_flatten_schema(child, full_name))
    return result


def _parameter_values(path: Path, *, label: str) -> dict[str, Any]:
    value = _parameter_raw(path, label=label)
    return _parameter_values_from_document(value, label=label)


def _parameter_raw(path: Path, *, label: str) -> dict[str, Any]:
    value = _yaml(path, label=label)
    _parameter_values_from_document(value, label=label)
    return value


def _parameter_values_from_document(value: Any, *, label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or "/**" not in value:
        raise ReconciliationError(f"{label} must contain the '/**' ROS scope")
    scope = value["/**"]
    if not isinstance(scope, dict) or set(scope) != {"ros__parameters"}:
        raise ReconciliationError(f"{label} ROS scope is malformed")
    parameters = scope["ros__parameters"]
    if not isinstance(parameters, dict):
        raise ReconciliationError(f"{label} parameters must be a mapping")
    for name in parameters:
        if not isinstance(name, str) or not PARAMETER_NAME.fullmatch(name):
            raise ReconciliationError(f"{label} contains malformed parameter {name!r}")
    return dict(parameters)


def _parameter_document(
    values: Mapping[str, Any], base: Mapping[str, Any] | None = None
) -> bytes:
    document = deepcopy(dict(base)) if base is not None else {}
    document["/**"] = {"ros__parameters": dict(sorted(values.items()))}
    return yaml.safe_dump(
        document,
        sort_keys=False,
    ).encode("utf-8")


def _strict_type(parameter_type: str, value: Any) -> bool:
    if parameter_type == "bool":
        return isinstance(value, bool)
    if parameter_type == "int":
        return isinstance(value, int) and not isinstance(value, bool)
    if parameter_type == "float":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if parameter_type == "string":
        return isinstance(value, str)
    array_types = {
        "bool_array": lambda item: isinstance(item, bool),
        "int_array": lambda item: isinstance(item, int) and not isinstance(item, bool),
        "integer_array": lambda item: isinstance(item, int)
        and not isinstance(item, bool),
        "float_array": lambda item: isinstance(item, (int, float))
        and not isinstance(item, bool),
        "string_array": lambda item: isinstance(item, str),
    }
    if parameter_type in array_types:
        return isinstance(value, list) and all(
            array_types[parameter_type](item) for item in value
        )
    return False


def _bound_value(expression: Any, candidates: Mapping[str, Any]) -> float | int:
    if isinstance(expression, (int, float)) and not isinstance(expression, bool):
        return expression
    if not isinstance(expression, str) or not expression.strip():
        raise ReconciliationError("numeric bound is malformed")
    rewritten = expression
    for reference in sorted(
        set(BOUND_REFERENCE.findall(expression)), key=len, reverse=True
    ):
        value = candidates.get(reference)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ReconciliationError(
                f"numeric bound references invalid parameter: {reference}"
            )
        rewritten = rewritten.replace(reference, repr(value))
    try:
        tree = ast.parse(rewritten, mode="eval")
    except SyntaxError as exc:
        raise ReconciliationError(
            f"numeric bound expression is invalid: {expression}"
        ) from exc
    allowed = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.USub,
        ast.UAdd,
        ast.Constant,
    )
    if any(not isinstance(node, allowed) for node in ast.walk(tree)):
        raise ReconciliationError(f"numeric bound expression is unsafe: {expression}")
    result = eval(
        compile(tree, "<configuration-bound>", "eval"), {"__builtins__": {}}, {}
    )
    if not isinstance(result, (int, float)) or isinstance(result, bool):
        raise ReconciliationError("numeric bound did not produce a number")
    return result


def _validate_candidates(
    candidates: Mapping[str, Any], entries: Mapping[str, Mapping[str, Any]]
) -> tuple[bool, tuple[str, ...]]:
    reasons: list[str] = []
    if set(candidates) != set(entries):
        reasons.append("parameter inventory differs from schema")
    for name in sorted(set(candidates) & set(entries)):
        entry = entries[name]
        value = candidates[name]
        parameter_type = entry.get("type")
        if not isinstance(parameter_type, str) or not _strict_type(
            parameter_type, value
        ):
            reasons.append(f"{name}: value does not match type {parameter_type}")
            continue
        options = entry.get("options")
        if options is not None and (
            not isinstance(options, list) or value not in options
        ):
            reasons.append(f"{name}: value is not an allowed option")
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            try:
                if "min" in entry and value < _bound_value(entry["min"], candidates):
                    reasons.append(f"{name}: value is below minimum")
                if "max" in entry and value > _bound_value(entry["max"], candidates):
                    reasons.append(f"{name}: value is above maximum")
            except ReconciliationError as exc:
                reasons.append(f"{name}: {exc}")
    return not reasons, tuple(reasons)


def _contract_inputs(
    contract: InstalledConfigurationContract, runtime_profile: str
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    descriptor = contract.profile(runtime_profile)
    entries = _flatten_schema(
        _contract_yaml(
            contract,
            "schema/parameter_manifest.yaml",
            label="installed parameter schema",
        )
    )
    default_descriptor = contract.default_set(descriptor.parameter_profile)
    default_document = _contract_yaml(
        contract,
        default_descriptor.path.as_posix(),
        label="installed tracked default",
    )
    if not isinstance(default_document, dict):
        raise ReconciliationError("installed tracked default is malformed")
    defaults = _parameter_values_from_document(
        default_document, label="installed tracked default"
    )
    valid, reasons = _validate_candidates(defaults, entries)
    if not valid:
        raise ReconciliationError(
            "installed tracked default is invalid: " + "; ".join(reasons)
        )
    return entries, defaults


def _retained_contract_documents(
    contract: InstalledConfigurationContract,
) -> dict[str, bytes]:
    """Return the authenticated bundle bytes needed for later offline migration."""

    prefix = Path("contracts") / contract.manifest_id
    result = {
        (prefix / "package-manifest.json").as_posix(): _contract_bytes(
            contract,
            "package-manifest.json",
            label="configuration package manifest",
        )
    }
    for relative, _digest in contract.artifact_hashes:
        safe = _safe_relative(relative, label="configuration contract artifact")
        result[(prefix / Path(*safe.parts)).as_posix()] = _contract_bytes(
            contract,
            relative,
            label=f"configuration contract artifact {relative}",
        )
    return result


def _selector(root: Path, scope: str) -> tuple[str, bytes | None]:
    path = root / "profiles" / f"{scope}.yaml"
    if not path.exists():
        return "tracked/default.yaml", None
    value = _yaml(path, label="runtime profile selector")
    if not isinstance(value, dict) or set(value) != {"version", "active_parameter_set"}:
        raise ReconciliationError("runtime profile selector is malformed")
    if value["version"] != 1 or not isinstance(value["active_parameter_set"], str):
        raise ReconciliationError(
            "runtime profile selector version/reference is invalid"
        )
    reference = _safe_relative(
        value["active_parameter_set"], label="active parameter set reference"
    ).as_posix()
    return reference, _regular_bytes(path, label="runtime profile selector")


def _set_paths(root: Path, scope: str) -> dict[str, Path]:
    base = root / "parameter_sets" / scope
    if not base.exists():
        return {}
    if base.is_symlink() or not base.is_dir():
        raise ReconciliationError("parameter-set profile root is unsafe")
    result: dict[str, Path] = {}
    for path in sorted(base.rglob("*")):
        if path.is_symlink():
            raise ReconciliationError(f"parameter set is unsafe: {path}")
        if path.is_dir():
            continue
        if not path.is_file():
            raise ReconciliationError(f"parameter set contains a special path: {path}")
        if path.suffix != ".yaml":
            continue
        reference = path.relative_to(base).as_posix()
        _safe_relative(reference, label="parameter set reference")
        result[reference] = path
    return result


def _shadow_path(root: Path, scope: str, manifest_id: str, reference: str) -> Path:
    relative = _safe_relative(reference, label="shadow set reference")
    return (
        root
        / "shadows"
        / scope
        / manifest_id
        / Path(*relative.parts).with_suffix(".json")
    )


def _load_shadow(path: Path) -> dict[str, Any]:
    value = _json(path, label="legacy shadow")
    if value.get("schema") != SHADOW_SCHEMA or value.get("shadow_id") != _identity(
        value, "shadow_id"
    ):
        raise ReconciliationError(f"legacy shadow identity is invalid: {path}")
    required = {
        "schema",
        "shadow_id",
        "target_id",
        "selector_scope",
        "parameter_profile",
        "set_reference",
        "retired_by_manifest_id",
        "retired_by_schema_version",
        "retired_at_release",
        "source_manifest_id",
        "source_release_id",
        "entries",
    }
    if set(value) != required or not isinstance(value["entries"], dict):
        raise ReconciliationError(f"legacy shadow fields are invalid: {path}")
    for name, entry in value["entries"].items():
        if not isinstance(name, str) or not PARAMETER_NAME.fullmatch(name):
            raise ReconciliationError(f"legacy shadow parameter is invalid: {path}")
        if not isinstance(entry, dict) or set(entry) != {
            "value",
            "restorable",
            "active_at_retirement",
            "source_manifest_id",
            "source_release_id",
            "source_set_sha256",
        }:
            raise ReconciliationError(f"legacy shadow entry is malformed: {path}")
        if (
            not isinstance(entry["restorable"], bool)
            or not isinstance(entry["active_at_retirement"], bool)
            or entry["restorable"] != entry["active_at_retirement"]
            or not isinstance(entry["source_manifest_id"], str)
            or not HASH.fullmatch(entry["source_manifest_id"])
            or not isinstance(entry["source_release_id"], str)
            or not entry["source_release_id"]
            or not isinstance(entry["source_set_sha256"], str)
            or not HASH.fullmatch(entry["source_set_sha256"])
        ):
            raise ReconciliationError(
                f"legacy shadow entry provenance is invalid: {path}"
            )
    return value


def _existing_shadows(
    root: Path, scope: str
) -> tuple[tuple[Path, dict[str, Any]], ...]:
    base = root / "shadows" / scope
    if not base.exists():
        return ()
    if base.is_symlink() or not base.is_dir():
        raise ReconciliationError("legacy shadow root is unsafe")
    rows: list[tuple[Path, dict[str, Any]]] = []
    for path in sorted(base.rglob("*")):
        if path.is_symlink():
            raise ReconciliationError(f"legacy shadow path is unsafe: {path}")
        if path.is_dir():
            continue
        if not path.is_file() or path.suffix != ".json":
            raise ReconciliationError(f"legacy shadow path is unsafe: {path}")
        shadow = _load_shadow(path)
        relative = path.relative_to(base)
        if len(relative.parts) < 2:
            raise ReconciliationError(f"legacy shadow path is malformed: {path}")
        reference = Path(*relative.parts[1:]).with_suffix(".yaml").as_posix()
        if (
            shadow["selector_scope"] != scope
            or shadow["retired_by_manifest_id"] != relative.parts[0]
            or shadow["set_reference"] != reference
        ):
            raise ReconciliationError(
                f"legacy shadow path and provenance disagree: {path}"
            )
        rows.append((path, shadow))
    return tuple(rows)


def _tree_state_id(root: Path, scope: str) -> str:
    inventory: list[dict[str, str]] = []
    candidates = [root / "profiles" / f"{scope}.yaml"]
    candidates.extend(_set_paths(root, scope).values())
    shadow_root = root / "shadows" / scope
    if shadow_root.exists():
        candidates.extend(sorted(shadow_root.rglob("*.json")))
    state_path = root / "state" / scope / "contract.json"
    if state_path.exists():
        candidates.append(state_path)
    for path in candidates:
        if not path.exists():
            continue
        data = _regular_bytes(path, label="configuration state input")
        inventory.append(
            {"path": path.relative_to(root).as_posix(), "sha256": _sha256(data)}
        )
    return _identity({"scope": scope, "files": inventory})


def _state_binding(root: Path, scope: str) -> dict[str, Any] | None:
    path = root / "state" / scope / "contract.json"
    if not path.exists():
        return None
    value = _json(path, label="configuration state binding")
    if value.get("schema") != STATE_SCHEMA or value.get("state_id") != _identity(
        value, "state_id"
    ):
        raise ReconciliationError("configuration state binding identity is invalid")
    if set(value) != {
        "schema",
        "state_id",
        "target_id",
        "runtime_profile",
        "parameter_profile",
        "selector_scope",
        "release_id",
        "manifest_id",
        "schema_version",
        "source_state_id",
        "set_results",
    }:
        raise ReconciliationError("configuration state binding fields are invalid")
    return value


def _reintroduction_candidates(
    shadows: tuple[tuple[Path, dict[str, Any]], ...],
    *,
    target_id: str,
    scope: str,
    profile: str,
    reference: str,
    name: str,
    retired_by_manifest_id: str,
) -> list[dict[str, Any]]:
    result = []
    for _path, shadow in shadows:
        if (
            shadow["target_id"] == target_id
            and shadow["selector_scope"] == scope
            and shadow["parameter_profile"] == profile
            and shadow["set_reference"] == reference
            and shadow["retired_by_manifest_id"] == retired_by_manifest_id
            and name in shadow["entries"]
        ):
            entry = shadow["entries"][name]
            if not isinstance(entry, dict):
                raise ReconciliationError("legacy shadow entry is malformed")
            result.append({**entry, "shadow_id": shadow["shadow_id"]})
    return result


def plan_reconciliation(
    *,
    old_immutable_root: Path,
    new_immutable_root: Path,
    writable_state_root: Path,
    operations_root: Path,
    operation_id: str,
    runtime_profile: str,
    target_id: str,
    old_release_id: str,
    new_release_id: str,
    mode: str,
    purpose: str = "activation",
) -> ReconciliationPlan:
    """Create a state-bound, no-write reconciliation plan."""

    writable = _require_absolute(writable_state_root, label="writable state root")
    operations = _require_absolute(operations_root, label="operations root")
    old_root = _require_absolute(old_immutable_root, label="old immutable root")
    new_root = _require_absolute(new_immutable_root, label="new immutable root")
    _require_operation_id(operation_id)
    for path, label in (
        (writable, "writable state root"),
        (operations, "operations root"),
    ):
        if path.exists() and (path.is_symlink() or not path.is_dir()):
            raise ReconciliationError(f"{label} is unsafe")
    for name in ("profiles", "parameter_sets", "shadows", "state", "contracts"):
        managed = writable / name
        if managed.exists() and (managed.is_symlink() or not managed.is_dir()):
            raise ReconciliationError(f"managed writable root is unsafe: {managed}")
    if mode not in ALLOWED_MODES:
        raise ReconciliationError("reconciliation mode is unsupported")
    if purpose not in {"activation", "rollback", "startup", "reset"}:
        raise ReconciliationError("reconciliation purpose is unsupported")
    if not target_id or len(target_id) > 128:
        raise ReconciliationError("logical target identity is invalid")
    _require_release_id(old_release_id, label="old release identity")
    _require_release_id(new_release_id, label="new release identity")

    old = load_installed_contract(old_root).contract
    new = load_installed_contract(new_root).contract
    compatibility = plan_compatibility(
        old_immutable_root=old_root,
        new_immutable_root=new_root,
        writable_state_root=writable,
        runtime_profile=runtime_profile,
    )
    if not compatibility.compatible:
        raise ReconciliationError(
            f"configuration contracts are incompatible: {compatibility.reason}"
        )
    descriptor = new.profile(runtime_profile)
    if mode == "simulation" and descriptor.parameter_profile != "sim":
        raise ReconciliationError("simulation reconciliation may mutate only sim state")
    if mode == "receiver-staged" and descriptor.parameter_profile != "real":
        raise ReconciliationError(
            "receiver reconciliation may mutate only aircraft state"
        )

    old_entries, _old_defaults = _contract_inputs(old, runtime_profile)
    new_entries, new_defaults = _contract_inputs(new, runtime_profile)
    new_default_descriptor = new.default_set(descriptor.parameter_profile)
    new_default_document = _contract_yaml(
        new,
        new_default_descriptor.path.as_posix(),
        label="installed tracked default",
    )
    _parameter_values_from_document(
        new_default_document, label="installed tracked default"
    )
    selected, selector_bytes = _selector(writable, descriptor.selector_scope)
    paths = _set_paths(writable, descriptor.selector_scope)
    if selected not in paths and paths:
        raise ReconciliationError("active parameter-set selector does not resolve")
    references = sorted(set(paths) | {"tracked/default.yaml"})
    shadows = _existing_shadows(writable, descriptor.selector_scope)
    if any(
        shadow["target_id"] != target_id
        or shadow["parameter_profile"] != descriptor.parameter_profile
        for _path, shadow in shadows
    ):
        raise ReconciliationError(
            "legacy shadow store contains another target or parameter profile"
        )
    binding = _state_binding(writable, descriptor.selector_scope)
    if binding is not None:
        if (
            binding.get("target_id") != target_id
            or binding.get("selector_scope") != descriptor.selector_scope
            or binding.get("parameter_profile") != descriptor.parameter_profile
            or binding.get("manifest_id") != old.manifest_id
            or binding.get("release_id") != old_release_id
        ):
            raise ReconciliationError(
                "writable configuration is bound to another target, release, profile, or manifest"
            )
    elif old.manifest_id != new.manifest_id or old_release_id != new_release_id:
        raise ReconciliationError(
            "unbound writable configuration cannot be migrated across releases"
        )

    documents: dict[str, bytes] = _retained_contract_documents(new)
    shadow_documents: dict[str, bytes] = {}
    set_plans: list[ParameterSetPlan] = []
    review_items: list[dict[str, Any]] = []
    for reference in references:
        path = paths.get(reference)
        existing_document = (
            _parameter_raw(path, label=f"parameter set {reference}")
            if path is not None
            else {}
        )
        existing = (
            dict(existing_document["/**"]["ros__parameters"])
            if existing_document
            else {}
        )
        result = dict(new_defaults)
        preserved = sorted(set(existing) & set(new_entries))
        for name in preserved:
            result[name] = existing[name]
        valid, reasons = _validate_candidates(result, new_entries)
        if not valid:
            raise ReconciliationError(
                f"parameter set {reference} has invalid retained values: "
                + "; ".join(reasons)
            )
        added = sorted(set(new_entries) - set(existing))
        removed = sorted(set(existing) - set(new_entries))
        reintroduced: list[str] = []
        for name in list(added):
            candidates = _reintroduction_candidates(
                shadows,
                target_id=target_id,
                scope=descriptor.selector_scope,
                profile=descriptor.parameter_profile,
                reference=reference,
                name=name,
                retired_by_manifest_id=old.manifest_id,
            )
            restorable = [
                candidate
                for candidate in candidates
                if candidate.get("restorable") is True
            ]
            if len(restorable) > 1:
                raise ReconciliationError(
                    f"multiple canonical restoration candidates exist for {reference}:{name}"
                )
            if not candidates:
                continue
            reintroduced.append(name)
            candidate = restorable[0] if restorable else candidates[-1]
            candidate_values = dict(result)
            candidate_values[name] = candidate.get("value")
            old_valid, validation_reasons = _validate_candidates(
                candidate_values, new_entries
            )
            selectable = bool(restorable) and old_valid
            item = {
                "set_reference": reference,
                "parameter": name,
                "old_canonical_value": candidate.get("value"),
                "new_default": new_defaults[name],
                "old_value_valid": old_valid,
                "old_value_selectable": selectable,
                "validation": list(validation_reasons),
                "provenance": {
                    key: candidate.get(key)
                    for key in (
                        "shadow_id",
                        "source_manifest_id",
                        "source_release_id",
                        "active_at_retirement",
                    )
                },
                "choices": ["use_old", "use_new_default"],
                "decision": None,
            }
            if purpose == "rollback" and selectable:
                result[name] = candidate["value"]
                item["decision"] = "use_old"
                item["decision_source"] = "deterministic-compatible-rollback"
            else:
                review_items.append(item)

        if removed:
            entries: dict[str, Any] = {}
            for name in removed:
                active_at_retirement = reference == selected
                entries[name] = {
                    "value": existing[name],
                    "restorable": active_at_retirement,
                    "active_at_retirement": active_at_retirement,
                    "source_manifest_id": old.manifest_id,
                    "source_release_id": old_release_id,
                    "source_set_sha256": _sha256(
                        _parameter_document(existing, existing_document)
                    ),
                }
            shadow = {
                "schema": SHADOW_SCHEMA,
                "shadow_id": "0" * 64,
                "target_id": target_id,
                "selector_scope": descriptor.selector_scope,
                "parameter_profile": descriptor.parameter_profile,
                "set_reference": reference,
                "retired_by_manifest_id": new.manifest_id,
                "retired_by_schema_version": new.schema_version,
                "retired_at_release": new_release_id,
                "source_manifest_id": old.manifest_id,
                "source_release_id": old_release_id,
                "entries": entries,
            }
            shadow["shadow_id"] = _identity(shadow, "shadow_id")
            shadow_path = _shadow_path(
                writable, descriptor.selector_scope, new.manifest_id, reference
            )
            shadow_documents[shadow_path.relative_to(writable).as_posix()] = _canonical(
                shadow
            )

        result_document = deepcopy(new_default_document)
        for scope_name, scope_value in existing_document.items():
            if scope_name != "/**":
                result_document[scope_name] = deepcopy(scope_value)
        data = _parameter_document(result, result_document)
        relative = (
            Path("parameter_sets") / descriptor.selector_scope / Path(reference)
        ).as_posix()
        documents[relative] = data
        set_plans.append(
            ParameterSetPlan(
                reference=reference,
                selected=reference == selected,
                source_sha256=(
                    _sha256(_regular_bytes(path, label=f"parameter set {reference}"))
                    if path is not None
                    else None
                ),
                result_sha256=_sha256(data),
                preserved=tuple(preserved),
                added=tuple(added),
                removed=tuple(removed),
                reintroduced=tuple(sorted(reintroduced)),
            )
        )

    selector_document = yaml.safe_dump(
        {"version": 1, "active_parameter_set": selected}, sort_keys=False
    ).encode("utf-8")
    documents[(Path("profiles") / f"{descriptor.selector_scope}.yaml").as_posix()] = (
        selector_document if selector_bytes is None else selector_bytes
    )
    initial_state_id = _tree_state_id(writable, descriptor.selector_scope)
    state = {
        "schema": STATE_SCHEMA,
        "state_id": "0" * 64,
        "target_id": target_id,
        "runtime_profile": runtime_profile,
        "parameter_profile": descriptor.parameter_profile,
        "selector_scope": descriptor.selector_scope,
        "release_id": new_release_id,
        "manifest_id": new.manifest_id,
        "schema_version": new.schema_version,
        "source_state_id": initial_state_id,
        "set_results": {item.reference: item.result_sha256 for item in set_plans},
    }
    state["state_id"] = _identity(state, "state_id")
    state_relative = (
        Path("state") / descriptor.selector_scope / "contract.json"
    ).as_posix()
    documents[state_relative] = _canonical(state)
    mutations = tuple(sorted((*shadow_documents, *documents)))
    identity_input = {
        "schema": PLAN_SCHEMA,
        "operation_id": operation_id,
        "mode": mode,
        "purpose": purpose,
        "target_id": target_id,
        "old_release_id": old_release_id,
        "new_release_id": new_release_id,
        "runtime_profile": runtime_profile,
        "parameter_profile": descriptor.parameter_profile,
        "selector_scope": descriptor.selector_scope,
        "old_manifest_id": old.manifest_id,
        "new_manifest_id": new.manifest_id,
        "old_schema_version": old.schema_version,
        "new_schema_version": new.schema_version,
        "initial_state_id": initial_state_id,
        "compatibility_plan_id": compatibility.plan_id,
        "sets": [item.as_dict() for item in set_plans],
        "review_items": review_items,
        "mutations": list(mutations),
        "document_hashes": {
            path: _sha256(data)
            for path, data in sorted({**shadow_documents, **documents}.items())
        },
    }
    plan_id = _identity(identity_input)
    return ReconciliationPlan(
        plan_id=plan_id,
        operation_id=operation_id,
        mode=mode,
        purpose=purpose,
        target_id=target_id,
        old_release_id=old_release_id,
        new_release_id=new_release_id,
        runtime_profile=runtime_profile,
        parameter_profile=descriptor.parameter_profile,
        selector_scope=descriptor.selector_scope,
        writable_state_root=writable,
        operations_root=operations,
        old_manifest_id=old.manifest_id,
        new_manifest_id=new.manifest_id,
        old_schema_version=old.schema_version,
        new_schema_version=new.schema_version,
        initial_state_id=initial_state_id,
        compatibility=compatibility,
        sets=tuple(set_plans),
        review_required=bool(review_items),
        review_items=tuple(review_items),
        mutations=mutations,
        _documents=documents,
        _shadow_documents=shadow_documents,
    )


def _operation_dir(plan: ReconciliationPlan) -> Path:
    path = plan.operations_root / plan.operation_id
    if path.parent != plan.operations_root:
        raise ReconciliationError("operation directory escapes its fixed root")
    return path


def write_reintroduction_review(plan: ReconciliationPlan) -> Path:
    review = {
        "schema": REVIEW_SCHEMA,
        "review_id": "0" * 64,
        "operation_id": plan.operation_id,
        "plan_id": plan.plan_id,
        "target_id": plan.target_id,
        "runtime_profile": plan.runtime_profile,
        "parameter_profile": plan.parameter_profile,
        "selector_scope": plan.selector_scope,
        "old_release_id": plan.old_release_id,
        "new_release_id": plan.new_release_id,
        "old_manifest_id": plan.old_manifest_id,
        "new_manifest_id": plan.new_manifest_id,
        "initial_state_id": plan.initial_state_id,
        "items": [dict(item) for item in plan.review_items],
    }
    review["review_id"] = _identity(review, "review_id")
    path = _operation_dir(plan) / "reconciliation-review.json"
    if path.exists():
        existing = _json(path, label="reintroduction review")
        if existing != review:
            raise ReconciliationError(
                "existing operation review differs from current plan"
            )
        return path
    _atomic_document(path, review)
    return path


def write_reintroduction_decisions(
    review_path: Path,
    decisions: Mapping[str, str],
) -> Path:
    review = _json(_require_absolute(review_path, label="review path"), label="review")
    if review.get("schema") != REVIEW_SCHEMA or review.get("review_id") != _identity(
        review, "review_id"
    ):
        raise ReconciliationError("review identity is invalid")
    expected = {
        f"{item['set_reference']}:{item['parameter']}": item
        for item in review.get("items", [])
    }
    if set(decisions) != set(expected):
        raise ReconciliationError(
            "reintroduction decisions are incomplete or unexpected"
        )
    normalized: dict[str, str] = {}
    for key, decision in decisions.items():
        if decision not in {"use_old", "use_new_default"}:
            raise ReconciliationError(f"reintroduction decision is invalid: {key}")
        if decision == "use_old" and not expected[key]["old_value_selectable"]:
            raise ReconciliationError(f"invalid legacy value cannot be selected: {key}")
        normalized[key] = decision
    value = {
        "schema": DECISIONS_SCHEMA,
        "decision_id": "0" * 64,
        "review_id": review["review_id"],
        "operation_id": review["operation_id"],
        "plan_id": review["plan_id"],
        "initial_state_id": review["initial_state_id"],
        "target_id": review["target_id"],
        "new_release_id": review["new_release_id"],
        "new_manifest_id": review["new_manifest_id"],
        "decisions": dict(sorted(normalized.items())),
    }
    value["decision_id"] = _identity(value, "decision_id")
    path = review_path.parent / "reconciliation-decisions.json"
    if path.exists():
        existing = _json(path, label="reintroduction decisions")
        if existing != value:
            raise ReconciliationError(
                "reintroduction decisions were already sealed differently"
            )
        return path
    _atomic_document(path, value)
    return path


def validate_reintroduction_decisions(
    plan: ReconciliationPlan,
    decisions: Mapping[str, str],
) -> dict[str, str]:
    """Validate a complete decision set without creating review artifacts."""

    if not plan.review_required:
        if decisions:
            raise ReconciliationError(
                "unexpected decisions supplied for a plan without review"
            )
        return {}
    expected = {
        f"{item['set_reference']}:{item['parameter']}": item
        for item in plan.review_items
    }
    if set(decisions) != set(expected):
        raise ReconciliationError(
            "reintroduction decisions are incomplete or contain extra keys"
        )
    normalized: dict[str, str] = {}
    for key, decision in decisions.items():
        if decision not in {"use_old", "use_new_default"}:
            raise ReconciliationError(f"decision is invalid: {key}")
        if decision == "use_old" and not expected[key]["old_value_selectable"]:
            raise ReconciliationError(f"invalid legacy value cannot be selected: {key}")
        normalized[key] = decision
    return dict(sorted(normalized.items()))


def _verified_decisions(plan: ReconciliationPlan, path: Path | None) -> dict[str, str]:
    if not plan.review_required:
        if path is not None:
            raise ReconciliationError(
                "unexpected decisions supplied for a plan without review"
            )
        return {}
    review_path = write_reintroduction_review(plan)
    if path is None:
        return {}
    decisions_path = _require_absolute(path, label="decisions path")
    if decisions_path.parent != review_path.parent:
        raise ReconciliationError("decisions are outside the bound operation directory")
    review = _json(review_path, label="reintroduction review")
    decisions = _json(decisions_path, label="reintroduction decisions")
    if (
        decisions.get("schema") != DECISIONS_SCHEMA
        or decisions.get("decision_id") != _identity(decisions, "decision_id")
        or decisions.get("review_id") != review["review_id"]
        or decisions.get("operation_id") != plan.operation_id
        or decisions.get("plan_id") != plan.plan_id
        or decisions.get("initial_state_id") != plan.initial_state_id
        or decisions.get("target_id") != plan.target_id
        or decisions.get("new_release_id") != plan.new_release_id
        or decisions.get("new_manifest_id") != plan.new_manifest_id
    ):
        raise ReconciliationError("decisions are stale, edited, or cross-bound")
    values = decisions.get("decisions")
    if not isinstance(values, dict):
        raise ReconciliationError("decisions are malformed")
    return validate_reintroduction_decisions(plan, values)


def _apply_review_decisions(
    plan: ReconciliationPlan, decisions: Mapping[str, str]
) -> dict[str, bytes]:
    documents = dict(plan._documents)
    if not decisions:
        return documents
    items = {
        f"{item['set_reference']}:{item['parameter']}": item
        for item in plan.review_items
    }
    for key, decision in decisions.items():
        if decision != "use_old":
            continue
        item = items[key]
        relative = (
            Path("parameter_sets") / plan.selector_scope / Path(item["set_reference"])
        ).as_posix()
        value = yaml.safe_load(documents[relative])
        value["/**"]["ros__parameters"][item["parameter"]] = item["old_canonical_value"]
        documents[relative] = _parameter_document(
            value["/**"]["ros__parameters"], value
        )
    state_relative = (Path("state") / plan.selector_scope / "contract.json").as_posix()
    state = json.loads(documents[state_relative])
    state["set_results"] = {
        item.reference: _sha256(
            documents[
                (
                    Path("parameter_sets") / plan.selector_scope / Path(item.reference)
                ).as_posix()
            ]
        )
        for item in plan.sets
    }
    state["state_id"] = _identity(state, "state_id")
    documents[state_relative] = _canonical(state)
    return documents


def _resolved_set_plans(
    plan: ReconciliationPlan, documents: Mapping[str, bytes]
) -> tuple[ParameterSetPlan, ...]:
    resolved: list[ParameterSetPlan] = []
    for item in plan.sets:
        relative = (
            Path("parameter_sets") / plan.selector_scope / Path(item.reference)
        ).as_posix()
        resolved.append(
            ParameterSetPlan(
                reference=item.reference,
                selected=item.selected,
                source_sha256=item.source_sha256,
                result_sha256=_sha256(documents[relative]),
                preserved=item.preserved,
                added=item.added,
                removed=item.removed,
                reintroduced=item.reintroduced,
            )
        )
    return tuple(resolved)


def _journal_path(plan: ReconciliationPlan) -> Path:
    return _operation_dir(plan) / "reconciliation-journal.json"


def _journal(
    plan: ReconciliationPlan,
    *,
    phase: str,
    completed_paths: list[str],
    state_id: str,
) -> dict[str, Any]:
    value = {
        "schema": JOURNAL_SCHEMA,
        "journal_id": "0" * 64,
        "operation_id": plan.operation_id,
        "plan_id": plan.plan_id,
        "initial_state_id": plan.initial_state_id,
        "phase": phase,
        "completed_paths": list(completed_paths),
        "state_id": state_id,
    }
    value["journal_id"] = _identity(value, "journal_id")
    _atomic_document(_journal_path(plan), value)
    return value


def _load_journal(plan: ReconciliationPlan) -> dict[str, Any] | None:
    path = _journal_path(plan)
    if not path.exists():
        return None
    value = _json(path, label="reconciliation journal")
    if (
        value.get("schema") != JOURNAL_SCHEMA
        or value.get("journal_id") != _identity(value, "journal_id")
        or value.get("operation_id") != plan.operation_id
        or value.get("plan_id") != plan.plan_id
        or value.get("initial_state_id") != plan.initial_state_id
    ):
        raise ReconciliationError("reconciliation journal is corrupt or cross-bound")
    return value


def _assert_receiver_stage(plan: ReconciliationPlan) -> None:
    if plan.mode != "receiver-staged":
        return
    marker_path = plan.writable_state_root / ".iii-reconciliation-stage.json"
    if not marker_path.is_file() or marker_path.is_symlink():
        raise ReconciliationError("receiver may mutate only its bound staged copy")
    marker = _json(marker_path, label="receiver reconciliation stage marker")
    if marker != {
        "schema": "iii.configuration-reconciliation-stage/v1",
        "operation_id": plan.operation_id,
        "target_id": plan.target_id,
    }:
        raise ReconciliationError("receiver may mutate only its bound staged copy")


def execute_reconciliation(
    plan: ReconciliationPlan,
    *,
    decisions_path: Path | None = None,
    decisions: Mapping[str, str] | None = None,
) -> ReconciliationResult:
    """Execute or resume an exact plan using fsynced atomic replacements."""

    _assert_receiver_stage(plan)
    if decisions_path is not None and decisions is not None:
        raise ReconciliationError(
            "decisions path and direct decisions are mutually exclusive"
        )
    if decisions is not None:
        normalized = validate_reintroduction_decisions(plan, decisions)
        if plan.review_required:
            review = write_reintroduction_review(plan)
            decisions_path = write_reintroduction_decisions(review, normalized)
    resolved_decisions = _verified_decisions(plan, decisions_path)
    review_path = (
        _operation_dir(plan) / "reconciliation-review.json"
        if plan.review_required
        else None
    )
    if plan.review_required and not resolved_decisions:
        return ReconciliationResult(
            operation_id=plan.operation_id,
            plan_id=plan.plan_id,
            status="review-required",
            state_id=plan.initial_state_id,
            journal_path=_journal_path(plan),
            review_path=review_path,
            sets=plan.sets,
            changed_paths=(),
        )

    journal = _load_journal(plan)
    completed = list(journal.get("completed_paths", [])) if journal else []
    current_state_id = _tree_state_id(plan.writable_state_root, plan.selector_scope)
    if journal is None and current_state_id != plan.initial_state_id:
        raise ReconciliationError("reconciliation plan is stale against writable state")
    if journal is not None and journal["phase"] == "complete":
        completed_documents = _apply_review_decisions(plan, resolved_decisions)
        return ReconciliationResult(
            operation_id=plan.operation_id,
            plan_id=plan.plan_id,
            status="complete",
            state_id=journal["state_id"],
            journal_path=_journal_path(plan),
            review_path=review_path,
            sets=_resolved_set_plans(plan, completed_documents),
            changed_paths=tuple(completed),
        )

    active_documents = _apply_review_decisions(plan, resolved_decisions)
    resolved_sets = _resolved_set_plans(plan, active_documents)
    documents = {**plan._shadow_documents, **active_documents}
    order = [*sorted(plan._shadow_documents), *sorted(active_documents)]
    _journal(
        plan,
        phase="prepared",
        completed_paths=completed,
        state_id=current_state_id,
    )
    for relative in order:
        target = plan.writable_state_root / Path(*PurePosixPath(relative).parts)
        expected = documents[relative]
        if relative in completed:
            if not target.is_file() or _sha256(
                _regular_bytes(target, label=relative)
            ) != _sha256(expected):
                raise ReconciliationError(
                    f"completed reconciliation path drifted: {relative}"
                )
            continue
        _atomic_bytes(target, expected)
        completed.append(relative)
        current_state_id = _tree_state_id(plan.writable_state_root, plan.selector_scope)
        _journal(
            plan,
            phase="applying",
            completed_paths=completed,
            state_id=current_state_id,
        )
    final_state_id = _tree_state_id(plan.writable_state_root, plan.selector_scope)
    _journal(
        plan,
        phase="complete",
        completed_paths=completed,
        state_id=final_state_id,
    )
    return ReconciliationResult(
        operation_id=plan.operation_id,
        plan_id=plan.plan_id,
        status="complete",
        state_id=final_state_id,
        journal_path=_journal_path(plan),
        review_path=review_path,
        sets=resolved_sets,
        changed_paths=tuple(completed),
    )


def plan_simulation_reconciliation(
    *,
    immutable_root: Path,
    writable_state_root: Path,
    operations_root: Path,
    runtime_profile: str = "sim",
    target_id: str = "sim",
    release_id: str,
) -> ReconciliationPlan:
    """Plan the same reconciliation that simulation startup will execute."""

    current = load_installed_contract(immutable_root).contract
    descriptor = current.profile(runtime_profile)
    binding = _state_binding(writable_state_root, descriptor.selector_scope)
    if binding is None:
        old_root = immutable_root
        old_release_id = release_id
        old_manifest_id = current.manifest_id
    else:
        old_manifest_id = str(binding["manifest_id"])
        old_release_id = str(binding["release_id"])
        old_root = (
            immutable_root
            if old_manifest_id == current.manifest_id
            else writable_state_root / "contracts" / old_manifest_id
        )
        if not old_root.is_dir():
            raise ReconciliationError(
                "previous installed configuration contract is unavailable; simulation startup is blocked"
            )
    initial = _tree_state_id(writable_state_root, descriptor.selector_scope)
    operation_id = (
        f"sim-startup-{old_manifest_id[:10]}-{current.manifest_id[:10]}-{initial[:10]}"
    )
    return plan_reconciliation(
        old_immutable_root=old_root,
        new_immutable_root=immutable_root,
        writable_state_root=writable_state_root,
        operations_root=operations_root,
        operation_id=operation_id,
        runtime_profile=runtime_profile,
        target_id=target_id,
        old_release_id=old_release_id,
        new_release_id=release_id,
        mode="simulation",
        purpose="startup",
    )


def reconcile_simulation_startup(
    *,
    immutable_root: Path,
    writable_state_root: Path,
    operations_root: Path,
    runtime_profile: str = "sim",
    target_id: str = "sim",
    release_id: str,
) -> ReconciliationResult:
    """Reconcile all sim sets before any runtime path is selected."""

    plan = plan_simulation_reconciliation(
        immutable_root=immutable_root,
        writable_state_root=writable_state_root,
        operations_root=operations_root,
        runtime_profile=runtime_profile,
        target_id=target_id,
        release_id=release_id,
    )
    decisions_path = (
        plan.operations_root / plan.operation_id / "reconciliation-decisions.json"
    )
    result = execute_reconciliation(
        plan,
        decisions_path=decisions_path if decisions_path.is_file() else None,
    )
    if result.status != "complete":
        raise ReconciliationError(
            f"simulation configuration is blocked by {result.status}: {result.review_path}"
        )
    return result


def plan_configuration_checkpoint(
    *,
    writable_state_root: Path,
    checkpoint_root: Path,
    target_id: str,
    runtime_profile: str,
    schema_version: int | None = None,
    release_id: str | None = None,
    manifest_id: str | None = None,
) -> dict[str, Any]:
    """Plan a content-addressed checkpoint without creating any path."""

    source = _require_absolute(writable_state_root, label="writable state root")
    destination_root = _require_absolute(checkpoint_root, label="checkpoint root")
    if source.is_symlink() or not source.is_dir():
        raise ReconciliationError("configuration checkpoint source is unsafe")
    if destination_root.exists() and (
        destination_root.is_symlink() or not destination_root.is_dir()
    ):
        raise ReconciliationError("configuration checkpoint destination is unsafe")
    files: list[dict[str, Any]] = []
    for path in sorted(source.rglob("*")):
        if path.is_symlink():
            raise ReconciliationError(f"configuration checkpoint contains link: {path}")
        if path.is_file() and path.name != ".iii-reconciliation-stage.json":
            data = _regular_bytes(path, label="configuration checkpoint input")
            files.append(
                {
                    "path": path.relative_to(source).as_posix(),
                    "sha256": _sha256(data),
                    "size": len(data),
                }
            )
    value = {
        "schema": CHECKPOINT_SCHEMA,
        "checkpoint_id": "0" * 64,
        "target_id": target_id,
        "profile": runtime_profile,
        "schema_version": schema_version,
        "release_id": release_id,
        "configuration_manifest_id": manifest_id,
        "files": files,
    }
    value["checkpoint_id"] = _identity(value, "checkpoint_id")
    return {
        **value,
        "path": str(destination_root / value["checkpoint_id"]),
        "source": str(source),
        "mutations": [str(destination_root / value["checkpoint_id"])],
    }


def plan_reconciled_checkpoint(
    plan: ReconciliationPlan,
    *,
    source_checkpoint: Path,
    checkpoint_root: Path,
    decisions: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Predict the exact receiver checkpoint produced by an unresolved-free plan."""

    if plan.mode != "receiver-staged":
        raise ReconciliationError(
            "only receiver reconciliation produces aircraft checkpoints"
        )
    normalized = validate_reintroduction_decisions(plan, decisions or {})
    source = _require_absolute(source_checkpoint, label="source checkpoint")
    destination = _require_absolute(checkpoint_root, label="checkpoint root")
    verify_configuration_checkpoint(source)
    inventory: dict[str, bytes] = {}
    for path in sorted(source.rglob("*")):
        if path == source / "checkpoint.json":
            continue
        if path.is_symlink():
            raise ReconciliationError("source configuration checkpoint contains a link")
        if path.is_file():
            inventory[path.relative_to(source).as_posix()] = _regular_bytes(
                path, label="source configuration checkpoint content"
            )
    inventory.update(plan._shadow_documents)
    inventory.update(_apply_review_decisions(plan, normalized))
    inventory.pop(".iii-reconciliation-stage.json", None)
    files = [
        {"path": relative, "sha256": _sha256(data), "size": len(data)}
        for relative, data in sorted(inventory.items())
    ]
    value = {
        "schema": CHECKPOINT_SCHEMA,
        "checkpoint_id": "0" * 64,
        "target_id": plan.target_id,
        "profile": plan.runtime_profile,
        "schema_version": plan.new_schema_version,
        "release_id": plan.new_release_id,
        "configuration_manifest_id": plan.new_manifest_id,
        "files": files,
    }
    value["checkpoint_id"] = _identity(value, "checkpoint_id")
    return {
        **value,
        "path": str(destination / value["checkpoint_id"]),
        "source_checkpoint": str(source),
        "reconciliation_plan_id": plan.plan_id,
        "mutations": [str(destination / value["checkpoint_id"])],
    }


def seal_configuration_checkpoint(
    *,
    writable_state_root: Path,
    checkpoint_root: Path,
    target_id: str,
    runtime_profile: str,
    schema_version: int | None = None,
    release_id: str | None = None,
    manifest_id: str | None = None,
) -> dict[str, Any]:
    """Seal a content-addressed, independently verifiable tree checkpoint."""

    source = _require_absolute(writable_state_root, label="writable state root")
    destination_root = _require_absolute(checkpoint_root, label="checkpoint root")
    planned = plan_configuration_checkpoint(
        writable_state_root=source,
        checkpoint_root=destination_root,
        target_id=target_id,
        runtime_profile=runtime_profile,
        schema_version=schema_version,
        release_id=release_id,
        manifest_id=manifest_id,
    )
    value = {
        key: item
        for key, item in planned.items()
        if key not in {"path", "source", "mutations"}
    }
    final = destination_root / value["checkpoint_id"]
    if final.exists():
        existing = verify_configuration_checkpoint(final)
        if existing != value:
            raise ReconciliationError(
                "existing checkpoint path contains another authenticated checkpoint"
            )
        return {**value, "path": str(final), "deduplicated": True}
    destination_root.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".checkpoint-", dir=destination_root))
    try:
        shutil.copytree(source, temporary, dirs_exist_ok=True, symlinks=False)
        marker = temporary / ".iii-reconciliation-stage.json"
        marker.unlink(missing_ok=True)
        copied = plan_configuration_checkpoint(
            writable_state_root=temporary,
            checkpoint_root=destination_root,
            target_id=target_id,
            runtime_profile=runtime_profile,
            schema_version=schema_version,
            release_id=release_id,
            manifest_id=manifest_id,
        )
        if copied["checkpoint_id"] != value["checkpoint_id"]:
            raise ReconciliationError(
                "configuration changed while its checkpoint was being sealed"
            )
        _atomic_document(temporary / "checkpoint.json", value, mode=0o444)
        for path in temporary.rglob("*"):
            if path.is_file():
                path.chmod(0o444)
            elif path.is_dir():
                path.chmod(0o555)
        os.replace(temporary, final)
        _fsync_directory(destination_root)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    verify_configuration_checkpoint(final)
    return {**value, "path": str(final), "deduplicated": False}


def verify_configuration_checkpoint(path: Path) -> dict[str, Any]:
    root = _require_absolute(path, label="checkpoint path")
    if root.is_symlink() or not root.is_dir():
        raise ReconciliationError("configuration checkpoint root is unsafe")
    value = _json(root / "checkpoint.json", label="configuration checkpoint manifest")
    if (
        set(value)
        != {
            "schema",
            "checkpoint_id",
            "target_id",
            "profile",
            "schema_version",
            "release_id",
            "configuration_manifest_id",
            "files",
        }
        or value.get("schema") != CHECKPOINT_SCHEMA
        or value.get("checkpoint_id") != _identity(value, "checkpoint_id")
        or root.name != value.get("checkpoint_id")
    ):
        raise ReconciliationError("configuration checkpoint identity is invalid")
    if (
        not isinstance(value["target_id"], str)
        or not value["target_id"]
        or not isinstance(value["profile"], str)
        or not value["profile"]
        or (
            value["schema_version"] is not None
            and (
                not isinstance(value["schema_version"], int)
                or isinstance(value["schema_version"], bool)
                or value["schema_version"] < 1
            )
        )
        or (
            value["release_id"] is not None
            and (not isinstance(value["release_id"], str) or not value["release_id"])
        )
        or (
            value["configuration_manifest_id"] is not None
            and (
                not isinstance(value["configuration_manifest_id"], str)
                or not HASH.fullmatch(value["configuration_manifest_id"])
            )
        )
        or not isinstance(value["files"], list)
    ):
        raise ReconciliationError("configuration checkpoint metadata is invalid")
    expected: dict[str, dict[str, Any]] = {}
    ordered_paths: list[str] = []
    for row in value["files"]:
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256", "size"}
            or not isinstance(row["path"], str)
            or row["path"] == "checkpoint.json"
            or not isinstance(row["sha256"], str)
            or not HASH.fullmatch(row["sha256"])
            or not isinstance(row["size"], int)
            or isinstance(row["size"], bool)
            or row["size"] < 0
        ):
            raise ReconciliationError(
                "configuration checkpoint file inventory is malformed"
            )
        safe = _safe_relative(row["path"], label="checkpoint file path").as_posix()
        if safe in expected:
            raise ReconciliationError(
                "configuration checkpoint file inventory is duplicated"
            )
        expected[safe] = row
        ordered_paths.append(safe)
    if ordered_paths != sorted(ordered_paths):
        raise ReconciliationError(
            "configuration checkpoint file inventory is not canonical"
        )
    observed: dict[str, dict[str, Any]] = {}
    for path_item in sorted(root.rglob("*")):
        if path_item == root / "checkpoint.json":
            continue
        if path_item.is_symlink():
            raise ReconciliationError("configuration checkpoint contains a link")
        if path_item.is_file():
            data = _regular_bytes(path_item, label="configuration checkpoint content")
            observed[path_item.relative_to(root).as_posix()] = {
                "path": path_item.relative_to(root).as_posix(),
                "sha256": _sha256(data),
                "size": len(data),
            }
    if expected != observed:
        raise ReconciliationError(
            "configuration checkpoint inventory or content differs"
        )
    return value


def materialize_receiver_stage(
    *,
    source_checkpoint: Path,
    stage_root: Path,
    operation_id: str,
    target_id: str,
) -> Path:
    """Copy an immutable aircraft checkpoint into a receiver-owned private stage."""

    _require_operation_id(operation_id)
    source = _require_absolute(source_checkpoint, label="source checkpoint")
    destination = _require_absolute(stage_root, label="stage root")
    manifest = verify_configuration_checkpoint(source)
    if manifest.get("target_id") != target_id:
        raise ReconciliationError("source checkpoint targets another aircraft")
    if destination.exists():
        raise ReconciliationError("receiver stage already exists")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.", dir=destination.parent)
    )
    try:
        shutil.copytree(
            source,
            temporary / "state",
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns("checkpoint.json"),
        )
        for path in (temporary / "state").rglob("*"):
            path.chmod(0o750 if path.is_dir() else 0o640)
        (temporary / "state").chmod(0o750)
        marker = {
            "schema": "iii.configuration-reconciliation-stage/v1",
            "operation_id": operation_id,
            "target_id": target_id,
        }
        _atomic_document(temporary / "state/.iii-reconciliation-stage.json", marker)
        os.replace(temporary, destination)
        _fsync_directory(destination.parent)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return destination / "state"
