"""Durable tuning-session and configuration-transaction state.

This module contains no ROS imports.  The configuration server supplies validation,
distributed application/readback, and active-set persistence adapters while this
store owns identity, concurrency, write-ahead journaling, crash recovery, and the
explicit divergent fault boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Callable, Mapping, Sequence

SESSION_SCHEMA = "iii.configuration-tuning-session/v1"
BASELINE_SCHEMA = "iii.configuration-tuning-baseline/v1"
STATE_SCHEMA = "iii.configuration-tuning-state/v1"
WAL_SCHEMA = "iii.configuration-tuning-wal-entry/v1"
PLAN_SCHEMA = "iii.configuration-transaction-plan/v1"
RESULT_SCHEMA = "iii.configuration-transaction-result/v1"
SELECTOR_SCHEMA = "iii.configuration-tuning-selector/v1"
CHECKPOINT_SCHEMA = "iii.configuration-tuning-checkpoint/v1"

HASH = re.compile(r"^[a-f0-9]{64}$")
IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:@+-]{0,127}$")
PARAMETER = re.compile(r"^/[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*$")
RESTART_KINDS = frozenset({"none", "node", "runtime"})


class TuningError(RuntimeError):
    """A tuning contract, journal, concurrency, or recovery invariant failed."""


def _canonical(value: Any) -> bytes:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise TuningError(f"value is not canonical JSON: {exc}") from exc


def _identity(value: Mapping[str, Any], field: str) -> str:
    payload = {key: item for key, item in value.items() if key != field}
    return hashlib.sha256(_canonical(payload)).hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _safe_directory(path: Path, *, create: bool = False) -> Path:
    root = path.expanduser().absolute()
    if root.exists() and (root.is_symlink() or not root.is_dir()):
        raise TuningError(f"tuning root is not a safe directory: {root}")
    if create and not root.exists():
        root.mkdir(parents=True, mode=0o750)
        _fsync_directory(root.parent)
    return root


def _atomic_bytes(path: Path, data: bytes, *, mode: int = 0o640) -> None:
    if path.exists() and (path.is_symlink() or not path.is_file()):
        raise TuningError(f"refusing to replace unsafe tuning path: {path}")
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o750)
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
    _atomic_bytes(path, _canonical(value) + b"\n", mode=mode)


def _read_document(path: Path, *, label: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise TuningError(f"{label} is missing or linked")
    try:
        raw = path.read_bytes()
        value = json.loads(raw)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TuningError(f"cannot read {label}: {exc}") from exc
    if not isinstance(value, dict) or raw != _canonical(value) + b"\n":
        raise TuningError(f"{label} is not canonical JSON")
    return value


def _exact(value: Mapping[str, Any], fields: set[str], *, label: str) -> None:
    if set(value) != fields:
        raise TuningError(f"{label} fields do not match the fixed contract")


def _require_identity(value: Any, *, label: str, allow_hash: bool = True) -> str:
    if not isinstance(value, str) or not value:
        raise TuningError(f"{label} is missing")
    if allow_hash and HASH.fullmatch(value):
        return value
    if not IDENTITY.fullmatch(value):
        raise TuningError(f"{label} is malformed")
    return value


def _values(value: Any, *, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TuningError(f"{label} must be an object")
    result: dict[str, Any] = {}
    for name, item in value.items():
        if not isinstance(name, str) or not PARAMETER.fullmatch(name):
            raise TuningError(f"{label} contains an invalid parameter name")
        _canonical(item)
        result[name] = item
    return dict(sorted(result.items()))


def _safe_slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", value).strip("-")


@dataclass(frozen=True)
class TransactionPlan:
    session_id: str
    transaction_id: str
    request_id: str
    expected_revision: int
    revision_after: int
    operator_id: str
    edits: tuple[dict[str, Any], ...]
    previous_active_values: Mapping[str, Any]
    previous_persisted_values: Mapping[str, Any]
    previous_pending_boot_values: Mapping[str, Any]
    desired_active_values: Mapping[str, Any]
    desired_persisted_values: Mapping[str, Any]
    desired_pending_boot_values: Mapping[str, Any]
    request_fingerprint: str
    schema: str = PLAN_SCHEMA

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "session_id": self.session_id,
            "transaction_id": self.transaction_id,
            "request_id": self.request_id,
            "expected_revision": self.expected_revision,
            "revision_after": self.revision_after,
            "operator_id": self.operator_id,
            "edits": [dict(item) for item in self.edits],
            "previous_active_values": dict(self.previous_active_values),
            "previous_persisted_values": dict(self.previous_persisted_values),
            "previous_pending_boot_values": dict(self.previous_pending_boot_values),
            "desired_active_values": dict(self.desired_active_values),
            "desired_persisted_values": dict(self.desired_persisted_values),
            "desired_pending_boot_values": dict(self.desired_pending_boot_values),
            "request_fingerprint": self.request_fingerprint,
        }


class TuningSessionStore:
    """Append-only, checksummed tuning session with atomic state checkpoints."""

    def __init__(
        self,
        *,
        root: Path,
        target_id: str,
        runtime_profile: str,
        release_id: str,
        workspace_id: str,
        manifest_id: str,
        now: Callable[[], str],
    ) -> None:
        self.root = _safe_directory(root)
        self.target_id = _require_identity(target_id, label="logical target")
        self.runtime_profile = _require_identity(
            runtime_profile, label="runtime profile", allow_hash=False
        )
        self.release_id = _require_identity(release_id, label="release identity")
        self.workspace_id = _require_identity(workspace_id, label="workspace identity")
        if not isinstance(manifest_id, str) or not HASH.fullmatch(manifest_id):
            raise TuningError("configuration manifest identity is malformed")
        self.manifest_id = manifest_id
        self.now = now

    @property
    def selector_path(self) -> Path:
        name = f"{_safe_slug(self.target_id)}--{_safe_slug(self.runtime_profile)}.json"
        return self.root / "selectors" / name

    def _session_root(self, session_id: str) -> Path:
        if not HASH.fullmatch(session_id):
            raise TuningError("tuning session identity is malformed")
        path = self.root / "sessions" / session_id
        if path.parent != self.root / "sessions":
            raise TuningError("tuning session path escapes its fixed root")
        return path

    @staticmethod
    def _state_id(value: Mapping[str, Any]) -> str:
        return _identity(value, "state_id")

    def _validate_baseline(self, value: Mapping[str, Any]) -> None:
        _exact(
            value,
            {
                "schema",
                "baseline_id",
                "session_id",
                "created_at",
                "target_id",
                "runtime_profile",
                "release_id",
                "workspace_id",
                "manifest_id",
                "values",
            },
            label="tuning baseline",
        )
        if (
            value["schema"] != BASELINE_SCHEMA
            or value["baseline_id"] != _identity(value, "baseline_id")
            or not HASH.fullmatch(str(value["session_id"]))
        ):
            raise TuningError("tuning baseline identity is invalid")
        _values(value["values"], label="tuning baseline values")

    def _validate_state(self, value: Mapping[str, Any]) -> None:
        _exact(
            value,
            {
                "schema",
                "state_id",
                "session_id",
                "baseline_id",
                "revision",
                "wal_sequence",
                "wal_checksum",
                "active_values",
                "persisted_values",
                "pending_boot_values",
                "divergent",
                "divergent_observations",
                "last_result",
                "updated_at",
            },
            label="tuning state",
        )
        if (
            value["schema"] != STATE_SCHEMA
            or value["state_id"] != self._state_id(value)
            or not HASH.fullmatch(str(value["session_id"]))
            or not HASH.fullmatch(str(value["baseline_id"]))
            or isinstance(value["revision"], bool)
            or not isinstance(value["revision"], int)
            or value["revision"] < 0
            or isinstance(value["wal_sequence"], bool)
            or not isinstance(value["wal_sequence"], int)
            or value["wal_sequence"] < 0
            or not isinstance(value["divergent"], bool)
            or not isinstance(value["divergent_observations"], dict)
            or value["last_result"] is not None
            and not isinstance(value["last_result"], dict)
        ):
            raise TuningError("tuning state metadata is invalid")
        if value["wal_sequence"] == 0:
            if value["wal_checksum"] is not None:
                raise TuningError("empty tuning WAL has a checksum")
        elif not isinstance(value["wal_checksum"], str) or not HASH.fullmatch(
            value["wal_checksum"]
        ):
            raise TuningError("tuning state WAL checksum is invalid")
        for field in ("active_values", "persisted_values", "pending_boot_values"):
            _values(value[field], label=field.replace("_", " "))

    def _validate_selector(self, value: Mapping[str, Any]) -> None:
        _exact(
            value,
            {
                "schema",
                "session_id",
                "state_id",
                "target_id",
                "runtime_profile",
                "release_id",
                "workspace_id",
                "manifest_id",
            },
            label="tuning selector",
        )
        if value["schema"] != SELECTOR_SCHEMA:
            raise TuningError("tuning selector schema is unsupported")
        expected = {
            "target_id": self.target_id,
            "runtime_profile": self.runtime_profile,
        }
        if any(value.get(key) != item for key, item in expected.items()):
            raise TuningError("tuning selector belongs to another runtime identity")
        if not HASH.fullmatch(str(value["session_id"])) or not HASH.fullmatch(
            str(value["state_id"])
        ):
            raise TuningError("tuning selector identities are malformed")

    def _load_current(self) -> tuple[dict[str, Any], dict[str, Any]] | None:
        if not self.selector_path.exists():
            return None
        selector = _read_document(self.selector_path, label="tuning selector")
        self._validate_selector(selector)
        if any(
            selector[field] != expected
            for field, expected in {
                "release_id": self.release_id,
                "workspace_id": self.workspace_id,
                "manifest_id": self.manifest_id,
            }.items()
        ):
            return None
        session_root = self._session_root(selector["session_id"])
        state = _read_document(session_root / "state.json", label="tuning state")
        self._validate_state(state)
        if state["session_id"] != selector["session_id"]:
            raise TuningError("tuning selector and state sessions differ")
        baseline = _read_document(
            session_root / "baseline.json", label="tuning baseline"
        )
        self._validate_baseline(baseline)
        if baseline["baseline_id"] != state["baseline_id"]:
            raise TuningError("tuning state is bound to another baseline")
        entries = self._read_wal(session_root)
        last_sequence = entries[-1]["sequence"] if entries else 0
        last_checksum = entries[-1]["checksum"] if entries else None
        if state["wal_sequence"] > last_sequence:
            raise TuningError("tuning state advances beyond its WAL")
        if state["wal_sequence"]:
            retained = entries[state["wal_sequence"] - 1]
            if retained["checksum"] != state["wal_checksum"]:
                raise TuningError("tuning state is not anchored in its WAL")
        elif state["wal_checksum"] is not None:
            raise TuningError("empty tuning state has a WAL checksum")
        if state["wal_sequence"] < last_sequence:
            state = self._replay_state_lag(
                session_root, state, entries[state["wal_sequence"] :]
            )
        elif (
            state["wal_checksum"] != last_checksum
            or state["state_id"] != selector["state_id"]
        ):
            raise TuningError("tuning selector, state, and WAL head differ")
        return state, baseline

    def _replay_state_lag(
        self,
        session_root: Path,
        state: Mapping[str, Any],
        entries: Sequence[Mapping[str, Any]],
    ) -> dict[str, Any]:
        """Rebuild an atomically lagging state file from fsynced WAL records."""
        next_state = dict(state)
        checkpoint = False
        for entry in entries:
            kind = entry["kind"]
            body = entry["body"]
            result = body.get("result")
            plan = body.get("plan")
            if kind in {"committed", "recovered-commit"}:
                if not isinstance(plan, dict) or not isinstance(result, dict):
                    raise TuningError("committed WAL record lacks its plan/result")
                next_state["revision"] = entry["revision"]
                next_state["active_values"] = _values(
                    plan.get("desired_active_values"), label="replayed active values"
                )
                next_state["persisted_values"] = _values(
                    plan.get("desired_persisted_values"),
                    label="replayed persisted values",
                )
                next_state["pending_boot_values"] = _values(
                    plan.get("desired_pending_boot_values"),
                    label="replayed pending boot values",
                )
                next_state["last_result"] = dict(result)
                checkpoint = True
            elif kind in {"rejected", "aborted"}:
                if not isinstance(result, dict):
                    raise TuningError("terminal WAL record lacks its result")
                next_state["last_result"] = dict(result)
            elif kind == "divergent":
                if not isinstance(result, dict):
                    raise TuningError("divergent WAL record lacks its result")
                next_state["last_result"] = dict(result)
                next_state["divergent"] = True
                next_state["divergent_observations"] = _values(
                    result.get("observed_values", {}),
                    label="divergent observations",
                )
                checkpoint = True
            elif kind == "boot-confirmed":
                observed = _values(
                    body.get("observed_values", {}),
                    label="boot confirmation observations",
                )
                active = dict(next_state["active_values"])
                for name in next_state["pending_boot_values"]:
                    if name not in observed:
                        raise TuningError(
                            "boot confirmation WAL lacks required readback"
                        )
                    active[name] = observed[name]
                next_state["active_values"] = active
                next_state["pending_boot_values"] = {}
                checkpoint = True
            elif kind == "divergence-reconciled":
                if not isinstance(result, dict):
                    raise TuningError(
                        "divergence reconciliation WAL record lacks its result"
                    )
                next_state["last_result"] = dict(result)
                next_state["divergent"] = False
                next_state["divergent_observations"] = {}
                checkpoint = True
            elif kind != "prepared":
                raise TuningError(f"unsupported tuning WAL kind: {kind}")
            next_state["state_id"] = ""
            next_state["wal_sequence"] = entry["sequence"]
            next_state["wal_checksum"] = entry["checksum"]
            next_state["updated_at"] = entry["timestamp"]
            next_state["state_id"] = self._state_id(next_state)
        if checkpoint:
            self._write_checkpoint(session_root, next_state)
        self._write_state_and_selector(session_root, next_state)
        return next_state

    def status(self) -> dict[str, Any]:
        current = self._load_current()
        if current is None:
            return {
                "session_id": None,
                "baseline_id": None,
                "target_id": self.target_id,
                "runtime_profile": self.runtime_profile,
                "release_id": self.release_id,
                "workspace_id": self.workspace_id,
                "manifest_id": self.manifest_id,
                "revision": 0,
                "wal_sequence": 0,
                "wal_checksum": None,
                "created_at": None,
                "updated_at": None,
                "active_values": {},
                "persisted_values": {},
                "pending_boot_values": {},
                "divergent": False,
                "divergent_observations": {},
                "last_result": None,
            }
        state, baseline = current
        return {
            "session_id": state["session_id"],
            "baseline_id": baseline["baseline_id"],
            "target_id": self.target_id,
            "runtime_profile": self.runtime_profile,
            "release_id": self.release_id,
            "workspace_id": self.workspace_id,
            "manifest_id": self.manifest_id,
            "revision": state["revision"],
            "wal_sequence": state["wal_sequence"],
            "wal_checksum": state["wal_checksum"],
            "created_at": baseline["created_at"],
            "updated_at": state["updated_at"],
            "active_values": dict(state["active_values"]),
            "persisted_values": dict(state["persisted_values"]),
            "pending_boot_values": dict(state["pending_boot_values"]),
            "divergent": state["divergent"],
            "divergent_observations": dict(state["divergent_observations"]),
            "last_result": (
                None if state["last_result"] is None else dict(state["last_result"])
            ),
        }

    def journal_batch(
        self,
        *,
        session_id: str | None,
        after_sequence: int,
        limit: int,
    ) -> dict[str, Any]:
        if (
            isinstance(after_sequence, bool)
            or not isinstance(after_sequence, int)
            or after_sequence < 0
        ):
            raise TuningError("journal after-sequence is invalid")
        if (
            isinstance(limit, bool)
            or not isinstance(limit, int)
            or not 1 <= limit <= 1000
        ):
            raise TuningError("journal batch limit is invalid")
        current = self._load_current()
        if session_id is not None and (
            current is None or current[0]["session_id"] != session_id
        ):
            state, baseline, entries = self._load_historical_session(session_id)
        elif current is None:
            if after_sequence != 0:
                raise TuningError(
                    "journal cursor advances beyond the authoritative head"
                )
            return {
                "schema": "iii.configuration-journal-batch/v1",
                "session": None,
                "baseline_values": {},
                "after_sequence": after_sequence,
                "through_sequence": after_sequence,
                "head_sequence": 0,
                "head_checksum": None,
                "entries": [],
                "complete": True,
            }
        else:
            state, baseline = current
            entries = self._read_wal(self._session_root(state["session_id"]))
        if after_sequence > len(entries):
            raise TuningError("journal cursor advances beyond the authoritative head")
        selected = entries[after_sequence : after_sequence + limit]
        through = selected[-1]["sequence"] if selected else after_sequence
        return {
            "schema": "iii.configuration-journal-batch/v1",
            "session": {
                "session_id": state["session_id"],
                "baseline_id": baseline["baseline_id"],
                "target_id": baseline["target_id"],
                "runtime_profile": baseline["runtime_profile"],
                "release_id": baseline["release_id"],
                "workspace_id": baseline["workspace_id"],
                "manifest_id": baseline["manifest_id"],
                "created_at": baseline["created_at"],
                "updated_at": (
                    entries[-1]["timestamp"] if entries else state["updated_at"]
                ),
                "revision": entries[-1]["revision"] if entries else state["revision"],
            },
            "baseline_values": dict(baseline["values"]),
            "after_sequence": after_sequence,
            "through_sequence": through,
            "head_sequence": len(entries),
            "head_checksum": entries[-1]["checksum"] if entries else None,
            "entries": [dict(entry) for entry in selected],
            "complete": through == len(entries),
        }

    def ensure_session(
        self,
        *,
        baseline_values: Mapping[str, Any],
        persisted_values: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Open the implicit session without inventing a parameter transaction."""
        self._ensure_session(
            baseline_values=baseline_values,
            persisted_values=persisted_values,
        )
        return self.status()

    def _load_historical_session(
        self, session_id: str
    ) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
        """Authenticate a retained session without changing the living selector."""
        session_root = self._session_root(session_id)
        if not session_root.is_dir() or session_root.is_symlink():
            raise TuningError("requested tuning session does not exist")
        state = _read_document(session_root / "state.json", label="tuning state")
        baseline = _read_document(
            session_root / "baseline.json", label="tuning baseline"
        )
        self._validate_state(state)
        self._validate_baseline(baseline)
        if (
            state["session_id"] != session_id
            or baseline["session_id"] != session_id
            or state["baseline_id"] != baseline["baseline_id"]
        ):
            raise TuningError("retained tuning session identities differ")
        if (
            baseline["target_id"] != self.target_id
            or baseline["runtime_profile"] != self.runtime_profile
        ):
            raise TuningError("retained tuning session belongs to another target")
        entries = self._read_wal(session_root)
        if state["wal_sequence"] > len(entries):
            raise TuningError("retained tuning state advances beyond its WAL")
        if state["wal_sequence"]:
            if entries[state["wal_sequence"] - 1]["checksum"] != state["wal_checksum"]:
                raise TuningError("retained tuning state is not anchored in its WAL")
        elif state["wal_checksum"] is not None:
            raise TuningError("empty retained tuning state has a WAL checksum")
        return state, baseline, entries

    def _new_session(
        self,
        *,
        baseline_values: Mapping[str, Any],
        persisted_values: Mapping[str, Any],
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        baseline_values = _values(dict(baseline_values), label="baseline values")
        persisted_values = _values(dict(persisted_values), label="persisted values")
        created_at = self.now()
        seed = {
            "created_at": created_at,
            "target_id": self.target_id,
            "runtime_profile": self.runtime_profile,
            "release_id": self.release_id,
            "workspace_id": self.workspace_id,
            "manifest_id": self.manifest_id,
            "values": baseline_values,
        }
        session_id = hashlib.sha256(_canonical(seed)).hexdigest()
        baseline: dict[str, Any] = {
            "schema": BASELINE_SCHEMA,
            "baseline_id": "",
            "session_id": session_id,
            **seed,
        }
        baseline["baseline_id"] = _identity(baseline, "baseline_id")
        state: dict[str, Any] = {
            "schema": STATE_SCHEMA,
            "state_id": "",
            "session_id": session_id,
            "baseline_id": baseline["baseline_id"],
            "revision": 0,
            "wal_sequence": 0,
            "wal_checksum": None,
            "active_values": baseline_values,
            "persisted_values": persisted_values,
            "pending_boot_values": {},
            "divergent": False,
            "divergent_observations": {},
            "last_result": None,
            "updated_at": created_at,
        }
        state["state_id"] = self._state_id(state)
        session_root = self._session_root(session_id)
        if session_root.exists():
            raise TuningError("new tuning session identity already exists")
        session_root.mkdir(parents=True, mode=0o750)
        _fsync_directory(session_root.parent)
        _atomic_document(session_root / "baseline.json", baseline, mode=0o440)
        self._write_checkpoint(session_root, state)
        self._write_state_and_selector(session_root, state)
        return state, baseline

    def _ensure_session(
        self,
        *,
        baseline_values: Mapping[str, Any],
        persisted_values: Mapping[str, Any],
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        current = self._load_current()
        if current is not None:
            return current
        _safe_directory(self.root, create=True)
        return self._new_session(
            baseline_values=baseline_values, persisted_values=persisted_values
        )

    def _read_wal(self, session_root: Path) -> list[dict[str, Any]]:
        path = session_root / "journal.jsonl"
        if not path.exists():
            return []
        if path.is_symlink() or not path.is_file():
            raise TuningError("tuning WAL is unsafe")
        entries: list[dict[str, Any]] = []
        previous: str | None = None
        try:
            lines = path.read_bytes().splitlines(keepends=True)
        except OSError as exc:
            raise TuningError(f"cannot read tuning WAL: {exc}") from exc
        for index, raw in enumerate(lines, start=1):
            if not raw.endswith(b"\n"):
                raise TuningError("tuning WAL ends with a partial record")
            try:
                value = json.loads(raw)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise TuningError(f"tuning WAL record is invalid: {exc}") from exc
            if not isinstance(value, dict) or raw != _canonical(value) + b"\n":
                raise TuningError("tuning WAL record is not canonical")
            _exact(
                value,
                {
                    "schema",
                    "sequence",
                    "previous_checksum",
                    "checksum",
                    "kind",
                    "timestamp",
                    "session_id",
                    "transaction_id",
                    "request_id",
                    "revision",
                    "body",
                },
                label="tuning WAL record",
            )
            if (
                value["schema"] != WAL_SCHEMA
                or value["sequence"] != index
                or value["previous_checksum"] != previous
                or value["checksum"] != _identity(value, "checksum")
                or value["session_id"] != session_root.name
                or not isinstance(value["body"], dict)
            ):
                raise TuningError("tuning WAL chain is invalid")
            previous = value["checksum"]
            entries.append(value)
        return entries

    def _append(
        self,
        session_root: Path,
        state: Mapping[str, Any],
        *,
        kind: str,
        transaction_id: str,
        request_id: str,
        revision: int,
        body: Mapping[str, Any],
    ) -> dict[str, Any]:
        if not IDENTITY.fullmatch(kind):
            raise TuningError("tuning WAL kind is malformed")
        sequence = int(state["wal_sequence"]) + 1
        value: dict[str, Any] = {
            "schema": WAL_SCHEMA,
            "sequence": sequence,
            "previous_checksum": state["wal_checksum"],
            "checksum": "",
            "kind": kind,
            "timestamp": self.now(),
            "session_id": state["session_id"],
            "transaction_id": transaction_id,
            "request_id": request_id,
            "revision": revision,
            "body": dict(body),
        }
        value["checksum"] = _identity(value, "checksum")
        path = session_root / "journal.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o750)
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o640)
        try:
            with os.fdopen(descriptor, "ab", closefd=True) as stream:
                stream.write(_canonical(value) + b"\n")
                stream.flush()
                os.fsync(stream.fileno())
            _fsync_directory(path.parent)
        except Exception:
            raise
        return value

    def _write_checkpoint(self, session_root: Path, state: Mapping[str, Any]) -> None:
        value: dict[str, Any] = {
            "schema": CHECKPOINT_SCHEMA,
            "checkpoint_id": "",
            "session_id": state["session_id"],
            "state_id": state["state_id"],
            "revision": state["revision"],
            "wal_sequence": state["wal_sequence"],
            "wal_checksum": state["wal_checksum"],
            "created_at": state["updated_at"],
        }
        value["checkpoint_id"] = _identity(value, "checkpoint_id")
        path = (
            session_root
            / "checkpoints"
            / f"{int(state['revision']):020d}-{int(state['wal_sequence']):020d}.json"
        )
        if path.exists():
            existing = _read_document(path, label="tuning checkpoint")
            if existing != value:
                raise TuningError(
                    "tuning checkpoint revision is already bound to other state"
                )
            return
        _atomic_document(path, value, mode=0o440)

    def _write_state_and_selector(
        self, session_root: Path, state: Mapping[str, Any]
    ) -> None:
        self._validate_state(state)
        _atomic_document(session_root / "state.json", state)
        selector = {
            "schema": SELECTOR_SCHEMA,
            "session_id": state["session_id"],
            "state_id": state["state_id"],
            "target_id": self.target_id,
            "runtime_profile": self.runtime_profile,
            "release_id": self.release_id,
            "workspace_id": self.workspace_id,
            "manifest_id": self.manifest_id,
        }
        _atomic_document(self.selector_path, selector)

    @staticmethod
    def _result_from_entry(entry: Mapping[str, Any]) -> dict[str, Any] | None:
        if entry["kind"] not in {
            "committed",
            "recovered-commit",
            "aborted",
            "rejected",
            "divergent",
        }:
            return None
        result = entry["body"].get("result")
        return dict(result) if isinstance(result, dict) else None

    def _request_history(
        self, session_root: Path, request_id: str
    ) -> tuple[str | None, dict[str, Any] | None, str | None]:
        fingerprint: str | None = None
        result: dict[str, Any] | None = None
        transaction_id: str | None = None
        for entry in self._read_wal(session_root):
            if entry["request_id"] != request_id:
                continue
            body_fingerprint = entry["body"].get("request_fingerprint")
            if isinstance(body_fingerprint, str):
                fingerprint = body_fingerprint
            terminal = self._result_from_entry(entry)
            if terminal is not None:
                result = terminal
                transaction_id = entry["transaction_id"]
        return fingerprint, result, transaction_id

    def prepare(
        self,
        *,
        baseline_values: Mapping[str, Any],
        persisted_values: Mapping[str, Any],
        pending_boot_values: Mapping[str, Any],
        request_id: str,
        expected_revision: int,
        operator_id: str,
        edits: Sequence[Mapping[str, Any]],
        validate_candidate: Callable[[Mapping[str, Any]], None],
    ) -> TransactionPlan | dict[str, Any]:
        request_id = _require_identity(
            request_id, label="configuration request identity", allow_hash=True
        )
        operator_id = _require_identity(
            operator_id, label="configuration operator identity", allow_hash=True
        )
        if (
            isinstance(expected_revision, bool)
            or not isinstance(expected_revision, int)
            or expected_revision < 0
        ):
            raise TuningError("expected tuning revision is invalid")
        state, _baseline = self._ensure_session(
            baseline_values=baseline_values, persisted_values=persisted_values
        )
        session_root = self._session_root(state["session_id"])
        if state["divergent"]:
            raise TuningError(
                "configuration is divergent; reconcile exact observed values before further writes"
            )
        normalized_edits: list[dict[str, Any]] = []
        seen: set[str] = set()
        for item in edits:
            if set(item) != {"name", "node_id", "value", "restart_required"}:
                raise TuningError(
                    "configuration edit fields do not match the fixed contract"
                )
            name = item["name"]
            if (
                not isinstance(name, str)
                or not PARAMETER.fullmatch(name)
                or name in seen
            ):
                raise TuningError(
                    "configuration edits contain invalid or duplicate names"
                )
            node_id = _require_identity(
                item["node_id"], label="configuration node identity", allow_hash=False
            )
            restart = item["restart_required"]
            if restart not in RESTART_KINDS:
                raise TuningError("configuration edit has invalid restart semantics")
            _canonical(item["value"])
            seen.add(name)
            normalized_edits.append(
                {
                    "name": name,
                    "node_id": node_id,
                    "value": item["value"],
                    "restart_required": restart,
                }
            )
        if not normalized_edits:
            raise TuningError("configuration transaction has no edits")
        normalized_edits.sort(key=lambda item: (item["name"], item["node_id"]))
        pending = _values(dict(pending_boot_values), label="pending boot values")
        active = _values(dict(baseline_values), label="current active values")
        persisted = _values(dict(persisted_values), label="current persisted values")
        if active != state["active_values"] or persisted != state["persisted_values"]:
            raise TuningError(
                "runtime configuration differs from the durable tuning session; recovery is required"
            )
        if pending != state["pending_boot_values"]:
            raise TuningError(
                "runtime pending boot state differs from durable tuning state"
            )
        request_value = {
            "session_id": state["session_id"],
            "request_id": request_id,
            "expected_revision": expected_revision,
            "operator_id": operator_id,
            "edits": normalized_edits,
        }
        request_fingerprint = hashlib.sha256(_canonical(request_value)).hexdigest()
        old_fingerprint, old_result, old_transaction_id = self._request_history(
            session_root, request_id
        )
        if old_fingerprint is not None:
            if old_fingerprint != request_fingerprint:
                raise TuningError(
                    "configuration request identity was reused with other content"
                )
            if old_result is not None:
                return {
                    **old_result,
                    "idempotent_replay": True,
                    "transaction_id": old_transaction_id,
                }
            raise TuningError(
                "configuration request is already prepared and requires recovery"
            )
        if expected_revision != state["revision"]:
            transaction_id = hashlib.sha256(_canonical(request_value)).hexdigest()
            result = {
                "schema": RESULT_SCHEMA,
                "ok": False,
                "status": "rejected",
                "session_id": state["session_id"],
                "transaction_id": transaction_id,
                "request_id": request_id,
                "revision": state["revision"],
                "reason": "stale expected revision",
                "observed_values": {},
                "results": [
                    {
                        "node_id": edit["node_id"],
                        "name": edit["name"],
                        "success": False,
                        "message": "stale expected revision",
                        "applied_value": state["active_values"].get(edit["name"]),
                        "persisted_value": state["persisted_values"].get(edit["name"]),
                        "restart_required": edit["restart_required"],
                    }
                    for edit in normalized_edits
                ],
                "idempotent_replay": False,
            }
            entry = self._append(
                session_root,
                state,
                kind="rejected",
                transaction_id=transaction_id,
                request_id=request_id,
                revision=state["revision"],
                body={"request_fingerprint": request_fingerprint, "result": result},
            )
            self._advance_state(state, entry=entry, last_result=result)
            return result
        desired_active = dict(state["active_values"])
        desired_persisted = dict(state["persisted_values"])
        desired_pending = dict(state["pending_boot_values"])
        for edit in normalized_edits:
            desired_persisted[edit["name"]] = edit["value"]
            if edit["restart_required"] != "none":
                if state["active_values"].get(edit["name"]) == edit["value"]:
                    desired_pending.pop(edit["name"], None)
                else:
                    desired_pending[edit["name"]] = edit["value"]
            else:
                desired_active[edit["name"]] = edit["value"]
                desired_pending.pop(edit["name"], None)
        effective = dict(desired_active)
        effective.update(desired_pending)
        try:
            validate_candidate(effective)
        except Exception as exc:
            transaction_id = hashlib.sha256(_canonical(request_value)).hexdigest()
            result = {
                "schema": RESULT_SCHEMA,
                "ok": False,
                "status": "rejected",
                "session_id": state["session_id"],
                "transaction_id": transaction_id,
                "request_id": request_id,
                "revision": state["revision"],
                "reason": str(exc),
                "observed_values": {},
                "results": [
                    {
                        "node_id": edit["node_id"],
                        "name": edit["name"],
                        "success": False,
                        "message": str(exc),
                        "applied_value": state["active_values"].get(edit["name"]),
                        "persisted_value": state["persisted_values"].get(edit["name"]),
                        "restart_required": edit["restart_required"],
                    }
                    for edit in normalized_edits
                ],
                "idempotent_replay": False,
            }
            entry = self._append(
                session_root,
                state,
                kind="rejected",
                transaction_id=transaction_id,
                request_id=request_id,
                revision=state["revision"],
                body={"request_fingerprint": request_fingerprint, "result": result},
            )
            self._advance_state(state, entry=entry, last_result=result)
            return result
        plan_value = {
            **request_value,
            "revision_after": state["revision"] + 1,
            "previous_active_values": state["active_values"],
            "previous_persisted_values": state["persisted_values"],
            "previous_pending_boot_values": state["pending_boot_values"],
            "desired_active_values": dict(sorted(desired_active.items())),
            "desired_persisted_values": dict(sorted(desired_persisted.items())),
            "desired_pending_boot_values": dict(sorted(desired_pending.items())),
            "request_fingerprint": request_fingerprint,
        }
        transaction_id = hashlib.sha256(_canonical(plan_value)).hexdigest()
        plan = TransactionPlan(
            session_id=state["session_id"],
            transaction_id=transaction_id,
            request_id=request_id,
            expected_revision=expected_revision,
            revision_after=state["revision"] + 1,
            operator_id=operator_id,
            edits=tuple(normalized_edits),
            previous_active_values=dict(state["active_values"]),
            previous_persisted_values=dict(state["persisted_values"]),
            previous_pending_boot_values=dict(state["pending_boot_values"]),
            desired_active_values=dict(sorted(desired_active.items())),
            desired_persisted_values=dict(sorted(desired_persisted.items())),
            desired_pending_boot_values=dict(sorted(desired_pending.items())),
            request_fingerprint=request_fingerprint,
        )
        entry = self._append(
            session_root,
            state,
            kind="prepared",
            transaction_id=transaction_id,
            request_id=request_id,
            revision=state["revision"],
            body={
                "request_fingerprint": request_fingerprint,
                "plan": plan.as_dict(),
            },
        )
        self._advance_state(state, entry=entry, last_result=state["last_result"])
        return plan

    def _advance_state(
        self,
        state: Mapping[str, Any],
        *,
        entry: Mapping[str, Any],
        last_result: Mapping[str, Any] | None,
        revision: int | None = None,
        active_values: Mapping[str, Any] | None = None,
        persisted_values: Mapping[str, Any] | None = None,
        pending_boot_values: Mapping[str, Any] | None = None,
        divergent: bool | None = None,
        divergent_observations: Mapping[str, Any] | None = None,
        checkpoint: bool = False,
    ) -> dict[str, Any]:
        next_state = dict(state)
        next_state.update(
            {
                "state_id": "",
                "revision": state["revision"] if revision is None else revision,
                "wal_sequence": entry["sequence"],
                "wal_checksum": entry["checksum"],
                "active_values": dict(
                    state["active_values"] if active_values is None else active_values
                ),
                "persisted_values": dict(
                    state["persisted_values"]
                    if persisted_values is None
                    else persisted_values
                ),
                "pending_boot_values": dict(
                    state["pending_boot_values"]
                    if pending_boot_values is None
                    else pending_boot_values
                ),
                "divergent": state["divergent"] if divergent is None else divergent,
                "divergent_observations": dict(
                    state["divergent_observations"]
                    if divergent_observations is None
                    else divergent_observations
                ),
                "last_result": None if last_result is None else dict(last_result),
                "updated_at": entry["timestamp"],
            }
        )
        next_state["state_id"] = self._state_id(next_state)
        root = self._session_root(next_state["session_id"])
        if checkpoint:
            self._write_checkpoint(root, next_state)
        self._write_state_and_selector(root, next_state)
        return next_state

    def commit(
        self,
        plan: TransactionPlan,
        *,
        observed_values: Mapping[str, Any],
        persistence_reference: str,
        recovered: bool = False,
    ) -> dict[str, Any]:
        current = self._load_current()
        if current is None:
            raise TuningError("tuning session disappeared before commit")
        state, _baseline = current
        if (
            state["session_id"] != plan.session_id
            or state["revision"] != plan.expected_revision
        ):
            raise TuningError("tuning session changed before commit")
        observed = _values(dict(observed_values), label="transaction readback")
        for edit in plan.edits:
            if edit["restart_required"] != "none":
                continue
            if observed.get(edit["name"]) != edit["value"]:
                raise TuningError(
                    "transaction readback does not match the prepared values"
                )
        results = [
            {
                "node_id": edit["node_id"],
                "name": edit["name"],
                "success": True,
                "message": (
                    "persisted for next cold restart"
                    if edit["restart_required"] != "none"
                    else "applied, read back, and persisted"
                ),
                "applied_value": (
                    state["active_values"].get(edit["name"])
                    if edit["restart_required"] != "none"
                    else edit["value"]
                ),
                "persisted_value": edit["value"],
                "restart_required": edit["restart_required"],
            }
            for edit in plan.edits
        ]
        result = {
            "schema": RESULT_SCHEMA,
            "ok": True,
            "status": "recovered-commit" if recovered else "committed",
            "session_id": plan.session_id,
            "transaction_id": plan.transaction_id,
            "request_id": plan.request_id,
            "revision": plan.revision_after,
            "reason": None,
            "observed_values": observed,
            "results": results,
            "persistence_reference": persistence_reference,
            "idempotent_replay": False,
        }
        session_root = self._session_root(plan.session_id)
        entry = self._append(
            session_root,
            state,
            kind="recovered-commit" if recovered else "committed",
            transaction_id=plan.transaction_id,
            request_id=plan.request_id,
            revision=plan.revision_after,
            body={
                "request_fingerprint": plan.request_fingerprint,
                "plan": plan.as_dict(),
                "result": result,
            },
        )
        self._advance_state(
            state,
            entry=entry,
            last_result=result,
            revision=plan.revision_after,
            active_values=plan.desired_active_values,
            persisted_values=plan.desired_persisted_values,
            pending_boot_values=plan.desired_pending_boot_values,
            checkpoint=True,
        )
        return result

    def abort(
        self,
        plan: TransactionPlan,
        *,
        reason: str,
        observed_values: Mapping[str, Any],
        compensation_succeeded: bool,
    ) -> dict[str, Any]:
        current = self._load_current()
        if current is None:
            raise TuningError("tuning session disappeared before abort")
        state, _baseline = current
        observed = _values(
            dict(observed_values), label="transaction failure observations"
        )
        status = "aborted" if compensation_succeeded else "divergent"
        result = {
            "schema": RESULT_SCHEMA,
            "ok": False,
            "status": status,
            "session_id": plan.session_id,
            "transaction_id": plan.transaction_id,
            "request_id": plan.request_id,
            "revision": state["revision"],
            "reason": reason,
            "observed_values": observed,
            "results": [
                {
                    "node_id": edit["node_id"],
                    "name": edit["name"],
                    "success": False,
                    "message": reason,
                    "applied_value": observed.get(edit["name"]),
                    "persisted_value": state["persisted_values"].get(edit["name"]),
                    "restart_required": edit["restart_required"],
                }
                for edit in plan.edits
            ],
            "idempotent_replay": False,
        }
        session_root = self._session_root(plan.session_id)
        entry = self._append(
            session_root,
            state,
            kind=status,
            transaction_id=plan.transaction_id,
            request_id=plan.request_id,
            revision=state["revision"],
            body={
                "request_fingerprint": plan.request_fingerprint,
                "plan": plan.as_dict(),
                "result": result,
            },
        )
        self._advance_state(
            state,
            entry=entry,
            last_result=result,
            divergent=not compensation_succeeded,
            divergent_observations=observed if not compensation_succeeded else {},
            checkpoint=not compensation_succeeded,
        )
        return result

    @staticmethod
    def _plan_from_document(value: Mapping[str, Any]) -> TransactionPlan:
        _exact(
            value,
            {
                "schema",
                "session_id",
                "transaction_id",
                "request_id",
                "expected_revision",
                "revision_after",
                "operator_id",
                "edits",
                "previous_active_values",
                "previous_persisted_values",
                "previous_pending_boot_values",
                "desired_active_values",
                "desired_persisted_values",
                "desired_pending_boot_values",
                "request_fingerprint",
            },
            label="prepared transaction plan",
        )
        if value["schema"] != PLAN_SCHEMA or not isinstance(value["edits"], list):
            raise TuningError("prepared transaction plan is malformed")
        return TransactionPlan(
            session_id=str(value["session_id"]),
            transaction_id=str(value["transaction_id"]),
            request_id=str(value["request_id"]),
            expected_revision=int(value["expected_revision"]),
            revision_after=int(value["revision_after"]),
            operator_id=str(value["operator_id"]),
            edits=tuple(dict(item) for item in value["edits"]),
            previous_active_values=_values(
                value["previous_active_values"], label="previous active values"
            ),
            previous_persisted_values=_values(
                value["previous_persisted_values"],
                label="previous persisted values",
            ),
            previous_pending_boot_values=_values(
                value["previous_pending_boot_values"],
                label="previous pending boot values",
            ),
            desired_active_values=_values(
                value["desired_active_values"], label="desired active values"
            ),
            desired_persisted_values=_values(
                value["desired_persisted_values"],
                label="desired persisted values",
            ),
            desired_pending_boot_values=_values(
                value["desired_pending_boot_values"],
                label="desired pending boot values",
            ),
            request_fingerprint=str(value["request_fingerprint"]),
        )

    def recover_prepared(
        self,
        *,
        active_values: Mapping[str, Any],
        persisted_values: Mapping[str, Any],
        pending_boot_values: Mapping[str, Any],
        persistence_reference: str,
    ) -> dict[str, Any] | None:
        """Terminalize the one open prepared transaction from observed truth.

        Exact desired truth commits; exact pre-transaction truth aborts. Any mixed
        state becomes divergent and blocks subsequent writes.
        """
        current = self._load_current()
        if current is None:
            return None
        state, _baseline = current
        root = self._session_root(state["session_id"])
        entries = self._read_wal(root)
        terminal = {
            entry["transaction_id"]
            for entry in entries
            if entry["kind"]
            in {"committed", "recovered-commit", "aborted", "rejected", "divergent"}
        }
        prepared = [
            entry
            for entry in entries
            if entry["kind"] == "prepared" and entry["transaction_id"] not in terminal
        ]
        if not prepared:
            return None
        if len(prepared) != 1:
            raise TuningError("multiple unterminated tuning transactions exist")
        plan_value = prepared[0]["body"].get("plan")
        if not isinstance(plan_value, dict):
            raise TuningError("prepared tuning transaction lacks its bound plan")
        plan = self._plan_from_document(plan_value)
        active = _values(dict(active_values), label="recovery active values")
        persisted = _values(dict(persisted_values), label="recovery persisted values")
        pending = _values(dict(pending_boot_values), label="recovery pending values")
        if (
            active == plan.desired_active_values
            and persisted == plan.desired_persisted_values
            and pending == plan.desired_pending_boot_values
        ):
            result = self.commit(
                plan,
                observed_values=active,
                persistence_reference=persistence_reference,
                recovered=True,
            )
            return result
        if (
            active == plan.previous_active_values
            and persisted == plan.previous_persisted_values
            and pending == plan.previous_pending_boot_values
        ):
            return self.abort(
                plan,
                reason="interrupted transaction had not changed durable/runtime truth",
                observed_values=active,
                compensation_succeeded=True,
            )
        observations = dict(active)
        observations.update(
            {
                name: value
                for name, value in persisted.items()
                if active.get(name) != value
            }
        )
        return self.abort(
            plan,
            reason="interrupted transaction left mixed observed state",
            observed_values=observations,
            compensation_succeeded=False,
        )

    def confirm_pending_boot(
        self, *, observed_values: Mapping[str, Any]
    ) -> dict[str, Any]:
        current = self._load_current()
        if current is None:
            return {"confirmed": [], "revision": 0}
        state, _baseline = current
        pending = dict(state["pending_boot_values"])
        if not pending:
            return {"confirmed": [], "revision": state["revision"]}
        observed = _values(dict(observed_values), label="boot activation readback")
        mismatches = {
            name: {"expected": value, "observed": observed.get(name)}
            for name, value in pending.items()
            if observed.get(name) != value
        }
        if mismatches:
            raise TuningError(
                "fresh runtime readback does not match pending boot values: "
                + ", ".join(sorted(mismatches))
            )
        transaction_id = hashlib.sha256(
            _canonical(
                {
                    "session_id": state["session_id"],
                    "revision": state["revision"],
                    "pending": pending,
                    "observed": {name: observed[name] for name in sorted(pending)},
                }
            )
        ).hexdigest()
        result = {
            "confirmed": sorted(pending),
            "revision": state["revision"],
            "transaction_id": transaction_id,
        }
        session_root = self._session_root(state["session_id"])
        entry = self._append(
            session_root,
            state,
            kind="boot-confirmed",
            transaction_id=transaction_id,
            request_id="system-cold-restart",
            revision=state["revision"],
            body={"observed_values": observed, "result": result},
        )
        active = dict(state["active_values"])
        active.update(pending)
        self._advance_state(
            state,
            entry=entry,
            last_result=state["last_result"],
            active_values=active,
            pending_boot_values={},
            checkpoint=True,
        )
        return result

    def reconcile_divergence(
        self,
        *,
        observed_values: Mapping[str, Any],
        persisted_values: Mapping[str, Any],
        pending_boot_values: Mapping[str, Any],
        persistence_reference: str,
    ) -> dict[str, Any]:
        """Clear a fault only after exact prior durable state is re-established."""

        current = self._load_current()
        if current is None:
            raise TuningError("there is no tuning session to reconcile")
        state, _baseline = current
        if not state["divergent"]:
            return {
                "schema": "iii.configuration-divergence-reconciliation-result/v1",
                "reconciled": False,
                "session_id": state["session_id"],
                "revision": state["revision"],
                "reason": "configuration is not divergent",
            }
        observed = _values(
            dict(observed_values), label="divergence reconciliation readback"
        )
        persisted = _values(
            dict(persisted_values), label="divergence reconciliation persistence"
        )
        pending = _values(
            dict(pending_boot_values), label="divergence reconciliation pending state"
        )
        mismatches: list[str] = []
        if observed != state["active_values"]:
            mismatches.append("active readback")
        if persisted != state["persisted_values"]:
            mismatches.append("persisted values")
        if pending != state["pending_boot_values"]:
            mismatches.append("pending boot values")
        if mismatches:
            raise TuningError(
                "divergent configuration does not match its prior durable state: "
                + ", ".join(mismatches)
            )
        result = {
            "schema": "iii.configuration-divergence-reconciliation-result/v1",
            "reconciled": True,
            "session_id": state["session_id"],
            "revision": state["revision"],
            "observed_values": observed,
            "persistence_reference": persistence_reference,
        }
        transaction_id = hashlib.sha256(
            _canonical(
                {
                    "session_id": state["session_id"],
                    "revision": state["revision"],
                    "observed_values": observed,
                    "persisted_values": persisted,
                    "pending_boot_values": pending,
                    "persistence_reference": persistence_reference,
                }
            )
        ).hexdigest()
        root = self._session_root(state["session_id"])
        entry = self._append(
            root,
            state,
            kind="divergence-reconciled",
            transaction_id=transaction_id,
            request_id="system-divergence-reconciliation",
            revision=state["revision"],
            body={"result": result},
        )
        self._advance_state(
            state,
            entry=entry,
            last_result=result,
            divergent=False,
            divergent_observations={},
            checkpoint=True,
        )
        return result

    def restore_baseline(self, *, request_id: str, operator_id: str) -> TransactionPlan:
        current = self._load_current()
        if current is None:
            raise TuningError("there is no tuning session to restore")
        state, baseline = current
        names = sorted(set(state["persisted_values"]) | set(baseline["values"]))
        edits = [
            {
                "name": name,
                "node_id": "configuration",
                "value": baseline["values"].get(name),
                "restart_required": "none",
            }
            for name in names
            if name in baseline["values"]
            and state["persisted_values"].get(name) != baseline["values"][name]
        ]
        plan = self.prepare(
            baseline_values=state["active_values"],
            persisted_values=state["persisted_values"],
            pending_boot_values=state["pending_boot_values"],
            request_id=request_id,
            expected_revision=state["revision"],
            operator_id=operator_id,
            edits=edits,
            validate_candidate=lambda _value: None,
        )
        if not isinstance(plan, TransactionPlan):
            raise TuningError("baseline restore was unexpectedly terminal")
        return plan

    def compact(self) -> dict[str, Any]:
        """Verify and retain the complete current session and every checkpoint.

        Only closed-session archival may discard WAL history.  P4 deliberately has
        no operator-visible close operation, so active-session compaction is a
        validated no-op rather than silent evidence loss.
        """
        current = self._load_current()
        if current is None:
            return {"compacted": False, "reason": "no active session", "records": 0}
        state, _baseline = current
        root = self._session_root(state["session_id"])
        records = self._read_wal(root)
        checkpoints = sorted((root / "checkpoints").glob("*.json"))
        for path in checkpoints:
            _read_document(path, label="tuning checkpoint")
        return {
            "compacted": False,
            "reason": "current session history is retained in full",
            "records": len(records),
            "checkpoints": [path.name for path in checkpoints],
        }
