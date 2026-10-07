from __future__ import annotations

import json
from pathlib import Path

import pytest

from iii_drone_configuration.tuning import (
    TransactionPlan,
    TuningError,
    TuningSessionStore,
)

BASELINE = {"/control/gain": 1.0, "/control/mode": "auto", "/frame": "map"}


class Clock:
    def __init__(self) -> None:
        self.value = 0

    def __call__(self) -> str:
        self.value += 1
        return f"2026-08-27T12:00:{self.value:02d}Z"


def store(
    tmp_path: Path, clock: Clock | None = None, *, profile: str = "real"
) -> TuningSessionStore:
    return TuningSessionStore(
        root=tmp_path / "tuning",
        target_id="drone-1",
        runtime_profile=profile,
        release_id="a" * 64,
        workspace_id="workspace-test",
        manifest_id="b" * 64,
        now=clock or Clock(),
    )


def edits(*items: tuple[str, object, str]) -> list[dict]:
    return [
        {
            "node_id": "controller",
            "name": name,
            "value": value,
            "restart_required": restart,
        }
        for name, value, restart in items
    ]


def prepare(
    tuning: TuningSessionStore,
    *,
    request_id: str = "request-1",
    revision: int = 0,
    values: dict | None = None,
    persisted: dict | None = None,
    pending: dict | None = None,
    requested: list[dict] | None = None,
    validate=lambda _candidate: None,
) -> TransactionPlan | dict:
    return tuning.prepare(
        baseline_values=values or BASELINE,
        persisted_values=persisted or values or BASELINE,
        pending_boot_values=pending or {},
        request_id=request_id,
        expected_revision=revision,
        operator_id="operator-1",
        edits=requested or edits(("/control/gain", 2.0, "none")),
        validate_candidate=validate,
    )


def test_session_baseline_is_immutable_and_commit_is_revisioned_durable_and_replayable(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)
    plan = prepare(tuning)
    assert isinstance(plan, TransactionPlan)
    assert plan.expected_revision == 0 and plan.revision_after == 1
    result = tuning.commit(
        plan,
        observed_values={**BASELINE, "/control/gain": 2.0},
        persistence_reference="snapshots/runtime-1.yaml",
    )
    assert result["ok"] is True and result["revision"] == 1
    status = tuning.status()
    assert status["revision"] == 1 and status["divergent"] is False

    root = tmp_path / "tuning/sessions" / status["session_id"]
    baseline = json.loads((root / "baseline.json").read_text())
    state = json.loads((root / "state.json").read_text())
    assert baseline["values"] == BASELINE
    assert state["active_values"]["/control/gain"] == 2.0
    assert baseline["baseline_id"] != state["state_id"]
    assert len(list((root / "checkpoints").glob("*.json"))) == 2

    replay = prepare(
        tuning,
        request_id="request-1",
        revision=0,
        values=state["active_values"],
        persisted=state["persisted_values"],
    )
    assert isinstance(replay, dict)
    assert replay["idempotent_replay"] is True
    assert replay["revision"] == 1


def test_ros_scoped_configuration_node_identity_is_accepted(tmp_path: Path) -> None:
    tuning = store(tmp_path)
    requested = edits(("/control/gain", 2.0, "none"))
    requested[0]["node_id"] = "control/maneuver_controller/maneuver_controller"

    plan = prepare(tuning, requested=requested)

    assert isinstance(plan, TransactionPlan)
    assert plan.edits[0]["node_id"] == "control/maneuver_controller/maneuver_controller"


@pytest.mark.parametrize("node_id", ["/absolute/node", "control/../node", "control//node"])
def test_configuration_node_identity_rejects_path_syntax(
    tmp_path: Path, node_id: str
) -> None:
    tuning = store(tmp_path)
    requested = edits(("/control/gain", 2.0, "none"))
    requested[0]["node_id"] = node_id

    with pytest.raises(TuningError, match="configuration node identity is malformed"):
        prepare(tuning, requested=requested)


def test_validate_all_rejection_and_stale_revision_never_mutate_values(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)

    def reject(candidate):
        assert candidate["/control/gain"] == 3.0
        raise ValueError("gain and mode combination is invalid")

    rejected = prepare(
        tuning,
        requested=edits(
            ("/control/gain", 3.0, "none"),
            ("/control/mode", "manual", "none"),
        ),
        validate=reject,
    )
    assert isinstance(rejected, dict)
    assert rejected["status"] == "rejected"
    assert tuning.status()["revision"] == 0

    stale = prepare(
        tuning,
        request_id="request-stale",
        revision=4,
    )
    assert isinstance(stale, dict)
    assert stale["reason"] == "stale expected revision"
    assert tuning.status()["revision"] == 0


def test_runtime_restart_values_remain_pending_until_fresh_matching_readback(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)
    plan = prepare(
        tuning,
        requested=edits(("/frame", "odom", "runtime")),
    )
    assert isinstance(plan, TransactionPlan)
    assert plan.desired_active_values["/frame"] == "map"
    assert plan.desired_persisted_values["/frame"] == "odom"
    result = tuning.commit(
        plan,
        observed_values=BASELINE,
        persistence_reference="snapshots/runtime-boot.yaml",
    )
    assert result["results"][0]["applied_value"] == "map"
    assert tuning.status()["pending_boot_values"] == {"/frame": "odom"}

    with pytest.raises(TuningError, match="fresh runtime readback"):
        tuning.confirm_pending_boot(observed_values=BASELINE)
    assert tuning.status()["pending_boot_values"] == {"/frame": "odom"}

    confirmation = tuning.confirm_pending_boot(
        observed_values={**BASELINE, "/frame": "odom"}
    )
    assert confirmation["confirmed"] == ["/frame"]
    assert tuning.status()["pending_boot_values"] == {}


def test_compensated_failure_aborts_but_failed_compensation_faults_and_blocks(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)
    first = prepare(tuning)
    assert isinstance(first, TransactionPlan)
    aborted = tuning.abort(
        first,
        reason="second node rejected update",
        observed_values=BASELINE,
        compensation_succeeded=True,
    )
    assert aborted["status"] == "aborted"
    assert tuning.status()["divergent"] is False

    second = prepare(tuning, request_id="request-2")
    assert isinstance(second, TransactionPlan)
    divergent = tuning.abort(
        second,
        reason="rollback readback differed",
        observed_values={**BASELINE, "/control/gain": 1.5},
        compensation_succeeded=False,
    )
    assert divergent["status"] == "divergent"
    assert tuning.status()["divergent_observations"]["/control/gain"] == 1.5
    with pytest.raises(TuningError, match="configuration is divergent"):
        prepare(tuning, request_id="request-3")

    with pytest.raises(TuningError, match="prior durable state"):
        tuning.reconcile_divergence(
            observed_values={**BASELINE, "/control/gain": 1.5},
            persisted_values=BASELINE,
            pending_boot_values={},
            persistence_reference="tracked/default.yaml",
        )
    reconciled = tuning.reconcile_divergence(
        observed_values=BASELINE,
        persisted_values=BASELINE,
        pending_boot_values={},
        persistence_reference="tracked/default.yaml",
    )
    assert reconciled["reconciled"] is True
    assert tuning.status()["divergent"] is False
    assert isinstance(prepare(tuning, request_id="request-3"), TransactionPlan)


def test_prepared_power_loss_recovers_exact_before_after_or_mixed_truth(
    tmp_path: Path,
) -> None:
    before_root = tmp_path / "before"
    before = store(before_root)
    before_plan = prepare(before)
    assert isinstance(before_plan, TransactionPlan)
    recovered_before = store(before_root).recover_prepared(
        active_values=BASELINE,
        persisted_values=BASELINE,
        pending_boot_values={},
        persistence_reference="tracked/default.yaml",
    )
    assert recovered_before and recovered_before["status"] == "aborted"
    assert store(before_root).status()["revision"] == 0

    after_root = tmp_path / "after"
    after = store(after_root)
    after_plan = prepare(after)
    assert isinstance(after_plan, TransactionPlan)
    desired = dict(after_plan.desired_active_values)
    recovered_after = store(after_root).recover_prepared(
        active_values=desired,
        persisted_values=after_plan.desired_persisted_values,
        pending_boot_values=after_plan.desired_pending_boot_values,
        persistence_reference="snapshots/recovered.yaml",
    )
    assert recovered_after and recovered_after["status"] == "recovered-commit"
    assert store(after_root).status()["revision"] == 1

    mixed_root = tmp_path / "mixed"
    mixed = store(mixed_root)
    mixed_plan = prepare(mixed)
    assert isinstance(mixed_plan, TransactionPlan)
    recovered_mixed = store(mixed_root).recover_prepared(
        active_values={**BASELINE, "/control/gain": 1.5},
        persisted_values=BASELINE,
        pending_boot_values={},
        persistence_reference="tracked/default.yaml",
    )
    assert recovered_mixed and recovered_mixed["status"] == "divergent"
    assert store(mixed_root).status()["divergent"] is True


def test_wal_tamper_partial_record_cross_identity_and_compaction_fail_closed(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)
    plan = prepare(tuning)
    assert isinstance(plan, TransactionPlan)
    tuning.abort(
        plan,
        reason="test abort",
        observed_values=BASELINE,
        compensation_succeeded=True,
    )
    status = tuning.status()
    root = tmp_path / "tuning/sessions" / status["session_id"]
    compacted = tuning.compact()
    assert compacted["compacted"] is False
    assert compacted["records"] == 2
    assert compacted["checkpoints"]

    wal = root / "journal.jsonl"
    original = wal.read_bytes()
    wal.write_bytes(original + b'{"partial":')
    with pytest.raises(TuningError, match="partial record"):
        tuning.status()
    wal.write_bytes(original)

    selector = tmp_path / "tuning/selectors/drone-1--real.json"
    value = json.loads(selector.read_text())
    value["target_id"] = "other-drone"
    selector.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n")
    with pytest.raises(TuningError, match="another runtime identity"):
        tuning.status()


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_same_transaction_conformance_and_baseline_restore_for_both_profiles(
    tmp_path: Path, profile: str
) -> None:
    tuning = store(tmp_path, profile=profile)
    first = prepare(tuning)
    assert isinstance(first, TransactionPlan)
    changed = {**BASELINE, "/control/gain": 2.0}
    tuning.commit(
        first,
        observed_values=changed,
        persistence_reference="snapshots/changed.yaml",
    )

    restore = tuning.restore_baseline(
        request_id="restore-baseline", operator_id="test-runner"
    )
    assert restore.previous_persisted_values["/control/gain"] == 2.0
    assert restore.desired_persisted_values["/control/gain"] == 1.0
    result = tuning.commit(
        restore,
        observed_values=BASELINE,
        persistence_reference="tracked/default.yaml",
    )
    assert result["revision"] == 2
    assert tuning.status()["active_values"] == BASELINE


def test_fsynced_commit_wal_rebuilds_state_after_atomic_state_write_interruption(
    tmp_path: Path, monkeypatch
) -> None:
    tuning = store(tmp_path)
    plan = prepare(tuning)
    assert isinstance(plan, TransactionPlan)

    def interrupt_state_write(*_args, **_kwargs):
        raise OSError("simulated power loss after WAL fsync")

    monkeypatch.setattr(tuning, "_write_state_and_selector", interrupt_state_write)
    with pytest.raises(OSError, match="simulated power loss"):
        tuning.commit(
            plan,
            observed_values={**BASELINE, "/control/gain": 2.0},
            persistence_reference="snapshots/interrupted.yaml",
        )

    recovered = store(tmp_path).status()
    assert recovered["revision"] == 1
    assert recovered["active_values"]["/control/gain"] == 2.0
    assert recovered["last_result"]["status"] == "committed"


def test_fsynced_divergence_reconciliation_replays_after_state_write_interruption(
    tmp_path: Path, monkeypatch
) -> None:
    tuning = store(tmp_path)
    plan = prepare(tuning)
    assert isinstance(plan, TransactionPlan)
    tuning.abort(
        plan,
        reason="rollback readback differed",
        observed_values={**BASELINE, "/control/gain": 1.5},
        compensation_succeeded=False,
    )

    def interrupt_state_write(*_args, **_kwargs):
        raise OSError("simulated power loss after reconciliation WAL fsync")

    monkeypatch.setattr(tuning, "_write_state_and_selector", interrupt_state_write)
    with pytest.raises(OSError, match="reconciliation WAL fsync"):
        tuning.reconcile_divergence(
            observed_values=BASELINE,
            persisted_values=BASELINE,
            pending_boot_values={},
            persistence_reference="tracked/default.yaml",
        )

    recovered = store(tmp_path).status()
    assert recovered["divergent"] is False
    assert recovered["revision"] == 0
    assert recovered["last_result"]["reconciled"] is True


def test_capture_can_open_the_implicit_session_without_inventing_a_transaction(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)

    opened = tuning.ensure_session(
        baseline_values=BASELINE,
        persisted_values=BASELINE,
    )
    repeated = tuning.ensure_session(
        baseline_values=BASELINE,
        persisted_values=BASELINE,
    )

    assert opened == repeated
    assert opened["session_id"] and opened["baseline_id"]
    assert opened["revision"] == 0
    assert opened["wal_sequence"] == 0
    assert list((tmp_path / "tuning/sessions").iterdir()) == [
        tmp_path / "tuning/sessions" / opened["session_id"]
    ]


def test_journal_batches_are_cursor_bound_and_retained_across_release_turnover(
    tmp_path: Path,
) -> None:
    tuning = store(tmp_path)
    plan = prepare(tuning)
    assert isinstance(plan, TransactionPlan)
    tuning.commit(
        plan,
        observed_values={**BASELINE, "/control/gain": 2.0},
        persistence_reference="snapshots/runtime.yaml",
    )
    old_status = tuning.status()
    first = tuning.journal_batch(
        session_id=old_status["session_id"], after_sequence=0, limit=1
    )
    second = tuning.journal_batch(
        session_id=old_status["session_id"], after_sequence=1, limit=10
    )
    assert first["through_sequence"] == 1 and first["complete"] is False
    assert second["through_sequence"] == 2 and second["complete"] is True
    assert second["entries"][0]["previous_checksum"] == first["entries"][0]["checksum"]

    next_release = TuningSessionStore(
        root=tmp_path / "tuning",
        target_id="drone-1",
        runtime_profile="real",
        release_id="c" * 64,
        workspace_id="workspace-next",
        manifest_id="d" * 64,
        now=Clock(),
    )
    assert next_release.status()["session_id"] is None
    retained = next_release.journal_batch(
        session_id=old_status["session_id"], after_sequence=0, limit=10
    )
    assert retained["complete"] is True
    assert retained["session"]["release_id"] == "a" * 64
    assert retained["session"]["workspace_id"] == "workspace-test"
    assert retained["session"]["manifest_id"] == "b" * 64

    with pytest.raises(TuningError, match="beyond the authoritative head"):
        next_release.journal_batch(
            session_id=old_status["session_id"], after_sequence=3, limit=10
        )


def commit_many(tuning: TuningSessionStore, count: int, *, start: int = 0) -> dict:
    """``count`` committed single-parameter updates starting at revision ``start``."""
    values = dict(tuning.status()["active_values"] or BASELINE)
    for index in range(start, start + count):
        gain = float(index + 2)
        plan = prepare(
            tuning,
            request_id=f"request-{index}",
            revision=index,
            values=values,
            requested=edits(("/control/gain", gain, "none")),
        )
        assert isinstance(plan, TransactionPlan)
        values = {**values, "/control/gain": gain}
        result = tuning.commit(
            plan,
            observed_values=values,
            persistence_reference=f"snapshots/runtime-{index}.yaml",
        )
        assert result["ok"] is True and result["revision"] == index + 1
    return values


def session_root(tmp_path: Path, tuning: TuningSessionStore) -> Path:
    return tmp_path / "tuning/sessions" / tuning.status()["session_id"]


def complete_wal_bytes(root: Path) -> bytes:
    import lzma

    sealed = b"".join(
        lzma.decompress(path.read_bytes())
        for path in sorted((root / "journal-segments").glob("*.jsonl.xz"))
    )
    return sealed + (root / "journal.jsonl").read_bytes()


def test_wal_is_sealed_into_lossless_segments_and_the_active_file_stays_bounded(
    tmp_path: Path, monkeypatch
) -> None:
    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    segmented = store(tmp_path / "segmented")
    commit_many(segmented, 21)
    root = session_root(tmp_path / "segmented", segmented)

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 10**9)
    monkeypatch.setattr(module, "WAL_SEGMENT_BYTES", 10**12)
    plain = store(tmp_path / "plain")
    commit_many(plain, 21)
    plain_root = session_root(tmp_path / "plain", plain)
    assert not (plain_root / "journal-segments").exists()

    # the same 42 records, byte for byte, in both layouts
    reference = (plain_root / "journal.jsonl").read_bytes()
    assert complete_wal_bytes(root) == reference
    segments = sorted(path.name for path in (root / "journal-segments").glob("*.jsonl.xz"))
    assert [name[:41] for name in segments] == [
        f"{first:020d}-{first + 7:020d}" for first in (1, 9, 17, 25, 33)
    ]
    assert len((root / "journal.jsonl").read_bytes().splitlines()) == 2
    assert segmented.status() == {**plain.status()}

    # a restarted store authenticates the sealed head from the segment names
    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    reopened = store(tmp_path / "segmented")
    assert reopened.status()["revision"] == 21
    assert reopened.status()["wal_sequence"] == 42
    compacted = reopened.compact()
    assert compacted["records"] == 42 and compacted["sealed_records"] == 40
    assert compacted["sealed_segments"] == segments


def test_updates_do_not_read_sealed_history(tmp_path: Path, monkeypatch) -> None:
    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    tuning = store(tmp_path)
    commit_many(tuning, 20)

    def refuse(_path):
        raise AssertionError("a sealed segment was read on the update path")

    parsed: list[int] = []
    original = TuningSessionStore._parse_wal

    def counting(data, **kwargs):
        parsed.append(len(data.splitlines()))
        return original(data, **kwargs)

    monkeypatch.setattr(TuningSessionStore, "_segment_bytes", staticmethod(refuse))
    monkeypatch.setattr(TuningSessionStore, "_parse_wal", staticmethod(counting))
    commit_many(tuning, 3, start=20)
    assert tuning.status()["revision"] == 23
    # only newly appended records are parsed, never the whole active file again
    assert parsed and max(parsed) <= 2
    # a restarted store needs the active file only
    assert store(tmp_path).status()["revision"] == 23


def test_journal_batches_and_idempotent_replay_reach_into_sealed_segments(
    tmp_path: Path, monkeypatch
) -> None:
    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    tuning = store(tmp_path)
    values = commit_many(tuning, 15)
    root = session_root(tmp_path, tuning)
    records = [json.loads(line) for line in complete_wal_bytes(root).splitlines()]
    assert len(records) == 30

    collected: list[dict] = []
    cursor = 0
    while True:
        batch = tuning.journal_batch(session_id=None, after_sequence=cursor, limit=7)
        collected.extend(batch["entries"])
        cursor = batch["through_sequence"]
        assert batch["head_sequence"] == 30
        assert batch["head_checksum"] == records[-1]["checksum"]
        if batch["complete"]:
            break
    assert collected == records
    with pytest.raises(TuningError, match="beyond the authoritative head"):
        tuning.journal_batch(session_id=None, after_sequence=31, limit=1)

    # request-2 was committed in the first sealed segment
    same = edits(("/control/gain", 4.0, "none"))
    replay = prepare(
        store(tmp_path), request_id="request-2", revision=2, values=values, requested=same
    )
    assert isinstance(replay, dict)
    assert replay["idempotent_replay"] is True and replay["revision"] == 3
    with pytest.raises(TuningError, match="reused with other content"):
        prepare(
            store(tmp_path),
            request_id="request-2",
            revision=2,
            values=values,
            requested=edits(("/control/mode", "manual", "none")),
        )

    # the request index is derived: without it the segment is consulted directly
    for path in (root / "journal-segments").glob("*.requests.json"):
        path.unlink()
    replay = prepare(
        store(tmp_path), request_id="request-2", revision=2, values=values, requested=same
    )
    assert isinstance(replay, dict) and replay["idempotent_replay"] is True


def test_interrupted_rotation_is_repaired_without_loss(tmp_path: Path, monkeypatch) -> None:
    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    tuning = store(tmp_path)
    commit_many(tuning, 3)
    root = session_root(tmp_path, tuning)
    real_atomic = module._atomic_bytes

    def lose_power_before_the_cut(path, data, **kwargs):
        if path.name == "journal.jsonl":
            raise OSError("simulated power loss after sealing")
        return real_atomic(path, data, **kwargs)

    monkeypatch.setattr(module, "_atomic_bytes", lose_power_before_the_cut)
    with pytest.raises(OSError, match="simulated power loss"):
        commit_many(tuning, 1, start=3)
    monkeypatch.setattr(module, "_atomic_bytes", real_atomic)
    # sealed and still present in the active file
    assert len(list((root / "journal-segments").glob("*.jsonl.xz"))) == 1
    before = (root / "journal.jsonl").read_bytes()
    assert len(before.splitlines()) == 8

    recovered = store(tmp_path)
    assert recovered.status()["revision"] == 4
    assert (root / "journal.jsonl").read_bytes() == b""
    assert complete_wal_bytes(root) == before
    commit_many(recovered, 2, start=4)
    assert recovered.status()["wal_sequence"] == 12
    assert len(recovered._read_wal(root)) == 12


def test_sealed_segment_tamper_and_gaps_fail_closed(tmp_path: Path, monkeypatch) -> None:
    import lzma

    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    tuning = store(tmp_path)
    commit_many(tuning, 9)
    root = session_root(tmp_path, tuning)
    first, second = sorted((root / "journal-segments").glob("*.jsonl.xz"))

    original = first.read_bytes()
    first.chmod(0o640)
    first.write_bytes(lzma.compress(lzma.decompress(original).replace(b"2.0", b"9.0", 1)))
    with pytest.raises(TuningError):
        store(tmp_path).compact()
    with pytest.raises(TuningError):
        store(tmp_path).journal_batch(session_id=None, after_sequence=0, limit=4)
    first.write_bytes(original)
    assert store(tmp_path).compact()["records"] == 18

    # the state is anchored in the newest segment's identity
    renamed = second.with_name(second.name.replace(second.name[42:106], "0" * 64))
    second.rename(renamed)
    with pytest.raises(TuningError, match="not anchored|differ|chain is invalid"):
        store(tmp_path).status()
    renamed.rename(second)

    hidden = first.with_suffix(".hidden")
    first.rename(hidden)
    with pytest.raises(TuningError, match="not contiguous"):
        store(tmp_path).status()
    hidden.rename(first)
    assert store(tmp_path).status()["revision"] == 9


def test_open_prepared_transaction_is_never_sealed_and_recovers(
    tmp_path: Path, monkeypatch
) -> None:
    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 4)
    tuning = store(tmp_path)
    values = commit_many(tuning, 1)
    root = session_root(tmp_path, tuning)
    plan = prepare(
        tuning,
        request_id="request-open",
        revision=1,
        values=values,
        requested=edits(("/control/gain", 7.0, "none")),
    )
    assert isinstance(plan, TransactionPlan)
    rejected = prepare(tuning, request_id="request-stale", revision=0, values=values)
    assert isinstance(rejected, dict) and rejected["ok"] is False
    # four records, but one transaction is still open
    assert len((root / "journal.jsonl").read_bytes().splitlines()) == 4
    assert not list((root / "journal-segments").glob("*.jsonl.xz"))

    recovered = store(tmp_path).recover_prepared(
        active_values={**values, "/control/gain": 7.0},
        persisted_values={**values, "/control/gain": 7.0},
        pending_boot_values={},
        persistence_reference="snapshots/recovered.yaml",
    )
    assert recovered is not None and recovered["status"] == "recovered-commit"
    assert (root / "journal.jsonl").read_bytes() == b""
    assert len(list((root / "journal-segments").glob("*.jsonl.xz"))) == 2
    assert store(tmp_path).status()["revision"] == 2


def test_existing_unsegmented_journal_is_migrated_in_place(tmp_path: Path, monkeypatch) -> None:
    import iii_drone_configuration.tuning as module

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 10**9)
    monkeypatch.setattr(module, "WAL_SEGMENT_BYTES", 10**12)
    legacy = store(tmp_path)
    commit_many(legacy, 10)
    root = session_root(tmp_path, legacy)
    before = (root / "journal.jsonl").read_bytes()

    monkeypatch.setattr(module, "WAL_SEGMENT_RECORDS", 8)
    upgraded = store(tmp_path)
    assert upgraded.status()["revision"] == 10
    commit_many(upgraded, 1, start=10)
    names = sorted(path.name[:41] for path in (root / "journal-segments").glob("*.jsonl.xz"))
    assert names == [f"{a:020d}-{b:020d}" for a, b in ((1, 8), (9, 16), (17, 22))]
    assert (root / "journal.jsonl").read_bytes() == b""
    assert complete_wal_bytes(root).startswith(before)
    assert len(complete_wal_bytes(root).splitlines()) == 22
