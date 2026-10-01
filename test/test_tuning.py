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
