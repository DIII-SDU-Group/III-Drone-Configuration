from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import stat

import pytest
import yaml

from iii_drone_configuration import (
    ReconciliationError,
    execute_reconciliation,
    materialize_receiver_stage,
    plan_simulation_reconciliation,
    plan_reconciliation,
    resolve_installed_contract_root,
    reconcile_simulation_startup,
    seal_configuration_checkpoint,
    verify_configuration_checkpoint,
    validate_reintroduction_decisions,
    write_reintroduction_decisions,
    write_reintroduction_review,
)


def _canonical(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def _rewrite_manifest(root: Path) -> None:
    path = root / "package-manifest.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    for row in value["artifacts"]:
        row["sha256"] = hashlib.sha256((root / row["path"]).read_bytes()).hexdigest()
    for row in value["tracked_sets"]:
        row["sha256"] = hashlib.sha256((root / row["path"]).read_bytes()).hexdigest()
    value["manifest_id"] = hashlib.sha256(
        _canonical({key: item for key, item in value.items() if key != "manifest_id"})
    ).hexdigest()
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _contract(
    tmp_path: Path,
    name: str,
    *,
    extra_default: float | None = None,
    maximum: float = 10.0,
) -> Path:
    root = tmp_path / name
    shutil.copytree(resolve_installed_contract_root(), root, symlinks=False)
    schema_path = root / "schema/parameter_manifest.yaml"
    schema = yaml.safe_load(schema_path.read_text(encoding="utf-8"))
    if extra_default is not None:
        schema["reconciliation_fixture"] = {
            "retired_gain": {
                "type": "float",
                "value": extra_default,
                "min": 0.0,
                "max": maximum,
            }
        }
    schema_path.write_text(yaml.safe_dump(schema, sort_keys=False), encoding="utf-8")
    for profile in ("real", "sim"):
        default_path = root / f"tracked_defaults/{profile}/default.yaml"
        value = yaml.safe_load(default_path.read_text(encoding="utf-8"))
        parameters = value["/**"]["ros__parameters"]
        if extra_default is None:
            parameters.pop("/reconciliation_fixture/retired_gain", None)
        else:
            parameters["/reconciliation_fixture/retired_gain"] = extra_default
        default_path.write_text(
            yaml.safe_dump(value, sort_keys=False), encoding="utf-8"
        )
    _rewrite_manifest(root)
    return root


def _manifest(root: Path) -> str:
    return json.loads((root / "package-manifest.json").read_text())["manifest_id"]


def _plan(
    *,
    old: Path,
    new: Path,
    state: Path,
    operations: Path,
    operation: str,
    old_release: str,
    new_release: str,
    mode: str = "simulation",
    purpose: str = "activation",
):
    return plan_reconciliation(
        old_immutable_root=old,
        new_immutable_root=new,
        writable_state_root=state,
        operations_root=operations,
        operation_id=operation,
        runtime_profile="sim" if mode == "simulation" else "real",
        target_id="sim" if mode == "simulation" else "aircraft-01",
        old_release_id=old_release,
        new_release_id=new_release,
        mode=mode,
        purpose=purpose,
    )


def _initial_state(contract: Path, root: Path, *, profile: str = "sim") -> None:
    plan = plan_reconciliation(
        old_immutable_root=contract,
        new_immutable_root=contract,
        writable_state_root=root,
        operations_root=root.parent / "operations",
        operation_id=f"initial-{profile}-0001",
        runtime_profile=profile,
        target_id="sim" if profile == "sim" else "aircraft-01",
        old_release_id="release-old",
        new_release_id="release-old",
        mode="simulation" if profile == "sim" else "receiver-staged",
        purpose="startup" if profile == "sim" else "activation",
    )
    if profile == "real":
        (root / ".iii-reconciliation-stage.json").parent.mkdir(
            parents=True, exist_ok=True
        )
        (root / ".iii-reconciliation-stage.json").write_text(
            json.dumps(
                {
                    "schema": "iii.configuration-reconciliation-stage/v1",
                    "operation_id": plan.operation_id,
                    "target_id": "aircraft-01",
                }
            )
        )
    assert execute_reconciliation(plan).status == "complete"


def _set_value(path: Path, name: str, value: object) -> None:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["/**"]["ros__parameters"][name] = value
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _select(root: Path, reference: str) -> None:
    path = root / "profiles/sim.yaml"
    path.write_text(
        yaml.safe_dump(
            {"version": 1, "active_parameter_set": reference}, sort_keys=False
        ),
        encoding="utf-8",
    )


def test_preserves_adds_retires_all_sets_and_requires_bound_review(tmp_path: Path):
    old = _contract(tmp_path, "old", extra_default=2.0)
    removed = _contract(tmp_path, "removed")
    reintroduced = _contract(tmp_path, "reintroduced", extra_default=3.0)
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    _initial_state(old, state)

    tracked = state / "parameter_sets/sim/tracked/default.yaml"
    snapshot = state / "parameter_sets/sim/snapshots/alternate.yaml"
    snapshot.parent.mkdir(parents=True)
    shutil.copyfile(tracked, snapshot)
    _set_value(tracked, "/reconciliation_fixture/retired_gain", 8.0)
    _set_value(snapshot, "/reconciliation_fixture/retired_gain", 7.0)
    _set_value(snapshot, "/control/dt", 0.3)
    _select(state, "snapshots/alternate.yaml")

    retire = _plan(
        old=old,
        new=removed,
        state=state,
        operations=operations,
        operation="retire-parameter-0001",
        old_release="release-old",
        new_release="release-removed",
    )
    assert len(retire.sets) == 2
    assert all(
        "/reconciliation_fixture/retired_gain" in item.removed for item in retire.sets
    )
    assert execute_reconciliation(retire).status == "complete"
    assert (
        "/reconciliation_fixture/retired_gain"
        not in yaml.safe_load(snapshot.read_text())["/**"]["ros__parameters"]
    )
    shadows = [
        json.loads(path.read_text()) for path in state.glob("shadows/sim/**/*.json")
    ]
    selected_shadow = next(
        item for item in shadows if item["set_reference"] == "snapshots/alternate.yaml"
    )
    inactive_shadow = next(
        item for item in shadows if item["set_reference"] == "tracked/default.yaml"
    )
    key = "/reconciliation_fixture/retired_gain"
    assert selected_shadow["entries"][key]["value"] == 7.0
    assert selected_shadow["entries"][key]["restorable"] is True
    assert inactive_shadow["entries"][key]["value"] == 8.0
    assert inactive_shadow["entries"][key]["restorable"] is False

    review_plan = plan_simulation_reconciliation(
        immutable_root=reintroduced,
        writable_state_root=state,
        operations_root=operations,
        runtime_profile="sim",
        target_id="sim",
        release_id="release-reintroduced",
    )
    assert review_plan.review_required is True
    direct_decisions = {
        "snapshots/alternate.yaml:/reconciliation_fixture/retired_gain": "use_old",
        "tracked/default.yaml:/reconciliation_fixture/retired_gain": "use_new_default",
    }
    assert validate_reintroduction_decisions(review_plan, direct_decisions) == dict(
        sorted(direct_decisions.items())
    )
    assert not (operations / review_plan.operation_id).exists()
    blocked = execute_reconciliation(review_plan)
    assert blocked.status == "review-required"
    assert (
        yaml.safe_load(snapshot.read_text())["/**"]["ros__parameters"].get(key) is None
    )
    review = write_reintroduction_review(review_plan)
    decisions = write_reintroduction_decisions(
        review,
        direct_decisions,
    )
    completed = reconcile_simulation_startup(
        immutable_root=reintroduced,
        writable_state_root=state,
        operations_root=operations,
        runtime_profile="sim",
        target_id="sim",
        release_id="release-reintroduced",
    )
    assert completed.status == "complete"
    assert yaml.safe_load(snapshot.read_text())["/**"]["ros__parameters"][key] == 7.0
    assert yaml.safe_load(tracked.read_text())["/**"]["ros__parameters"][key] == 3.0
    assert (
        execute_reconciliation(review_plan, decisions_path=decisions).state_id
        == completed.state_id
    )


def test_invalid_legacy_value_is_visible_but_not_selectable(tmp_path: Path):
    old = _contract(tmp_path, "old", extra_default=20.0, maximum=200.0)
    removed = _contract(tmp_path, "removed")
    narrow = _contract(tmp_path, "narrow", extra_default=2.0, maximum=10.0)
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    _initial_state(old, state)
    plan = _plan(
        old=old,
        new=removed,
        state=state,
        operations=operations,
        operation="retire-invalid-0001",
        old_release="release-old",
        new_release="release-removed",
    )
    execute_reconciliation(plan)
    review_plan = _plan(
        old=removed,
        new=narrow,
        state=state,
        operations=operations,
        operation="review-invalid-0001",
        old_release="release-removed",
        new_release="release-narrow",
    )
    item = review_plan.review_items[0]
    assert item["old_canonical_value"] == 20.0
    assert item["old_value_valid"] is False
    assert item["old_value_selectable"] is False
    review = write_reintroduction_review(review_plan)
    with pytest.raises(ReconciliationError, match="cannot be selected"):
        write_reintroduction_decisions(
            review,
            {"tracked/default.yaml:/reconciliation_fixture/retired_gain": "use_old"},
        )


def test_compatible_rollback_rehydrates_only_canonical_active_value(tmp_path: Path):
    old = _contract(tmp_path, "old", extra_default=2.0)
    removed = _contract(tmp_path, "removed")
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    _initial_state(old, state)
    tracked = state / "parameter_sets/sim/tracked/default.yaml"
    key = "/reconciliation_fixture/retired_gain"
    _set_value(tracked, key, 7.0)
    execute_reconciliation(
        _plan(
            old=old,
            new=removed,
            state=state,
            operations=operations,
            operation="retire-for-rollback-0001",
            old_release="release-old",
            new_release="release-removed",
        )
    )
    rollback = _plan(
        old=removed,
        new=old,
        state=state,
        operations=operations,
        operation="rollback-rehydrate-0001",
        old_release="release-removed",
        new_release="release-old",
        purpose="rollback",
    )
    assert rollback.review_required is False
    assert execute_reconciliation(rollback).status == "complete"
    assert yaml.safe_load(tracked.read_text())["/**"]["ros__parameters"][key] == 7.0


def test_shadow_corruption_blocks_planning_without_active_state_mutation(
    tmp_path: Path,
):
    old = _contract(tmp_path, "old", extra_default=2.0)
    removed = _contract(tmp_path, "removed")
    reintroduced = _contract(tmp_path, "reintroduced", extra_default=3.0)
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    _initial_state(old, state)
    execute_reconciliation(
        _plan(
            old=old,
            new=removed,
            state=state,
            operations=operations,
            operation="retire-before-corrupt-0001",
            old_release="release-old",
            new_release="release-removed",
        )
    )
    tracked = state / "parameter_sets/sim/tracked/default.yaml"
    before = tracked.read_bytes()
    shadow = next(state.glob("shadows/sim/**/*.json"))
    value = json.loads(shadow.read_text())
    value["entries"]["/reconciliation_fixture/retired_gain"]["value"] = 9.0
    shadow.write_text(json.dumps(value))
    with pytest.raises(ReconciliationError, match="shadow identity"):
        _plan(
            old=removed,
            new=reintroduced,
            state=state,
            operations=operations,
            operation="reject-corrupt-shadow-0001",
            old_release="release-removed",
            new_release="release-reintroduced",
        )
    assert tracked.read_bytes() == before


def test_shadow_path_provenance_mismatch_fails_closed(tmp_path: Path):
    old = _contract(tmp_path, "old", extra_default=2.0)
    removed = _contract(tmp_path, "removed")
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    _initial_state(old, state)
    execute_reconciliation(
        _plan(
            old=old,
            new=removed,
            state=state,
            operations=operations,
            operation="retire-before-shadow-move-0001",
            old_release="release-old",
            new_release="release-removed",
        )
    )
    shadow = next(state.glob("shadows/sim/**/*.json"))
    wrong = shadow.parents[1] / ("f" * 64) / shadow.name
    wrong.parent.mkdir()
    shadow.rename(wrong)
    with pytest.raises(ReconciliationError, match="path and provenance disagree"):
        _plan(
            old=removed,
            new=removed,
            state=state,
            operations=operations,
            operation="reject-shadow-move-0001",
            old_release="release-removed",
            new_release="release-removed",
        )


def test_review_tamper_partial_cross_binding_and_state_drift_fail(tmp_path: Path):
    old = _contract(tmp_path, "old", extra_default=2.0)
    removed = _contract(tmp_path, "removed")
    reintroduced = _contract(tmp_path, "new", extra_default=3.0)
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    _initial_state(old, state)
    execute_reconciliation(
        _plan(
            old=old,
            new=removed,
            state=state,
            operations=operations,
            operation="retire-review-0001",
            old_release="release-old",
            new_release="release-removed",
        )
    )
    plan = _plan(
        old=removed,
        new=reintroduced,
        state=state,
        operations=operations,
        operation="review-binding-0001",
        old_release="release-removed",
        new_release="release-new",
    )
    review = write_reintroduction_review(plan)
    with pytest.raises(ReconciliationError, match="incomplete"):
        write_reintroduction_decisions(review, {})
    decisions = write_reintroduction_decisions(
        review,
        {
            "tracked/default.yaml:/reconciliation_fixture/retired_gain": "use_new_default"
        },
    )
    value = json.loads(decisions.read_text())
    value["target_id"] = "other-target"
    decisions.write_text(json.dumps(value))
    with pytest.raises(ReconciliationError, match="stale, edited, or cross-bound"):
        execute_reconciliation(plan, decisions_path=decisions)


def test_interrupted_reconciliation_resumes_and_receiver_requires_private_stage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    contract = _contract(tmp_path, "contract", extra_default=2.0)
    state = tmp_path / "state"
    operations = tmp_path / "ops"
    plan = _plan(
        old=contract,
        new=contract,
        state=state,
        operations=operations,
        operation="interrupted-plan-0001",
        old_release="release-old",
        new_release="release-old",
    )
    import iii_drone_configuration.reconciliation as module

    original = module._atomic_bytes
    calls = 0

    def interrupted(path, data, *, mode=0o640):
        nonlocal calls
        calls += 1
        if calls == 4:
            raise OSError("simulated power loss")
        return original(path, data, mode=mode)

    monkeypatch.setattr(module, "_atomic_bytes", interrupted)
    with pytest.raises(OSError, match="power loss"):
        execute_reconciliation(plan)
    monkeypatch.setattr(module, "_atomic_bytes", original)
    assert execute_reconciliation(plan).status == "complete"

    aircraft_state = tmp_path / "aircraft-state"
    receiver = _plan(
        old=contract,
        new=contract,
        state=aircraft_state,
        operations=tmp_path / "aircraft-ops",
        operation="receiver-stage-0001",
        old_release="release-old",
        new_release="release-old",
        mode="receiver-staged",
    )
    with pytest.raises(ReconciliationError, match="bound staged copy"):
        execute_reconciliation(receiver)


def test_checkpoint_round_trip_and_receiver_stage_never_edits_only_copy(tmp_path: Path):
    contract = _contract(tmp_path, "contract")
    state = tmp_path / "state"
    _initial_state(contract, state)
    checkpoint = seal_configuration_checkpoint(
        writable_state_root=state,
        checkpoint_root=tmp_path / "checkpoints",
        target_id="sim",
        runtime_profile="sim",
        schema_version=1,
        release_id="release-old",
        manifest_id=_manifest(contract),
    )
    verified = verify_configuration_checkpoint(Path(checkpoint["path"]))
    assert verified["checkpoint_id"] == checkpoint["checkpoint_id"]
    assert stat.S_IMODE(Path(checkpoint["path"]).stat().st_mode) == 0o555
    assert checkpoint["checkpoint_id"] == hashlib.sha256(
        _canonical(
            {
                key: item
                for key, item in verified.items()
                if key != "checkpoint_id"
            }
        )
    ).hexdigest()
    wrong_name = Path(checkpoint["path"]).parent / ("f" * 64)
    shutil.copytree(checkpoint["path"], wrong_name)
    with pytest.raises(ReconciliationError, match="checkpoint identity"):
        verify_configuration_checkpoint(wrong_name)
    original = (state / "parameter_sets/sim/tracked/default.yaml").read_bytes()
    stage = materialize_receiver_stage(
        source_checkpoint=Path(checkpoint["path"]),
        stage_root=tmp_path / "stages/receiver-stage-0001",
        operation_id="receiver-stage-0001",
        target_id="sim",
    )
    (stage / "parameter_sets/sim/tracked/default.yaml").write_text("changed\n")
    assert (state / "parameter_sets/sim/tracked/default.yaml").read_bytes() == original
    assert (
        verify_configuration_checkpoint(Path(checkpoint["path"]))["checkpoint_id"]
        == checkpoint["checkpoint_id"]
    )


def test_wrong_profile_and_incompatible_contract_fail_before_mutation(tmp_path: Path):
    contract = _contract(tmp_path, "contract")
    with pytest.raises(ReconciliationError, match="only sim"):
        plan_reconciliation(
            old_immutable_root=contract,
            new_immutable_root=contract,
            writable_state_root=tmp_path / "state",
            operations_root=tmp_path / "ops",
            operation_id="wrong-profile-0001",
            runtime_profile="real",
            target_id="sim",
            old_release_id="old",
            new_release_id="new",
            mode="simulation",
        )
    assert not (tmp_path / "state").exists()
