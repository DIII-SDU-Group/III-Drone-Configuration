from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil

import pytest

from iii_drone_configuration.installed_contracts import (
    ConfigurationContractError,
    load_installed_contract,
    plan_compatibility,
    resolve_installed_contract_root,
)
from iii_drone_configuration import InstalledConfigurationContract


def _canonical(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _rewrite_manifest(root: Path, mutate) -> None:
    path = root / "package-manifest.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    mutate(value)
    value["manifest_id"] = hashlib.sha256(
        _canonical({key: item for key, item in value.items() if key != "manifest_id"})
    ).hexdigest()
    _write_json(path, value)


def _replace_artifact_hash(root: Path, relative: str) -> None:
    digest = hashlib.sha256((root / relative).read_bytes()).hexdigest()

    def mutate(value):
        row = next(item for item in value["artifacts"] if item["path"] == relative)
        row["sha256"] = digest

    _rewrite_manifest(root, mutate)


def _copy_contract(tmp_path: Path, name: str) -> Path:
    target = tmp_path / name
    shutil.copytree(resolve_installed_contract_root(), target, symlinks=False)
    return target


def test_installed_contract_authenticates_profiles_defaults_and_artifacts() -> None:
    root = resolve_installed_contract_root()
    result = load_installed_contract(root)
    contract = result.contract

    assert isinstance(contract, InstalledConfigurationContract)
    assert contract.schema_version == 1
    assert contract.profile("real").parameter_profile == "real"
    assert contract.profile("sim").parameter_profile == "sim"
    assert contract.profile("opti_track").parameter_profile == "real"
    assert contract.profile("opti_track").selector_scope == "opti_track"
    assert contract.profile("hil").parameter_profile == "sim"
    assert contract.profile("hil").selector_scope == "hil"
    assert contract.profile("hil").bootable is False
    assert contract.default_set("real").set_id == "default"
    assert contract.default_set("sim").set_id == "default"
    assert len(result.verified_artifacts) == 6


def test_compatibility_planning_is_typed_deterministic_and_side_effect_free(
    tmp_path: Path,
) -> None:
    root = resolve_installed_contract_root()
    writable = tmp_path / "must-not-be-opened"

    first = plan_compatibility(
        old_immutable_root=root,
        new_immutable_root=root,
        writable_state_root=writable,
        runtime_profile="opti_track",
    )
    second = plan_compatibility(
        old_immutable_root=root,
        new_immutable_root=root,
        writable_state_root=writable,
        runtime_profile="opti_track",
    )

    assert first == second
    assert first.compatible is True
    assert first.direction == "same-schema"
    assert first.parameter_profile == "real"
    assert first.selector_scope == "opti_track"
    assert first.mutations == ()
    assert first.as_dict()["plan_id"] == first.plan_id
    assert not writable.exists()


def test_upgrade_downgrade_ranges_and_unknown_versions_fail_closed(
    tmp_path: Path,
) -> None:
    old = _copy_contract(tmp_path, "old")
    new = _copy_contract(tmp_path, "new")
    migrations_path = new / "migrations.json"
    migrations = json.loads(migrations_path.read_text(encoding="utf-8"))
    migrations["current_schema_version"] = 2
    migrations["upgrade_from"] = {"minimum": 1, "maximum": 1}
    migrations["downgrade_from"] = {"minimum": 2, "maximum": 2}
    _write_json(migrations_path, migrations)
    _replace_artifact_hash(new, "migrations.json")

    def to_v2(value):
        value["configuration_schema_version"] = 2
        value["compatibility"] = {
            "readable": {"minimum": 2, "maximum": 2},
            "upgrade_from": {"minimum": 1, "maximum": 1},
            "downgrade_from": {"minimum": 2, "maximum": 2},
        }

    _rewrite_manifest(new, to_v2)
    plan = plan_compatibility(
        old_immutable_root=old,
        new_immutable_root=new,
        writable_state_root=tmp_path / "state",
        runtime_profile="real",
    )
    assert plan.compatible is True and plan.direction == "upgrade"

    migrations["upgrade_from"] = {"minimum": 2, "maximum": 2}
    _write_json(migrations_path, migrations)
    _replace_artifact_hash(new, "migrations.json")

    def incompatible(value):
        value["compatibility"]["upgrade_from"] = {"minimum": 2, "maximum": 2}

    _rewrite_manifest(new, incompatible)
    rejected = plan_compatibility(
        old_immutable_root=old,
        new_immutable_root=new,
        writable_state_root=tmp_path / "state",
        runtime_profile="real",
    )
    assert rejected.compatible is False
    assert rejected.direction == "upgrade"


def test_malformed_manifest_artifact_tamper_and_source_shadow_are_rejected(
    tmp_path: Path,
) -> None:
    root = _copy_contract(tmp_path, "contract")
    (root / "profiles.json").write_text("{}\n", encoding="utf-8")
    with pytest.raises(ConfigurationContractError, match="hash mismatch"):
        load_installed_contract(root)

    unknown = _copy_contract(tmp_path, "unknown")

    def unknown_schema(value):
        value["schema"] = "iii.configuration-package/v99"

    _rewrite_manifest(unknown, unknown_schema)
    with pytest.raises(ConfigurationContractError, match="unsupported"):
        load_installed_contract(unknown)

    source = Path(__file__).resolve().parents[1] / "config" / "configuration_contract"
    with pytest.raises(ConfigurationContractError, match="source tree"):
        load_installed_contract(source)


def test_non_default_tracked_sets_are_extensible_without_a_second_default(
    tmp_path: Path,
) -> None:
    root = _copy_contract(tmp_path, "contract")
    extra = root / "tracked_defaults/real/experimental.yaml"
    shutil.copyfile(root / "tracked_defaults/real/default.yaml", extra)
    digest = hashlib.sha256(extra.read_bytes()).hexdigest()

    def add_set(value):
        value["artifacts"].append(
            {
                "path": "tracked_defaults/real/experimental.yaml",
                "sha256": digest,
            }
        )
        value["artifacts"] = sorted(value["artifacts"], key=lambda item: item["path"])
        value["tracked_sets"].insert(
            1,
            {
                "profile": "real",
                "set_id": "experimental",
                "default": False,
                "path": "tracked_defaults/real/experimental.yaml",
                "sha256": digest,
            },
        )

    _rewrite_manifest(root, add_set)
    contract = load_installed_contract(root).contract
    assert [
        item.set_id for item in contract.tracked_sets if item.profile == "real"
    ] == [
        "default",
        "experimental",
    ]
    assert contract.default_set("real").set_id == "default"


def test_explicit_roots_and_ament_only_resolution_reject_implicit_inputs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with pytest.raises(ConfigurationContractError, match="absolute"):
        load_installed_contract(Path("relative-contract"))

    source_share = Path(__file__).resolve().parents[1] / "config"
    monkeypatch.setattr(
        "ament_index_python.packages.get_package_share_directory",
        lambda _name: str(source_share),
    )
    with pytest.raises(ConfigurationContractError):
        resolve_installed_contract_root()
