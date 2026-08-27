from pathlib import Path

import pytest

from iii_drone_configuration.schema_utils import (
    persist_default_parameter_file_name,
    resolve_active_parameter_file,
    resolve_default_parameter_file_name,
    resolve_schema_file,
    seed_runtime_configuration,
)
from iii_drone_configuration.installed_contracts import resolve_installed_contract_root
from iii_drone_configuration.reconciliation import ReconciliationError

from conftest import write_bootstrap_parameter_file


@pytest.fixture(autouse=True)
def isolated_operations(monkeypatch, tmp_path):
    monkeypatch.setenv("III_OPERATIONS_ROOT", str(tmp_path / "operations"))


def test_resolve_schema_file_uses_only_installed_immutable_contract(
    monkeypatch, tmp_path
):
    monkeypatch.delenv("III_DRONE_SCHEMA_FILE", raising=False)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("WORKSPACE_DIR", str(Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("SIMULATION", "true")

    schema_path = resolve_schema_file()

    assert (
        schema_path
        == resolve_installed_contract_root() / "schema" / "parameter_manifest.yaml"
    )
    assert not schema_path.resolve().is_relative_to(Path(__file__).resolve().parents[1])


def test_seed_runtime_configuration_populates_config_root_without_overwriting(
    monkeypatch, tmp_path
):
    monkeypatch.delenv("III_DRONE_SCHEMA_FILE", raising=False)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("WORKSPACE_DIR", str(Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("SIMULATION", "true")

    seeded = seed_runtime_configuration("sim")

    assert (tmp_path / "iii_drone" / "profiles" / "sim.yaml").exists()
    assert (
        tmp_path / "iii_drone" / "parameter_sets" / "sim" / "tracked" / "default.yaml"
    ).exists()
    assert not (tmp_path / "iii_drone" / "parameters").exists()
    assert resolve_active_parameter_file("sim") == (
        tmp_path / "iii_drone" / "parameter_sets" / "sim" / "tracked" / "default.yaml"
    )
    assert (
        resolve_schema_file()
        == resolve_installed_contract_root() / "schema" / "parameter_manifest.yaml"
    )
    assert "profiles/sim.yaml" in seeded

    selector = tmp_path / "iii_drone" / "profiles" / "sim.yaml"
    selector.write_text("custom: true\n", encoding="utf-8")
    with pytest.raises(ReconciliationError, match="selector is malformed"):
        seed_runtime_configuration("sim")


def test_seed_runtime_configuration_never_copies_schema_or_overwrites_parameter_sets(
    monkeypatch, tmp_path
):
    monkeypatch.delenv("III_DRONE_SCHEMA_FILE", raising=False)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("WORKSPACE_DIR", str(Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("SIMULATION", "true")

    seed_runtime_configuration("sim")
    tracked = (
        tmp_path / "iii_drone" / "parameter_sets" / "sim" / "tracked" / "default.yaml"
    )
    tracked.write_text("custom_parameter_set: true\n", encoding="utf-8")

    with pytest.raises(ReconciliationError, match="must contain the '/\*\*' ROS scope"):
        seed_runtime_configuration("sim")

    assert not (tmp_path / "iii_drone" / "parameters").exists()


def test_aircraft_and_reserved_profiles_never_seed_at_runtime(monkeypatch, tmp_path):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))

    with pytest.raises(ReconciliationError, match="receiver-reconciled"):
        seed_runtime_configuration("real")
    with pytest.raises(ReconciliationError, match="receiver-reconciled"):
        seed_runtime_configuration("opti_track")
    with pytest.raises(ReconciliationError, match="non-bootable"):
        seed_runtime_configuration("hil")

    config = tmp_path / "iii_drone"
    assert not config.exists()


def test_active_parameter_file_prefers_default_snapshot(monkeypatch, tmp_path):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    write_bootstrap_parameter_file(
        tmp_path, active_parameter_file="snapshots/custom.yaml"
    )
    snapshot_dir = tmp_path / "iii_drone" / "parameter_sets" / "sim" / "snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    snapshot_file = snapshot_dir / "custom.yaml"
    snapshot_file.write_text(
        "/**:\n  ros__parameters:\n    /control/mode: manual\n", encoding="utf-8"
    )

    assert resolve_default_parameter_file_name("sim") == "snapshots/custom.yaml"
    assert resolve_active_parameter_file("sim") == snapshot_file


def test_persist_default_parameter_file_name_updates_bootstrap_file(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    selector = write_bootstrap_parameter_file(tmp_path)

    persist_default_parameter_file_name("sim", "snapshots/next.yaml")

    assert resolve_default_parameter_file_name("sim") == "snapshots/next.yaml"
    assert "snapshots/next.yaml" in selector.read_text(encoding="utf-8")


def test_resolve_active_parameter_file_uses_profile_pointer(monkeypatch, tmp_path):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))

    config_dir = tmp_path / "iii_drone"
    profile_dir = config_dir / "profiles"
    parameter_set_dir = config_dir / "parameter_sets" / "sim" / "tracked"
    profile_dir.mkdir(parents=True)
    parameter_set_dir.mkdir(parents=True)

    active_file = parameter_set_dir / "default.yaml"
    active_file.write_text("/**:\n  ros__parameters: {}\n")
    (profile_dir / "sim.yaml").write_text(
        "version: 1\nactive_parameter_set: tracked/default.yaml\n"
    )

    seed_runtime_configuration("sim")

    assert resolve_active_parameter_file("sim") == active_file
