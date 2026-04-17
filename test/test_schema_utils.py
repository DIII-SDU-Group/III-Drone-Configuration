from pathlib import Path

from iii_drone_configuration.schema_utils import (
    persist_default_parameter_file_name,
    resolve_active_parameter_file,
    resolve_default_parameter_file_name,
    resolve_schema_file,
    seed_runtime_configuration,
)

from conftest import write_bootstrap_parameter_file


def test_resolve_schema_file_falls_back_to_workspace_source(monkeypatch, tmp_path):
    monkeypatch.delenv("III_DRONE_SCHEMA_FILE", raising=False)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("WORKSPACE_DIR", str(Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("SIMULATION", "true")

    schema_path = resolve_schema_file()

    assert schema_path == Path(__file__).resolve().parents[1] / "config" / "parameters" / "parameter_manifest.yaml"


def test_seed_runtime_configuration_populates_config_root_without_overwriting(monkeypatch, tmp_path):
    monkeypatch.delenv("III_DRONE_SCHEMA_FILE", raising=False)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("WORKSPACE_DIR", str(Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("SIMULATION", "true")

    seeded = seed_runtime_configuration("sim")

    assert (tmp_path / "iii_drone" / "ros_params_sim.yaml").exists()
    assert (tmp_path / "iii_drone" / "parameters" / "parameter_manifest.yaml").exists()
    assert (tmp_path / "iii_drone" / "parameter_snapshots").is_dir()
    assert resolve_active_parameter_file("sim") == tmp_path / "iii_drone" / "ros_params_sim.yaml"
    assert resolve_schema_file() == tmp_path / "iii_drone" / "parameters" / "parameter_manifest.yaml"
    assert "bootstrap" in seeded

    bootstrap = tmp_path / "iii_drone" / "ros_params_sim.yaml"
    bootstrap.write_text("custom: true\n", encoding="utf-8")
    seed_runtime_configuration("sim")

    assert bootstrap.read_text(encoding="utf-8") == "custom: true\n"


def test_active_parameter_file_prefers_default_snapshot(monkeypatch, tmp_path):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    write_bootstrap_parameter_file(tmp_path, default_snapshot_file="custom.yaml")
    snapshot_dir = tmp_path / "iii_drone" / "parameter_snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    snapshot_file = snapshot_dir / "custom.yaml"
    snapshot_file.write_text("/**:\n  ros__parameters:\n    /control/mode: manual\n", encoding="utf-8")

    assert resolve_default_parameter_file_name("sim") == "custom.yaml"
    assert resolve_active_parameter_file("sim") == snapshot_file


def test_persist_default_parameter_file_name_updates_bootstrap_file(monkeypatch, tmp_path):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    bootstrap = write_bootstrap_parameter_file(tmp_path)

    persist_default_parameter_file_name("sim", "next.yaml")

    assert resolve_default_parameter_file_name("sim") == "next.yaml"
    assert "next.yaml" in bootstrap.read_text(encoding="utf-8")
