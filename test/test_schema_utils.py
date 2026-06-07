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

    assert (tmp_path / "iii_drone" / "profiles" / "sim.yaml").exists()
    assert (tmp_path / "iii_drone" / "parameter_sets" / "sim" / "tracked" / "default.yaml").exists()
    assert (tmp_path / "iii_drone" / "parameters" / "parameter_manifest.yaml").exists()
    assert (tmp_path / "iii_drone" / "parameter_sets" / "sim" / "snapshots").is_dir()
    assert resolve_active_parameter_file("sim") == (
        tmp_path / "iii_drone" / "parameter_sets" / "sim" / "tracked" / "default.yaml"
    )
    assert resolve_schema_file() == tmp_path / "iii_drone" / "parameters" / "parameter_manifest.yaml"
    assert "profiles/sim.yaml" in seeded

    selector = tmp_path / "iii_drone" / "profiles" / "sim.yaml"
    selector.write_text("custom: true\n", encoding="utf-8")
    seed_runtime_configuration("sim")

    assert selector.read_text(encoding="utf-8") == "custom: true\n"


def test_seed_runtime_configuration_refreshes_schema_without_overwriting_parameter_sets(monkeypatch, tmp_path):
    monkeypatch.delenv("III_DRONE_SCHEMA_FILE", raising=False)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("WORKSPACE_DIR", str(Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("SIMULATION", "true")

    seed_runtime_configuration("sim")
    schema = tmp_path / "iii_drone" / "parameters" / "parameter_manifest.yaml"
    tracked = tmp_path / "iii_drone" / "parameter_sets" / "sim" / "tracked" / "default.yaml"
    schema.write_text("stale_schema: true\n", encoding="utf-8")
    tracked.write_text("custom_parameter_set: true\n", encoding="utf-8")

    seed_runtime_configuration("sim")

    assert "stale_schema" not in schema.read_text(encoding="utf-8")
    assert "custom_parameter_set: true\n" == tracked.read_text(encoding="utf-8")


def test_active_parameter_file_prefers_default_snapshot(monkeypatch, tmp_path):
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))
    monkeypatch.setenv("SIMULATION", "true")
    write_bootstrap_parameter_file(tmp_path, active_parameter_file="snapshots/custom.yaml")
    snapshot_dir = tmp_path / "iii_drone" / "parameter_sets" / "sim" / "snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    snapshot_file = snapshot_dir / "custom.yaml"
    snapshot_file.write_text("/**:\n  ros__parameters:\n    /control/mode: manual\n", encoding="utf-8")

    assert resolve_default_parameter_file_name("sim") == "snapshots/custom.yaml"
    assert resolve_active_parameter_file("sim") == snapshot_file


def test_persist_default_parameter_file_name_updates_bootstrap_file(monkeypatch, tmp_path):
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
    (profile_dir / "sim.yaml").write_text("version: 1\nactive_parameter_set: tracked/default.yaml\n")

    seed_runtime_configuration("sim")

    assert resolve_active_parameter_file("sim") == active_file
