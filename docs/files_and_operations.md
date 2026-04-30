# Files And Operations

## Important Files

### Schema

- `config/parameters/parameter_manifest.yaml`

This is the active runtime schema file for managed parameters.

Each managed parameter is defined once here with:
- type
- default value
- optional validation constraints
- optional mutability rules

### Profiles And Parameter Sets

- `config/profiles/real.yaml`
- `config/profiles/sim.yaml`
- `config/parameter_sets/real/tracked/default.yaml`
- `config/parameter_sets/sim/tracked/default.yaml`

The profile selector chooses the active parameter set for a profile:

```yaml
version: 1
active_parameter_set: tracked/default.yaml
```

Every parameter-set file is a standalone ROS parameter file with canonical full names under `/**: ros__parameters:`.
Snapshots live alongside tracked files in:
- `~/.config/iii_drone/parameter_sets/<profile>/snapshots/`

### Compatibility / Install Files

- `config/parameters/parameters.yaml`
- `config/parameters/parameter_manifest.yaml`
- `scripts/update_installed_parameters.py`
- `scripts/install.sh`

These support install/update workflows that propagate parameter definitions into a target configuration directory. The runtime design should treat:
- `parameter_manifest.yaml` as the schema source
- `ros_params_*.yaml` as runtime value files

## Common Operations

### Add A New Managed Parameter

1. Add the parameter to `config/parameters/parameter_manifest.yaml`.
2. Add the runtime value to the tracked parameter-set files for any relevant profiles.
3. In the owning node, call `Configurator::DeclareParameter(...)` or `Configurator.declare_parameter(...)`.
4. If a nested subsystem needs live access, include it in a `Configuration`.
5. If the parameter is meant to be shared across nodes, reuse the exact same canonical full name.

### Add Validation

Validation belongs in the schema manifest, not in duplicated node-local defaults.

Supported forms:
- `constant: true`
- numeric `min`
- numeric `max`
- string `options`
- expression-based numeric constraints referencing other canonical keys

### Save A Snapshot

When the configuration server is running, it can save the current converged parameter state to a full parameter-set file.

Operator parameter changes through `set_parameter_from_gc` also maintain an automatic runtime snapshot named `runtime_parameters_<timestamp>.yaml`. The server updates this file after each successful live change, points the active profile selector at it, and deletes the previous automatic runtime snapshot from the same server run. If values return to the boot configuration, the automatic runtime snapshot is removed and the selector is restored to the boot-selected parameter set.

### Load A Snapshot

The server can load a saved parameter set, validate it, and push values to discovered managed nodes.

### Resolve The Active Schema Path

Schema helper:
- `iii_drone_configuration/schema_utils.py`

Active parameter-set resolution order:

1. `III_SYSTEM_PARAMETER_FILE`, if explicitly set
2. `SIMULATION` selects `sim` or `real`
3. `~/.config/iii_drone/profiles/<profile>.yaml` chooses the active parameter set
4. That selector resolves into `~/.config/iii_drone/parameter_sets/<profile>/<reference>`

## Current Caveats

- `config/parameters/parameters.yaml` still exists as a compatibility artifact and should not be treated as the schema source of truth.
- The service layer still uses `default_parameter_file` naming in a few message fields for compatibility, even though the runtime model is now selector-driven parameter sets.
- The package has both C++ and Python validation implementations; they are conceptually aligned, but maintenance should keep them in sync.
