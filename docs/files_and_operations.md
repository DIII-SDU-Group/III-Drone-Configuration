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

### ROS Startup / Snapshot Files

- `config/ros_params_real.yaml`
- `config/ros_params_sim.yaml`

These are flattened ROS parameter files suitable for:
- launch-time loading
- value overrides
- snapshot save/load

They contain canonical full names directly as ROS parameter keys.

### Compatibility / Install Files

- `config/parameters/parameters.yaml`
- `scripts/update_installed_parameters.py`
- `scripts/install.sh`

These are used for install/update workflows that propagate parameter definitions into a target configuration directory. They are still present, but the runtime design should treat:
- `parameter_manifest.yaml` as the schema source
- `ros_params_*.yaml` as runtime value files

## Common Operations

### Add A New Managed Parameter

1. Add the parameter to `config/parameters/parameter_manifest.yaml`.
2. Add the runtime value to `config/ros_params_real.yaml` and `config/ros_params_sim.yaml` if appropriate.
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

When the configuration server is running, it can save the current converged parameter state to a ROS parameter file in the configured snapshot directory.

Relevant support parameters:
- `parameter_snapshots_path_postfix`
- `default_snapshot_file`
- `sim_snapshot_file`

### Load A Snapshot

The server can load a saved ROS parameter snapshot, validate it, and push values to discovered managed nodes.

### Resolve The Active Schema Path

Resolution order:

1. `III_DRONE_SCHEMA_FILE`, if set
2. `CONFIG_BASE_DIR`, if set, otherwise `~/.config`
3. `SIMULATION` determines whether `default_parameter_file` or `sim_parameter_file` is used

The helper for this logic is:
- `iii_drone_configuration/schema_utils.py`

## Current Caveats

- The package still contains some legacy maintenance scripts oriented around `config/parameters/parameters.yaml`.
- The terminal client and server still use some older naming in their service layer, even though the runtime node-side model is now local-first.
- The package has both C++ and Python validation implementations; they are conceptually aligned, but maintenance should keep them in sync.
