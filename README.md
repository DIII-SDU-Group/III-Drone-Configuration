# III-Drone-Configuration

`iii_drone_configuration` is the configuration package for the III-Drone ROS 2 workspace.

It provides:
- the schema and validation source of truth for managed parameters
- local-first `Configurator` helpers for C++ and Python nodes
- `Configuration` read-only live views for nested subsystems
- an optional configuration server for discovery, synchronization, live runtime updates, and parameter-set persistence
- the canonical manifest and parameter-set files used by the rest of the workspace

## Current Design

The package is no longer built around a mandatory startup server.

The current model is:
- each node declares its managed parameters locally through `Configurator`
- the declaration default comes from the central schema manifest
- ROS parameter files may override those values during startup
- the node validates values locally against the shared schema
- nested classes receive `Configuration` objects, which proxy live reads to the node's current ROS parameters
- the configuration server is optional and only coordinates runtime synchronization and snapshot management

This removes the old hard dependency on a central configuration server during bringup while keeping:
- one schema source of truth
- live parameter validation
- runtime synchronization when a server is present
- reproducible snapshots

## Main Concepts

### Schema Manifest

The schema manifest defines managed parameters, their types, defaults, mutability, and validation constraints.

Primary file:
- [`config/parameters/parameter_manifest.yaml`](./config/parameters/parameter_manifest.yaml)

Each leaf entry defines at least:
- `type`
- `value`

Optional fields include:
- `constant`
- `min`
- `max`
- `options`

Example:

```yaml
control:
  dt:
    type: float
    value: 0.2
    min: 0.01
```

The manifest uses hierarchical YAML, but managed parameter identities are canonical full names such as:
- `/control/dt`
- `/tf/world_frame_id`
- `/perception/pl_mapper/kf_r`

### Profiles And Parameter Sets

The runtime configuration now separates:
- profile selection
- actual parameter values

Source defaults:
- [`config/profiles/real.yaml`](./config/profiles/real.yaml)
- [`config/profiles/sim.yaml`](./config/profiles/sim.yaml)
- [`config/parameter_sets/real/tracked/default.yaml`](./config/parameter_sets/real/tracked/default.yaml)
- [`config/parameter_sets/sim/tracked/default.yaml`](./config/parameter_sets/sim/tracked/default.yaml)

Build/install captures these inputs into an authenticated immutable contract under
`share/iii_drone_configuration/configuration_contract`. The captured package
manifest binds schema version 1, compatibility ranges, migration metadata, runtime
profile aliases, the managed-parameter schema, and exactly one `default` tracked
set for each `real` and `sim` parameter profile. The tracked-set descriptor is a
list so later releases can add reviewed non-default tracked sets without changing
the default-selection contract.

Runtime profiles and parameter profiles are deliberately distinct:

- `real -> real`, bootable, selector scope `real`
- `sim -> sim`, bootable, selector scope `sim`
- `opti_track -> real`, bootable, selector scope `opti_track`
- `hil -> sim`, reserved/non-bootable, selector scope `hil`

Alias selectors therefore remain independent even when their initial immutable
default bytes come from the same parameter profile.

At runtime these are reconciled into `$CONFIG_BASE_DIR/iii_drone`. Development
profiles set `CONFIG_BASE_DIR` to the current clone's Git-ignored `.config/` root;
the aircraft uses receiver-owned persistent checkpoints under `/var/lib/iii`.
Simulation reconciles automatically before startup. Aircraft build/install/runtime
startup never mutates this state; only receiver activation creates a staged and
content-addressed successor checkpoint.

The selector file is the authority for which parameter set is active:

```yaml
version: 1
active_parameter_set: tracked/default.yaml
```

Every parameter-set file is a standalone ROS parameter file with flattened keys under `/**: ros__parameters:`.
Parameter-set files never point to other parameter-set files.

### Configurator

`Configurator` is a local node helper, not a central authority.

Responsibilities:
- locate and load the schema manifest
- declare managed ROS parameters with schema defaults
- verify that declared type matches schema type
- validate initial values
- reject invalid runtime updates through a normal ROS 2 parameter callback
- construct named `Configuration` views for nested subsystems
- announce managed nodes to the optional configuration server

It does not:
- own a second parameter store
- distribute startup values to nodes
- require the server to be running

Public C++ headers:
- [`include/iii_drone_configuration/configurator.hpp`](./include/iii_drone_configuration/configurator.hpp)
- [`include/iii_drone_configuration/configuration.hpp`](./include/iii_drone_configuration/configuration.hpp)
- [`include/iii_drone_configuration/schema_validator.hpp`](./include/iii_drone_configuration/schema_validator.hpp)

Python equivalents:
- [`iii_drone_configuration/configurator.py`](./iii_drone_configuration/configurator.py)
- [`iii_drone_configuration/configuration.py`](./iii_drone_configuration/configuration.py)
- [`iii_drone_configuration/parameter_handler.py`](./iii_drone_configuration/parameter_handler.py)

### Configuration

`Configuration` is a small, read-only, named view over a subset of a node's managed parameters.

It is intended for nested classes and subsystems that should not know about:
- `Configurator`
- schema files
- ROS parameter declaration details

It always yields the latest value because it resolves reads through the owning node's parameter getter.

### Configuration Server

The configuration server is optional runtime infrastructure.

Current responsibilities:
- discover nodes exposing managed parameters
- infer shared parameters by canonical full name
- synchronize late-joining nodes
- keep server-owned runtime values converged while the server is active
- validate a complete operator edit batch before applying any key
- durably journal intent, distributed readback, commit, rejection, and compensation
- hold node/runtime-restart values as pending until a fresh whole-graph readback
- block all further writes if exact compensation cannot restore a consistent graph
- save and load full parameter-set files
- maintain an automatic runtime parameter-set snapshot after successful live operator changes

The internal tuning session is opened or resumed automatically on the first
transaction. It binds an immutable baseline and monotonically revised current
state to target, profile, release/workspace, and installed-manifest identities.
There is intentionally no operator-facing start/end session workflow. Simulation
stores this evidence under the clone's Git-ignored `.iii/tuning/`; the real host
sets `III_TUNING_STATE_ROOT=/var/lib/iii/tuning`.

Implementation:
- [`iii_drone_configuration/configuration_server_node.py`](./iii_drone_configuration/configuration_server_node.py)

Terminal client:
- [`iii_drone_configuration/configuration_client_node.py`](./iii_drone_configuration/configuration_client_node.py)

## Typical Node Workflow

### C++

```cpp
configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>>(
    this,
    "my_node"
);

configurator_->DeclareParameter("/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE);
configurator_->DeclareParameter("/tf/world_frame_id", rclcpp::ParameterType::PARAMETER_STRING);

configurator_->CreateConfiguration("controller", {
    {"/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE},
    {"/tf/world_frame_id", rclcpp::ParameterType::PARAMETER_STRING},
});

configurator_->validate();

auto controller_configuration = configurator_->GetConfiguration("controller");
```

### Python

```python
self.configurator = Configurator(node=self)

self.configurator.declare_parameter("/control/dt", rclpy.parameter.Parameter.Type.DOUBLE)
self.configurator.declare_parameter("/tf/world_frame_id", rclpy.parameter.Parameter.Type.STRING)

self.configurator.validate()
```

## Environment And Path Resolution

Schema resolution is controlled by:
- `III_DRONE_SCHEMA_FILE`
- `CONFIG_BASE_DIR`

By default, both Python and C++ resolve the schema only through the ament-indexed,
build-captured immutable contract. `WORKSPACE_DIR`, editable source files, and a
writable `~/.config/iii_drone/parameters` shadow are never implicit schema inputs.
`III_DRONE_SCHEMA_FILE` remains an explicit debug/test override.

The side-effect-free public Python API is exported from
`iii_drone_configuration.installed_contracts` and the package root:

- `resolve_installed_contract_root()` performs strict ament-only resolution.
- `load_installed_contract(immutable_root)` authenticates every declared artifact
  and returns a typed `ContractLoadResult`.
- `plan_compatibility(old_immutable_root=..., new_immutable_root=...,
  writable_state_root=..., runtime_profile=...)` returns a content-identified
  typed plan without opening or creating the writable root.
- `plan_reconciliation(...)` and `execute_reconciliation(...)` implement the
  shared preserve/add/retire/review transaction for simulation and receiver stages.
- `seal_configuration_checkpoint(...)` and
  `verify_configuration_checkpoint(...)` provide the content-addressed tree
  boundary used by reset, activation, rollback, capture, and portable backup.

All roots are explicit absolute paths. Compatibility can therefore be evaluated
before runtime shutdown and before any writable-state migration begins.

Helpers:
- [`iii_drone_configuration/schema_utils.py`](./iii_drone_configuration/schema_utils.py)

Active parameter-set resolution is controlled by:
- `SIMULATION`, which selects the `sim` or `real` profile
- `$CONFIG_BASE_DIR/iii_drone/profiles/<profile>.yaml`, which points at the active parameter set
- `III_SYSTEM_PARAMETER_FILE`, only as an explicit override/debug escape hatch

## Package Layout

### C++ library

- [`include/iii_drone_configuration/configurator.hpp`](./include/iii_drone_configuration/configurator.hpp)
- [`include/iii_drone_configuration/configuration.hpp`](./include/iii_drone_configuration/configuration.hpp)
- [`include/iii_drone_configuration/schema_validator.hpp`](./include/iii_drone_configuration/schema_validator.hpp)
- [`src/configurator.cpp`](./src/configurator.cpp)
- [`src/configuration.cpp`](./src/configuration.cpp)
- [`src/schema_validator.cpp`](./src/schema_validator.cpp)

### Python package

- [`iii_drone_configuration/configurator.py`](./iii_drone_configuration/configurator.py)
- [`iii_drone_configuration/configuration.py`](./iii_drone_configuration/configuration.py)
- [`iii_drone_configuration/parameter_handler.py`](./iii_drone_configuration/parameter_handler.py)
- [`iii_drone_configuration/schema_utils.py`](./iii_drone_configuration/schema_utils.py)
- [`iii_drone_configuration/installed_contracts.py`](./iii_drone_configuration/installed_contracts.py)
- [`iii_drone_configuration/reconciliation.py`](./iii_drone_configuration/reconciliation.py)
- [`iii_drone_configuration/tuning.py`](./iii_drone_configuration/tuning.py)
- [`iii_drone_configuration/configuration_server_node.py`](./iii_drone_configuration/configuration_server_node.py)
- [`iii_drone_configuration/configuration_client_node.py`](./iii_drone_configuration/configuration_client_node.py)

### Config files

- [`config/parameters/parameter_manifest.yaml`](./config/parameters/parameter_manifest.yaml)
- [`config/profiles/real.yaml`](./config/profiles/real.yaml)
- [`config/profiles/sim.yaml`](./config/profiles/sim.yaml)
- [`config/parameter_sets/real/tracked/default.yaml`](./config/parameter_sets/real/tracked/default.yaml)
- [`config/parameter_sets/sim/tracked/default.yaml`](./config/parameter_sets/sim/tracked/default.yaml)

### Retired mutation entry points

- [`scripts/install.sh`](./scripts/install.sh)
- [`scripts/update_installed_parameters.py`](./scripts/update_installed_parameters.py)

Both scripts fail with status 64 and direct operators to `iii config sim`; they do
not propagate or mutate configuration.

## Additional Documentation

- [`docs/architecture.md`](./docs/architecture.md)
- [`docs/api.md`](./docs/api.md)
- [`docs/files_and_operations.md`](./docs/files_and_operations.md)

## Notes

- `config/parameters/parameters.yaml` still exists as a compatibility artifact. The active schema source is the authenticated installed copy of `parameter_manifest.yaml`.
- The package currently supports both C++ and Python validation paths.
- Nodes should declare managed parameters explicitly in code. There is no central preset registry anymore.
