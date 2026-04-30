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

At runtime these are seeded into:
- `~/.config/iii_drone/profiles/<profile>.yaml`
- `~/.config/iii_drone/parameter_sets/<profile>/tracked/*.yaml`
- `~/.config/iii_drone/parameter_sets/<profile>/snapshots/*.yaml`

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
- reject live operator changes unless every affected running node applies the change
- reject live updates for boot-only parameters
- save and load full parameter-set files
- maintain an automatic runtime parameter-set snapshot after successful live operator changes

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

By default, the package resolves the schema to:
- `~/.config/iii_drone/parameters/parameter_manifest.yaml`

unless overridden.

Helpers:
- [`iii_drone_configuration/schema_utils.py`](./iii_drone_configuration/schema_utils.py)

Active parameter-set resolution is controlled by:
- `SIMULATION`, which selects the `sim` or `real` profile
- `~/.config/iii_drone/profiles/<profile>.yaml`, which points at the active parameter set
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
- [`iii_drone_configuration/configuration_server_node.py`](./iii_drone_configuration/configuration_server_node.py)
- [`iii_drone_configuration/configuration_client_node.py`](./iii_drone_configuration/configuration_client_node.py)

### Config files

- [`config/parameters/parameter_manifest.yaml`](./config/parameters/parameter_manifest.yaml)
- [`config/profiles/real.yaml`](./config/profiles/real.yaml)
- [`config/profiles/sim.yaml`](./config/profiles/sim.yaml)
- [`config/parameter_sets/real/tracked/default.yaml`](./config/parameter_sets/real/tracked/default.yaml)
- [`config/parameter_sets/sim/tracked/default.yaml`](./config/parameter_sets/sim/tracked/default.yaml)

### Scripts

- [`scripts/install.sh`](./scripts/install.sh)
- [`scripts/update_installed_parameters.py`](./scripts/update_installed_parameters.py)

## Additional Documentation

- [`docs/architecture.md`](./docs/architecture.md)
- [`docs/api.md`](./docs/api.md)
- [`docs/files_and_operations.md`](./docs/files_and_operations.md)

## Notes

- `config/parameters/parameters.yaml` still exists as a compatibility artifact. The active schema source for runtime and install/update flows is `parameter_manifest.yaml`.
- The package currently supports both C++ and Python validation paths.
- Nodes should declare managed parameters explicitly in code. There is no central preset registry anymore.
