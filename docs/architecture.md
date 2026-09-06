# Architecture

## Immutable package contract boundary

Configuration release inputs are captured at CMake configure time into the build
tree and installed through the ament package share. Even under `--symlink-install`,
installed contract files point to build-captured artifacts rather than editable
workspace source. The package manifest authenticates the parameter schema,
profiles, migration metadata, JSON schema, and tracked defaults by SHA-256 and a
content identity over the complete manifest.

Compatibility and reconciliation planning are pure pre-mutation operations.
Callers supply old and new immutable roots plus the intended writable-state root.
Planning authenticates the complete living tree, every retained legacy shadow,
and both contracts, but creates no files. Execution is journaled and writes shadow
records before removing retired values from active sets.

Simulation startup executes the shared engine against the clone-local
`$CONFIG_BASE_DIR/iii_drone` tree before selecting a set. Aircraft runtime and
build/install never reconcile living state. The root receiver copies the currently
selected immutable checkpoint into a private stage, executes the same plan there,
seals a new content-addressed checkpoint, and switches code/configuration/catalog
only as one activation transaction. Explicit rollback restores its already paired
checkpoint; compatible schema rollback can deterministically rehydrate the latest
canonical active-at-retirement shadow value.

Reintroduced keys block before mutation. A review binds every old value/default,
validation result, release, manifest, target, state, set, and operation. Planning
validates complete `use_old|use_new_default` decisions without writing; accepted
receiver execution seals the review and decisions before applying them.

## Purpose

`iii_drone_configuration` exists to make configuration:
- explicit in node code
- validated against a central schema
- available without a mandatory runtime server
- synchronizable when a server is running
- reproducible through ROS-native parameter files

## High-Level Model

There are four layers:

1. Schema
   File: `config/parameters/parameter_manifest.yaml`
   Defines parameter identity, type, default value, mutability, and validation constraints.

2. Effective node parameters
   Storage: native ROS 2 node parameters
   Each node owns its own parameters and uses normal ROS 2 APIs at runtime.

3. Local configuration binding
   Implementation: `Configurator`
   Bridges the schema onto a node by declaring managed parameters, validating them, and creating `Configuration` views.

4. Optional runtime coordination
   Implementation: `configuration_server_node.py`
   Discovers managed nodes, synchronizes shared parameter values, and handles save/load of snapshots.

## Local-First Startup

Normal startup does not require the server.

The expected sequence is:
- node constructs `Configurator`
- node calls `DeclareParameter` for each managed parameter it uses
- `Configurator` looks up the parameter in the schema
- the node's ROS parameter is declared with the schema default
- ROS launch or CLI param files may override that value
- node calls `validate()`
- `Configurator` validates the node's effective values against the schema
- node creates and passes `Configuration` objects to nested classes as needed

This means standalone node bringup is deterministic and does not depend on another process.

## Configuration Views

`Configuration` is intentionally small.

It is:
- named
- read-only
- bound to a specific subset of full parameter names
- backed by a live getter into the owning node

It is not:
- a cache
- a schema parser
- a parameter declaration mechanism
- a mutable parameter owner

Use it when nested classes need live parameter reads without depending on `Configurator`.

## Validation Model

Validation is shared between:
- the local node path
- the optional server path

The schema supports:
- exact type checking
- `constant` parameters
- numeric `min` and `max`
- string `options`
- expression-based bounds referencing other canonical parameter names

Examples:
- `/perception/pl_mapper/alive_cnt_low_thresh`
  max: `/perception/pl_mapper/alive_cnt_high_thresh-1`
- `/perception/pl_mapper/strict_min_point_dist`
  min: `/perception/pl_mapper/min_point_dist`

Validation happens:
- at declaration time
- at explicit startup validation
- during ROS runtime parameter updates
- in server-side operator workflows before fan-out

## Shared Parameters

There is no separate global transport at startup anymore.

Shared parameters are identified by canonical full names. If multiple nodes declare the same canonical key, they are treated as the same conceptual parameter.

That enables:
- one schema identity
- one snapshot key
- optional runtime convergence by the server

## Optional Server

The server is authoritative only while it is running.

Its job is:
- discover nodes that expose managed parameters
- query the standard ROS parameter services
- maintain server-side current values
- push updates to nodes
- transact an entire operator batch through a checksummed write-ahead journal
- compensate already-applied nodes if any update/readback/persistence step fails
- expose an explicit divergent fault if compensation cannot prove exact rollback
- resynchronize periodically
- save and load runtime snapshots as ROS parameter files

`tuning.py` owns content identities, the immutable session baseline, monotonic
revision, canonical append-only WAL, atomic checkpoints/selectors, idempotent
request replay, prepared-transaction recovery, and pending-boot confirmation.
The ROS server remains the adapter for schema validation, distributed mutation,
fresh readback, and active-set persistence. Baseline and current state are
separate authenticated documents and cannot overwrite one another.

It is not part of mandatory startup anymore.

## File Responsibilities

### Runtime schema

- `config/parameters/parameter_manifest.yaml`

### Profile selectors

- `config/profiles/real.yaml`
- `config/profiles/sim.yaml`

### Runtime value files

- `config/parameter_sets/real/tracked/default.yaml`
- `config/parameter_sets/sim/tracked/default.yaml`

### Immutable contract and reconciliation support

- `config/parameters/parameters.yaml`
- `config/parameters/parameter_manifest.yaml`
- `config/configuration_contract/`
- `iii_drone_configuration/installed_contracts.py`
- `iii_drone_configuration/reconciliation.py`
- `iii_drone_configuration/tuning.py`

`parameter_manifest.yaml` is captured into the installed contract and is never
propagated into writable state as a mutable schema copy. `parameters.yaml` remains
only as a declared compatibility artifact. The retired `scripts/install.sh` and
`scripts/update_installed_parameters.py` entry points fail with status 64 and name
the canonical `iii config sim` replacement; neither mutates files.
