# Architecture

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
- reject operator writes if a target node rejects them
- resynchronize periodically
- save and load runtime snapshots as ROS parameter files

It is not part of mandatory startup anymore.

## File Responsibilities

### Runtime schema

- `config/parameters/parameter_manifest.yaml`

### Runtime value files

- `config/ros_params_real.yaml`
- `config/ros_params_sim.yaml`

### Schema install/update support

- `config/parameters/parameters.yaml`
- `scripts/update_installed_parameters.py`
- `scripts/install.sh`

The last group is still used for install-time propagation and is not the preferred runtime source.
