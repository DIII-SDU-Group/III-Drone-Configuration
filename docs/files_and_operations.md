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
Snapshots live alongside tracked files in the writable runtime tree at
`$CONFIG_BASE_DIR/iii_drone/parameter_sets/<profile>/snapshots/`, including on
the Pi under `/home/iii/.config/iii_drone`.

### Installed Contract And Writable Reconciliation

- `config/parameters/parameters.yaml`
- `config/parameters/parameter_manifest.yaml`
- `config/configuration_contract/`
- `iii_drone_configuration/installed_contracts.py`
- `iii_drone_configuration/reconciliation.py`

Build captures these inputs and the tracked defaults into an authenticated ament
package-share contract. Reconciliation preserves valid living values, adds new
defaults, retires removed keys into target/profile/manifest/set-scoped shadow
records, and blocks reintroduced keys for explicit review. It normalizes every
selected and unselected set; snapshots never become restoration candidates unless
they were selected at the exact retirement boundary.

Legacy installer scripts are retained only as deterministic failure shims. Use:

- `iii config sim inspect`
- `iii config sim checkpoint`
- `iii config sim review --decision ...`
- confirmed `iii config sim reset [--restore CHECKPOINT_ID]`

Aircraft reconciliation is not exposed as a writable runtime command. Use
`iii deploy activate`; if a review is returned, continue it with
`iii deploy continue OPERATION_ID --decision ...`.

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

### Save A Simulation Checkpoint

`iii config sim checkpoint` seals the complete living sim tree, including
selectors, every set, shadows, state binding, and retained old contracts, under
the clone-local Git-ignored `.iii/configuration-checkpoints/` root. Repeated
identical checkpoints deduplicate by content identity.

### Reset Simulation State

Confirmed `iii config sim reset` first seals a recoverable checkpoint and then
recreates the living tree from the currently installed immutable tracked default.
`--restore CHECKPOINT_ID` restores an authenticated clone-local checkpoint. Neither
operation writes the source default.

### Save A Runtime Snapshot

When the configuration server is running, it can save the current converged parameter state to a full parameter-set file.

Operator parameter changes through `set_parameter_from_gc` also maintain an automatic runtime snapshot named `runtime_parameters_<timestamp>.yaml`. The server updates this file after each successful live change, points the active profile selector at it, and deletes the previous automatic runtime snapshot from the same server run. If values return to the boot configuration, the automatic runtime snapshot is removed and the selector is restored to the boot-selected parameter set.

An Apply is acknowledged only after its canonical WAL commit and active-set
replacement are file- and directory-synced. Request IDs make response-loss retry
idempotent; expected revisions reject a stale UI. Restart-required edits are
present in persisted state and `pending_boot_values`, but not active state, until
either a full managed stop/start or `system restart --cold` obtains fresh exact
readbacks. Warm or partial node restart cannot clear this indication.

Real-host tuning evidence lives at `/var/lib/iii/tuning`; simulation uses
`$WORKSPACE_DIR/.iii/tuning`. The directories contain selectors plus
content-addressed session baselines, state, checksummed JSONL WALs, and revision
checkpoints. Do not edit these files directly. A divergent status means automatic
compensation failed and all configuration writes remain blocked until the exact
observed state is reconciled.
Full-graph reconciliation retries the exact prior durable state; it clears the
fault only after every affected node, the active file, and pending state match.
Failed retries leave the fault and write block intact.

### Load A Snapshot

The server loads a saved parameter set through the same validate-all, journaled,
multi-node transaction used by GUI Apply. Loading never mutates the source
snapshot and does not bypass restart-required or divergent-state policy.

### Resolve The Active Schema Path

Schema helper:
- `iii_drone_configuration/schema_utils.py`

Active parameter-set resolution order:

1. `III_SYSTEM_PARAMETER_FILE`, if explicitly set
2. `SIMULATION` selects `sim` or `real`
3. `$CONFIG_BASE_DIR/iii_drone/profiles/<profile>.yaml` chooses the active parameter set
4. That selector resolves into `$CONFIG_BASE_DIR/iii_drone/parameter_sets/<profile>/<reference>`

## Current Caveats

- `config/parameters/parameters.yaml` still exists as a compatibility artifact and should not be treated as the schema source of truth.
- The service layer still uses `default_parameter_file` naming in a few message fields for compatibility, even though the runtime model is now selector-driven parameter sets.
- The package has both C++ and Python validation implementations; they are conceptually aligned, but maintenance should keep them in sync.
- `opti_track` shares the immutable `real` default and reconciles an independent
  writable selector for the reduced-mode OptiTrack lab profile (no cable,
  payload, or perception; PX4 takes its pose from motion capture through the
  pose relay, configured by the boot-only `/opti_track/pose_relay/*`
  parameters). `hil` shares the `sim` baseline and reconciles an independent
  writable selector for split-host bench operation.

Tracked defaults are source-owned developer inputs. Normal live tuning remains
in the writable profile tree; copy a value back to the source default only when
you deliberately want it to become the next developer baseline.
