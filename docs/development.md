# Development Notes

## Recommended Usage Pattern For New Nodes

For a new managed node:

1. Construct `Configurator`.
2. Declare each managed parameter explicitly in the node.
3. Call `validate()` once configuration is complete.
4. Create one or more `Configuration` views only when nested classes need live access.
5. Pass `Configuration` objects, not `Configurator`, to deep subsystems.

This keeps:
- top-level ownership explicit
- deep classes decoupled
- parameter identity canonical

## What Not To Reintroduce

Avoid:
- central startup registration requirements
- per-node manifest files describing parameter subsets
- parameter alias/remap layers
- cached parameter bundles with stale values
- local code defaults that bypass the central schema

## Testing Guidance

Good tests for this package or its consumers include:
- declaring a schema-managed parameter with the correct type
- failing declaration on schema/type mismatch
- accepting valid startup overrides from ROS param files
- rejecting invalid runtime updates
- verifying `Configuration` returns updated values after node parameter changes
- verifying server reconciliation of late-joining nodes

## Synchronization Expectations

Node-side rules:
- nodes always validate locally
- constant parameters reject runtime changes
- invalid values reject at the ROS parameter callback level

Server-side rules:
- server uses the same schema/validation logic
- if a node rejects a pushed update, the server should reject the operator action as well
- server periodically reconciles and cleans up offline nodes

## Cleanup Candidates

The following are known candidates for future cleanup:
- unify remaining Python and C++ validation implementations further
- modernize or replace the terminal client UX
- remove `config/parameters/parameters.yaml` once any remaining external compatibility need is gone
- add automated tests specifically for schema path resolution and snapshot save/load
