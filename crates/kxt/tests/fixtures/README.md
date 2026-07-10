# XT test fixtures

Provenance of every transmit file used by the `kxt` test suite.

None of the current external positive fixtures contains a true tolerant edge with
`EDGE.curve = null` and a per-fin trimmed SP-curve. That capability has standards-derived
self-round-trip coverage in `tests/write.rs`, but it remains intentionally uncertified
until an independently authored, redistributable positive fixture and a licensed
Parasolid-host round trip are added.

## Hand-authored (this repository)

- `block.x_t`, `block.x_b` — a 0.2 × 0.3 × 0.4 m solid block, written by
  hand at exactly base schema 13006 (text and neutral binary) per the
  published *Parasolid XT Format Reference*. The topology mirrors
  `ktopo::make::block` (8 vertices, 12 line edges, 6 planar faces, solid +
  void regions, and the void-exterior shell listing the faces as
  front-faces). Regenerate with `gen_block.py` (committed alongside); the
  files are committed as stable fixtures and must not silently change.

## Downloaded (public GitHub repositories)

Real-world files written by Parasolid-based applications; committed as
small test fixtures with their sources recorded here. They are trivial
geometric primitives / test parts from public repositories; no license
statement accompanied the individual files.

- `sphere.x_t` — cut solid sphere (one spherical face, one planar face,
  one ring edge; V27, schema `SCH_2700142_26105_13006`, `USFLD_SIZE=1`).
  From SCOREC/core (github.com/SCOREC/core,
  `python_wrappers/input/sphere.x_t`, commit `395d3a2`).
- `disk_nat.x_t` — planar sheet disk (V27, embedded schema, base 13006).
  From SCOREC/pumi-meshes (github.com/SCOREC/pumi-meshes,
  `disk/disk_nat.x_t`, commit `684e480`).
- `plate.x_t` — solid plate (V28, schema `SCH_2800180_28002_13006`).
  From SCOREC/pumi-meshes (`faceExtrusion/plate.x_t`, commit `684e480`).
- `longbar.x_t` — solid bar written by Parasolid V10 (schema
  `SCH_1000230_10004`, predating base schema 13006). Kept as the
  *negative* fixture: the Tier-0 reader must reject it with
  `UnsupportedSchema`. From ansys/example-data
  (github.com/ansys/example-data, `pymechanical/embedding/LONGBAR.x_t`,
  commit `f4582f1`; that repository is MIT-licensed).
