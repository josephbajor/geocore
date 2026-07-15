# XT test fixtures

Provenance of every transmit file used by the `kxt` test suite.

None of the current external positive fixtures contains a true tolerant edge with
`EDGE.curve = null` and a per-fin trimmed SP-curve. That capability has standards-derived
self-round-trip coverage in `tests/write.rs`, but it remains intentionally uncertified
until an independently authored, redistributable positive fixture and a licensed
Parasolid-host round trip are added.

`manifest.tsv` is the machine-readable source of truth for provenance, schema, size,
feature tags, stable unsupported-capability code, and expected parse/reconstruct/checker/tessellation stage outcomes. The
`xt_inspect` binary emits the corresponding observed JSON Lines record and the corpus
test ratchets every row against the manifest. Expected failures are retained as explicit
regression targets rather than removed from the denominator.

`discovery.tsv` records the same observations for useful public candidates that are
*not copied into this repository*. No license was found in their source repository at the
pinned revision, so a public URL is evidence of availability, not redistribution
permission. Those rows are leads for permission requests/replacement and do not count as
the committed corpus.

## Hand-authored (this repository)

- `block.x_t`, `block.x_b` — a 0.2 × 0.3 × 0.4 m solid block, written by
  hand at exactly base schema 13006 (text and neutral binary) per the
  published *Parasolid XT Format Reference*. The topology mirrors
  `ktopo::make::block` (8 vertices, 12 line edges, 6 planar faces, solid +
  void regions, and the void-exterior shell listing the faces as
  front-faces). Regenerate with `gen_block.py` (committed alongside); the
  files are committed as stable fixtures and must not silently change.
- `offset_plane.x_t` — the canonical G4a writer output for a bounded
  2 × 1 m planar sheet at `z = 0.25`, represented as a `+0.25` true-normal
  offset along the sensed normal of the world XY plane. It retains all four
  authored fin pcurves as
  trimmed SP-curves. Tests pin the complete byte stream and its
  reconstruct/evaluate/check/tessellate/re-export behavior. This synthetic
  fixture validates the published base-13006 layout locally; it does not
  claim acceptance by a modern external Parasolid oracle.

The G5 transmitted-intersection positive is generated structurally in
`tests/intersection_chart.rs` from the canonical writer's unit block rather
than committed as another byte fixture. It replaces one exact line with a
trimmed, finite open plane/plane `INTERSECTION`, two affine CHART positions,
`L/?` endpoint LIMITs, and an `INTERSECTION_DATA(204)` record containing two
ordered `[u0,v0,u1,v1]` tuples. A separate embedded-schema wire test pins the
modern appended pointer and inserted-field layouts. The later certified
constant-normal `Offset(B-surface)/B-surface` rung covers the exemplar's first
such chart. Endpoint-only equal limits, finite-open/end-terminated `T/F`
charts, and the first finite-open direct B-surface/Plane chart with paired-null
interior Plane UVs are now certified. General closed limits, omissions on a
NURBS trace or chart endpoint, and other nullable chart-data forms remain
unsupported.

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

## Owner-contributed exports

- `exemplar.x_t` — a large organic production part (96 faces, 68
  B-surfaces, 44 offset surfaces, 110 intersection curves, tolerant
  edges with per-fin trimmed SP-curves) exported by the repository owner
  from their own Onshape project as Parasolid text (Parasolid 37.1.212,
  schema `SCH_3701212_37102_13006`, 2026-07-11). Parses fully through
  the embedded-schema mechanism. Its three clamped periodic/closed B-surface
  leaves reconstruct through the certified position/C1 seam contract, and its
  first constant-normal `Offset(B-surface)/B-surface` chart now retains the
  live offset root, signed distance, periodic NURBS basis, paired pcurves, and
  whole-range unit-normal proof. Records 1828 and 2008 each reuse one `H/?`
  limit pointer as both start and end. Their endpoint-only,
  single-periodic-axis payloads now certify: record 1828 advances untouched
  production reconstruction through exact v3 `115485725/20/10`
  Work/Items/Depth, while record 2008 is independently pinned by a focused
  payload transplant at `124040223/22/10`. Production v4 then certifies the
  first finite-open/end-terminated `T/F` record at `116396069/20/10`.
  Production v5 additionally certifies record 1252, a direct B-surface/Plane
  chart whose six interior Plane UV pairs are null and recovered by exact frame
  inversion, at `117478445/20/10`. V6 certifies native direct-Plane `SP_CURVE`
  node 30 and derives FACE 1195's vertex-less periodic ring domain at
  `208228426/22/10`; v7 recovers record 5089's paired-null interior Plane UV and
  proves its Plane/Offset(B-surface) carrier at `272430166/22/10`; and v8
  certifies record 1984 by endpoint-only nonperiodic NURBS source-boundary
  normalization at `315245660/22/10`. Production v9 reaches
  `323814492/22/10` by certifying record 5945's finite-open three-sample
  Offset(B-surface)/Offset(B-surface) chart with canonical clamped quadratic
  carrier and pcurves plus two independent original-source interval proofs. The
  v10 reaches `336759900/22/10` by certifying record 3819's finite-open
  four-sample dual-offset chart with unique degree-3 clamped carrier and
  pcurves plus two independent original-source interval proofs. Historical
  v1-v9 profiles remain exact, and the isolated chart pins
  `12945408/4/10`. V11 accepts knot-set padding only when multiplicity zero is
  paired with null or a finite number. It certifies quadratic record 3790 at
  isolated `8593408/3/10`, then the exposed 11-sample Plane/Offset record 3745
  at isolated `42772491/11/10`, reaching `388125799/22/10` with historical
  v1-v10 parity. V12 certifies seven-sample dual-offset record 3615 with one
  common degree-1 open-clamped carrier/pcurve polyline and two independent
  original-source proofs. Its isolated cost is `26443776/7/10`; the corpus
  reaches `414569575/22/10` with historical v1-v11 parity. The separately
  transplanted two-sample dual-offset record 3595 now certifies as the exact
  common open-clamped line at isolated `4352000/2/10`, with residuals
  `[3.468467250779673e-5, 3.384554176162513e-5]`; it is not reached by the
  production traversal. Five-sample dual-offset record 4230, roots
  `[3320, 773]`, chart 4231, independently certifies as a common degree-1
  open-clamped polyline at exact isolated `17285120/5/10`. V13 admits that proof
  at exact `431854695/22/10` with historical v1-v12 parity. V14 admits two-sample
  direct Plane/Offset(B-surface) record 3609, chart 3607, at isolated `4277250`
  Work and exact cumulative `436131945/22/10`, then stops before two-sample
  dual-offset record 6044, chart 6043, whose isolated `4352000` Work would
  request cumulative `440483945`.
  The exemplar manifest therefore records a structured reconstruction failure
  with no unsupported capability. General closed limits, other nullable chart
  data, periodic-trace-range, broader carrier families, and noncanonical chart
  forms remain unsupported. Primary
  reference for modern writer conventions: TRIMMED_CURVE/GEOMETRIC_OWNER linkage,
  tolerant-edge fin curves, POINT ownership by vertex, and the resolved
  37102 node layouts (133/141 match base 13006 exactly).
- `cyl.x_t` — a plain solid cylinder exported by the repository owner
  from Onshape as Parasolid text (Parasolid 37.1.212, schema
  `SCH_3701212_37102_13006`, 2026-07-11). The only real file in the
  corpus with an analytic periodic wall face: two vertex-less ring
  edges bounding a two-loop CYLINDER face, no SP-curves, no vertices
  anywhere in the body. Settled the solid-cylinder emission questions:
  full-circle edges carry no vertex, and `EDGE.fin` points at the
  positive-sense fin (as in every modern real file). Passes every
  local stage.
- `solid_cone_onshape_reexport.x_t` and
  `solid_block_tolerant_edge_onshape_reexport.x_t` — Onshape's PS-37
  re-exports of this repository's own accepted fixtures, captured by the
  automated there-and-back loop (2026-07-11). Both parse cleanly and both
  fail reconstruction with checked-topology-commit invariant faults (1 and
  2 respectively) — the first real reader-gap regression fixtures produced
  by the compare leg. The tolerant re-export retains one curve-less
  tolerant edge with per-fin trimmed SP chains.

Metadata-only discoveries from SCOREC/pumi-meshes at commit `684e480`:

- `Kova_nat.x_t` — curved analytic solid written by the Parasolid acceptance tests
  (V31); currently passes every local stage.
- `annular.x_t` — Unigraphics annular solid (V26); currently passes every local stage.
- `blend_nat.x_t` — post-blend analytic solid (V27); currently passes every local stage.
- `fichera.x_t` — Unigraphics Fichera-corner solid (V26); currently passes every stage.
- `crack_nat.x_t` — curved multi-loop crack model (V27); exposed and now guards the rule
  that an X_T face's outer loop need not be first in its loop chain.
- `sph_vertical_slice_nat.x_t` — spherical vertical slice (V27); exposed and now guards
  the distinct bipolar-boundary tessellation case where one ring passes through both
  sphere poles and longitude winding is not a valid cap classifier.
- `upright.x_t` — 1,501-node SolidWorks 2013 / Parasolid 24.1 production-style,
  multi-loop part; now passes parse, reconstruction, checker, and tessellation stages.
- `model_nat.x_t` — V28 general body retained as a stable unsupported-capability case.

No repository license file or per-file license statement was present in pumi-meshes at
the pinned commit. Their bytes are therefore deliberately absent here. The URLs and
observed results are retained in `discovery.tsv`; permission must be confirmed or
licensed replacements found before any of them enters the committed corpus.
