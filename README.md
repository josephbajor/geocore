# cad_prototype

An open, performant B-rep modeling kernel built for interoperability with
Parasolid-based CAD systems (SolidWorks, Solid Edge, NX, Onshape) via XT
round-trip. It is the geometry and topology foundation for an eventual full
parametric CAD application; feature history and regeneration are later layers.

- **Specification:** [docs/kernel-spec.md](docs/kernel-spec.md)
- **Construction roadmap:** [docs/kernel-roadmap.md](docs/kernel-roadmap.md)
- **Machine-readable capability ledger:** [docs/kernel-support.tsv](docs/kernel-support.tsv)
- **Licensed-host certification loop:** [docs/oracle-loop.md](docs/oracle-loop.md) — automated via `scripts/oracle_loop.py`

## Layout

Application and product code should start at the `kernel` facade. The numbered
lower layers remain available for kernel implementation and trusted adapters,
but their raw storage and assembly APIs are not the application compatibility
boundary.

| Crate | Layer | Contents |
|---|---|---|
| [`crates/kcore`](crates/kcore) | L0 foundations | Deterministic robust `orient2d`, `orient3d`, positive-inside-CCW `incircle`, exact cyclic `polygon_orientation2d`, bounded exact quadratic/harmonic root classification, and exact affine or squared-distance signs with conservative floating filters or exact expansion evaluation, interval filters, tolerance policy (Parasolid numeric regime), typed errors, generational entity arenas with copy-on-write undo frames, deterministic parallel primitives, deterministic transcendental math (musl port — platform libm is banned in kernel code via clippy `disallowed-methods`) |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse), true 2D line/circle/NURBS pcurve evaluators with fail-open original-source affine ranges, and analytic surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller) with homogeneous 2D/3D knot operations and conservative active-subrange control-hull boxes, closest-point projection, deterministic trimmed-face tessellation with exact streaming trim-loop winding and explicit refinement-limit errors, evaluator conformance harness |
| [`crates/kgraph`](crates/kgraph) | L1.5 geometry graph | Immutable analytic, NURBS, and procedural geometry nodes with typed dependencies, deterministic identity, and bounded evaluation |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex), finite conservative face UV domains, typed entity-tolerance provenance and transaction-owned growth budgets, independent per-fin pcurves, bounded curve-less tolerant edges, reusable validated polygon-with-holes profiles and checked prism extrusion, transaction-owned pcurve-aware Euler edits, private generic Store mutation with transaction-scoped checked assembly, deterministic mutation/lineage/tolerance journals, journal-returning checked solid/sheet/wire/acorn constructors, shared incidence validation, and pcurve-driven watertight tessellation with source-certified periodic side-loop ordering |
| [`crates/kops`](crates/kops) | L3 operations | Provisional M4 intersection foundation: exact analytic special cases plus early sampled NURBS curve/curve, curve/surface, and surface/surface experiments; generic completeness and boolean-ready pcurve results remain gated |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic modern-schema Parasolid XT (`.x_t`/`.x_b`) import for the supported geometry subset, including zero-multiplicity null-knot normalization and proof-bearing transmitted intersection charts through bounded two-sample line, three-sample quadratic, four-sample cubic, five-sample polyline, and seven-sample polyline dual-offset NURBS slices, plus a deterministic schema-13006 text writer for self-authored analytic solids, sheets, wires, acorns, and bounded tolerant edges encoded as trimmed SP-curves over 2D B-curves (clean-room from the published XT Format Reference) |
| [`crates/kernel`](crates/kernel) | Supported native facade | Session/part lifecycle, opaque semantic IDs and views, contextual block/profile-extrusion construction, deterministic rigid body copy, checking/evaluation/tessellation, and typed X_T import/export without raw topology or graph leakage |
| [`examples/kernel-lifecycle`](examples/kernel-lifecycle) | Facade-only client | Executable application lifecycle with `kernel` as its only direct kernel dependency |

### Facade quickstart

Run the facade-only lifecycle client and write its deterministic X_T result:

```sh
cargo run -p kernel-lifecycle -- target/kernel-lifecycle.x_t
```

The client constructs a block, traverses semantic faces/edges/vertices, checks
and tessellates the body, evaluates one supporting surface, and exports through
the `kernel` API. Lower-layer X_T reconstruction and oracle tools remain
trusted adapter and conformance seams; they are not examples of the ordinary
application boundary.

## Current Status

- M0 foundations now include deterministic exact-fallback `orient2d`, `orient3d`,
  and `incircle`; M1 geometry and M2 topology/primitives also have implemented
  alpha slices. The public `polygon_orientation2d` slice and its non-copying
  streaming companion, `polygon_orientation2d_iter`, compute the exact cyclic
  shoelace expansion. Fewer than three vertices, any non-finite coordinate,
  and an exactly zero expansion
  return `Orientation::Zero`; cyclic rotation preserves the result, reversal
  flips every nonzero result, and repeated vertices are allowed under algebraic
  area semantics. Evidence includes 20,000 random integer polygons against an
  `i128` oracle, a `2^52`-translated unit square whose naive shoelace sum is
  zero, and the cross-platform determinism golden. `incircle` remains a public
  predicate with conformance evidence but has no production topology decision
  consumer yet. Strict first-chart SSI
  polygon convexity is the first audited
  exact decision consumer: it accepts only at least three finite vertices with
  `orient2d(...) == Orientation::Positive` at every turn, so exact collinear
  and all other nonpositive turns fail closed. Oblique profile extrusion is the
  next migrated consumer: `extrude_profile_along_in` classifies the stored-frame
  `(x, y, translation)` scalar triple with exact `orient3d`, rejects exact
  coplanarity before allocation, and reflects the chart for a negative result.
  Its integer-source adversary has normal dot `+3` while the former normalized
  `translation.dot(frame.z())` rounds to zero; both directions now construct
  deterministically, while ordinary oblique extrusion remains Full-valid.
  Coincident bounded Plane/Plane window construction now also uses exact
  `orient2d` in both monotone hull chains: only exact `Positive` turns are
  retained, while `Zero` and `Negative` turns pop. A private production-helper
  fixture near `2^52` has exact `i128` determinant `+1` where the former rounded
  turn is zero, and pins middle-vertex retention, repeat, input
  rotation/reversal, and exact-collinear removal. Rectangle-overlap boundaries
  draw consecutive directions from two axis pairs, so that cancellation triple
  cannot be encoded as consecutive exact public window edges; the public seam
  instead pins an eight-vertex `Complete` Region, cyclic exact-positive turns,
  repeatability, operand-swap semantic parity, and non-finite range rejection.
  `kgeom` trim cleaning plus `TrimmedSurface` outer/hole winding now consume the
  streaming exact sign without allocating a coordinate copy; rounded
  `TrimLoop::signed_area` is reporting only. `kops` polygonal-region
  canonicalization now compares exact winding in both parameter charts to
  derive `Same` or `Reversed`, rejects zero or a contradictory declared chart
  relation, and normalizes a negative first chart by reversal. Its
  integer-source adversary has exact doubled area `+2` where the former
  origin-relative floating sum is zero. `ktopo`
  `face_case_a` now requires exactly one exact-positive outer loop; exact-zero
  and non-finite loops fail closed before the existing periodic anchoring and
  outer-first ordering. Its `2^52`-translated unit-square adversary also has
  exact doubled area `+2` while naive shoelace summation is zero. The `ktopo`
  periodic side-face path no longer orders its `+u` and `-u` winding loops by
  sampled mean `v` when pcurves exist. `Curve2d::source_affine_range` supplies
  conservative authored-data ranges for Line2d, Circle2d, and active original
  positive-weight NurbsCurve2d controls. After pcurve-definition validation,
  complete all-pcurve loops on non-`v`-periodic charts union those ranges and
  require strict `top.lo > bottom.hi`. Certified reversal is invalid geometry;
  overlapping or unavailable proof returns capability
  `ktopo.tessellation.periodic-loop-vertical-separation`. The sampled mean
  remains only when both complete loops are truly pcurve-less. Mixed coverage,
  vertical periodicity, nonzero vertical chart shifts, and unsupported pcurve
  classes fail open. A degree-five NURBS whose five seed samples all have
  `v = 1` but whose original source reaches `v = -41/64` now stops
  deterministically instead of being accepted above a `v = 0` loop. The
  `ktopo` checker now has a separate strict authority for planar straight-loop
  orientation: whole-interval-certified line uses must form finite, nonzero,
  bit-identically closed, exactly simple rings with nonzero
  `polygon_orientation2d_iter` signs, and robust strict containment identifies
  the unique outer without an area-magnitude proxy. The former sampled
  shoelace, largest-absolute-area selection, and periodic sample unwrapping no
  longer decide faults. Fast emits `WrongLoopOrientation` only when the complete
  layout is certified; curved, periodic, nonlinear-chart, tolerance-joined,
  exact-zero, and non-finite loops remain silent in Fast and receive per-loop
  Full `LoopOrientation` gaps, while unresolved outer/hole roles remain the
  separate `LoopContainment` obligation. Bounded exact conic root-count
  classification is also landed. `quadratic_discriminant` interval-certifies
  ordinary `b² - 4ac` signs and uses normal-range expansions when cancellation
  defeats the filter; non-finite or out-of-envelope fallback cases return
  `None`, never a guessed miss. Shared `harmonic_half_angle_roots` applies an
  exactly reversible power-of-two normalization, classifies the number of
  roots from the original
  `cosine² + sine² - constant²` identity rather than the rounded transformed
  quadratic, and represents the exact projective `t = π` root separately. The
  ordinary positive path retains the prior quadratic-formula bits and ordering;
  exact-positive fallback uses stable `q` construction. Every `kops` conic and
  planar-conic consumer turns classification failure into `Indeterminate`,
  while `kgraph` exposes the typed
  `IntersectionCertificateError::HarmonicRootClassification`. Evidence pins
  exact positive/zero/negative quadratic cancellation, a `2^52` harmonic
  cancellation, a transformed-sign-mismatch bit pattern, two transverse contacts for
  `cos(t) + 0.991 = 0` despite a `0.01` metric tolerance, `2^±700` scaling,
  ordinary repeat bits, and the debug/release numeric golden.
  Planar circle/ellipse-by-plane and circle-by-sphere containment no longer
  treats a sub-tolerance harmonic amplitude as identity. Exact source signs
  distinguish identity, a nonzero constant complete miss, and a general
  harmonic before overlap is emitted. Plane classification respects the
  semantic orthonormal `Frame` contract for identical or opposite stored
  normals and otherwise uses exact affine signs; circle/sphere uses exact
  center-axis affine signs plus the exact squared-distance/radius predicate
  below. General root locations still use the computed finite harmonic
  coefficients, and source-general relations erased to a rounded identity
  remain `Indeterminate`. The `kops` periodic harmonic adapter uses parameter
  tolerance only to admit or clamp a representative: it no longer merges
  distinct exactly classified roots, and an exact numeric-parameter collision
  fails closed. Its contained-conic window partition also retains
  coefficient/root-index provenance at every cut. Distinct numeric roots remain
  separate even below parameter tolerance, different equations may share an
  exact chart corner, and two roots of one harmonic at the same numeric cut
  return `Indeterminate`. Only an exactly zero-width curve range collapses to a
  point; bit-exact full-period surface axes contribute one canonical seam
  equation while near-full windows retain both bounds. Final contacts can still
  merge within model-space linear tolerance as the intentional physical
  coincidence policy. The `kgraph` oblique spherical seam certifier now checks
  source-derived harmonic coefficient signs against the finite coefficients,
  retains per-root provenance and exact derivative direction, proves seam side
  with bounded algebraic signs, preserves distinct close roots, and fails with
  typed evidence on unrepresentable collisions, singular charts, or ranges
  wider than one period. `IntersectionCertificateError` now owns stable
  class/code/capability metadata for every certificate failure. The graph
  surface adapter preserves its published aggregate branch-certificate code
  and adapter-owned class while retaining the exact leaf error through
  `source()`. Rigid body copy exposes a typed
  `copy_body_rigid_with_source` path whose `BodyCopyError` survives all six
  certificate reissuers and `KernelError::BodyCopy`, remains paired with the
  operation report, and rolls the transaction back atomically; the historical
  `copy_body_rigid` entry remains a compatibility `kcore::Result` wrapper.
  Analytic
  surface/surface solvers that construct a circle already proved to lie on a
  sphere preserve that construction proof through a circle-only sphere-window
  clipping seam. Plane/Sphere likewise preserves its plane proof. These seams
  do not ask a re-normalized carrier frame or rounded square-root radius to
  reproduce identities already owned by the construction.
  `affine_dot3(normal, point, origin, bias)` now interval-certifies
  `normal · (point - origin) + bias` without making rounded subtraction a
  decision authority. Cancellation falls back to an exact expansion sum of the
  six products `normal[i] * point[i]` and `-normal[i] * origin[i]`, followed by
  the exact bias; non-finite input or fallback outside the conservative
  exact-product/expansion envelope returns `None`. Evidence covers a 20,000-case `i128`
  oracle, ordinary approximation bits, exact zero, raw-zero oblique
  cancellation, threshold reversal, a subtraction residue lost by rounded
  `point - origin`, and forced-fallback participation in the reviewed
  debug/release golden.
  `squared_distance_difference3(point, origin, first_radius, second_radius)`
  similarly classifies
  `|point - origin|² + first_radius² - second_radius²` without making a rounded
  coordinate difference authoritative. Its interval filter expands each axis
  as `point² - 2·point·origin + origin²`; cancellation falls back to exact
  expansions of those nine products and the two squared radii. Evidence covers
  a separate 20,000-case `i128` oracle, ordinary approximation bits, exact
  3-4-5 zero, radius threshold/reversal signs, and an axial `5e-9` offset whose
  legacy result is zero but exact sign is positive. The current cross-platform
  debug/release numeric golden is `0xEED3_9864_73A4_C2D2`.
  Bounded line/Torus intersection now constructs its source-world quartic with
  checked exact expansions and isolates distinct roots with a bounded signed
  pseudo-Sturm chain. For nonidentity auxiliary polynomials, a second exact
  squared-stationarity quartic with an unsquared stationarity interval filter,
  the radial-axis quadratic, and both line endpoints cover tolerance-near
  extrema. Partial torus windows, auxiliary identity cases, unsafe arithmetic
  envelopes, unit/orthonormal invariant failures caught by defensive solver
  checks, unrepresentable root separation, and tolerance-only discoveries
  remain `Indeterminate`; rounded local roots are discovery-only. Distinct
  algebraic roots remain separate in the isolator, while the final
  physical-contact list still uses world-space linear-resolution consolidation.
  `Vec3::normalized` now rejects non-finite components, preserves the established
  ordinary finite-norm bits, and rescales only when squared-length overflow would
  otherwise collapse a finite direction. `Line::new` inherits that contract.
  `Frame::new` and `Frame::from_z` preserve their ordinary valid construction
  bits and use a homogeneous cross/cross projection only when the ordinary
  candidate cannot satisfy the orthonormal-frame contract, with scale-aware
  degeneracy rejection and a final orthonormality gate. The existing
  linear-resolution floor is unchanged. Evidence covers power-of-two and
  near-maximum directions, projection overflow, huge parallel and near-parallel
  hints, the unchanged geometry determinism golden in debug and release,
  line/line bit parity, and direct/generic line/Torus parity.
  Remaining
  concrete decision-audit debt includes generic curved-pcurve signed line
  integrals and broader curved or periodic containment beyond the landed
  vertical-separation slice, the outer amplitude metric
  policy in the still-unmigrated higher conic/primitive families, affine,
  squared-distance, and harmonic fallback outside their reviewed exponent
  envelopes, full source-exact harmonic discriminant construction beyond the
  landed coefficient sign/zero agreement, higher-polynomial root and window
  classification beyond the bounded line/Torus quartic slice, general NURBS
  root classification, raw extreme-scale `Vec3`
  `norm`/`norm_sq`/dot/cross/distance/subtraction behavior, overflow-safe 2D
  direction normalization for `Line2d` and `Circle2d` after choosing zero-only
  versus linear-resolution-floor semantics, and other raw topological sign
  branches.
  `insphere`, an `incircle` production decision consumer when required, the
  broader topological-decision audit, and full conformance remain ahead.
- M2.5 is in progress and remains the architecture gate. Transaction-owned checked
  topology, pcurve-aware Euler edits, deterministic journals, tolerance provenance,
  bounded operation contexts, `Fast`/`Full` checking, adaptive face-domain proofs, and
  the immutable geometry graph with persistent verified intersection descriptors have
  landed. The Q2 production-clean v2 ladder now pins exact ordinary-commit phase
  boundaries at 4/16/64/256 production solids while retaining equal store/index
  snapshots and the prior 39 Q2 output digests. General NURBS/mixed-parameter
  incidence, periodic and unsupported mixed-boundary containment, curved-loop/shell
  proofs, operation-specific tolerance rules, phase optimization and full-rebuild
  phase instrumentation, broader heterogeneous production edits, and production
  assembly benchmarks remain.
- M3 X_T interchange is in progress. Modern text and neutral-binary reading, atomic
  reconstruction, deterministic analytic text writing, bounded tolerant-edge SP-curves,
  safe offset surfaces, certified clamped periodic/closed B-surfaces, canonical
  transmitted Plane/Offset/NURBS intersection-chart slices, and bounded
  noncanonical affine direct-Plane/B-surface, safe-Offset(Plane)/B-surface,
  direct-Plane/Offset(B-surface), direct constant-normal Offset(B-surface)/direct
  B-surface, independent direct one-descriptor Offset(B-surface)/Offset(B-surface),
  and direct-B-surface/B-surface slices are implemented. The noncanonical direct
  offset slices are limited to two through five finite-open samples, preserve
  ordered roots and paired UVs, and cover polynomial or rational bases and
  operand swap at exact `14336/28672/43008/57344` Work, `N` Items, and Depth
  10. Nested, shared-basis, multi-offset, null/mixed, and out-of-range
  noncanonical forms remain unsupported. The committed
  corpus includes a production 7,423-node Onshape part and reconstructs under the exact v15
  profile at `440483945/22/10` Work/Items/Depth. Two-sample dual-offset record 3595
  certifies independently; v13 admits five-sample record 4230 at isolated
  `17285120/5/10`, v14 admits two-sample Plane/Offset record 3609 at isolated
  `4277250/2/10`, and v15 admits two-sample dual-offset record 6044 at isolated
  `4352000/2/10`. Production next stops atomically before four-sample dual-offset
  record 5921 at its exact `454258793`-Work request. At that attempted budget,
  record 5921 still fails closed: its canonical cubic first pcurve materially
  leaves the original open nonperiodic source domain, so the retained report
  remains the exact v15 prefix with an empty rollback. Broader cyclic
  B-geometry, periodic/circular pcurves,
  remaining intersection/procedural families, assemblies, older schemas, neutral-binary
  writing, and broader external Parasolid certification remain.
- M4 intersections are useful but provisional. Shared `Complete`/`Indeterminate`
  evidence, source-provenanced adaptive NURBS covers, interval implicit exclusion,
  Work-bounded polishing, exact algebraic seed/overlap certificates, selected paired
  pcurves, and a bounded clamped NURBS/Plane sign migration have landed. For
  NURBS/Plane, homogeneous range evaluation and exact affine signs over the
  active original Euclidean controls are the only authority for plane-slab
  sides/containment and asymmetric plane-`u`/plane-`v` window bands. Rounded
  restricted, Bezier-extracted, and split controls guide subdivision and sign
  variation only; original-source samples and bisection use `affine_dot3`, and
  emitted points still pass the shared residual admission. Inconclusive
  `Candidate` leaves and the static depth-72/node-65,536 root and clipping caps
  cannot upgrade completion: every result remains diagnostic `Indeterminate`.
  Overlaps merge only across actual overlap or bit-exact parameter contact, not
  across a tolerance-sized gap. Evidence pins the ordinary crossing at exact
  `t = 0.5` with repeat-bit parity, three legacy-failing adversaries—oblique
  rounded-dot false overlap with zero-range rejection and curve/normal
  reversal, rounded split controls losing an exact midpoint plane contact, and
  rounded split controls hiding a plane-window excursion—and separated,
  touching, and nested overlap-merge behavior.
  Complete root isolation, contextual rather than static node/depth budgets,
  proof of unresolved UV-boundary cells, broader NURBS/higher-polynomial roots
  beyond the bounded line/Torus quartic, and affine fallback outside the
  reviewed exponent envelope remain open.
  A first exact varying-normal Offset(NURBS) arm with global-X-,
  global-Y-, and global-Z-normal planar-NURBS, analytic-Plane, or one-descriptor
  safe-Offset(Plane) peers, plus the complete one- through four-descriptor
  rational-quarter-cylinder family against direct global-axis analytic Planes
  only, and bounded
  coincident
  Plane/Cylinder/Sphere/Cone/Torus regions including exact polar-cap and
  same-row adjacent, exact same-column vertical (reviewed at
  `[0,2]`/`[1,2]`), either exact full latitude row, and an exact mixed-axis
  three-cell L path in two real orientations and the generic exact four-
  positive shared-seam path family in two real orientations, plus the disjoint
  exact lower-stem `[0,0]`/`[0,1]`/`[0,2]`/`[1,1]` and upper-stem
  `[0,1]`/`[1,0]`/`[1,1]`/`[1,2]` T trees and the exact left
  `[0,0]`/`[0,1]`/`[1,0]`/`[1,1]` and right
  `[0,1]`/`[0,2]`/`[1,1]`/`[1,2]` four-positive 2×2 cycle family with two
  certified-empty siblings, plus the exact disconnected outer-column vertical
  pairs `[0,0]`/`[0,2]`/`[1,0]`/`[1,2]` with middle-column `[0,1]`/`[1,1]`
  certified empty, all four exact disconnected isolated-corner plus three-cell
  mixed-axis L layouts—`[0,0]` + `[0,2]`/`[1,1]`/`[1,2]`, `[1,2]` +
  `[0,0]`/`[0,1]`/`[1,0]`, `[1,0]` + `[0,1]`/`[0,2]`/`[1,2]`, and
  `[0,2]` + `[0,0]`/`[1,0]`/`[1,1]`—with their respective two omitted
  graph-cut siblings certified empty, and exactly five positive cells with one
  certified-empty sibling, in the polar-by-wide family have landed. The
  connected four-positive routes are disjoint by degree sequence: path
  `2,2,1,1`, T `3,1,1,1`, and cycle `2,2,2,2`. Each T certifies the other two
  siblings empty,
  simultaneously proves and removes exactly three reverse-oriented bit-exact
  seams, requires one outer cycle with no artificial seam, and restores the
  parent map and maximum child/parent residual. Its real fixtures pin
  repeat/swap while one-ULP mutation and duplicate-edge ambiguity fail closed.
  The cycle route simultaneously proves and removes exactly four reverse-
  oriented bit-exact adjacencies, requires one outer cycle with no artificial
  seam, and restores the parent map and maximum child/parent residual. Its real
  fixtures pin repeat/swap while one-ULP mutation and duplicate-edge ambiguity
  fail closed. The outer-column disconnected arm merges both exact latitude
  seams into exactly two canonical regions, excludes both longitude separators,
  restores parent maps and maximum child/parent residuals, and pins repeat/swap
  plus one-ULP/ambiguity rejection. Each singleton-plus-L arm proves and removes
  both reverse-oriented bit-exact L seams, requires zero occupied-boundary contact
  with either empty cut separator and no bit-exact contact between the singleton
  and merged component, and likewise returns exactly two canonical regions with
  restored parent maps and maximum child/parent residuals. Together these
  routes exhaust the exact disconnected four-positive graph layouts in the 2×3
  decomposition, with repeat/swap, exact 6/5 piece, 147/146 pair, and 588/587
  arc N/N-1 evidence plus one-ULP/ambiguity rejection. The five-positive arm
  simultaneously removes every
  internal reverse-oriented
  bit-exact seam and requires one outer cycle; corner-empty cycle-plus-tail and
  edge-middle-empty tree fixtures pin repeat/swap, parent mapping, residuals,
  and one-ULP/ambiguity rejection. The all-six-positive arm now admits the
  complete 2×3 grid with no empty sibling only after all seven reverse bit-
  exact internal adjacencies are removed simultaneously, leaving one
  unambiguous outer cycle and no artificial seam edge. Its real fixture pins
  parent mapping, residuals, repeat/swap, and one-ULP/ambiguity rejection. The
  non-cap row retains eight vertices and every six-cell decomposition keeps the
  exact 6/5, 147/146, and 588/587 piece/pair/arc N/N-1 ceilings; other polar
  layouts remain unsupported.
  The both-wide Cartesian 3×3 arm now also admits all four exact disconnected
  seven-positive rotations in which one occupied corner is a singleton, its
  two orthogonal neighbor cells certify empty, and the other six occupied cells
  form one exact component. The existing connected-seven merger keeps
  precedence. The disconnected proof requires zero occupied-boundary contact
  at both empty separators, cancels the six-cell component's six internal seams
  simultaneously, and rejects any surviving artificial seam or bit-exact
  contact between components. Success returns exactly two canonical parent-
  mapped regions with maximum child/parent residual propagation. Real fixtures
  pin all rotations, repeat/swap, exact 9/8 piece, 252/251 pair, and 1,008/1,007
  arc N/N-1 admission; one-ULP seam and duplicate-edge ambiguity fail closed.
  Other seven-positive 3×3 layouts outside the exact connected and corner-
  singleton-plus-six-component families remain indeterminate.
  Varying-normal chains against a direct analytic Plane retain exact outer-to-
  inner metadata, prove every intermediate and final radius finite and positive
  from the original basis, use the derived sheet only for discovery, preserve
  certificate budgets, and pin exact 2–5 graph Work/depth with N/N-1 evidence;
  other peers remain one descriptor and chain depth five or greater remains
  unsupported. Compatible intersecting
  planar constant-normal dual Offset(NURBS) chains now cover the full 1–4×1–4
  matrix with original-source proof at exact 14,336/1,024/depth-10 certificate
  use and maximum 10 Work/depth-5 graph traversal; the strict-separated
  complete-miss arm remains. Exact algebraic curve-pair search keeps magnitude
  twelve as its compatibility default and stable prefix while explicit
  configuration admits the reviewed thirteen and fourteen shells. Fourteen
  owns exactly 254 carrier forms and 9,825 residual forms—24 and 1,704 more than
  thirteen—and alone certifies the reviewed noncoplanar normalized-`1/3`
  fixture after every ceiling through thirteen returns no certificate. Earlier
  goldens remain unchanged; invalid ceilings and broken, overflowing, non-
  finite, or out-of-range forms fail closed. Polygonal
  profiles with holes, checked complete-body rigid copy with direct Plane or
  safe finite Offset(Plane)-backed Plane/Plane lines, and Plane/Sphere latitude
  or oblique circles backed by direct Plane or safe finite Offset(Plane) sources
  plus direct Sphere or safe finite Offset(Sphere) sources whose effective
  Sphere radius is positive and finite, in either source order and through
  leaf-inclusive depth at most 64, and checked nonzero-normal oblique
  polygonal-profile extrusion are the first M4 modeling consumers. Rigid copy
  now reissues every current operation-generated verified-NURBS intersection
  family, including one- through four-level nested dual offsets and oblique
  frames. Its first transmitted tranche covers Plane/Plane over direct or safe
  nested exact-plane roots, direct Plane/NURBS in both orders, direct
  NURBS/NURBS, direct one-descriptor Offset(NURBS)/NURBS in both orders,
  exactly one-descriptor Offset(NURBS)/direct-Plane charts in both orders, and
  only the canonical finite-open two-sample degree-1, witnessed three-sample
  quadratic, witnessed four-sample cubic, canonical five-sample degree-1, or
  canonical seven-sample degree-1 dual Offset(NURBS) charts in either ordered-
  root arrangement. These complete the existing canonical 2/3/4/5/7-sample set.
  The two-sample line, witnessed three-sample quadratic, and witnessed four-
  sample cubic each admit independent exact ordered chains of one through four
  descriptors per root across a full 4×4 matrix and both trace orders; the five-
  and seven-sample families remain exactly one descriptor per root. All five
  require distinct ordered roots with distinct direct nonperiodic terminal
  NURBS basis handles.
  The line uses unweighted two-
  control carrier/pcurves on `[0,0,1,1]` over `[0,1]` without witnesses; the
  quadratic uses unweighted degree-2 three-control carrier/pcurves on
  `[0,0,0,2,2,2]` over `[0,2]`; and the cubic uses unweighted degree-3 four-
  control carrier/pcurves on `[0,0,0,0,3,3,3,3]` over `[0,3]`. Both witnessed
  higher-order families retain exact position and paired-UV interpolation
  witnesses. The five-sample family uses unweighted degree-1 five-control
  carrier/pcurves on `[0,0,1,2,3,4,4]` over `[0,4]`; the seven-sample family
  uses unweighted degree-1 seven-control carrier/pcurves on
  `[0,0,1,2,3,4,5,6,6]` over `[0,6]`. Neither polyline family has interpolation
  witnesses or a carrier period. Generic graph persistence walks each dual root
  to its direct NURBS terminal and binds its complete outer-to-inner distance
  order bit-for-bit; it atomically rejects reordered same-total, extra, missing,
  or stale chains. The graph trace constructor retains at most four exact
  ordered descriptors per trace and rejects a fifth; a live depth-five source
  paired to the maximum-depth trace therefore fails atomically at graph
  insertion. Broader depth remains graph representation/binding work. Rigid
  copy transforms both distinct terminal bases and the line, five-sample,
  or seven-sample carrier; for either witnessed higher-order family it transforms
  exact positions and rebuilds carrier controls by the public interpolation
  formula. It copies both ordered roots and full offset/basis and pcurve chains,
  preserves exact distance order, terminal source binding, metadata, tolerance,
  and exact UV witnesses, publicly recertifies—including every witnessed
  quadratic and cubic 4×4 pair—and protects the copied roots and complete basis
  closures transitively.
  Facade rejection happens before scope creation, while lower-copy rejection
  restores every Body/Region/Shell/Edge/Vertex and Curve/Surface/Pcurve/Point
  count plus future point identity. Graph-valid shared-basis or periodic charts,
  nested five-/seven-sample roots, Offset(Plane) peers, altered higher-order
  witnesses, and other sample counts remain copy-unsupported;
  attributes are blocked on an authorable storage contract, and non-rigid
  transforms remain.
  General root discovery, complete verified residuals across every result family, and
  boolean-ready paired-pcurve branches remain gated.
- M5-M8 are not started: there are no end-to-end booleans, general sweeps/sewing,
  blends/offsets/shelling, stable C API, or production hardening yet.

The authoritative foundation priority and handoff order live in
[`docs/projects/foundation-projects.md`](docs/projects/foundation-projects.md).
The longer milestone contracts remain in the construction roadmap; they are
not a second execution queue.

## Building

```sh
# Tight inner loop for one package target.
python3 scripts/test_lanes.py focused -p kxt -t read

# Normal edit/commit gate: workspace unit tests plus representative integration
# coverage across determinism, topology, operations, interchange, and facade use.
python3 scripts/test_lanes.py fast

# Broad local gate: every non-corpus integration target plus tooling.
python3 scripts/test_lanes.py standard

# Architectural API-boundary examples and other workspace doctests.
python3 scripts/test_lanes.py docs

# Mandatory pre-merge/handoff gate, including every corpus ratchet.
python3 scripts/test_lanes.py full

cargo clippy --all-targets -- -D warnings
```

Direct `cargo test --workspace` remains complete and supported. The
[test-throughput contract](docs/projects/test-throughput.md) documents the
fail-closed lane inventory, timing evidence, and CI scheduling policy.

Requires stable Rust (1.93+). The lane runner requires Python 3.11+ and uses
only the standard library. The workspace is dependency-free by policy at L0.

## Determinism contract

Same input → bit-identical output on every platform, thread count, and run.
CI enforces this with golden-hash tests across Linux/macOS/Windows in both
debug and release. Changing a golden value is a reviewed, intentional event.
