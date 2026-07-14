# Kernel foundation project portfolio

Status: active implementation portfolio; convergence phase

This portfolio turns the current foundation review into bounded projects with
explicit dependencies and exit criteria. Projects should preserve the kernel's
existing determinism, failure atomicity, completion evidence, and checked
topology boundaries.

## Planning authority and handoff rule

This file is the authoritative source for **current foundation priority,
project status, and handoff order**. `docs/kernel-roadmap.md` remains
authoritative for milestone contracts, dependency rationale, and long-horizon
exit criteria, but its milestone ordering is not a second execution queue.
Project-specific design files remain authoritative for their local contracts.

At handoff, update this portfolio first. A project file may refine its own next
slice, and the milestone roadmap may record changed evidence, but neither may
silently reorder the portfolio. If evidence changes the order, revise this
section and link the reason from the affected project file in the same change.

## Progress

| Project | Current state |
| --- | --- |
| F0 | Implemented: curve/curve operand swapping preserves completion evidence and canonical order. |
| F1 | G1-G4a plus the F2 graph-budget adapter are implemented: `kgraph` and `ktopo::Store` own one transactional geometry graph; exact offsets evaluate through accepted/attempted node/depth accounting, check, tessellate, and round-trip through the declared X_T subset without basis duplication. Reverse dependencies use deterministic insertion-ordered adjacency with direct key/membership lookup and no full-order rebuilds; traversal keeps vector-defined output/path order with indexed active/completed membership. Q2a/Q2b preserve exact graph, index, traversal, rollback, and accounting evidence; the Q2a v2 production-descriptor diamonds also pin deduplicated shared-basis closure. G5a supplies invertible carrier/pcurve maps plus private-field whole-interval certificates for plane/plane lines and both common-axis and genuinely oblique plane/sphere circles. Its graph-aware exact-field adapter preserves source handles, raw-result parity, typed stale/unsupported failures, canonical swap behavior, and deterministic certified branch vertices/edges. Direct planes/spheres and context-accounted constant-offset chains share these arms while every sphere-chain radius remains positive and finite. The common-axis fast path retains exact `t`/`-t` longitude maps across shifted, seam-crossing, full-turn, and overwide finite windows. General finite secants use a stable nonperiodic `SphericalCircle` pcurve with analytic derivatives through order three, continuous seam unwrapping, conservative finite bounds, whole-branch pole clearance, and typed chart-window rejection. Certified line and circle branches persist atomically as stable `Intersection` curve descriptors with ordered source/pcurve dependencies and their paired proof; altered or stale proof sources fail before allocation or roll the batch back. Direct or safe finite Offset(Plane) fields, and direct Sphere fields, against genuinely non-planar direct NURBS now run the existing marcher in one owner scope, retain paired degree-1 traces, certify both lifts at fixed depth 10, and persist the original ordered source identity in a distinct non-transmitted verified-NURBS descriptor atomically. M3c uses the same ownership seam for canonical finite-open X_T `INTERSECTION` charts whose two ordered exact-plane sources may each be direct or a safe finite offset chain; two-offset roots must have independent basis chains. Transmitted positions and modern paired UVs persist as a degree-1 carrier plus pcurves with a whole-span residual certificate; the proof binds effective planes while retaining actual ordered source handles and protecting every transitive offset basis. Plane/B-surface, safe-Offset(Plane)/B-surface, B-surface/B-surface, and direct constant-normal Offset(B-surface)/B-surface charts use one persistent NURBS-intersection descriptor and whole-range original-source interval certificate for each polynomial or rational NURBS trace; a plane trace may bind a direct plane or safe finite offset chain, while an Offset(B-surface) trace retains the live root, signed distance, and original basis and proves its whole-box unit-normal field; every ordered root and transitive basis remains protected. Certified clamped periodic/closed B-surfaces now use BODY-toleranced polynomial or exact-homogeneous rational position/C1 seams, wrapped evaluation, seam-aware bounds, and writer-preserved flags. Those three leaves, the first constant-normal `Offset(B-surface)/B-surface` chart, the endpoint-only equal-limit records 1828 and 2008, the finite-open/end-terminated `T/F` records 1671 and 1678, and finite-open direct B-surface/Plane record 1252 with paired-null interior Plane UVs now certify; production reconstruction advances to procedural `SP_CURVE` node 30 (`xt.geometry.procedural-curves`) at exact v5 `117478445/20/10` Work/Items/Depth. The historical 14-file writer bundle is host-certified and machine-fingerprinted; the declared set now adds a locally verified genuinely curved B-surface fixture, and the certification remains stale pending the complete 15-file licensed-host rerun. Broader G4 corpus coverage, Offset(NURBS)/NURBS, NURBS/NURBS and further procedural G5 arms, further carrier families, broader persistent descriptor families, and the exemplar's procedural-curve, null/general-closed, other nullable-data, non-endpoint-only periodic-trace-range, and noncanonical transmitted-chart variants plus arbitrary unclamped cyclic B-geometry remain. |
| F2 | Stage 1, Stage 1b composition, the bounded NURBS/NURBS Stage 3 scale gate, two Stage 2 pilots, and contextual face/body-tessellation, projection, checker, and generic curve/curve entries are implemented. `OperationContext` owns family-default < session < request budget precedence for graph evaluation, Full checking, tessellation, projection, exact curve-pair and surface-patch isolation, overlap-equivalence Work/Items admission, and bounded cell-local seed attempts, including canonical root stops and accounting-mode validation. Whole-body tessellation owns one scope across graph evaluation, projection fallback, refinement/storage, per-patch work, and retained output. X_T reconstruction and generic curve/curve dispatch account owned iterative work through one scope; exact compatibility, N/N+1 limits, numerical-policy propagation, rollback/completion evidence are pinned. The exact-plane X_T chart profile pre-admits two certificate Work units per position, retained-position Items, and one proof Depth; direct, one-offset, and two-offset two-position fixtures pin exact `4/2/1` N/N-1 crossings before allocation; direct two-offset roots consume 34 graph visits/depth 2 and two nested roots consume 36/depth 3. Plane/B-surface and Offset/B-surface charts pre-admit `N+(N-1)*2^10*(6R+1)` Work, `N` Items, and Depth 10 from validated source dimensions; the one-span fixture remains exact at `7170/2/10`. B/B and direct constant-normal Offset(B)/B sum both original-source trace proofs; canonical one-span pairs pin `14336/2/10`, and the offset trace additionally proves a positive whole-box normal bound while binding the live root, signed distance, and basis. Historical X_T import profile v1 remains at 131,072 Work, v2 at its pre-equal-limit 81,267,732-Work boundary, v3 at exact equal-limit `115485725/20/10`, and v4 at exact end-terminator `116396069/20/10`; owner reconstruction uses corpus-backed v5 with exact finite-open omitted-Plane-data `117478445/20/10` admission and rollback, while records 2008 and 1678 independently pin `124040223/22/10` and `116413476` Work. Exact overlap scans, common-knot reconstruction, and bounded checked inverse-refinement state pre-admit conservative logical work and temporary items; per-resource N/N-1 crossings return structured indeterminate evidence before allocation. Curve-pair range boxes pre-admit every inspected original-source knot-span slot; the one-span depth-six default is 6,828 units. Surface source bounds now admit `R*(6R+1)` before contextual BVH build and `1+4*(6R+1)` before each adaptive parent, including repeated/empty tensor slots; denial retains the source cover. Each oblique spherical-pcurve certificate pre-admits exactly 128 proof-subdivision Work units, and its composed graph-surface profile pins exact N/N-1 crossings alongside safe-offset node/depth accounting. The same profile composes exact direct/safe-Offset(Plane)-field/NURBS and direct-Sphere/NURBS marching. Plane traces precharge `C + S*2^10*(6T+1)` certificate Work; the curved one-segment fixture pins exact 7,170/7,169 and one Offset(Plane) pins exact 2/1 graph visits. Sphere traces precharge `S*2^10*(6T+2)` and observe paired proof cells/depth; the one-segment fixture pins exact 8,192/8,191 Work, 1,024/1,023 Items, and 10/9 Depth. Failed residual proof retains attempted resources. Internal graph-owned facade coverage pins distinct checked-ancestor identities, clipped reversed extents, exact Work/Items N admission, and isolated per-resource N-1 evidence. Q4 curve isolation v4, implicit isolation v3, and solve v18 record source-range enclosure/certificate digests, multi-span surface Work, primitive-integer completion through magnitude twelve, common-refinement and inverse-history recovery, altered-history rejection, and independent overlap Work/Items denial. Body/standalone-face tessellation and both standalone projection wrappers are closed to new production callers by the CI retirement ratchet. Corpus-backed face/body `bounded_v1` presets now carry finite stage and root caps; all matrix rows pass, exact observed root N/N-1 crossings are pinned, and facade/X_T clients opt in without changing compatibility defaults. Segment conditioning, input/dedup slack, other intersection-family minimizer/evidence migrations, and broader migrations remain. |
| F3 | Implemented slices include centralized class inspection; shared periodic/scalar fitting and first-wins candidate emission; shared finite-range validation and window fitting; canonical unordered-pair normalization; typed one-arm-per-pair routing; contextual/shared-scope generic curve/curve dispatch; source-provenanced NURBS curve-pair isolation with bounded verified polishing; and unique exact transverse-root certificates. Candidate cells retain shared original-curve provenance: rounded split controls are partition/seed machinery only. Curve-pair exclusion and stored bounds use outward interval position enclosures over original-source ranges, tightened by the conservative whole-source hull and failed open to it. A cubic/line adversary whose exact shared midpoint rounds outside every generated child hull proves that subdivision cannot create a false empty result. Partial certificates use direct outward interval de Boor bounds of the original homogeneous source and derivative B-splines over each requested knot span. Source-range Poincare face signs plus a range-local P-matrix cover coplanar roots; noncoplanar existence includes bounded exact `{mid,lo,hi}` samples, in-range full-multiplicity knots, and algebraic parameter-correspondence lifts. Exact same/reversed normalized knots, globally proportional positive rational weights, and a strictly monotone source-exact carrier remain mandatory. Canonical primitive integer carriers and omitted-coordinate residuals through coefficient magnitude twelve remain the compatibility proof family; an explicit validated search configuration now admits the magnitude-thirteen shell without changing that default. The magnitude-twelve enumeration is an exact stable prefix, direct homogeneous derivative bounds preserve correlation through the reviewed magnitude-thirteen corridor, and every broken, overflowing, non-finite, or out-of-range configuration fails closed. Surface BVH, plane/implicit pruning, adaptive children, and `NurbsSurface::bounding_box` now evaluate source rectangles by outward tensor interval de Boor, active source-support hulls, and a centered derivative mean-value bound; an exact cubic extrusion plus exact source-rectangle Work N/N-1 coverage prove rounded child hulls and denied scans cannot erase a real contact. Exact affine parameterizations, common-knot reconstruction, and checked exact-reinsertion ancestors produce complete clipped same/reversed overlap extents; altered refinements and sampled near-coincidence stay indeterminate. Curve/curve and surface/surface swapping restore canonical first-operand ordering without weakening completion evidence. Bounded coincident Plane/Plane, exact coincident Cylinder/Cylinder, exact common-axis Sphere/Sphere windows, exact nonparallel signed-axis and arbitrary-frame Sphere/Sphere octants, exact coincident coaxial Cone/Cone, and exact coincident Torus/Torus windows now return canonical paired regions with chart orientation and outward whole-region residual bounds; cone regions preserve affine chart correspondence and split at the apex, torus regions split both periodic axes and collapse to exact latitude/meridian circles or tangent points, common-axis windows remain seam-aware rectangles, and sphere octants retain explicit nonlinear bidirectional chart correspondence, exact robust halfspace topology, and exact physical boundary anchors. The first general non-octant arbitrary-axis arm covers positive-area pole-clear windows narrower than π: all 28 boundary-halfspace pairs, one connected degree-2 cycle, and a strict interior witness are mandatory before Complete, while containment, seam crossing, and swap retain authoritative nonlinear window correspondence. A fixed scan of at most 112 arrangement arcs returns Complete empty only after excluding every boundary component; its disjoint exemplar pins exact 96/95 witness evidence. Bit-exact opposing boundary planes collapse one equality lock to interval-certified tangent circle arcs and two independent locks to interval-certified tangent points; the collapsed-curve exemplar pins exact 12/11 arc-witness evidence. A bounded wide arm decomposes exactly one pole-clear wide operand into three closed sub-π cells and completes only for three certified-empty cells or one positive region with two certified-empty siblings, pinning exact 3/2 piece, 84/83 pair, and 336/335 arc ceilings. A second empty-only arm decomposes both pole-clear wide operands into a Cartesian 3×3 grid and completes only after all nine child intersections certify empty, pinning exact 9/8 piece-pair, 252/251 boundary-pair, and 1,008/1,007 arc ceilings. Multi-cell-positive two-wide unions, polar, non-exact tangent, ambiguous, and multiple-cycle cases remain Indeterminate. Edge and point contacts collapse dimensionally, cone apexes and sphere poles remain singular, and disjoint windows or octants are proven empty. The shared complete-support-curve SSI emitter owns clipping, periodic membership, candidate reacceptance, and first-wins dedup for cylinder/cylinder, cone/cylinder, cone/cone, cone/sphere, cylinder/sphere, and sphere/sphere with byte-identical representative results. Circle/ellipse × cylinder, circle/ellipse × cone, circle/ellipse × torus, and circle/ellipse × sphere each use one config-driven curve/surface pipeline with pre-refactor debug/release bit signatures, exact diagnostics, and unchanged completion; this completes the analytic primitive-surface conic family. Circle/ellipse × NURBS shares one marcher with explicit radial-circle and closest-projection ellipse strategies. Circle/circle, circle/ellipse, and ellipse/ellipse share one bounded conic-pair orchestration layer while their distinct quadratic/quartic and projection arithmetic remains strategy-local; frozen debug/release result and contextual report digests prove bit-exact behavior. Direct or safe finite Offset(Plane) fields, and direct Sphere fields, against genuinely non-planar direct NURBS now retain the lower marcher's degree-1 paired traces, prove both lifts over the whole range in one owner scope, and persist ordered source identity in a distinct non-transmitted verified descriptor. Plane fixtures pin exact 7,170/7,169 certificate and 2/1 offset graph-visit evidence; the Sphere fixture pins exact 8,192/8,191 Work, 1,024/1,023 Items, and 10/9 Depth. Offset(Sphere)/NURBS, Offset(NURBS)/NURBS, NURBS/NURBS, further verified carrier/descriptor families, coefficient forms beyond the reviewed magnitude-thirteen ceiling, and multi-cell/two-wide/polar or non-exact collapsed general sphere-window contracts remain. |
| F4 | Phase 1, representative Phase 2 slices, and three Phase 3 pilots are implemented: graph evaluation owns stable classification; SSI and NURBS curve/curve retain ordered structured incomplete evidence through limits, numeric/method stops, canonicalization, swapping, dispatch, and facade adaptation. Broader result-family and legacy migrations remain. |
| F5 | K1-K3, typed K4 interchange and facade journal views, K5 adoption, and facade body tessellation are implemented: the `kernel` facade owns lifecycle, opaque IDs, classified sources, one-scope outcomes, safe checker subjects, child-accounted procedural evaluation, atomic typed X_T import/export, graph-owned bounded curve/curve intersection with facade-owned proof results, and immutable conforming body meshes with exact closed-solid or true-sheet incidence plus facade-safe face/edge identities. Committed journals expose exact-size part-qualified net-mutation, all-five-form lineage, tolerance-budget, and tolerance-event iterators; deleted topology/geometry identity remains reportable and stored points use a journal-only opaque ID rather than leaking arena handles. The standalone `kernel-lifecycle` client depends directly only on `kernel` and proves construction, semantic inspection, Full checking, body tessellation, curve intersection, surface evaluation, byte-stable X_T export/import/re-export, and facade-only journal traversal. Semantic edit transactions remain. |
| F6 | First slice implemented: shared surface inversion, chart normalization, and distance services consumed by checker and tessellation. Module splits remain. |
| F7 | Q0-Q2b, Q8, and the first Q3-Q6 slices are implemented: CI now enforces Python/oracle freshness, compiles and smoke-runs the excluded benchmark package including graph construction/traversal, contextual body/face tessellation, and curve-pair isolation/solve, and runs both pinned fuzz targets within fixed limits. Q2a drove the reverse-index replacement and its v2 ladder now pins zero full-order rebuilds across 21 rows, including four real verified-intersection diamonds whose dependency-first closure visits a shared basis exactly once. Q2b v2 has ten deterministic closure/path rows through 1,000 edges plus real verified-intersection diamond closure and missing-path cases; the timed closure visits the shared basis exactly once after traversal membership indexing. Q3 `body-tessellation.v3` has 32 rows and pins all 21 composed counters: twenty generalized legacy solids plus four tiers each for a locally verified genuinely curved NURBS block and historically host-certified plane and full-period cylinder sheets. Its 24 solids and eight sheets pin exact directed incidence, topological boundary, face-sense orientation, and the applicable signed-volume or faceted-area measure. Q3 face v2 crosses three representations, three trim topologies, and two tolerances with lift, orientation, boundary, area, mesh, and report evidence. Both matrices pass finite `bounded_v1` profiles and pin root Work N/N-1. Q4 implicit isolation v3 has eight cases including the surface roundoff adversary and multi-span Work N-1; curve-pair isolation v4 has nine span-accounted cases; solve v18 has twenty-eight cases including coordinate, unit-form, and primitive magnitude-two through magnitude-twelve algebraic `1/3` certificates, common-refinement and inverse-history success, altered-history rejection, and exact per-path overlap Work/Items denial. The benchmark manifest now contains 156 total cases. Q3-Q5 expansion, exact coefficient forms beyond twelve, more Q6 targets/corpora, and Q7 remain. |

## Current direction and handoff order

The foundation has enough vertical proof. The current phase prioritizes
convergence, adoption, and continuous enforcement over adding more parallel
surface area:

Read the ordered queue below literally. At this handoff, item 1's facade-owned
body-tessellation replacement and state-4 compatibility ratchet are closed,
and item 2's evidence, finite-preset, matrix-admission, and client-adoption
work is closed. The first unclosed foundation obligation is item 3. Its
explicitly bounded magnitude-thirteen/configurable-search component is
complete: magnitude twelve remains the compatibility default and exact stable
prefix, while opt-in thirteen is the only reviewed extension. The exemplar's
equal-limit records 1828 and 2008 are certified for the endpoint-only,
single-periodic-axis form, and its end-terminated records 1671 and 1678 are
certified for the finite-open/`T/F` singular form. Production reconstruction
now uses the exact v5 corpus profile, certifies finite-open direct
B-surface/Plane record 1252 with interior-only paired-null Plane UV recovery at
`117478445/20/10` Work/Items/Depth, and stops at procedural `SP_CURVE` node 30.
The remaining parallel legs continue the certified non-octant sphere fallback
beyond its pole-clear, sub-π hit, certified-disjoint, exact-boundary-lock
collapsed, first single-wide decomposition, and two-wide certified-empty grid
arms to multi-cell-positive or polar unions and non-exact tangent cases; and
extend the contextual verified graph arms beyond exact direct/safe-Offset(Plane)
and direct Sphere fields to Offset(NURBS) or NURBS/NURBS fields. These legs do not
silently leapfrog items 1 or 2.

### External-evidence lane — current

The historical 14-file bundle, including `offset_plane.x_t`, has licensed-host
evidence from Onshape. The declared writer set now has a fifteenth, genuinely
curved B-surface solid with deterministic local import/check/tessellation
evidence and pending host certification. `docs/oracle-certification.json`
fingerprints the certified writer inputs and every historical host payload; Q8
regenerates the declared bundle and rejects a falsely current record. The
post-certification facade accessor migration changed writer source, and the
new curved payload expands the declared set, so the 14-fixture record remains
correctly stale until the standing complete licensed-host rerun. Host findings
remain ratcheted in `docs/oracle-results.tsv`.

### Ordered code queue

1. **Adopt and ratchet the completed contextual paths — completed.** X_T reconstruction
   and checked-commit Fast validation share one facade-owned scope and
   cumulative graph allowance. Whole-body tessellation now has equivalent
   contextual and shared-scope entries, composes its projection fallbacks, and
   its remaining `ktopo`/`kxt` production clients now use one contextual
   operation per body. The enforced legacy-API source audit closes new
   production calls to the body wrapper while preserving compatibility tests.
   Standalone surface projection is now closed to new production callers;
   X_T owns a composed graph/projection profile and ellipse intersection owns
   one contextual projection scope. Both standalone projectors are now closed
   to new production callers by the source ratchet.
   `kernel::Part::tessellate_body` now owns the complete body profile and one
   operation scope, maps ordered face ranges and edge polylines to opaque
   part-qualified identities, preserves exact lower mesh/report/error evidence,
   and is exercised by the facade-only lifecycle client. The legacy
   `ktopo::btess::tessellate_body` compatibility wrapper is state 4 deprecated;
   its v1 behavior remains pinned while the source ratchet prevents facade or
   production code from resetting context through it.
2. **Finish hostile-input tessellation policy.** Exact per-face split/vertex/
   triangle admission and body-wide edge/iso split, prepared-patch, and retained-
   triangle stages have landed, including physical representability checks,
   atomic rejection, deterministic diagnostics, and composition evidence.
   Prepared UV/patch copies and final nondegenerate triangles are admitted
   before their first body-owned allocation; later moves do not recharge them.
   Pre-UV edge face-use, seed, recursive-interior, retained-sample, and record
   slots plus final edge-polyline records and indices now share one exact
   `Items/Cumulative` stage, including pre-allocation arithmetic and atomic
   N/N+1 evidence. The compatibility-v1 preparation, edge-storage, structural,
   and body-triangle totals intentionally remain accounting-only at `u64::MAX`
   because no truthful finite legacy cap exists. A distinct structural-items
   stage now admits the single first-seen topology plan, deterministic
   membership scratch, `vgids`, `face_ranges`, outer loop/chain and patch-hole
   collections, `trim_loops`, and torus arc-row holders. The reviewed block total is 84, and
   closed-surface, multi-hole, atomic N/N+1, shared-scope, overflow, diagnostic,
   legacy, and execution-policy evidence has landed. Q3's contextual analytic
   `body-tessellation.v3` ladder now records all 21 aggregate stages across 32
   rows and preserves the reviewed legacy mesh bits. Its twenty generalized
   legacy solids are joined by four tiers each for a locally verified genuinely
   curved NURBS block and historically host-certified plane and full-period
   cylinder sheets, for 24 solids and eight sheets. Exact directed incidence,
   topological boundary, and face-sense orientation apply to both body kinds;
   solids prove signed volume while sheets prove faceted area. The curved
   block's finest admitted tier is `5e-4`: `3e-4` truthfully fails because 25
   refinement passes exceed compatibility v1's limit of 24. The full-period
   cylinder sheet uses a proven rectangular chart split into four
   quarter-period patches and requires no area exception. The body
   representation evidence gate is closed. The standalone face v2 matrix also
   closes its named representation/trim gate with 18 plane, analytic
   half-cylinder, and exact
   rational-quadratic NURBS rows across outer, one-hole, and three-hole trims
   at two tolerances. It pins trim/boundary identity, exact lifts, orientation,
   UV/model area, and all face stages. The reviewed finite presets use the next
   power of two at or above twice each nonzero matrix maximum, preserve zero,
   and retain smaller existing algorithm caps. Every row passes, with exact
   face 222/221 and body 2,822/2,821 root-Work crossings. Facade lifecycle and
   X_T tool clients opt in explicitly; compatibility defaults are unchanged.
   Item 2 is closed. In the body ladder, zero
   face-boundary use is the required frozen-boundary invariant, not missing
   evidence.
   Do not describe compatibility-default tessellation as hostile-input bounded,
   use allocator-dependent byte counts, or silently tune the legacy v1 wrapper.
3. **Resume algorithm/API expansion behind the completed gates.** The first facade
   graph-aware intersection family now adopts F3's contextual generic
   curve/curve dispatcher with exact report parity, identity precedence, and
   classified limit evidence. Exact adaptive NURBS pair exclusion now feeds
   one bounded cell-local seed/polish attempt per retained cell; accepted
   discoveries carry re-evaluated tolerance witnesses. Complete isolation now
   grants completion only when every deterministic candidate component has a
   unique-root certificate and verified representative; partial certificates
   remain proof evidence on indeterminate results. Exact shared grid vertices
   define joined components, and their validated bounding-range proofs now
   complete rational boundary roots and separated multi-root cases. Interval
   Euclidean hull-distance bounds now remove diagonal tolerance-empty cells
   without weakening the inclusive boundary. Exact affine parameterizations
   with matching normalized knots and globally proportional rational weights
   now produce complete clipped same/reversed overlap extents. Noncoplanar
   pairs with an exact shared 3D corner can certify uniqueness through an
   interval-global injective coordinate projection; sampled near-coincidence
   and unsupported spatial existence cases stay indeterminate. Deterministic
   knot-insertion descendants now inherit the same complete overlap result
   only when reconstruction to a common knot multiset compares exactly;
   differing rounded insertion histories now recover only through bounded
   inverse candidates whose production reinsertion exactly reproduces both
   descendants; altered refinements stay indeterminate. Full-multiplicity
   interior knots add an exact noncoplanar 3D existence witness before the same
   injectivity proof. Candidate cells now retain shared original-source
   provenance after an adversarial `2^-53` case demonstrated that rounded
   restricted endpoints could otherwise create a false exact witness. Exclusion
   bounds now come from outward interval evaluation of the original source over
   each child range, failed open to the whole-source hull; a distinct exact
   midpoint adversary proves rounded children cannot erase a real contact. Partial
   proofs evaluate only the original sources: outward interval de Boor bounds
   over source knot spans supply range-local Poincare face signs and P-matrix
   derivatives, while a bounded exact `{mid,lo,hi}` parameter cross product and
   in-range full-multiplicity knots supply noncoplanar existence. Exact
   normalized parameter correspondence now adds a non-sampled algebraic lift:
   proportional rational denominators, a strictly monotone shared carrier, and
   exact omitted-coordinate controls turn a projected Poincare/P-matrix root
   into a source-exact 3D root. Signed `x±y` carriers and
   `z+a*x+b*y`, `a,b∈{-1,0,1}`, residuals extend that proof to source pairs
   with no shared coordinate controls while retaining exact homogeneous
   derivative and arithmetic gates. Rounded
   generated controls never establish a root; arithmetic, witness, or interval
   failure stays inconclusive without regressing rational boundary or separated
   multi-root completion. Overlap
   scans/reconstruction pre-admit Work and Items, and every curve-pair
   source-range enclosure pre-admits its inspected knot-span slots; limit crossings return
   structured indeterminate evidence. Surface patch exclusion likewise retains
   original-source rectangles through outward tensor intervals, source-support
   hulls, and centered derivative bounds; an exact extrusion adversary prevents
   rounded child hulls from creating a false empty result. Contextual surface
   BVH and adaptive-child evaluation now pre-admit their complete tensor-span
   scan formulas, with repeated-knot and roundoff N/N-1 parent retention.
   Internal facade coverage, Q4 isolation v4, implicit-isolation v3, and Q4
   solve v18 now ratchet this accounting. G5a owns graph-aware plane/plane lines
   and common-axis plus genuinely oblique plane/sphere circles with paired
   whole-interval pcurve residual certificates. The common-axis fast path covers
   both sphere-axis orientations, rotated plane charts, and seam-aware finite
   longitude windows. The general path persists a bounded nonlinear inverse
   sphere-chart pcurve, proves pole clearance, and pre-admits 128 Work units per
   retained branch. Context-accounted safe plane/sphere offset fields and
   atomically persistent verified line/circle descriptors share both arms. The
   first M3c consumers now import canonical finite-open Plane/Plane,
   Plane/Offset, and Offset/Offset transmitted charts by
   retaining their model-space positions and modern paired
   UVs on a shared degree-1 basis, proving both lifts over every span, and
   committing the carrier, ordered sources, pcurves, metadata, and certificate
   atomically. Certificates bind effective exact planes while retaining actual
   source handles and protecting safe nested offset bases; two offset roots
   must have independent chains. Canonical finite-open Plane/B-surface,
   Offset/B-surface, B-surface/B-surface, and every applicable reversed operand
   order now persist polynomial or rational NURBS proof sources and use
   original-source interval point/partial enclosures to prove every span at
   binary depth 10, with exact Work/Items/Depth N/N-1 rollback coverage. A
   plane trace may bind a direct plane or safe finite offset chain while
   retaining the actual root and protecting every transitive basis; B/B
   retains and protects both ordered original sources. Direct constant-normal
   Offset(B-surface)/B-surface charts additionally retain the live offset root,
   signed distance, and original basis while outwardly proving a positive
   whole-box unit-normal field. Historical import profile v1 remains at 131,072
   Work and v2 remains the pre-equal-limit compatibility contract at
   `81267732`. Historical corpus-backed v3 admits record 1828 and every later
   equal-limit chart at exact `115485725/20/10`
   Work/Items/Depth before the preserved terminated `T/F` limit boundary.
   Record 2008 is masked by that earlier traversal stop, so its payload is
   independently pinned by an exact transplant at `124040223/22/10`. Both
   shared `H/?` equal-limit forms require one position, spatial closure,
   exactly one certified periodic NURBS axis, and endpoint-only period
   unwrapping; null, distinct closed, interior-out-of-domain, or off-seam forms
   remain typed and atomic. Corpus-backed v4 then certifies the finite-open
   start plus end `T/F` record 1671 at exact `116396069/20/10`: its terminator
   stores a distinct tolerance-close `[singularity, branch]` pair, the branch
   matches the chart endpoint, and one extra UV tuple proves the appended final
   span. Paired-null Plane UVs are recovered analytically; NURBS endpoint
   roundoff may snap only within `16384 * EPSILON * domain-scale` before the
   whole-range proof. Record 1678 is independently pinned by transplant at
   `116413476` Work. Corpus-backed v5 next certifies finite-open direct
   B-surface/Plane record 1252 at exact `117478445/20/10`; only its six interior
   Plane UV pairs may be null, and exact direct-plane inversion recovers them
   before the original two-lift whole-range proof. Endpoints, NURBS UVs,
   partial pairs, offset operands, and other null/non-finite chart data remain
   typed and atomic. The corpus boundary is now procedural `SP_CURVE` node 30.
   The exact
   algebraic family now includes canonical primitive carrier/residual
   coefficients through magnitude twelve as the compatibility default. A
   validated `CurvePairAlgebraicSearchConfig` now opts the standalone and
   candidate-cell exact certifiers into the magnitude-thirteen shell only;
   twelve remains an exact stable prefix, the supported 12/13 form counts are
   pinned, and invalid ceilings fail before search. The contextual non-plane
   graph arm now marches direct or safe finite Offset(Plane), or direct Sphere,
   fields against a genuinely curved direct NURBS in one owner scope, retains
   paired degree-1 traces, certifies both lifts over the whole range at depth
   10, and persists the ordered live source identity in a non-transmitted
   verified NURBS descriptor atomically. The Plane fixture pins exact 7170/7169
   certificate Work and one Offset(Plane) pins exact 2/1 graph visits. The
   Sphere fixture pins exact 8192/8191 Work, 1024/1023 Items, and 10/9 Depth for
   outward centered-mean-value Sphere and original-source NURBS lifts. Planar
   encodings, Offset(Sphere), Offset(NURBS), and NURBS/NURBS remain explicit
   unsupported boundaries. Further fields and
   carrier families, plus coefficients above thirteen, remain unsupported. All six complete-support-
   circle SSI pipelines share one emitter; circle/ellipse × cylinder,
   circle/ellipse × cone, circle/ellipse × torus, and circle/ellipse × sphere
   each share one bit-pinned config driver. This completes the analytic
   primitive-surface conic family; circle/ellipse × NURBS now shares one
   bit-pinned marcher with explicit strategies. Circle/circle, circle/ellipse,
   and ellipse/ellipse now share one bit-pinned orchestration layer without
   merging their distinct root/projection arithmetic. Bounded coincident
   Plane/Plane, exact coincident Cylinder/Cylinder, exact common-axis
   Sphere/Sphere windows, exact nonparallel signed-axis and arbitrary-frame
   sphere octants, exact
   coincident coaxial Cone/Cone windows, and exact coincident Torus/Torus
   windows now
   emit paired polygonal or nonlinear-correspondence regions, tangent collapsed
   contacts, singular pole points, or complete empty evidence according to
   intersection dimension. Cone charts preserve affine correspondence across
   shifted reference origins and radii, transverse-frame phase, and reversed
   axes; regions split at the apex, isolated apexes are singular, and whole-
   region residuals are outward. Torus charts split both periodic axes,
   preserve antiparallel signed correspondence, and collapse to exact latitude
   or meridian circle branches and tangent points. Signed-axis periodic
   representatives are accepted
   only while their outward endpoint phase bound fits the active angular
   tolerance, so remote phase drift fails closed before it can change that
   dimension. Arbitrary-frame octants use robust six-halfspace topology and one
   private parameter allowance shared by nonlinear membership, anchor
   validation, and the whole-region residual. A first certified general-window
   arm now covers positive-area, pole-clear arbitrary-axis windows with
   longitude width below π. It checks all 28 boundary-plane pairs, requires
   interval-certified mutual membership, one connected degree-2 boundary
   cycle, and a strict interior witness, and preserves nonlinear source-window
   correspondence across containment, seam crossing, and operand swap. Wider,
   polar, non-exact tangent, multiple-cycle, and ambiguous cases stay `Indeterminate`.
   A fixed 112-witness arrangement scan now certifies disjoint supported
   windows: the pinned empty exemplar succeeds at 96 witnesses and fails at 95.
   Bit-exact opposing boundary planes additionally reduce one equality lock to
   interval-certified tangent circle arcs and two independent locks to tangent
   points; the pinned collapsed-curve exemplar succeeds at 12 arc witnesses and
   fails at 11. A first wide arm decomposes exactly one pole-clear wide operand
   into three closed sub-π cells. It returns `Complete` only when all cells are
   certified empty or exactly one positive region has two certified-empty
   siblings, which cancels every artificial seam before restoring the parent
   correspondence; piece/pair/arc ceilings pin exact 3/2, 84/83, and 336/335
   evidence. A two-wide empty-only arm now decomposes both parents into the
   same three closed sub-π cells and returns `Complete` only after all nine
   Cartesian child pairs are certified empty; exact 9/8 piece-pair, 252/251
   boundary-pair, and 1,008/1,007 arc ceilings preserve fail-closed admission.
   Multi-cell-positive two-wide, polar, and non-exact collapsed contacts are
   the next sphere boundaries.
   The K4 facade journal adapter now preserves mutation order, all semantic
   lineage shapes, and transaction-owned tolerance evidence. Semantic K4 edit
   transactions follow the K5 adoption pass. F6 splits and F4
   legacy cleanup land only with an owner-level behavioral migration. The Q2a/
   Q2b ladders are executable in CI; any graph-index/traversal representation change
   still requires a recorded stable-host before/after comparison.

No C ABI, plugin ABI, broad topology privacy break, speculative facade family,
or file-size-only module split is part of this convergence phase.

## Dependency outline

```text
F0 Completion-preserving result symmetry        (independent corrective fix)
F1 Procedural geometry graph                    (blocks procedural geometry)
F2 Operation context and numerical policy       (blocks generic solver growth)
F3 Intersection engine consolidation            (after F2 foundations; uses F1 types later)
F4 Kernel error and capability taxonomy         (independent, coordinate with F2/F3)
F5 Kernel facade and topology encapsulation     (after F1, F2, and F4 contracts)
F6 Shared surface services/module decomposition (independent first slice)
F7 Quality and performance harnesses             (independent and continuous)
```

The original independent foundations have landed. Work is no longer scheduled
as broad parallel expansion: Q8 made the harness protective; K5 tested the
facade against a consumer; the completed F2 profile/scale gates make bounded F3
fallback work eligible; X_T reconstruction and checked-commit Fast checking now
share one graph child in one scope. Contextual body tessellation now composes
projection and sequential graph/face work in one scope; its `ktopo`/`kxt`
production callers are contextual and its internal legacy-use ratchet is
enforced. Exact body edge-line and remaining structural-holder admission have
landed. The first graph-owned facade curve/curve family is adopted, and its
NURBS pair path now consumes exact isolation cells through bounded verified
discovery.
Facade body tessellation and corpus-backed bounded tessellation presets are
adopted; broader contextual intersection families remain.
The Q2a/Q2b ladders now protect graph construction, reverse indexing, and
dependency traversal through the current 1,000-edge procedural scale.

### Standing handoff ratchets

- Writer-reachable byte changes invalidate the affected licensed-host evidence;
  local read/write round-trip does not restore it.
- A proven contextual replacement closes the door to new crate-internal legacy
  calls even while source-compatible public wrappers remain.
- Excluded benchmark, fuzz, and Python tooling is protective only when its
  contracts run in CI.
- The facade-only lifecycle client keeps exactly `kernel` as its direct
  dependency, exercises graph-owned curve intersection as well as the original
  lifecycle, and the reviewed `kernel` package inventory stays enforced in CI.
- Large-import work exercises the graph-construction ladder; representation
  optimization includes a stable-host before/after measurement and preserves
  deterministic ordering.

## Reconciled F1/F2/F4 boundary

The geometry-graph and operation-context projects use one normative ownership
model:

- `kgeom` keeps total, context-free leaf evaluators for analytic and NURBS
  values.
- `kgraph` owns geometry handles, descriptors, dependency traversal, cycle
  detection, and a fallible per-query `EvalContext`.
- `OperationContext` owns immutable session/numerical/execution policy;
  `OperationScope` owns the top-level deterministic work ledger and ordered
  diagnostics.
- An operation scope deterministically reserves graph node-visit/depth work,
  then constructs a graph evaluator with that `EvalLimits` reservation and a
  copy of the operation's model-acceptance `Tolerances`.
- The graph evaluator owns no session policy, executor, cancellation contract,
  topology state, or operation diagnostic buffer. The operation context owns no
  graph handles, caches, cycle stack, or descriptor knowledge.
- F1 and F2 may introduce typed local evaluation/limit data. F4 standardizes
  stable capability, stage, and public error identifiers without erasing those
  distinctions or introducing graph types into `kcore`.

This contract is the integration gate for implementing either design. Changes
that create a second session/context abstraction require an explicit portfolio
revision.

## F0 — Completion-preserving result symmetry

**Purpose:** prevent operand-order normalization from weakening proof evidence.

**Scope:** add first-class curve/curve result swapping that preserves points,
overlaps, ordering, orientation, and completion; route reversed dispatch through
it; add symmetry regressions.

**Exit criteria:** complete hits and misses remain complete in either operand
order; indeterminate reasons survive swapping; all `kops` tests pass.

## F1 — Procedural geometry graph

**Purpose:** represent offset, intersection, swept, spun, and blend geometry as
exact dependent geometry without duplicating owned basis objects or introducing
topology-to-geometry dependency cycles.

**Scope:** define graph ownership and handles, serializable descriptors, a
fallible evaluation context, dependency traversal and cycle rejection, class
identity, and integration boundaries for `ktopo`, `kops`, and `kxt`. Prove the
design with the narrow offset-surface import/evaluation slice.

**Non-goals:** general caching, concurrency optimization, every procedural
class, or a public plugin ABI.

**Exit criteria:** an imported offset surface references its basis surface by
handle, evaluates position/derivatives through a typed context, rejects cycles
deterministically, remains exactly classifiable for X_T, and is consumable by a
topology face without owned surface duplication.

## F2 — Operation context and numerical policy

**Purpose:** stop model tolerances, solver conditioning thresholds, proof limits,
and fixed work caps from becoming unrelated per-module policy.

**Scope:** define the context and policy types, ownership/lifetime rules,
deterministic work accounting, structured limit diagnostics, and a staged
migration for intersections, checker proofs, construction, projection, and
tessellation.

**Non-goals:** making the Parasolid model-space regime arbitrarily configurable,
introducing nondeterministic cancellation behavior, or tuning all algorithms in
the first change.

**Exit criteria:** one representative intersection and one refinement/checking
algorithm consume explicit policy; defaults reproduce existing golden results;
limits are test-overridable and failures report stage plus consumed/allowed
work.

**Current convergence gate:** operation-family composition and the
scale-sensitive contact/minimizer gate are complete. Contextual graph
evaluation and checked commit use the same scope/child-reservation model.
Projection's standalone contextual entries have landed and body tessellation
now consumes them in one shared scope. Body production callers and its
internal-use ratchet are complete. Projection caller adoption/ratcheting,
other intersection-family incomplete-evidence and minimizer migrations,
hostile-input tessellation allocation bounds, and facade construction
composition remain.

## F3 — Intersection engine consolidation

**Purpose:** keep analytic special cases while preventing quadratic dispatch and
helper duplication from becoming the architecture.

**Scope:** introduce stable geometry-class inspection, centralized pair
normalization/swapping, shared range and periodic-parameter utilities, shared
candidate deduplication/emission, and one generic certified fallback contract.
Migrate one curve/curve family and one surface/surface family before expanding.

**Non-goals:** rewriting correct closed-form solvers or completing every NURBS
case in the same project.

**Dependencies:** F2 policy types; coordinate descriptor identity with F1.

**Exit criteria:** adding a new geometry class does not require hand-writing both
operand orders; specialized and fallback paths return the same result contract;
completion and structured limits survive dispatch transformations.

## F4 — Kernel error and capability taxonomy

**Purpose:** let callers and metrics distinguish invalid input, unsupported valid
input, incomplete proof, exhausted resources, and violated invariants without
parsing diagnostic strings.

**Scope:** define stable capability/stage identifiers, structured algorithm-limit
data, and layer-appropriate error/outcome boundaries. Migrate intersection
dispatch and one topology/checking path. Retain human-readable context.

**Exit criteria:** unsupported geometry is not `InvalidGeometry`; limit telemetry
is machine-readable; X_T wrapping retains kernel classifications; C-ABI mapping
can be defined without inspecting strings.

## F5 — Kernel facade and topology encapsulation

**Purpose:** give future application, bindings, and feature-history clients a
stable conceptual API without exposing arena layout and backlink vectors.

**Scope:** introduce a thin `Kernel` or `Session` facade, read-only entity views
and deterministic iterators, operation request/result types, and an explicitly
unstable low-level assembly boundary for interchange. Gradually privatize raw
topology fields where cross-crate construction no longer requires them.

**Dependencies:** stable first versions of F1, F2, and F4.

**Exit criteria:** ordinary clients can construct, query, mutate transactionally,
and export a body without importing raw entity structs; `kxt` still reconstructs
atomically; compile-fail tests protect raw mutation boundaries.

## F6 — Shared surface services and responsibility splits

**Purpose:** remove semantic drift before splitting large modules for size alone.

**First slice:** centralize analytic surface inversion/projection, periodic base
chart normalization, and point-to-surface distance in `kgeom`; migrate checker
and body tessellation to it.

**Later slices:** separate structural/incidence/domain/shell checking;
boundary/chart/triangulation tessellation; and X_T
planning/emission/serialization only when the corresponding contextual or
adoption work establishes a tested seam. File size alone is not a split
criterion.

**Exit criteria:** checker and tessellator share one inversion implementation and
the same class coverage; focused tests cover seams, singularities, and NURBS
projection; later moves are behavior-preserving.

## F7 — Quality, fuzzing, and performance harnesses

**Purpose:** make robustness and asymptotic expectations executable before broad
modeling operations land.

**Scope:** pin the Rust toolchain/MSRV; add benchmark ladders for checked commit,
index refresh, tessellation, implicit and curve-pair NURBS isolation, and X_T I/O; add initial fuzz
targets for X_T parsing, NURBS constructors, result canonicalization, and
transaction/Euler sequences; retain minimized regressions.

**Exit criteria:** benchmarks have named fixtures and recorded baselines; fuzz
targets run locally and in bounded CI smoke jobs; toolchain changes are explicit;
no benchmark depends on wall-clock ordering for correctness.

## Integration rules

Each project must state which capability changes, whether results are complete
or indeterminate, which tolerances and work budgets apply, how failure atomicity
is verified, what journal/checker evidence is produced, and which deterministic
or performance regression protects it. Cross-project shared types should land
in small contract commits before broad migrations.

During convergence, new production code must use the F2/F4 contracts, but F4
does not run a repository-wide cleanup campaign. Remaining legacy call sites
migrate opportunistically with their owning behavior change.
