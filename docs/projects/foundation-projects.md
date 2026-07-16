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
| F1 | G1-G4a plus the F2 graph-budget adapter are implemented: `kgraph` and `ktopo::Store` own one transactional geometry graph; exact offsets evaluate through accepted/attempted node/depth accounting, check, tessellate, and round-trip through the declared X_T subset without basis duplication. Reverse dependencies use deterministic insertion-ordered adjacency with direct key/membership lookup and no full-order rebuilds; traversal keeps vector-defined output/path order with indexed active/completed membership. Q2a/Q2b preserve exact graph, index, traversal, rollback, and accounting evidence; the Q2a v2 production-descriptor diamonds also pin deduplicated shared-basis closure. G5a supplies invertible carrier/pcurve maps plus private-field whole-interval certificates for plane/plane lines and both common-axis and genuinely oblique plane/sphere circles. Its graph-aware exact-field adapter preserves source handles, raw-result parity, typed stale/unsupported failures, canonical swap behavior, and deterministic certified branch vertices/edges. Direct planes/spheres and context-accounted constant-offset chains share these arms while every sphere-chain radius remains positive and finite. The common-axis fast path retains exact `t`/`-t` longitude maps across shifted, seam-crossing, full-turn, and overwide finite windows. General finite secants use a stable nonperiodic `SphericalCircle` pcurve with analytic derivatives through order three, continuous seam unwrapping, conservative finite bounds, whole-branch pole clearance, and typed chart-window rejection. Certified line and circle branches persist atomically as stable `Intersection` curve descriptors with ordered source/pcurve dependencies and their paired proof; altered or stale proof sources fail before allocation or roll the batch back. Direct Plane/Sphere or safe finite Offset(Plane)/Offset(Sphere) fields against genuinely non-planar direct NURBS, plus positive-area-clipped compatible genuinely non-planar direct NURBS/NURBS and one- through four-descriptor constant-normal Offset(NURBS)/NURBS unit charts, now run the existing marcher in one owner scope, retain paired degree-1 traces, certify both lifts at fixed depth 10, and persist the original ordered source identity in a distinct non-transmitted verified-NURBS descriptor atomically. The NURBS/NURBS and Offset(NURBS)/NURBS arms use rounded derived scalar surfaces only for discovery; outward original-control differences own complete misses. The Offset(NURBS) arm retains the live outer root, accumulated signed distance, terminal original constant-normal basis, direct peer, and paired pcurves while protecting the capped basis chain transitively; one, two, three, and four offset descriptors pin exact 2/depth-2, 3/depth-3, 4/depth-4, and 5/depth-5 node visits and dependency depth with N/N-1 admission. One exact rational-quarter-cylinder varying-normal root against a canonical global-X-, global-Y-, or global-Z-normal planar direct NURBS, analytic Plane, or one safe Offset(Plane) descriptor over a direct Plane basis adds a whole-window original-derivative normal proof and true parallel-surface discovery; the direct analytic Plane peer alone admits the complete one- through four-descriptor family after every intermediate and final radius proves finite and positive. X/Y generators pin combined 14343/14342 Work and 1024/1023 Items, while the certified 40-span Z-normal quarter-circle pins 573447/573446 Work and 40960/40959 Items; analytic-Plane X/Y peers pin 7177/7176 Work and 1024/1023 Items while their Z peer pins 286768/286767 Work and 40960/40959 Items; all chain lengths retain these unchanged certificate budgets and 10/9 certificate Depth. The direct-Plane arm consumes exact graph Work/depth 2, 3, 4, or 5 for one through four descriptors with N/N-1 admission; planar-NURBS and safe Offset(Plane) peers remain one-descriptor only, with the latter retaining live root/basis/distance identity at 4/3 graph Work and 2/1 dependency Depth. All retain the live root, original basis, peer, and paired pcurves; the proof retains every exact outer-to-inner distance, uses its derived rational effective sheet only for discovery while the original basis owns proof, and pins swap, persistence, complete misses, and atomic rollback for per-descriptor, same-sum, and stale-source mutations. Compatible pairs of independent one- through four-descriptor constant-normal Offset(NURBS) roots now cover the complete intersecting 4x4 matrix with one original-source-certified branch in either order, retain both live roots and transitive basis chains, and pin exact 14336/14335 Work, 1024/1023 Items, and 10/9 certificate Depth. Both this positive arm and the retained strict-separated graph-owned complete miss pin exact A+B+2 graph Work and max(A+1,B+1) dependency depth, including maximum 10/9 Work and 5/4 Depth admission; only the miss uses zero certificate budget and allocates no persistence. Five-or-more descriptors and incompatible or coincident effective sheets fail closed, as do altered or stale proof sources. Sphere-offset traces bind the effective sphere while retaining the ordered root and protecting direct or nested bases transitively; a direct root pins exact 2/1 node-visit and dependency-depth boundaries. M3c uses the same ownership seam for canonical finite-open X_T `INTERSECTION` charts whose two ordered exact-plane sources may each be direct or a safe finite offset chain; two-offset roots must have independent basis chains. Transmitted positions and modern paired UVs persist as a degree-1 carrier plus pcurves with a whole-span residual certificate; the proof binds effective planes while retaining actual ordered source handles and protecting every transitive offset basis. Plane/B-surface, safe-Offset(Plane)/B-surface, B-surface/B-surface, and direct constant-normal Offset(B-surface)/B-surface charts use one persistent NURBS-intersection descriptor and whole-range original-source interval certificate for each polynomial or rational NURBS trace; the bounded finite-open two-sample line, three-sample quadratic, four-sample cubic, five-sample polyline, and seven-sample polyline Offset(B-surface)/Offset(B-surface) slices use the same descriptor with their common degree-2, degree-3, or degree-1 carrier/pcurve basis and two independent original-source offset-NURBS interval proofs; a plane trace may bind a direct plane or safe finite offset chain, while an Offset(B-surface) trace retains the live root, signed distance, and original basis and proves its whole-box unit-normal field; every ordered root and transitive basis remains protected. A bounded finite-open two- through five-sample direct-Plane/B-surface, safe-Offset(Plane)/B-surface, direct-Plane/Offset(B-surface), direct constant-normal Offset(B-surface)/direct B-surface, independent direct one-descriptor Offset(B-surface)/Offset(B-surface), or direct-B-surface/B-surface slice retains finite positive noncanonical affine metadata on the canonical shared sample-index basis while preserving the unchanged original-source proof and ordered dependencies. The direct Offset(B)/B and independent direct Offset(B)/Offset(B) slices cover operand swap and polynomial/rational bases at exact `14336/2/10`, `28672/3/10`, `43008/4/10`, and `57344/5/10` Work/Items/Depth. The dual-offset arm retains both live roots, signed distances, independent direct bases, paired UVs, and the unchanged two-source proof; nested, shared-basis, multi-offset, null/mixed, and out-of-range noncanonical forms remain unsupported. Certified clamped periodic/closed B-surfaces now use BODY-toleranced polynomial or exact-homogeneous rational position/C1 seams, wrapped evaluation, seam-aware bounds, and writer-preserved flags. Those three leaves, the first constant-normal `Offset(B-surface)/B-surface` chart, the one-period equal-limit records 1828 and 2008 plus exact synthetic distinct same-point `H/?` limits and a non-endpoint interior alias range, the finite-open/end-terminated `T/F` records 1671 and 1678, finite-open direct B-surface/Plane record 1252 and Plane/Offset(B-surface) record 5089 with paired-null interior Plane UVs plus a canonical synthetic endpoint-null variant, native direct-Plane `SP_CURVE` node 30, nonperiodic endpoint-roundoff record 1984, canonical quadratic dual-offset records 5945 and 3790, canonical cubic dual-offset record 3819, exposed 11-sample Plane/Offset record 3745, seven-sample dual-offset polyline record 3615, independently certified five-sample polyline record 4230, and two-sample dual-offset line records 3595 and 6044 now certify. Historical v5 remains exact at `117478445/20/10` Work/Items/Depth and stops on the later 118,406,196-Work chart attempt; production v6 derives FACE 1195's vertex-less ring domain through certificate-owned periodic carrier semantics at exact `208228426/22/10` and pins its next denied attempt at 221,060,174 Work; production v7 certifies record 5089 by exact interior and synthetic endpoint Plane-UV recovery plus the whole-carrier Plane/Offset-NURBS proof at `272430166/22/10` and pins INTERSECTION 1984's 285,283,414-Work attempt; historical v8 certifies record 1984 through endpoint-only nonperiodic NURBS source-boundary normalization plus the unchanged whole-carrier proof at `315245660/22/10`, then stops at INTERSECTION 5945's 323,814,492-Work proof preflight; production v9 admits record 5945's exact quadratic three-sample dual-offset proof at `323814492/22/10` while preserving v1-v8 parity; production v10 admits record 3819's exact cubic four-sample dual-offset proof at `336759900/22/10`, with isolated `12945408/4/10` accounting and v1-v9 parity; production v11 safely normalizes null or finite-numeric zero-multiplicity nonperiodic source-knot padding, admits quadratic record 3790 at isolated `8593408/3/10`, exposes 11-sample Plane/Offset record 3745 at isolated `42772491/11/10`, and reaches `388125799/22/10` with v1-v10 parity; production v12 admits seven-sample dual-offset record 3615 at isolated `26443776/7/10` and reaches `414569575/22/10` with v1-v11 parity; two-sample record 3595 independently certifies at `4352000/2/10` but is not reached; production v13 admits five-sample record 4230 at isolated `17285120/5/10` and exact `431854695/22/10` with v1-v12 parity; production v14 admits two-sample direct Plane/Offset record 3609 at isolated `4277250/2/10` and exact `436131945/22/10` with v1-v13 parity; production v15 admits two-sample dual-offset record 6044 at isolated `4352000/2/10` and exact `440483945/22/10` with v1-v14 parity, then stops before four-sample dual-offset record 5921's cumulative `454258793`-Work request; at that exact attempted budget, the canonical cubic first pcurve materially leaves its original open nonperiodic source domain, so certification retains the v15 report and rolls back atomically. The historical 14-file writer bundle is host-certified and machine-fingerprinted; the declared set now adds a locally verified genuinely curved B-surface fixture, and the certification remains stale pending the complete 15-file licensed-host rerun. Broader G4 corpus coverage, descriptor-chain depth five or greater, multi-descriptor planar-NURBS/Offset(Plane) peers, nested-dual, or other varying-normal Offset(NURBS)/NURBS, broader NURBS/NURBS charts and further procedural G5 arms, further carrier families, broader persistent descriptor families, and the exemplar's broader procedural-curve families, null, mixed/non-`H`, or broader closed limits, remaining nullable chart-data, ambiguous or multi-period trace aliases, and noncanonical transmitted-chart variants outside the bounded affine direct-Plane/B-surface, safe-Offset(Plane)/B-surface, direct-Plane/Offset(B-surface), direct Offset(B-surface)/direct B-surface, independent direct one-descriptor Offset(B-surface)/Offset(B-surface), and direct-B-surface/B-surface slices; nested, shared-basis, multi-offset, null/mixed, and out-of-range forms plus arbitrary unclamped cyclic B-geometry remain. |
| F2 | Stage 1, Stage 1b composition, the bounded NURBS/NURBS Stage 3 scale gate, two Stage 2 pilots, and contextual face/body-tessellation, projection, checker, and generic curve/curve entries are implemented. `OperationContext` owns family-default < session < request budget precedence for graph evaluation, Full checking, tessellation, projection, exact curve-pair and surface-patch isolation, overlap-equivalence Work/Items admission, and bounded cell-local seed attempts, including canonical root stops and accounting-mode validation. Whole-body tessellation owns one scope across graph evaluation, projection fallback, refinement/storage, per-patch work, and retained output. X_T reconstruction and generic curve/curve dispatch account owned iterative work through one scope; exact compatibility, N/N+1 limits, numerical-policy propagation, rollback/completion evidence are pinned. The exact-plane X_T chart profile pre-admits two certificate Work units per position, retained-position Items, and one proof Depth; direct, one-offset, and two-offset two-position fixtures pin exact `4/2/1` N/N-1 crossings before allocation; direct two-offset roots consume 34 graph visits/depth 2 and two nested roots consume 36/depth 3. Plane/B-surface and Offset/B-surface charts pre-admit `N+(N-1)*2^10*(6R+1)` Work, `N` Items, and Depth 10 from validated source dimensions; the one-span fixture remains exact at `7170/2/10`; the bounded noncanonical affine record-5089 variant pins cumulative `139792442/4/10` with exact per-resource N/N-1 rollback; the corresponding direct Plane/B-surface structural family pins `7170/2/10`, and its five-sample record-1252-derived variant pins cumulative `127115320` Work; bounded noncanonical direct B/B, direct constant-normal Offset(B)/B, and independent direct one-descriptor Offset(B)/Offset(B) pin `14336/2/10`, `28672/3/10`, `43008/4/10`, and `57344/5/10`. B/B and both direct offset slices sum both original-source trace proofs; canonical one-span pairs pin `14336/2/10`, and each offset trace additionally proves a positive whole-box normal bound while binding its live root, signed distance, and basis; the dual arm retains independent bases, ordered paired UVs, and operand-swap identity. Historical X_T import profile v1 remains at 131,072 Work, v2 at its pre-equal-limit 81,267,732-Work boundary, v3 at exact equal-limit `115485725/20/10`, v4 at exact end-terminator `116396069/20/10`, and v5 at exact finite-open omitted-Plane-data `117478445/20/10`; v5 next attempts 118,406,196 Work, while historical v6 pins exact native-Plane-SP-curve `208228426/22/10` admission and its 221,060,174-Work next attempt, owner reconstruction uses corpus-backed v7 at exact finite-open Plane/Offset(B-surface) `272430166/22/10` admission before the 285,283,414-Work next attempt, corpus-backed v8 at exact endpoint-normalized nonperiodic NURBS `315245660/22/10` admission before the 323,814,492-Work next attempt, corpus-backed v9 at exact quadratic finite-open three-sample dual-offset `323814492/22/10` admission with historical v1-v8 parity, corpus-backed v10 at exact cubic finite-open four-sample dual-offset `336759900/22/10` admission with isolated `12945408/4/10` accounting and historical v1-v9 parity, corpus-backed v11 at exact `388125799/22/10` after strict zero-multiplicity null-or-finite source-knot normalization admits quadratic record 3790 (`8593408/3/10`) and exposed 11-sample Plane/Offset record 3745 (`42772491/11/10`) with historical v1-v10 parity, corpus-backed v12 admits seven-sample dual-offset record 3615 at exact `414569575/22/10` with isolated `26443776/7/10` accounting and historical v1-v11 parity; separately transplanted two-sample record 3595 pins isolated `4352000/2/10`; corpus-backed v13 admits five-sample record 4230 at isolated `17285120/5/10` and exact `431854695/22/10` with historical v1-v12 parity; and corpus-backed v14 admits two-sample direct Plane/Offset record 3609 at isolated `4277250/2/10` and exact `436131945/22/10` with historical v1-v13 parity; corpus-backed v15 admits two-sample dual-offset record 6044 at isolated `4352000/2/10` and exact `440483945/22/10` with historical v1-v14 parity, then pins the next atomic stop at record 5921's cumulative `454258793`-Work request; an exact-budget regression proves the subsequent original-domain certificate rejection, retained v15 report, and atomic rollback. Records 2008 and 1678 independently pin `124040223/22/10` and `116413476` Work. Exact overlap scans, common-knot reconstruction, and bounded checked inverse-refinement state pre-admit conservative logical work and temporary items; per-resource N/N-1 crossings return structured indeterminate evidence before allocation. Curve-pair range boxes pre-admit every inspected original-source knot-span slot; the one-span depth-six default is 6,828 units. Surface source bounds now admit `R*(6R+1)` before contextual BVH build and `1+4*(6R+1)` before each adaptive parent, including repeated/empty tensor slots; denial retains the source cover. Each oblique spherical-pcurve certificate pre-admits exactly 128 proof-subdivision Work units, and its composed graph-surface profile pins exact N/N-1 crossings alongside safe-offset node/depth accounting. The same profile composes exact direct/safe-Offset(Plane)-field/NURBS, direct/safe-Offset(Sphere)-field/NURBS, compatible direct-NURBS/NURBS, and one- through four-descriptor constant-normal Offset(NURBS)/NURBS marching. Plane traces precharge `C + S*2^10*(6T+1)` certificate Work; the curved one-segment fixture pins exact 7,170/7,169 and one Offset(Plane) pins exact 2/1 graph visits. Sphere traces precharge `S*2^10*(6T+2)` and observe paired proof cells/depth; the one-segment fixture pins exact 8,192/8,191 Work, 1,024/1,023 Items, and 10/9 Depth. Positive-area-clipped compatible direct NURBS/NURBS and one- through four-descriptor Offset(NURBS)/NURBS traces precharge `S*2^10*((6R_a+1)+(6R_b+1))`; their one-span fixtures pin exact 14,336/14,335 Work with the same 1,024/1,023 Items and 10/9 Depth, while two descriptors pin exact 3/2 node visits and dependency depth, three pin exact 4/3, and four pin exact 5/4. The rational-quarter-cylinder varying-normal arm with canonical global-X-, global-Y-, and global-Z-normal planar-NURBS, direct-Plane, or one-descriptor safe-Offset(Plane) peers adds exactly 7 Work, 1 Item, and Depth 1 for its original-derivative normal proof; only the direct-Plane peer admits the complete one- through four-descriptor family after every intermediate and final radius proves finite and positive. Planar-NURBS peers produce combined 14,343/14,342 Work and 1,024/1,023 Items for X/Y while their 40-span Z branch pins 573,447/573,446 Work and 40,960/40,959 Items; direct-Plane and safe-Offset(Plane) peers pin 7,177/7,176 Work for X/Y and 286,768/286,767 for Z with the same Items. Every admitted chain length retains those certificate budgets and 10/9 certificate Depth; one through four direct-Plane descriptors pin exact graph Work/depth 2, 3, 4, and 5 with N/N-1 admission, while planar-NURBS and safe-Offset(Plane) peers remain one descriptor only and the latter pins 4/3 graph Work and 2/1 dependency Depth. Per-descriptor, same-sum, and stale-source denial remains allocation-clean. The compatible planar constant-normal dual Offset(NURBS) arm covers every intersecting 1–4×1–4 pair with an original-source certificate at exact 14336/14335 Work, 1024/1023 Items, and 10/9 Depth while retaining both roots and transitive basis chains. It shares exact A+B+2 graph Work and max(A+1,B+1) dependency depth with the retained strict-separated complete-miss arm, including maximum 10/9 Work and 5/4 Depth admission; only the miss uses no branch-certificate work or persistence allocation. Failed residual proof retains attempted resources. Internal graph-owned facade coverage pins distinct checked-ancestor identities, clipped reversed extents, exact Work/Items N admission, and isolated per-resource N-1 evidence. Q4 curve isolation v4, implicit isolation v3, and solve v18 record source-range enclosure/certificate digests, multi-span surface Work, primitive-integer completion through magnitude twelve, common-refinement and inverse-history recovery, altered-history rejection, and independent overlap Work/Items denial. Body/standalone-face tessellation and both standalone projection wrappers are closed to new production callers by the CI retirement ratchet. Corpus-backed face/body `bounded_v1` presets now carry finite stage and root caps; all matrix rows pass, exact observed root N/N-1 crossings are pinned, and facade/X_T clients opt in without changing compatibility defaults. Segment conditioning, input/dedup slack, other intersection-family minimizer/evidence migrations, and broader migrations remain. |
| F3 | Implemented slices include centralized class inspection; shared periodic/scalar fitting and first-wins candidate emission; shared finite-range validation and window fitting; canonical unordered-pair normalization; typed one-arm-per-pair routing; contextual/shared-scope generic curve/curve dispatch; source-provenanced NURBS curve-pair isolation with bounded verified polishing; and unique exact transverse-root certificates. Candidate cells retain shared original-curve provenance: rounded split controls are partition/seed machinery only. Curve-pair exclusion and stored bounds use outward interval position enclosures over original-source ranges, tightened by the conservative whole-source hull and failed open to it. A cubic/line adversary whose exact shared midpoint rounds outside every generated child hull proves that subdivision cannot create a false empty result. Partial certificates use direct outward interval de Boor bounds of the original homogeneous source and derivative B-splines over each requested knot span. Source-range Poincare face signs plus a range-local P-matrix cover coplanar roots; noncoplanar existence includes bounded exact `{mid,lo,hi}` samples, in-range full-multiplicity knots, and algebraic parameter-correspondence lifts. Exact same/reversed normalized knots, globally proportional positive rational weights, and a strictly monotone source-exact carrier remain mandatory. Canonical primitive integer carriers and omitted-coordinate residuals through coefficient magnitude twelve remain the compatibility proof family; an explicit validated search configuration admits the magnitude-thirteen and magnitude-fourteen shells without changing that default. The magnitude-twelve enumeration remains an exact stable prefix and every through-thirteen enumeration and certificate golden is unchanged. The fourteen ceiling owns exactly 254 carrier forms and 9,825 residual forms, a shell delta of 24 and 1,704 over thirteen; its noncoplanar normalized-1/3 fixture is uncertifiable at every explicit ceiling through thirteen and certifies at fourteen. Direct homogeneous derivative bounds preserve correlation through the reviewed magnitude-fourteen corridor, while broken, overflowing, non-finite, out-of-range, or invalid-ceiling inputs fail closed. Surface BVH, plane/implicit pruning, adaptive children, and `NurbsSurface::bounding_box` now evaluate source rectangles by outward tensor interval de Boor, active source-support hulls, and a centered derivative mean-value bound; an exact cubic extrusion plus exact source-rectangle Work N/N-1 coverage prove rounded child hulls and denied scans cannot erase a real contact. Exact affine parameterizations, common-knot reconstruction, and checked exact-reinsertion ancestors produce complete clipped same/reversed overlap extents; altered refinements and sampled near-coincidence stay indeterminate. Curve/curve and surface/surface swapping restore canonical first-operand ordering without weakening completion evidence. Bounded coincident Plane/Plane, exact coincident Cylinder/Cylinder, exact common-axis Sphere/Sphere windows, exact nonparallel signed-axis and arbitrary-frame Sphere/Sphere octants, exact coincident coaxial Cone/Cone, and exact coincident Torus/Torus windows now return canonical paired regions with chart orientation and outward whole-region residual bounds; cone regions preserve affine chart correspondence and split at the apex, torus regions split both periodic axes and collapse to exact latitude/meridian circles or tangent points, common-axis windows remain seam-aware rectangles, and sphere octants retain explicit nonlinear bidirectional chart correspondence, exact robust halfspace topology, and exact physical boundary anchors. The first general non-octant arbitrary-axis arm covers positive-area pole-clear windows narrower than π: all 28 boundary-halfspace pairs, one connected degree-2 cycle, and a strict interior witness are mandatory before Complete, while containment, seam crossing, and swap retain authoritative nonlinear window correspondence. A fixed scan of at most 112 arrangement arcs returns Complete empty only after excluding every boundary component; its disjoint exemplar pins exact 96/95 witness evidence. Bit-exact opposing boundary planes collapse one equality lock to interval-certified tangent circle arcs and two independent locks to interval-certified tangent points; the collapsed-curve exemplar pins exact 12/11 arc-witness evidence. A bounded polar arm decomposes exactly one sub-pi bit-exact natural-pole source window into one closed pole-clear latitude cell plus one closed cap against a pole-clear sub-pi peer; Complete requires both cells empty or exactly one occupied with one certified-empty sibling, which excludes the artificial latitude seam before parent correspondence is restored. The cap omits the redundant degenerate pole-latitude constraint, retains one canonical singular anchor, and canonicalizes every exact source-pole longitude alias before frame mapping. Its triangular-cap exemplar pins exact repeat/swap, outward residuals, and 2/1 piece, 49/48 pair, and 196/195 arc ceilings; one-ULP near-poles, double- or wide-polar inputs, boundary tangencies, and two occupied cells fail closed. A polar-by-wide arm crosses the same two latitude cells with three closed longitude cells from exactly one pole-clear wide peer and completes for all six empty, one occupied child with five certified-empty siblings, or one exact adjacent same-row pair with four certified-empty siblings, or an exact same-column vertical pair with four certified-empty siblings and strict bit-exact latitude-seam cancellation (reviewed at `[0,2]`/`[1,2]`), or either exact full latitude row with three certified-empty opposite-row siblings, with the non-cap row retaining eight vertices, or one exact mixed-axis three-cell L path with the other three siblings empty in the reviewed cap-right `[0,2]`/`[1,1]`/`[1,2]` and lower-middle `[0,1]`/`[0,2]`/`[1,1]` orientations, or any exact connected four-positive grid path, lower/upper-stem T tree, or left/right 2×2 cycle with two empty siblings, or the exact disconnected outer-column vertical pairs with a certified-empty middle column, or any of the four exact isolated-corner plus three-cell mixed-axis L layouts with both omitted graph-cut siblings certified empty, or exactly five positive children with the sole sibling certified empty, or the exact all-six-positive 2×3 union with no empty sibling. Its `[1,1]` three-edge and `[1,0]`/`[1,1]` five-edge exemplars preserve the canonical singular alias; each L cancels one longitude and one latitude seam. Three disjoint four-positive routes are keyed by degree sequence. The path arm owns `2,2,1,1`, cancels its three exact seams, and has real cap-row-right `[0,2]`/`[1,0]`/`[1,1]`/`[1,2]` and zigzag `[0,1]`/`[0,2]`/`[1,0]`/`[1,1]` repeat/swap fixtures. The T arm owns `3,1,1,1` and admits only the exact lower-stem `[0,0]`/`[0,1]`/`[0,2]`/`[1,1]` and upper-stem `[0,1]`/`[1,0]`/`[1,1]`/`[1,2]` trees with two certified-empty siblings; it simultaneously proves and removes exactly three reverse bit-exact seams, requires one outer cycle with no artificial seam, restores the parent map/residual, and pins repeat/swap plus one-ULP and ambiguity rejection. The cycle arm owns `2,2,2,2` and admits exact left/right 2×2 cycles with two certified-empty siblings only after all four reverse bit-exact adjacencies prove simultaneously; it removes all four, requires one outer cycle with no artificial seam, restores the parent map/residual, and pins repeat/swap plus one-ULP and ambiguity rejection. One disconnected route admits `[0,0]`/`[0,2]`/`[1,0]`/`[1,2]`, the two outer-column vertical pairs, with middle-column `[0,1]`/`[1,1]` certified empty. It merges both exact latitude seams into exactly two canonical regions, excludes both longitude separators, restores the parent maps and maximum child/parent residuals, and pins repeat/swap, exact 6/5 piece, 147/146 pair, and 588/587 arc N/N-1 ceilings, plus one-ULP and ambiguity rejection. A second disconnected route admits all four isolated-corner plus three-cell mixed-axis L layouts: `[0,0]` + `[0,2]`/`[1,1]`/`[1,2]`, `[1,2]` + `[0,0]`/`[0,1]`/`[1,0]`, `[1,0]` + `[0,1]`/`[0,2]`/`[1,2]`, and `[0,2]` + `[0,0]`/`[1,0]`/`[1,1]`; respectively the omitted graph-cut sibling pairs `[0,1]`/`[1,0]`, `[0,2]`/`[1,1]`, `[0,0]`/`[1,1]`, and `[0,1]`/`[1,2]` certify empty. It proves and removes both reverse-oriented bit-exact L seams, requires zero occupied-boundary contact with every empty cut separator and no bit-exact contact between the singleton and merged component, returns exactly two canonical regions with restored parent maps and maximum child/parent residuals, and pins repeat/swap, the same exact 6/5 piece, 147/146 pair, and 588/587 arc N/N-1 ceilings, plus one-ULP and ambiguity rejection. Together the two routes exhaust the exact disconnected four-positive graph layouts in this polar-by-wide 2×3 decomposition. The exactly-five-positive arm simultaneously removes every internal reverse-oriented bit-exact seam and requires one unambiguous outer cycle; `[0,0]` corner-empty cycle-plus-tail and `[1,1]` edge-middle-empty tree fixtures pin repeat/swap, parent-map restoration, residuals, complete internal-seam removal, and one-ULP/ambiguity rejection. The exact all-six-positive arm has no empty sibling, simultaneously proves and removes all seven reverse bit-exact 2×3 adjacencies, requires one unambiguous outer cycle with no artificial seam, and restores the parent map and residual; its real `0.14716980102990423`-tilt fixture pins repeat/swap plus one-ULP and ambiguity rejection. Every admitted layout retains the exact 6/5 piece, 147/146 pair, and 588/587 arc ceilings. A bounded wide arm decomposes exactly one pole-clear wide operand into three closed sub-π cells and completes only for three certified-empty cells or one positive region with two certified-empty siblings, pinning exact 3/2 piece, 84/83 pair, and 336/335 arc ceilings. A second arm decomposes both pole-clear wide operands into a Cartesian 3×3 grid and completes only after all nine child intersections certify empty, exactly one child owns one positive region while its eight siblings certify empty, or exactly two children each own one positive region while the other seven certify empty, or exactly three positive children are pairwise non-edge-adjacent while all six siblings certify empty, or exactly three positive children comprise one exact adjacent pair plus one isolated component while all six siblings certify empty, or exactly three positive children form a two-edge grid path while the other six certify empty, or exactly four positive children form a three-edge grid path or one connected 2×2 cycle while the other five certify empty, or exactly five positive children form a four-edge grid path or an exact connected non-path union while the other four certify empty, or exactly six positive children form an exact connected path or non-path union while the other three certify empty, or exactly seven positive children form one exact connected union while the other two certify empty, or exactly seven positive children form an exact disconnected occupied-corner singleton plus connected-six-cell family in all four rotations, with both orthogonal neighbor cells certified empty, zero occupied-boundary contact at every empty separator, simultaneous exact cancellation of the six-cell component's six internal seams, no surviving artificial seam or bit-exact inter-component contact, exactly two canonical parent-mapped regions with maximum child/parent residual propagation, connected-seven precedence, repeat/swap, exact 9/8 piece, 252/251 pair, and 1,008/1,007 arc N/N-1 evidence, and one-ULP or duplicate-edge ambiguity rejection, or exactly eight positive children form one exact connected union while the sole other cell certifies empty, or exactly nine positive children form one exhaustive exact connected union, under sub-full-turn parent charts; two or three pairwise non-edge-adjacent children remain separate after closed-cell sibling emptiness excludes artificial seams and diagonal corner contact through both certified-empty orthogonal owners; a mixed three-cell layout merges its sole exact adjacent pair while retaining its certified-separated singleton in canonical grid order; path-connected children merge only from reverse-oriented shared boundary edges with exactly two consecutive bit-identical endpoint records before restoring parent correspondence; the four- through nine-cell non-path arms prove every internal seam through reverse-oriented bit-exact pairs or the bounded one-owner complementary-chart rule, cancel those edges, and require one unambiguous outer cycle; the five-cell 2×2-cycle-plus-tail exemplar has twelve boundary edges, the connected six-cell exemplar has fourteen, the exact connected seven-cell 2×3-block-plus-tail and opposite-corner-empty exemplars have fifteen and seventeen boundary edges respectively, the disconnected seven-positive corner-singleton exemplars return exactly two canonical regions, the exact eight-cell edge-empty and corner-empty exemplars have eighteen and seventeen boundary edges respectively, while the exhaustive exact nine-cell exemplar has seventeen, and an exact sibling-separated disconnected five-cell singleton-plus-block fixture retains canonical 3- and 8-edge components, while a physically coincident but bit-mismatched central seam fails closed. It pins exact 9/8 piece-pair, 252/251 boundary-pair, and 1,008/1,007 arc ceilings. Other seven-positive layouts outside the exact connected and corner-singleton-plus-six-component families, other eight- or nine-positive layouts, disconnected five-cell layouts without exact sibling separation, non-exact or ambiguous multi-edge shared-seam, or full-turn two-wide unions, other polar layouts outside the admitted exact adjacent same-row/same-column, full-row, mixed-axis three-cell-path, connected four-positive path/T/cycle, disconnected four-positive graph, exactly-five-positive, and all-six-positive families, non-exact tangent, ambiguous, and multiple-cycle cases remain Indeterminate. Edge and point contacts collapse dimensionally, cone apexes and sphere poles remain singular, and disjoint windows or octants are proven empty. The shared complete-support-curve SSI emitter owns clipping, periodic membership, candidate reacceptance, and first-wins dedup for cylinder/cylinder, cone/cylinder, cone/cone, cone/sphere, cylinder/sphere, and sphere/sphere with byte-identical representative results. Circle/ellipse × cylinder, circle/ellipse × cone, circle/ellipse × torus, and circle/ellipse × sphere each use one config-driven curve/surface pipeline with pre-refactor debug/release bit signatures, exact diagnostics, and unchanged completion; this completes the analytic primitive-surface conic family. Circle/ellipse × NURBS shares one marcher with explicit radial-circle and closest-projection ellipse strategies. Circle/circle, circle/ellipse, and ellipse/ellipse share one bounded conic-pair orchestration layer while their distinct quadratic/quartic and projection arithmetic remains strategy-local; frozen debug/release result and contextual report digests prove bit-exact behavior. Direct Plane/Sphere or safe finite Offset(Plane)/Offset(Sphere) fields against genuinely non-planar direct NURBS, plus positive-area-clipped compatible direct NURBS/NURBS and one- through four-descriptor constant-normal Offset(NURBS)/NURBS unit charts, now retain the lower marcher's degree-1 paired traces, prove both lifts over the whole range in one owner scope, and persist ordered source identity in a distinct non-transmitted verified descriptor. The NURBS/NURBS and offset-effective derived scalar surfaces are discovery-only; outward original-control differences own complete misses. The offset branch binds the live outer root, accumulated signed distance, terminal constant-normal basis, direct peer, and both pcurves; one, two, three, and four descriptors pin exact 2/depth 2, 3/depth 3, 4/depth 4, and 5/depth 5 traversal with N/N-1 admission. Plane fixtures pin exact 7,170/7,169 certificate and 2/1 offset graph-visit evidence; the Sphere fixture pins exact 8,192/8,191 Work, 1,024/1,023 Items, and 10/9 Depth, with direct sphere-offset roots additionally pinning exact 2/1 node visits and dependency depth; the compatible NURBS/NURBS and Offset(NURBS)/NURBS fixtures pin exact 14,336/14,335 Work with the same Items/Depth boundaries. An exact rational-quarter-cylinder varying-normal root against a canonical global-X-, global-Y-, or global-Z-normal planar direct NURBS, analytic Plane, or one safe Offset(Plane) descriptor over a direct Plane basis proves its original normal field for 7 Work; the direct analytic Plane arm alone admits the complete one- through four-descriptor family after every intermediate and final radius proves finite and positive, retains every exact outer-to-inner distance, and uses the derived rational effective sheet only for discovery while the original basis remains proof authority. It pins combined 14,343/14,342 Work and 1,024/1,023 Items for X/Y, while its 40-span Z-normal branch pins 573,447/573,446 Work and 40,960/40,959 Items; all chain lengths retain 10/9 certificate Depth and unchanged certificate budgets. One through four direct-Plane descriptors pin graph Work/depth 2, 3, 4, and 5 with N/N-1 admission; safe Offset(Plane) and planar-NURBS peers remain one descriptor only. All retain both original sources and pcurves, with swap, persistence, and complete misses plus per-descriptor, same-sum, and stale-source rollback pinned atomically. Compatible intersecting planar constant-normal dual Offset(NURBS) chains cover every 1–4×1–4 pair, retain both live roots, accumulated distances, original terminal bases and complete basis chains, and certify one paired-pcurve branch against both original sources at exact 14,336/14,335 Work, 1,024/1,023 Items, and 10/9 Depth. Their graph traversal pins exact `A+B+2` Work and `max(A+1,B+1)` depth, including maximum 10/9 Work and 5/4 Depth; the strict-separated miss remains at the same graph ceiling without certificate or persistence allocation. Collapsed or non-finite sphere-offset fields, descriptor-chain depth five or greater or other varying-normal Offset(NURBS), and multi-descriptor peers outside direct analytic Plane, incompatible, coincident, five-or-more-descriptor, altered, or stale dual Offset(NURBS), broader NURBS/NURBS charts, further verified carrier/descriptor families, coefficient forms beyond the reviewed magnitude-fourteen ceiling, and other seven-positive two-wide layouts outside the exact connected and corner-singleton-plus-six-component families, other eight- or nine-positive two-wide layouts, disconnected five-cell, non-exact or ambiguous multi-edge shared seams, other polar, or non-exact collapsed general sphere-window contracts remain. |
| F4 | Phase 1, representative Phase 2 slices, three Phase 3 pilots, and the first solver-local source-identity migration are implemented: graph evaluation owns stable classification; SSI and NURBS curve/curve retain ordered structured incomplete evidence through limits, numeric/method stops, canonicalization, swapping, dispatch, and facade adaptation. Ellipse/ellipse retains `ProjectionError::{InvalidQueryPoint, InvalidWindow, NoCandidate, NonFiniteEvaluation, Policy}` as `IntersectionError::Projection`; class, code, limit, `capability() == None`, and the direct source survive the concrete solver, generic intersection adapter, `GeometryIntersectionError`, and `KernelError`, while `Policy` additionally retains its `OperationPolicyError`. Its direct entry returns `IntersectionResult` to avoid a lossy `kcore::Error` conversion. Other solver-local collapses plus broader result-family and legacy migrations remain. |
| F5 | K1-K3, typed K4 interchange and facade journal views, checked semantic K4 edits through MVFS/KVFS, MEV/KEV, and KFMRH/MFKRH, K5 adoption, and facade body tessellation are implemented: the `kernel` facade owns lifecycle, opaque IDs, classified sources, one-scope outcomes, safe checker subjects, child-accounted procedural evaluation, atomic typed X_T import/export, graph-owned bounded curve/curve intersection with facade-owned proof results, immutable conforming body meshes with exact closed-solid or true-sheet incidence plus facade-safe face/edge identities, failure-atomic checked block and polygonal-profile extrusion construction, deterministic complete-body rigid copy with direct Plane or safe finite Offset(Plane)-backed Plane/Plane line and Plane/Sphere latitude/oblique circle certificate reissuance from direct Plane or safe finite Offset(Plane) sources plus direct Sphere or safe finite Offset(Sphere) sources whose effective Sphere radius is positive and finite in both orders with leaf-inclusive proof depth at most 64, plus certificate reissuance for every current operation-generated VerifiedNurbsIntersection family: Plane/NURBS, Sphere/NURBS, direct NURBS/NURBS, one- through four-descriptor Offset(NURBS)/NURBS, compatible one- through four by one- through four dual Offset(NURBS), and retained Offset(NURBS)/Plane variants under both supported orders, polynomial/rational traces, and oblique frames. It copies ordered roots and every transitive offset basis, transforms the carrier and original analytic/NURBS trace fields, preserves range, knots, weights, periodicity, paired pcurves and tolerance, and reruns the whole-range family certifier. The transmitted tranche covers Plane/Plane over direct or safe nested exact-plane roots, direct Plane/NURBS in both orders, direct NURBS/NURBS, direct one-descriptor Offset(NURBS)/NURBS in both orders, exactly one-descriptor Offset(NURBS)/direct Plane in both orders, and only the canonical finite-open two-sample degree-1, witnessed three-sample quadratic, witnessed four-sample cubic, canonical five-sample degree-1, and canonical seven-sample degree-1 dual Offset(NURBS) families, completing the existing five sample-count families. The two-sample line, witnessed three-sample quadratic, and witnessed four-sample cubic each admit independent exact ordered chains of one through four Offset(NURBS) descriptors per root across a full 4x4 matrix and both trace orders; the five- and seven-sample families remain exactly one descriptor per root. All five require two distinct ordered roots whose terminal sources are distinct direct nonperiodic NURBS basis handles. The line uses unweighted two-control carrier/pcurves on `[0,0,1,1]` over `[0,1]` without interpolation witnesses; the quadratic uses unweighted degree-2 three-control carrier/pcurves on `[0,0,0,2,2,2]` over `[0,2]`; the cubic uses unweighted degree-3 four-control carrier/pcurves on `[0,0,0,0,3,3,3,3]` over `[0,3]`; the five-sample family uses unweighted degree-1 five-control carrier/pcurves on `[0,0,1,2,3,4,4]` over `[0,4]`; and the seven-sample family uses unweighted degree-1 seven-control carrier/pcurves on `[0,0,1,2,3,4,5,6,6]` over `[0,6]`. Neither polyline family has interpolation witnesses or a carrier period. Both witnessed higher-order families retain exact position and paired-UV interpolation witnesses. Generic graph chain persistence walks each dual root to its direct NURBS terminal source and binds the complete outer-to-inner distance sequence bit-for-bit; it atomically rejects reordered same-total, extra, missing, or stale chains. The graph trace constructor retains at most four exact ordered descriptors per trace and rejects a fifth, so a live depth-five source paired to its maximum-depth trace fails atomically at graph insertion; broader depth remains graph representation/binding work rather than a facade-copy preflight case. Its public original-source recertifiers transform both terminal bases and the line, five-sample, or seven-sample carrier; for either witnessed higher-order family they transform exact position witnesses and rebuild carrier controls from them by the public interpolation formula. All five copy ordered roots and full offset/basis and pcurve chains, preserve metadata, tolerance, exact UV witnesses, distance order, and terminal source identity, rerun the corresponding public original-source recertifier, and protect each copied root and complete basis closure from removal transitively. Facade proof/source preflight runs before operation-scope creation; graph-valid shared-basis or periodic charts, nested five- or seven-sample roots, Offset(Plane) peers, altered higher-order witnesses, other sample counts, and altered or overdeep bindings fail closed, while the lower topology transaction restores every Body/Region/Shell/Edge/Vertex and Curve/Surface/Pcurve/Point count plus future point identity on rejection. Attributes remain blocked on an authorable storage contract, and non-rigid transforms remain; position-owning seed-body creation/removal plus affine-map- and incidence-metadata-aware pcurve strut creation/removal, face split/merge, bridge-edge removal/ring join, and face-as-hole merge/split composition. Committed journals expose exact-size part-qualified net-mutation, all-five-form lineage, tolerance-budget, and tolerance-event iterators; deleted topology/geometry identity remains reportable and stored points use a journal-only opaque ID rather than leaking arena handles. Semantic edits validate part-qualified live geometry, finite size-box positions, shell/loop shape, finite invertible edge-to-pcurve maps, integer-period charts, singular endpoints, closed-use winding, periodic seam roles, and moved-fin incidence before mutation, use checked contextual commit, return opaque results plus the committed facade journal, and restore topology and future identities on rollback or proof denial. Position-owning MVFS preflights its surface and finite size-box seed position before hidden-point allocation, exposes all created topology identities opaquely, and remains transient until later Euler composition completes it or facade KVFS removes it; the facade inverse deletes the point only when unshared while ordinary lower KVFS retains external geometry. Position-owning MEV preflights every fallible input before hidden-point allocation, and its facade KEV inverse deletes that point only when unshared while ordinary lower KEV retains external geometry. Operation-owned facade tolerance growth now accepts one ordered Face/Edge/Vertex batch: every target is part-qualified and live before value validation, targets are unique, final values meet the model-resolution floor, and provenance plus exact aggregate accounting complete before an infallible apply. It preserves imported origin, journals events in request order, returns only a journal-local non-authoring budget identity, and restores model and budget state on rollback or checked denial. The additive Full-assurance commit gate keeps Fast commit behavior unchanged, checks duplicate-free explicit roots before affected/store-audit roots, shares one scope across Fast graph work and Full proofs, and returns ordered part-qualified reports for committed or rollback-clean rejected decisions. `RequireValid` rejects any gap, `AllowIndeterminate` retains gaps, Full faults always reject, rejected decisions carry no journal, and proof/accounting denial restores tolerances, the committed index, and future identities with exact 306/305 graph-work coverage. Structural face/hole edits do not pre-certify geometric containment beyond supported Fast checks, so operation-specific unsupported containment remains a caller evidence obligation. `FinView` exposes facade-owned range, affine-map, chart, endpoint, winding, and seam values without leaking lower types. The standalone `kernel-lifecycle` client depends directly only on `kernel` and proves construction, semantic inspection, Full checking, Full-assurance edit commit, body tessellation, curve intersection, surface evaluation, byte-stable X_T export/import/re-export, facade-only edit request rollback, tolerance-batch journaling, and journal traversal. Broader semantic edit families and partition-history composition remain. |
| F6 | First slice implemented: shared surface inversion, chart normalization, and distance services consumed by checker and tessellation. Module splits remain. |
| F7 | Q0-Q2b, Q8, and the first Q3-Q6 slices are implemented: CI now enforces Python/oracle freshness, compiles and smoke-runs the excluded benchmark package including graph construction/traversal, contextual body/face tessellation, and curve-pair isolation/solve, and runs both pinned fuzz targets within fixed limits. Q2 now has 35 rows: the prior 28 remain unchanged, including the seven-row mixed-store affected-root cohort matrix with exact scope/order/result digests through 256 total and 64 affected bodies. Seven affected-solid `primitive_mix` rows form a crossed production grid: total 64 with 1/4/16/64 affected roots plus one affected root across totals 4/16/64/256, sharing the 64/1 row. The fixed-total axis retains exact `N × 1e-8` budget, N modified-face/event, affected/refreshed/checked/mutation, digest, and installed-index evidence. The total-size axis uses one ordinary checked operation scope for exactly one first-face growth to `2e-8` under an exact `1e-8` budget and pins one net Face mutation, one tolerance event, affected/refreshed/checked/mutations = 1, a stable affected digest across totals, before/after store and output ratchets, and installed-index equality. Q2a drove the reverse-index replacement and its v2 ladder now pins zero full-order rebuilds across 21 rows, including four real verified-intersection diamonds whose dependency-first closure visits a shared basis exactly once. Q2b v2 has ten deterministic closure/path rows through 1,000 edges plus real verified-intersection diamond closure and missing-path cases; the timed closure visits the shared basis exactly once after traversal membership indexing. Q3 `body-tessellation.v3` has 32 rows and pins all 21 composed counters: twenty generalized legacy solids plus four tiers each for a locally verified genuinely curved NURBS block and historically host-certified plane and full-period cylinder sheets. Its 24 solids and eight sheets pin exact directed incidence, topological boundary, face-sense orientation, and the applicable signed-volume or faceted-area measure. Q3 face v2 crosses three representations, three trim topologies, and two tolerances with lift, orientation, boundary, area, mesh, and report evidence. Both matrices pass finite `bounded_v1` profiles and pin root Work N/N-1. Q4 implicit isolation v3 has eight cases including the surface roundoff adversary and multi-span Work N-1; curve-pair isolation v4 has nine span-accounted cases; solve v18 has twenty-eight cases including coordinate, unit-form, and primitive magnitude-two through magnitude-twelve algebraic `1/3` certificates, common-refinement and inverse-history success, altered-history rejection, and exact per-path overlap Work/Items denial. The benchmark manifest now contains 170 total cases. Q3-Q5 expansion, exact coefficient forms beyond twelve, more Q6 targets/corpora, and Q7 remain. |

The M0 predicate-hardening ratchet now includes deterministic robust `incircle`.
Its conservative Shewchuk stage-A filter falls back to exact expansion
arithmetic whenever the floating determinant cannot certify its sign. The
oriented convention is positive for a point inside a counterclockwise defining
circle, negative outside, exactly zero for cocircular inputs, and sign-reversing
when the defining orientation reverses. A 20,000-case random `i128` oracle,
all six defining-point permutations, exact and one-unit near-cocircular
fixtures proven to force the fallback, degenerate/non-finite behavior, and the
cross-platform numeric golden pin the contract. This closes the named
`incircle` debt. `insphere` remains deferred until a 3D Delaunay or equivalent
consumer needs it.

The first bounded consumer migration in the repository-wide decision audit is
also landed: SSI region consolidation now treats a polygon as strictly convex
in its first parameter chart only when it has at least three vertices, all
first-chart coordinates are finite, and every consecutive
`orient2d(a, b, c)` result is exactly `Orientation::Positive`. Exact collinear
or otherwise nonpositive turns fail closed. The public consolidation fixture
uses integer coordinates near `2^52` whose first exact determinant is `+1`
while the former floating cross product rounds to zero, then proves identical
canonical output for repetition, rotation, and reversal. This does not expose
or claim a general polygon-orientation primitive. The broader audit remains
open, with oblique-extrusion direction and polygon-shoelace orientation signs
still explicit debt.

## Current direction and handoff order

The foundation has enough vertical proof. The current phase prioritizes
convergence, adoption, and continuous enforcement over adding more parallel
surface area:

The blocking F7 test-throughput checkpoint is closed. The fail-closed
`focused`/`fast`/`standard`/`docs`/`full` developer lanes, exact 14-target
production-corpus classification, concurrent three-OS debug/release CI
profiles, rolling Cargo caches, and first redundant v10/v11 full-exemplar
replay removals have landed. Warm `fast` passed in 14.231s and the integrated
`full` gate passed all targets, docs, and tooling in 1,726.501s on the named
development host. The explicit `docs` lane preserves compile-fail architecture
contracts while removing documentation compilation from `standard`; direct
post-split runs passed in 62.900s for `standard` and 176.581s for `docs`.
`full` still owns every corpus and doctest ratchet; further optimization
remains evidence-driven and no longer blocks resuming item 3. See
[`test-throughput.md`](test-throughput.md) for the contract and measurements.

Read the ordered queue below literally. At this handoff, item 1's facade-owned
body-tessellation replacement and state-4 compatibility ratchet are closed,
and item 2's evidence, finite-preset, matrix-admission, and client-adoption
work is closed. The first unclosed foundation obligation is item 3. Its first
M4d transform consumer is now complete: `PartEdit::copy_body_rigid` duplicates
the complete topology and geometry closure under an orientation-preserving
placement, preserves pcurve/chart/tolerance data, emits identity-complete
`DerivedFrom` lineage, and checked-commits atomically. Plane/Plane line
descriptors now copy direct Plane sources or safe finite Offset(Plane) chains.
Plane/Sphere latitude or oblique circle descriptors copy direct Plane or safe
finite Offset(Plane) sources plus direct Sphere or safe finite Offset(Sphere)
sources whose effective Sphere radius is positive and finite, in either
order, transform the carrier, copy aligned Plane/latitude pcurves into the
copied effective surface frames or regenerate the oblique spherical pcurve, and
reissue the whole-range certificate. The copied proof closure is leaf-inclusive
with depth at most 64. Facade preflight also admits every current
operation-generated `VerifiedNurbsIntersection` family: Plane/NURBS,
Sphere/NURBS, direct NURBS/NURBS, one- through four-descriptor
Offset(NURBS)/NURBS, compatible one- through four by one- through four dual
Offset(NURBS), and retained Offset(NURBS)/Plane variants in either supported
order, including polynomial/rational traces and oblique frames. Copy preserves
the ordered roots and full transitive basis chains, transforms the carrier and
both original proof fields, retains range, knot, weight, periodicity,
paired-pcurve, and tolerance data, and reruns the whole-range family certifier.
The transmitted copy tranche reruns public original-source certifiers for
Plane/Plane charts whose ordered roots are direct or safe nested exact-plane
fields, direct Plane/NURBS in both orders, direct NURBS/NURBS, direct one-
descriptor Offset(NURBS)/NURBS in both orders, exactly one-descriptor
Offset(NURBS)/direct-Plane charts in both orders, and only the canonical finite-
open two-sample degree-1, witnessed three-sample quadratic, witnessed four-
sample cubic, canonical five-sample degree-1, or canonical seven-sample degree-1
dual Offset(NURBS) charts in either ordered-root arrangement. These five
canonical sample families complete the existing set. The two-sample line,
witnessed three-sample quadratic, and witnessed four-sample cubic now each
admit independent exact ordered chains of one through four Offset(NURBS)
descriptors per root: each full 4×4 matrix certifies in both trace orders. The
five- and seven-sample families remain exactly one descriptor per root. All
five require two distinct ordered roots whose terminal sources are
distinct direct nonperiodic NURBS basis handles. The line uses matching
unweighted two-control carrier and pcurves on knots `[0,0,1,1]` over `[0,1]`
with no interpolation witnesses; the quadratic uses matching unweighted
degree-2 three-control carrier and pcurves on knots `[0,0,0,2,2,2]` over
`[0,2]`; and the cubic uses
matching unweighted degree-3 four-control carrier and pcurves on knots
`[0,0,0,0,3,3,3,3]` over `[0,3]`. Both witnessed higher-order families carry
exact position and paired-UV interpolation witnesses. The five-sample family
uses matching unweighted degree-1 five-control carrier and pcurves on knots
`[0,0,1,2,3,4,4]` over `[0,4]`; the seven-sample family uses matching
unweighted degree-1 seven-control carrier and pcurves on knots
`[0,0,1,2,3,4,5,6,6]` over `[0,6]`. Neither polyline family has interpolation
witnesses or a carrier period. Generic graph chain persistence walks each dual
root to its direct NURBS terminal source and binds the complete outer-to-inner
distance sequence bit-for-bit; it rejects reordered same-total, extra, missing,
or stale chains atomically. The graph trace constructor retains at most four
exact ordered descriptors per trace and rejects a fifth; a live depth-five
source paired to its maximum-depth trace therefore fails atomically at graph
insertion. Broader depth is graph representation/binding work, not a facade-
copy preflight case. Rigid copy transforms both distinct terminal bases
and the line, five-sample, or seven-sample carrier directly; for either
witnessed higher-order family it transforms the exact-position witness set and
rebuilds the quadratic or cubic carrier controls from the transformed positions
by the public interpolation formula. It copies both ordered roots, their full
offset/basis and pcurve chains; preserves exact distance order, terminal source
binding, chart metadata, tolerance, and exact UV witnesses; reruns the
corresponding public original-source recertifier, including public quadratic
and cubic recertification at every 4×4 chain pair; and protects both copied
roots and their complete basis closures from removal transitively. The facade
verifies the live roots and proof family before creating an operation scope.
Graph-valid shared-basis or periodic charts remain copy-unsupported, as do nested five- or
seven-sample roots. Offset(Plane) peer roots, altered higher-order
witnesses, other sample counts, unsupported or nonpositive/nonfinite effective-
sphere proofs, and altered or overdeep bindings fail closed.
The lower topology transaction proves rollback-clean Body/Region/Shell/Edge/
Vertex and Curve/Surface/Pcurve/Point counts plus future point-identity reuse on
rejection.
Attributes remain follow-on work because no authorable storage contract exists;
non-rigid transforms also remain explicit follow-on work. Its first extrusion
consumer is also complete: one validated polygonal profile with holes extrudes
along any finite translation with a
nonzero profile-normal component into exact planar caps and side
parallelograms with shared perimeter/sweep edges and per-fin pcurves in the
actual face frames. The builder commits atomically, the typed facade returns
opaque body/journal values, Full checking is `Valid`, and the oblique holed
fixture is watertight with signed volume 24 within `1e-9`; reverse translations
use a reflected equivalent profile chart without changing model-space geometry.
Curved profiles, zero-normal degenerate sweeps, revolve, and external X_T validation remain.
Item 3's explicitly bounded magnitude-thirteen/fourteen configurable-search
component is complete: magnitude twelve remains the compatibility default and
exact stable prefix, while opt-in thirteen and fourteen are the only reviewed
extensions. The exemplar's
equal-limit records 1828 and 2008 are certified for the single-periodic-axis,
one-period form with one shared or two distinct `H/?` limits identifying the
same point, including unique exact interior period aliases, and its end-
terminated records 1671 and 1678 are
certified for the finite-open/`T/F` singular form. Production reconstruction
now uses the exact v15 corpus profile. V5 remains fixed at
`117478445/20/10` Work/Items/Depth and stops on the newly exposed
118,406,196-Work chart attempt after certifying finite-open direct
B-surface/Plane record 1252 with interior-only paired-null Plane UV recovery.
V6 exactly lifts native direct-Plane `SP_CURVE` node 30 and advances through
the later supported charts at `208228426/22/10`. Its equal-limit certificate
promotes the closed carrier to periodic semantics, derives FACE 1195's
vertex-less ring domain, and pins its next denied attempt at 221,060,174 Work.
V7 recovers `INTERSECTION` 5089 / data 5092 sample 2 operand 0's paired-null
interior Plane UV through exact frame inversion, reruns the whole-carrier
Plane/Offset-NURBS certificate, and advances at exact `272430166/22/10` before
pinning `INTERSECTION` 1984's attempted 285,283,414 Work. A synthetic
endpoint-null Plane variant preserves the same v7 report and next crossing;
endpoint displacement still fails the unchanged certificate. A bounded two-
through five-sample direct-Plane/B-surface, safe-Offset(Plane)/B-surface,
direct-Plane/Offset(B-surface), direct constant-normal Offset(B-surface)/direct
B-surface, independent direct one-descriptor Offset(B-surface)/Offset(B-surface),
or direct-B-surface/B-surface slice now retains finite positive
noncanonical affine metadata on the
canonical shared sample-index basis. An exact record-5089 variant reuses
record 778's metadata and pins `139792442/4/10` cumulative Work/Items/Depth
with N/N-1 rollback; a five-sample record-1252-derived direct Plane/B-surface
variant pins cumulative `127115320` Work, while the shared structural family
pins `7170/2/10`. Direct B/B, direct Offset(B)/B, and independent direct
Offset(B)/Offset(B) structural costs are
`14336/2/10`, `28672/3/10`, `43008/4/10`, and `57344/5/10` for two through
five samples. The dual-offset arm covers polynomial/rational basis combinations
and operand swap while retaining both live roots, signed distances, independent
direct original bases, paired UVs, and the unchanged two-source whole-range
proof. Nested, shared-basis, multi-offset, null/mixed, and out-of-range
noncanonical forms remain typed unsupported; corpus
records 778 and 3620 remain original-domain certificate failures. V8 certifies record
1984 at exact `315245660/22/10` by snapping only its final first-trace `u`
across a source boundary within endpoint roundoff slack and rerunning the
unchanged original-source whole-carrier proof. Its historical v8 profile still
stops before record 5945's attempted 323,814,492 Work. V9 admits record 5945 at
exact `323814492/22/10`: its ordered Offset(B-surface) roots `[3338, 773]` use
common degree-2 clamped interpolants through exactly three transmitted positions
and canonicalized paired UV tuples, while independent original-source offset-
NURBS interval residuals remain the proof evidence. V10 admits record 3819 at
exact `336759900/22/10`. Its ordered Offset(B-surface) roots `[3370, 773]` use
unique common degree-3 clamped interpolants through four transmitted positions
and canonicalized paired UV tuples, while independent original-source offset-
NURBS interval residuals remain authoritative. Historical v1-v9 profiles retain
exact parity. V11 treats a nonperiodic source knot with multiplicity zero as
padding only when it is null or finite numeric. That admits quadratic record
3790 at isolated `8593408/3/10`, then exposes and certifies the existing
11-sample Plane/Offset(B-surface) record 3745 at isolated
`42772491/11/10`, reaching exact `388125799/22/10` with historical v1-v10
parity. V12 admits seven-sample dual-offset record 3615 at isolated
`26443776/7/10`: roots `[3374, 773]` retain bases `[3730, 1186]`, while exact
transmitted positions and paired UVs form a common degree-1 open-clamped
polyline and both original offset-NURBS sources retain independent whole-range
proofs. The corpus reaches exact `414569575/22/10` with historical v1-v11
parity. Two-sample dual-offset record 3595, roots `[783, 773]`, independently
certifies as a canonical open-clamped line at isolated `4352000/2/10`, with
residuals `[3.468467250779673e-5, 3.384554176162513e-5]` below its chordal
tolerance. Production does not reach it. Five-sample dual-offset record 4230,
roots `[3320, 773]`, chart 4231, independently certifies as a common degree-1
open-clamped polyline at isolated `17285120/5/10`. V13 admits it at exact
`431854695/22/10` with historical v1-v12 parity. V14 admits two-sample direct
Plane/Offset(B-surface) record 3609, chart 3607, at isolated `4277250` Work and
exact cumulative `436131945/22/10`, preserving historical v1-v13 parity. V15
admits two-sample dual-offset record 6044, chart 6043, at isolated
`4352000/2/10` and exact cumulative `440483945/22/10`, preserving historical
v1-v14 parity. It then stops before four-sample dual-offset record 5921, chart
6027, whose isolated `13774848/4/10` proof would request cumulative
`454258793` Work. At that exact attempted budget, the canonical cubic first
pcurve materially exits its original open nonperiodic source domain;
certification fails atomically and retains the v15 report.
The remaining parallel legs continue the certified non-octant sphere fallback
beyond its pole-clear, sub-π hit, certified-disjoint, exact-boundary-lock
collapsed, exact one-pole cap and polar-by-one-wide 2×3 decompositions including
exact adjacent pairs in either latitude row, exact same-column vertical pairs
(reviewed at `[0,2]`/`[1,2]`), and either exact full latitude row, with the
non-cap row retaining eight vertices, plus the exact mixed-axis three-cell L
path in cap-right `[0,2]`/`[1,1]`/`[1,2]` and lower-middle
`[0,1]`/`[0,2]`/`[1,1]` orientations at the same 6/147/588 ceilings,
first single-wide decomposition,
and two-wide all-empty, one-positive,
two-isolated-positive, three-isolated-positive, exact shared-edge adjacent-positive,
exact adjacent-pair-plus-isolated three-cell, exact three-cell path, exact
four-cell path, exact connected four-cell 2×2 cycle, exact five-cell path,
exact connected five-cell non-path, exact sibling-separated disconnected five-cell,
six-cell grid, exact seven-cell 2×3-block-plus-tail and opposite-corner-empty layouts,
all four exact disconnected seven-positive occupied-corner-singleton plus connected-six-cell rotations, exact connected
eight-cell edge- and corner-empty layouts, and exact exhaustive nine-cell arms to other
seven-positive layouts outside the connected and corner-singleton families, other
eight- or nine-positive layouts,
disconnected layouts without exact separation,
other polar, or non-exact tangent
cases; and
extend the contextual verified graph arms beyond exact direct/safe-Offset(Plane),
direct/safe-Offset(Sphere), positive-area-clipped compatible direct-NURBS/NURBS, and the capped
four-descriptor constant-normal Offset(NURBS)/NURBS unit-chart arm to
broader offset or
NURBS/NURBS fields. These legs do not silently
leapfrog items 1 or 2.

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
   equal-limit forms permit one shared or two distinct `H/?` limits only when
   they identify the same point, the chart closes spatially, and exactly one
   periodic NURBS axis is certified. Each transmitted coordinate may
   use at most one exact-period alias and must have one unique predecessor-
   relative lift; the normalized trace must traverse exactly one period and
   pass the unchanged original-source certificate. Null, mixed-limit,
   mismatched or multi-position closed, material, ambiguous, multi-period,
   nonperiodic-axis, or off-seam forms
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
   typed and atomic. V5 keeps that exact cap and stops on the later attempted
   118,406,196-Work chart proof. Corpus-backed v6 exactly lifts native direct-
   Plane `SP_CURVE` node 30 from its open, nonperiodic, nonrational 2D B-curve
   controls, preserves degree, knots, and parameterization, and then admits the
   later supported chart proofs at exact `208228426/22/10`. The equal-limit
   certificate mints periodic carrier semantics only for one complete certified
   seam crossing, so FACE 1195's vertex-less ring domain derives. V6 remains
   fixed at `208228426/22/10` and pins the next 221,060,174-Work attempt.
   Corpus-backed v7 then certifies finite-open Plane/Offset(B-surface) record
   5089 at exact `272430166/22/10` by recovering paired-null interior Plane
   samples through exact frame inversion before the unchanged whole-range
   proof. Canonical Plane/Offset endpoints now use the same recovery; a
   synthetic endpoint-null variant preserves the exact v7 report and next
   crossing. The bounded noncanonical affine direct-Plane/B-surface, safe-
   Offset(Plane)/B-surface, direct-Plane/Offset(B-surface), direct constant-normal
   Offset(B-surface)/direct B-surface, independent direct one-descriptor
   Offset(B-surface)/Offset(B-surface), or direct-B-surface/B-surface slice accepts
   two through five samples, retains
   finite positive chart
   metadata separately from the canonical sample-index basis, and preserves
   exact source/dependency identity. Safe offset-plane roots retain every
   nested Plane basis and pin structural 2–5 sample Work/Items/Depth at
   `7170/2/10`, `14339/3/10`, `21508/4/10`, and `28677/5/10`. Direct B/B and
   direct Offset(B)/B and independent direct Offset(B)/Offset(B) charts pin
   `14336/2/10`, `28672/3/10`, `43008/4/10`, and `57344/5/10`. The dual-offset
   arm covers operand swap and polynomial/rational bases while retaining both
   live roots, signed distances, independent direct bases, paired UVs, affine
   metadata, canonical sample-index carrier/pcurves, and the unchanged
   two-source proof. Six-sample, nested/shared/multi-offset, null/mixed, and
   other out-of-range noncanonical variants remain typed and atomic. A record-5089
   variant pins cumulative `139792442/4/10` Work/Items/Depth and exact N/N-1 rollback. No corpus chart
   contains a NURBS-side paired-null tuple; noncanonical records 778 and 3620
   materially leave their original NURBS domains and remain typed and atomic.
   V8 admits `INTERSECTION` 1984 at exact `315245660/22/10` by snapping
   only first/last NURBS coordinates whose source-domain overhang is within
   `16384 * EPSILON * domain-scale`; material/interior overhangs and displaced
   carriers remain rejected by the unchanged original-source proof. Historical
   v8 stops before `INTERSECTION` 5945's attempted 323,814,492-Work proof. V9
   admits that exact finite-open three-sample dual-offset chart at
   `323814492/22/10`; malformed samples, controls, knots, and witness tuples
   remain typed and atomic. V10 admits four-sample cubic dual-offset record
   3819 at exact `336759900/22/10`, with unique degree-3 clamped interpolants,
   independent original-source proofs, and historical v1-v9 parity. V11
   normalizes only null or finite-numeric zero-multiplicity nonperiodic
   knot padding, admits quadratic record 3790 and the exposed 11-sample
   Plane/Offset record 3745 at exact `388125799/22/10`, and preserves
   historical v1-v10 parity. V12 admits seven-sample dual-offset polyline
   record 3615 at exact `414569575/22/10`, with isolated `26443776/7/10`
   accounting and historical v1-v11 parity. Two-sample dual-offset record 3595
   independently certifies at isolated `4352000/2/10` with exact Work/Items/
   Depth N/N-1 rollback evidence. Five-sample record 4230 independently
   certifies at `17285120/5/10`; production v13 admits it at exact
   `431854695/22/10`. Production v14 admits Plane/Offset record 3609 at isolated
   `4277250/2/10` and reaches exact `436131945/22/10` with historical v1-v13
   parity. Production v15 admits two-sample dual-offset record 6044 at isolated
   `4352000/2/10`, reaches exact `440483945/22/10` with historical v1-v14
   parity, and stops before four-sample dual-offset record 5921's cumulative
   `454258793`-Work request. An exact-budget regression pins the subsequent
   original-domain certificate rejection, retained v15 report, and empty
   rollback.
   Half-null pairs, NURBS/Offset-NURBS omissions, and noncanonical forms outside
   the bounded direct-Plane/B-surface, safe-Offset(Plane)/B-surface, direct-
   Plane/Offset(B-surface), direct Offset(B-surface)/direct B-surface, and
   independent direct one-descriptor Offset(B-surface)/Offset(B-surface), and
   direct-B-surface/B-surface affine slices remain typed; nested, shared-basis,
   multi-offset, null/mixed, and out-of-range noncanonical forms remain outside
   the landed slice.
   Original-backed, tolerance-qualified, non-Plane, reversed-basis,
   periodic, closed, rational, non-2D, and foreign procedural curves remain
   typed unsupported.
   The exact
   algebraic family now includes canonical primitive carrier/residual
   coefficients through magnitude twelve as the compatibility default. A
   validated `CurvePairAlgebraicSearchConfig` now opts the standalone and
   candidate-cell exact certifiers into the magnitude-thirteen or magnitude-
   fourteen shell; twelve remains the exact compatibility prefix and every
   through-thirteen enumeration/certificate golden remains unchanged. The
   fourteen ceiling enumerates exactly 254 carrier forms and 9,825 residual
   forms, adding 24 and 1,704 respectively over thirteen. Its genuinely
   noncoplanar normalized-`1/3` fixture returns no certificate at every explicit
   ceiling through thirteen, certifies at fourteen, retains polynomial/rational
   source, candidate-cell, swap, and reversal parity, and rejects broken,
   overflowing, non-finite, or out-of-range inputs. Invalid ceilings fail before
   search. The contextual non-plane
   graph arm now marches direct or safe finite Offset(Plane) or Offset(Sphere)
   fields against a genuinely curved direct NURBS, plus compatible pairs of
   genuinely curved direct NURBS unit charts and a capped four-descriptor
   constant-normal Offset(NURBS)/NURBS unit chart with exact positive-area
   clipping across distinct finite operand windows, in one owner scope. It
   retains paired degree-1 traces, certifies both lifts over the whole range at depth
   10, and persists the ordered live source identity in a non-transmitted
   verified NURBS descriptor atomically. The Plane fixture pins exact 7170/7169
   certificate Work and one Offset(Plane) pins exact 2/1 graph visits. The
   Sphere fixture pins exact 8192/8191 Work, 1024/1023 Items, and 10/9 Depth for
   outward centered-mean-value Sphere and original-source NURBS lifts; a direct
   sphere-offset root additionally pins exact 2/1 node visits and dependency
   depth, while nested safe chains retain every basis transitively. The
   compatible NURBS/NURBS and Offset(NURBS)/NURBS fixtures pin exact
   14336/14335 Work with the same Items/Depth limits; their rounded scalar
   differences are discovery-only and outward original-control differences own
   complete misses. The offset proof also binds the live outer root, accumulated
   signed distance, terminal constant-+Z-normal basis, direct peer, and paired
   pcurves. A direct
   root retains exact 2/depth-2 graph traversal; two descriptors pin exact
   3/2 node-visit and dependency-depth admission, three pin exact 4/3, and
   four pin exact 5/4.
   A first varying-normal arm admits one offset descriptor whose original
   basis is the exact rational quarter-cylinder extrusion, against a canonical
   bilinear planar direct-NURBS, direct analytic Plane, or one safe
   Offset(Plane) descriptor over a direct Plane basis normal to the global X,
   Y, or Z axis. The direct analytic Plane arm alone admits the complete one-
   through four-descriptor family, retains every outer-to-inner signed
   distance as certificate metadata, and proves that every intermediate and
   final cylinder radius is finite and positive before reducing the chain to
   its effective parallel sheet. A 7-Work, 1-Item, Depth-1
   original-derivative enclosure proves a nonzero normal over the complete
   operand window before the true rational parallel surface is used for
   discovery. Orientation-selected original controls, radially scaled only for
   X/Y, own complete misses, while a
   positive branch retains the live root, original basis, direct peer, and
   paired pcurves under one scope. Planar-NURBS X/Y branches pin exact
   14343/14342 Work and 1024/1023 Items; their Z-normal 40-span branch pins
   573447/573446 Work and 40960/40959 Items. Analytic-Plane X/Y branches pin
   7177/7176 Work and 1024/1023 Items, while their 40-span Z branch pins
   286768/286767 Work and 40960/40959 Items. A safe Offset(Plane) peer retains
   those same certificate boundaries while pinning 4/3 graph Work and 2/1
   dependency Depth, with its live root, direct basis, and signed distance.
   Direct analytic Plane peers consume exact graph Work/depth 2, 3, 4, or 5
   for one, two, three, or four descriptors, with exact N/N-1 admission at
   every boundary. All orientations and chain lengths retain the existing
   10/9 certificate Depth and unchanged certificate Work/Items budgets. The
   derived rational effective sheet remains discovery-only; the original
   rational-quarter-cylinder basis and its source controls own proof and
   completion. Swap, persistence, and complete-miss evidence cover every
   admitted chain length. Persistence validates each descriptor distance, so
   per-descriptor alteration, including a four-descriptor same-sum mutation,
   and stale roots or peers are rejected without allocation and roll back
   atomically. Singular or inconclusive fields, descriptor-chain depth five or
   greater, more than one descriptor against planar NURBS or Offset(Plane), and
   incompatible peers fail closed.
   Incompatible planar or unaligned charts, disjoint or
   boundary-only offset/direct ranges, unequal weights, collapsed or non-finite
   sphere offsets, five-or-more-descriptor positive chains, other
   varying-normal Offset(NURBS) families, and multi-descriptor roots outside
   the direct analytic Plane arm,
   coincident or incompatible dual Offset(NURBS), and
   broader NURBS/NURBS remain explicit unsupported boundaries. Compatible
   planar constant-normal one- through four-descriptor Offset(NURBS) pairs
   cover the complete intersecting 4×4 matrix. They retain both live roots,
   accumulated distances, terminal original bases and full transitive chains,
   generate one paired-pcurve branch certificate against both originals at
   exact 14336/14335 Work, 1024/1023 Items, and 10/9 Depth, and preserve
   operand order under swap. Graph traversal pins exact `A+B+2` Work and
   `max(A+1,B+1)` dependency depth, including maximum 10/9 Work and 5/4 Depth
   admission. Strict outward original-control separation retains the
   graph-only complete miss at the same graph ceilings with zero certificate
   use and no persistence allocation. Five-or-more-descriptor roots,
   incompatible or coincident effective sheets, and altered or stale proof
   sources remain unsupported. Further fields and
   carrier families, plus coefficients above fourteen, remain unsupported. All six complete-support-
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
   other polar, non-exact tangent, multiple-cycle, and ambiguous cases stay `Indeterminate`.
   A fixed 112-witness arrangement scan now certifies disjoint supported
   windows: the pinned empty exemplar succeeds at 96 witnesses and fails at 95.
   Bit-exact opposing boundary planes additionally reduce one equality lock to
   interval-certified tangent circle arcs and two independent locks to tangent
   points; the pinned collapsed-curve exemplar succeeds at 12 arc witnesses and
   fails at 11. A bounded polar arm admits exactly one sub-π source window with
   one bit-exact natural-pole boundary and one pole-clear sub-π peer. It splits
   the polar latitude interval into a closed pole-clear cell and a closed cap,
   accepts both-empty or exactly one occupied cell with one certified-empty
   sibling, and restores the parent map only after that sibling excludes the
   artificial latitude seam. The cap drops the degenerate pole-latitude plane,
   retains one canonical singular pole anchor, and canonicalizes every exact
   source-pole longitude alias before frame mapping. The reviewed cap returns
   one three-anchor region with exact repeat/swap, outward residuals, and exact
   2/1 piece, 49/48 pair, and 196/195 arc ceilings. One-ULP near-poles,
   double-polar or wide-polar inputs, boundary tangencies, and two-occupied-cell
   decompositions remain indeterminate. A broader polar-by-wide arm crosses the
   two latitude cells with three closed sub-π longitude cells from exactly one
   pole-clear wide peer. It accepts all-empty, exactly one occupied child with
   five certified-empty siblings, exactly two edge-adjacent same-row children
   with four certified-empty siblings, an exact same-column vertical pair with
   its other four siblings certified empty, or the exact
   three-child row in either latitude row with all three opposite-row siblings
   certified empty, one exact mixed-axis three-cell L path with the other three
   siblings certified empty, any exact four-positive grid path, exact lower- or
   upper-stem T tree, or exact left/right 2×2 cycle with the other two siblings
   certified empty, the exact disconnected outer-column vertical pairs with
   the two middle-column siblings certified empty, or any of the four exact
   disconnected singleton-plus-three-cell mixed-axis L layouts with the two
   omitted graph-cut siblings certified empty, or exactly five positive children
   with the sole remaining sibling certified empty, or all six children positive
   with no empty sibling. The reviewed real L orientations are the
   cap-right `[0,2]`/`[1,1]`/`[1,2]` path and the lower-middle
   `[0,1]`/`[0,2]`/`[1,1]` path. The single-child path excludes every
   artificial seam through sibling emptiness. The two-child path requires one
   reverse-oriented, bit-exact shared edge on the applicable regular longitude
   or latitude seam, removes only that edge, rejects any surviving used seam or
   unused-longitude seam, and restores the parent map. The vertical slice
   requires bit-exact latitude-seam cancellation; its reviewed fixture is
   `[0,2]`/`[1,2]`.
   The reviewed `[1,1]` cap/middle-longitude fixture retains one canonical
   three-anchor region; the reviewed `[1,0]`/`[1,1]` pair retains one canonical
   five-anchor region. A one-turn-shifted lower-row pair retains one canonical
   six-anchor region without a pole alias. The `[0,2]`/`[1,2]` vertical fixture
   retains one merged region after strict latitude-seam cancellation and four
   empty siblings. A one-turn-shifted full-row fixture cancels both strict
   longitude seams and excludes the latitude seam. Its cap-row result retains
   one canonical 11-anchor region and the opposite non-cap row retains one
   canonical eight-vertex region. Each mixed-axis L cancels exactly one
   reverse-oriented longitude seam and one reverse-oriented latitude seam,
   rejects any surviving used seam or unused longitude seam, and restores one
   parent region. The generic four-positive arm requires a three-edge path with
   exactly two longitude adjacencies and one latitude adjacency, explores
   stable exact association orders, and cancels all three artificial seams.
   Its two real fixtures are the cap-row-right
   `[0,2]`/`[1,0]`/`[1,1]`/`[1,2]` path and the zigzag
   `[0,1]`/`[0,2]`/`[1,0]`/`[1,1]` path. One-ULP shared-edge mutation and
   duplicate-edge ambiguity fail closed. The path route remains exclusive to
   degree sequence `2,2,1,1`. A disjoint four-positive T route admits the exact
   lower stem `[0,0]`/`[0,1]`/`[0,2]`/`[1,1]` and upper stem
   `[0,1]`/`[1,0]`/`[1,1]`/`[1,2]` trees with the other two siblings certified
   empty. Each has degree sequence `3,1,1,1`, simultaneously proves and removes
   exactly three reverse-oriented bit-exact seams, requires one unambiguous
   outer cycle with no artificial seam edge, and restores the parent map and
   maximum child/parent residual. Both real fixtures pin repeat/swap; one-ULP
   seam mutation and duplicate-edge ambiguity fail closed. The third disjoint
   four-positive route admits the
   exact left `[0,0]`/`[0,1]`/`[1,0]`/`[1,1]` and right
   `[0,1]`/`[0,2]`/`[1,1]`/`[1,2]` 2×2 cycles with the other two siblings
   certified empty. Each simultaneously proves exactly four reverse-oriented,
   bit-exact internal adjacencies, removes all four, requires one unambiguous
   outer cycle with no artificial seam edge, and restores the parent map and
   maximum child/parent residual. Both real fixtures pin repeat/swap; one-ULP
   seam mutation and duplicate-edge ambiguity fail closed. The five-cell
   arm removes every internal reverse-oriented bit-exact seam simultaneously
   and requires one unambiguous outer cycle: its `[0,0]` corner-empty fixture is
   a 2×2 cycle plus a tail, while its `[1,1]` edge-middle-empty fixture is a
   tree. Both pin repeat/swap, restored parent correspondence, outward
   residuals, and absence of every internal latitude and longitude seam; a
   one-ULP seam mutation or duplicate-edge ambiguity fails closed. The all-six
   arm admits the complete 2×3 grid with no empty sibling only after all four
   longitude and three latitude adjacencies prove reverse-oriented and bit-
   exact simultaneously. It removes all seven internal edges, requires one
   unambiguous outer cycle with no artificial seam edge, and restores the
   parent mapping and maximum child/parent residual. Its real
   `0.14716980102990423`-tilt fixture pins all six cell owners, exact
   repeat/swap, one-ULP seam rejection, and duplicate-edge ambiguity rejection.
   The cap layouts preserve the singular-pole alias; every layout pins exact
   repeat/swap and outward residual evidence. Exact 6/5 piece, 147/146 pair,
   and 588/587 arc ceilings remain pinned. The path (`2,2,1,1`), T
   (`3,1,1,1`), and cycle (`2,2,2,2`) routes remain disjoint. One separate
   disconnected route admits the outer-column vertical pairs
   `[0,0]`/`[0,2]`/`[1,0]`/`[1,2]` with middle-column siblings
   `[0,1]`/`[1,1]` certified empty. It merges both exact latitude seams into
   exactly two canonical regions, excludes both longitude separators, restores
   the parent maps and maximum child/parent residuals, and pins repeat/swap,
   exact 6/5 piece, 147/146 pair, and 588/587 arc N/N-1 ceilings, plus one-ULP
   and duplicate-edge ambiguity rejection. A second disconnected route admits
   all four isolated-corner plus three-cell mixed-axis L layouts:
   `[0,0]` + `[0,2]`/`[1,1]`/`[1,2]`, `[1,2]` +
   `[0,0]`/`[0,1]`/`[1,0]`, `[1,0]` +
   `[0,1]`/`[0,2]`/`[1,2]`, and `[0,2]` +
   `[0,0]`/`[1,0]`/`[1,1]`; respectively the omitted graph-cut sibling pairs
   `[0,1]`/`[1,0]`, `[0,2]`/`[1,1]`, `[0,0]`/`[1,1]`, and
   `[0,1]`/`[1,2]` must certify empty. It proves and removes both reverse-
   oriented bit-exact L seams, requires zero occupied-boundary contact with
   every empty cut separator and no bit-exact contact between the singleton and
   merged component, then returns exactly two canonical regions with restored
   parent maps and maximum child/parent residuals. The four real fixtures pin
   repeat/swap, exact 6/5 piece, 147/146 pair, and 588/587 arc N/N-1 ceilings,
   plus one-ULP and duplicate-edge ambiguity rejection. Together with the
   outer-column vertical-pair route, these exhaust the exact disconnected four-
   positive graph layouts in this polar-by-wide 2×3 decomposition. Broader
   layouts outside the admitted exact same-row, same-column, full-row,
   mixed-axis three-cell-path, connected four-positive path/T/cycle,
   disconnected four-positive graph, exactly-five-positive, and all-six-
   positive families remain indeterminate.
   A first
   wide arm decomposes exactly one pole-clear wide operand
   into three closed sub-π cells. It returns `Complete` only when all cells are
   certified empty or exactly one positive region has two certified-empty
   siblings, which cancels every artificial seam before restoring the parent
   correspondence; piece/pair/arc ceilings pin exact 3/2, 84/83, and 336/335
   evidence. A two-wide arm now decomposes both parents into the same three
   closed sub-π cells and returns `Complete` after all nine Cartesian child
   pairs certify empty, for exactly one positive region with eight
   certified-empty siblings, for exactly two positive regions with seven
   certified-empty siblings, for exactly three pairwise non-edge-adjacent
   positives with six certified-empty siblings, for exactly three positives
   comprising one exact adjacent pair plus one isolated component with six
   certified-empty siblings, for exactly three positives forming a two-edge
   grid path with six certified-empty siblings, for exactly four positives
   forming a three-edge grid path with five certified-empty siblings, or for
   exactly five positives forming a four-edge grid path with four
   certified-empty siblings, for an exact connected four-, five-, or six-cell
   shared-seam union, for an exact connected seven-cell union with both other
   siblings certified empty and sub-full-turn parents, for an exact disconnected
   seven-positive occupied-corner singleton whose two orthogonal neighbor cells
   certify empty and whose other six occupied cells form one exact component,
   or for an exact
   connected eight-cell union whose sole
   remaining sibling certifies empty, or for nine positive cells whose
   exhaustive closed decomposition cancels every internal seam into one
   unambiguous outer cycle.
   Two or three pairwise non-edge-adjacent cells remain separate after closed
   sibling ownership excludes artificial seams and both certified-empty
   orthogonal owners exclude every diagonal corner contact.
   The sole adjacent pair in a mixed three-cell layout merges through that
   same exact seam rule while the certified-separated singleton retains its
   canonical grid component order. Edge-adjacent cells merge only when every internal seam is one
   reverse-oriented shared boundary edge with exactly two consecutive,
   bit-identical endpoint records. Removing each edge and splicing complementary
   paths restores one parent region; three- through six-cell paths recheck every
   remaining seam against the current merged boundary after each earlier splice.
   Connected four- through nine-cell non-path unions instead prove every
   internal seam simultaneously. Paired owners require reverse-oriented
   bit-exact edge records; one exact owner may replace a neighboring
   reconstruction only when a unique reverse consecutive edge has bit-identical
   endpoints in the unchanged pole-clear complementary chart and the endpoint
   is not a non-exact multi-owner grid corner. The remaining edges must trace
   one unambiguous outer cycle. The exact five-cell 2×2-cycle-plus-tail fixture produces one
   12-edge boundary, while the exact connected six-cell fixture produces one
   14-edge boundary. A physically coincident but bit-mismatched central-seam
   fixture remains indeterminate.
   Exactly five positive cells may instead retain multiple canonical components
   only when every cross-component pair has certified sibling emptiness,
   including both orthogonal empty owners at a diagonal corner. The pinned
   singleton-plus-four-cell fixture yields 3- and 8-edge boundaries; removing
   either separator owner fails closed.
   Exactly seven positive cells may instead form two canonical components in
   all four grid rotations: singleton `[0,0]` with cuts `[0,1]`/`[1,0]`,
   singleton `[0,2]` with cuts `[0,1]`/`[1,2]`, singleton `[2,0]` with cuts
   `[1,0]`/`[2,1]`, or singleton `[2,2]` with cuts `[1,2]`/`[2,1]`; every
   non-cut cell is occupied and the other six cells form one connected
   component. The existing connected-seven merger retains precedence. Only
   after it declines does this disconnected route require both cuts certified
   empty, zero occupied-boundary contact at every empty separator, and
   simultaneous exact proof and cancellation of all six internal seams in the
   six-cell component. Neither output may retain an artificial seam edge or
   share a bit-exact vertex with the other. Success returns exactly two
   canonical regions with the parent correspondence restored and each maximum
   residual propagated from its children and the parent bound. Real fixtures
   for all four rotations pin exact repeat/swap and 9/8 piece, 252/251 pair,
   and 1,008/1,007 arc N/N-1 admission; one-ULP seam mutation and duplicate-edge
   ambiguity fail closed.
   The reviewed seven-cell 2×3-block-plus-tail fixture now certifies one
   canonical 15-edge region. Seven adjacencies have paired bit-exact seam edges;
   `[1,0]/[1,1]` uses the bounded one-owner complementary-chart proof above.
   Closed-cell containment and pole-clear constraint topology establish the
   whole arc rather than merely matching endpoints. Operand swap is bit-exact.
   A second reviewed seven-cell fixture leaves opposite corner cells `[0,2]`
   and `[2,0]` certified empty. Its two nonadjacent notches produce a different
   adjacency graph from the 2×3-block-plus-tail fixture; the unchanged generic
   merger proves every current internal seam and returns one canonical 17-edge
   outer cycle with exact repeat/swap and parent correspondence.
   The reviewed eight-cell fixture occupies every grid cell except certified-
   empty `[2,1]`, proves all nine occupied adjacencies with paired bit-exact
   seams, and returns one canonical 18-edge outer cycle with exact repeat/swap.
   A second reviewed eight-cell fixture leaves corner cell `[2,0]` certified
   empty. Its distinct corner-notch topology passes the unchanged generic seam
   merger and returns one canonical 17-edge outer cycle with exact repeat/swap,
   proving the arm is broader than the original edge-empty layout.
   The reviewed nine-cell fixture needs no empty sibling because the closed
   3×3 decomposition exhausts the parent; all twelve internal adjacencies
   cancel and leave one canonical 17-edge outer cycle with no artificial seam.
   Exact 9/8 piece-pair, 252/251 boundary-pair, and 1,008/1,007 arc ceilings
   preserve fail-closed admission. Other seven-positive two-wide layouts outside
   the exact connected unions and four corner-singleton-plus-six-component
   rotations, other eight- or nine-positive two-wide layouts,
   disconnected five-cell layouts without exact sibling separation, non-exact
   or otherwise ambiguous multi-edge
   shared seams, full-turn aliases, other polar, and non-exact collapsed contacts are
   the next sphere boundaries.
   The first feature-ladder profile slice now accepts one exact polygonal
   outer boundary with any deterministic list of strictly contained,
   pairwise-disjoint, unnested polygonal holes. Robust orientation and segment
   predicates normalize outer/hole winding and reject touching, crossing,
   overlapping, nested, non-finite, degenerate, or out-of-size-box inputs
   before topology allocation. Checked planar-sheet construction authors one
   explicit line-pcurve loop per boundary, and checker v2 now proves every
   straight loop simple, the unique outer/hole containment relation, and the
   resulting single-face planar shell embedding. Full checking is `Valid`, and
   tessellation retains the exact holed area. The first checked consumer
   extrudes that same polygon-with-holes profile along any finite translation
   with a nonzero frame-normal component, with exact planar caps, one planar
   parallelogram per boundary segment, shared perimeter/sweep edges, and line
   pcurves on every use in the actual face frames. A builder-owned affine-prism
   proof makes Full checking `Valid`; the oblique holed fixture is watertight
   with signed volume 24 within `1e-9`, failed translations are allocation-clean,
   and the typed facade returns only opaque body and journal values. Reverse
   translations reflect the profile chart and reference normal together before
   using the same proof. Curved loops, nested material islands, zero-normal sweeps, and revolve
   remain.
   The K4 facade journal adapter now preserves mutation order, all semantic
   lineage shapes, and transaction-owned tolerance evidence. Its first checked
   semantic transactions compose affine-map- and incidence-metadata-aware
   position-owning pcurve strut creation/removal, face split/merge, bridge-edge
   removal/ring join, and face-as-hole merge/split, preserve exact rollback
   identity, and return the committed
   facade journal. KFMRH/MFKRH preserve moved pcurve identity and metadata,
   journal exact face merge/split lineage, and deliberately leave unsupported
   geometric hole containment as operation-specific evidence rather than a
   topology precondition. Ring joins
   preflight part/liveness, same-face distinct loops, selected tail vertices,
   bounded curve and both pcurves before checked mutation; bridge removal
   retains the lower KEMR same-loop/nonempty-split proof. Position-owning MVFS
   validates its surface and finite size-box seed position before allocating a
   hidden point and returns every created topology identity opaquely. Its exact
   scaffold is explicitly transient: checked commit rejects it until later
   Euler edits complete it or facade KVFS removes it in the same transaction.
   The facade KVFS inverse deletes and journals that point only when unshared,
   while ordinary lower KVFS retains external/shared geometry. Position-owning
   MEV preflights every fallible input before hidden-point allocation; its
   facade KEV inverse deletes and journals the point only when no live vertex
   shares it, without changing ordinary lower KEV ownership. Broader edit
   families and partition-history composition remain. MEF now copies complete
   optional face-tolerance provenance to the new face without growth; KEF
   selects the ordered `[surviving, absorbed]` maximum, breaks equal-value ties
   toward the survivor, and keeps two exact faces exact. The facade journal
   exposes part-qualified inputs, both merge values, the result, selected
   source, and selected provenance as descriptive evidence without budget or
   authoring authority. Checked/resource denial discards those records,
   restores tolerances, and reuses exact future face/edge/loop/fin identities.
   Policies for broader edit families remain. Operation-owned facade tolerance
   growth now batches unique part-qualified live Face/Edge/Vertex
   targets. Identity and liveness precede scalar validation; requested final
   values meet the model-resolution floor; exact aggregate accounting and
   imported-origin-preserving provenance complete before an infallible apply;
   and committed events retain request order under a journal-local budget ID
   that no authoring method accepts. N/N-1 exhaustion, rollback, and checked
   denial restore model and transaction-local budget state exactly. The additive
   Full-assurance commit gate now makes Full proofs load-bearing without changing
   Fast commit behavior: distinct explicit roots precede affected/store-audit roots;
   Fast graph validation and Full proofs share one scope; `RequireValid` rejects
   gaps while `AllowIndeterminate` retains them; and Full faults always reject.
   Committed or rollback-clean rejected results own ordered facade reports, rejected
   results have no journal, and proof/accounting denial restores tolerances, the
   committed index, and future identities, including exact 306/305 graph-work
   coverage. F6 splits and F4
   legacy cleanup land only with an
   owner-level behavioral migration. The Q2a/
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
The Q2 topology ladder now has 35 rows. Its prior 28 rows remain unchanged,
including the seven-row mixed-store cohort matrix that holds four affected
minimal bodies across 4/16/64/256 total bodies and separately sweeps
1/4/16/64 affected roots at 64 total bodies. Seven affected-solid
`primitive_mix` rows form a crossed production grid: 1/4/16/64 affected roots
at 64 total bodies and one affected root across 4/16/64/256 total bodies, with
the 64/1 row shared. The total-size ladder owns one ordinary checked operation
scope and grows exactly the first face to `2e-8` under an exact `1e-8` budget;
it pins one modified Face net mutation and one ordered tolerance event,
affected/refreshed/checked/mutations = 1, a stable affected digest across
totals, before/after store and full-output digest ratchets, and installed-index
equality. The fixed-total ladder retains exact `N × 1e-8`, N-face/event, scope,
digest, and index-equality evidence. These deterministic counters establish
scoped root count and production-solid total-size behavior; full graph
validation, committed-index cloning, body-order refresh, global ordinary-commit
cost, broader production edit footprints, and production assembly remain
explicit performance boundaries. The complete benchmark manifest now contains
170 cases.

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

**Implemented source-identity slice:** ellipse/ellipse projection failures retain
all five `ProjectionError` variants in `IntersectionError::Projection`. The
concrete and generic intersection layers plus `GeometryIntersectionError` and
`KernelError` delegate class, code, limit, `capability() == None`, and the exact
source; the policy arm also retains `OperationPolicyError`. The direct
`intersect_bounded_ellipses` API returns `IntersectionResult`, so no compatibility
adapter flattens the source into `kcore::Error::InvalidGeometry`. Other solver-local
collapses remain owner-driven migration debt.

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
