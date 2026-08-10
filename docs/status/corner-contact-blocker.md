# Corner-contact Boolean representability blocker

Stages 1–3 of the skew-cylinder corner-contact rung are complete. The exact
four-bound `Isolated` events now flow through kgraph and kops into first-class
Section isolated contacts, and the public corner fixture stitches one closed
eight-fragment component with two zero-dimensional members and zero gaps in
both operand orders under World and Oblique frames.

Stage 4 remains deliberately blocked and the production Boolean continues to
refuse atomically with `BooleanRefusal::AssemblyRejected`.

## Exact blocker

An allocation-free diagnostic extension of the general cylinder-pair boundary
pipeline consumed the two isolated contacts as source-ring split points and
carried all eight real Section fragments through periodic and disk
arrangements. Subtract then selected one edge-connected proposal with:

- 8 faces;
- 8 Section edges;
- 1 physical shell component; and
- boundary cells joined through the isolated point-contact strata.

This is the known point-pinched result class, not either existing bounded-skew
lobe class. The proposal could be allocated, but `RequireValid` Full checking
rejected it without committing. Its single body report had no faults and the
following unresolved proof obligations:

- three loops: `LoopOrientation` and `LoopSelfIntersection`;
- one shell: `ShellSelfIntersection` and `ShellOrientation`.

The blocking checker invariant is that a committed solid boundary must be a
Full-certified closed vertex-manifold shell with simple oriented face loops.
The existing bounded-skew lobe theorem recognizes a 4-face, 6-edge, 4-vertex
shell whose vertex links are cycles; it does not recognize the connected
8-face point-pinched proposal. None of the other existing shell theorems
certifies that topology. Treating the isolated contacts as ordinary loop
vertices would therefore leave loop simplicity and the shell vertex link
unproved. Merging through the points, duplicating them by fixture rule, or
adding a degenerate edge would change or disguise the published topology.

The trusted shell cascade and `LIVE_SHELL_CERTIFIERS` remain byte-identical at
their existing eight entries. No ninth certifier was added, and the diagnostic
Stage 4 experiment was discarded.

## Representation decision is oracle-bound

Decomposing the selected set into vertex-manifold bodies and extending the
store/checker contract for point-contact solids are hypotheses, not choices to
make from first principles. Parasolid X_T round-trip is the representation
contract, so Stage 4 stays blocked until licensed Parasolid shows what its own
Subtract emits for this exact topology.

The manual R5 `corner-contact` suite now constructs the two fixture cylinders
inside Onshape from the same exact literals and evaluates `first - second`:

- first cylinder: radius 13, axis `(0,0,16)` to `(0,0,17)`;
- second cylinder: radius 20, axis `(-14,0,0)` to `(5,0,0)`; and
- expected isolated contact positions: `(5,-12,16)` and `(5,12,16)`.

The probe does not assert a body count. It retains Onshape's native body,
face, loop, edge, and vertex structure; exports the result as raw X_T; imports
that X_T back into Onshape; retains the replay structure and re-export; and
compares normalized topology with host ids removed. This distinguishes at
least separate bodies, one body with topologically shared contact vertices,
and coincident-but-distinct vertices. The raw X_T remains authoritative for
shell/region structure not exposed by the body-details API.

The observed native and replay representation will select the implementation
route. Until that evidence is recorded and its topology can pass Full
validation without a new trusted certifier, `AssemblyRejected` is the required
covenant-preserving result.
