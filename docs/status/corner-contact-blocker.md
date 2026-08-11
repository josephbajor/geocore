# Corner-contact Boolean representation decision and resolution

The representation blocker is resolved. Parasolid's native result, observed in
[oracle run 31433897241](https://github.com/josephbajor/geocore/actions/runs/31433897241),
selects the point-contact representation the kernel now commits.

## Parasolid oracle

For the exact `first - second` fixture, Onshape/Parasolid produced one solid
body with one shell, 8 faces, 14 edges, and 8 vertices. The contact points are
topologically shared vertices at `(5,-12,16)` and `(5,12,16)`. Each has degree
five, its face link is a five-cycle, and the lower cap is split into three
simple planar faces. Native and X_T replay structure agreed. Parasolid did not
decompose the two point-touching lobes into separate bodies and did not encode
the contacts as degenerate edges.

## Kernel result

The exact `Isolated` events flow from the finite-window family through kgraph
and kops into first-class Section isolated contacts. Section stitches one
closed eight-fragment component with two zero-dimensional members and zero
gaps in both operand orders under World and Oblique frames. The unpublished
`Contact` stratum retains its typed fail-closed refusal.

Subtract now commits one Full-valid solid in both orders and frames:

- `first - second` is the oracle-aligned 8-face, 14-edge, 8-vertex shell with
  two degree-five contact vertices;
- `second - first` is a 7-face, 13-edge, 8-vertex shell with one two-loop
  cylinder face and one endpoint-free periodic ring; and
- every bounded edge has two distinct endpoints. No isolated point is
  disguised as a zero-length curve or legacy planar `SectionVertex`.

Both shapes certify through the existing bounded-skew theorem entry. Its
discovery consumes the complete family, exact isolated roots, source supports,
member adjacency, sheet occupancy, loop winding, and vertex-link evidence.
The trusted shell cascade remains at eight entries; no new certifier or checker
expectation downgrade was introduced.

## X_T queue

`corner_contact_first_minus_second.x_t` and
`corner_contact_second_minus_first.x_t` are appended to the deterministic
Boolean oracle bundle. Repeated export, local import, topology preservation,
public Fast checking, and independent re-export pass in both orders and rigid
frames. The eight shared point owners appear as the reader's expected skipped
non-geometric type-141 metadata.

Licensed-host R5 replay of the expanded 18-payload bundle is still pending.
The queued files therefore establish local round-trip stability only; they are
not yet a Parasolid X_T conformance claim.
