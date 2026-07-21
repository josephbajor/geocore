# External Parasolid Oracle Loop (M3b)

Self-round-trip cannot certify interchange: any convention shared by this repository's
reader and writer — the provisional v-fastest B-surface pole ordering is the standing
example — round-trips cleanly and stays invisible. The M3b exit gate therefore requires
every declared Tier 1 authoring capability to import into a licensed Parasolid host with
zero checker errors and survive a there-and-back comparison. This document is the
operating procedure for that loop. It requires an explicitly dispatched operator or agent
with host access; repository CI never contacts the licensed host.

## Certification invalidation rule

Licensed-host evidence certifies the exact writer behavior that produced the
tested bytes, not the repository's writer in perpetuity. A change to
`crates/kxt/src/write.rs`, writer planning/schema code, writer-reachable geometry
serialization, or any generated fixture bytes makes the corresponding prior
host evidence stale. Treat “writer bytes changed since the last oracle run” as
a standing re-test trigger even when local round-trip and checker tests pass.

After such a change:

1. mark the affected certification record `stale` and queue the change;
2. identify every fixture whose bytes changed, conservatively treating an
   unclear impact as the full bundle;
3. batch queued writer changes until the next catch-up checkpoint—do not spend
   host requests certifying a superseded intermediate revision;
4. regenerate the final deterministic bundle and retain its manifest hashes;
5. re-run the affected final bytes in a licensed Parasolid host before making a
   current writer-conformance claim; and
6. append results with the writer Git revision and bundle identity in the
   notes field of `docs/oracle-results.tsv` until those fields receive a
   versioned schema of their own.

CI does not replace the licensed host. Its deterministic freshness checks
compare regenerated writer/bundle identities with the last committed licensed
evidence. A stale record blocks a conformance claim, not unrelated development.

`current` means only that the regenerated identities match the exact bytes cited
by the committed host record. It does not mean CI contacted the host, every
fixture passed, or the writer is generally Parasolid-conformant. `stale` retains
historical evidence but cannot support a current-byte claim. A partial catch-up
may append result rows, but the record stays stale until the final queued batch
and its exact identity are fully recorded.

## Catch-up cadence and request budget

Onshape allows 2,500 API requests per year. The workflow's default dispatched
session cap is 200 requests and its hard maximum is 400. Uploads, translation
polls, element lookup, re-export attempts, and credential checks all consume the
same allowance. Size the queued fixture batch before dispatch, keep requests
serial, and stop rather than exceed the cap or retry a rate-limited session.
Partial sessions remain stale and resume in a later manually dispatched batch.

**Last licensed-host run (2026-07-20, writer=b596027):** the declared base
matrix imported 12/15 and compared 7/12 clean. Wire/acorn parse rejection,
analytic NURBS canonicalization, cone/tolerant-edge reader gaps, and the offset
sheet's missing re-export remain open. The curved B-surface is accepted and
compares clean. The supplemental public Boolean matrix imported 6/6 and
compared 6/6 clean, including fragmented shared-surface solids, disjoint union
bodies, and a two-shell finite-void cavity. Evidence: `docs/oracle-results.tsv`.

**Current certification state (2026-07-21): base stale, Boolean stale.**
Finite-cylinder Full proof changed the regenerated base manifest's checker
evidence after writer `b596027`; its 15 payload hashes are unchanged, but the
base bundle needs licensed-host re-certification. The six-payload Boolean record
retains historical 2026-07-20 evidence, but the generator now adds eight queued
Plane/Cylinder payloads: bounded-arc intersection, both ordered
planar-minus-cylinder bodies, rectangular cap-retaining Unite/cylinder-left
Subtract, five-portal variants of both operations, and seam-crossing five-portal
cylinder-left Subtract.
The complete regenerated fourteen-payload bundle has deterministic offline identity
`6a869c653b5864fd17b96f631610119f981395f20818effd84a03d4389a42936`
and awaits a fresh licensed-host run; this queued identity is not host
certification.

## 1. Generate the bundle

```sh
cargo run --release -p kxt --bin xt_oracle -- export oracle/outbox
```

This writes the current 15-file declared set—one `.x_t` file per declared Tier
1 writer capability plus the canonical `offset_plane.x_t`—and `manifest.tsv`
(expected topology counts, enclosed volume, checker outcomes, byte count, FNV-1a hash).
Generation is deterministic — same source, same bytes — and every file is re-imported
and re-checked locally before it is written, so a host is never handed a file this
repository's own pipeline rejects. `oracle/` is gitignored transport space; the
committed record is `docs/oracle-results.tsv`.

The bundle retains the host-canonicalized exactly planar
`solid_block_nurbs_face.x_t` and the host-preserved
`solid_block_curved_nurbs_face.x_t`. The curved fixture preserves exact linear
block boundaries while displacing the sole interior biquadratic control point;
it therefore cannot be canonicalized to a plane. **Every base-matrix
certification session must include both B-surface fixtures.**

The exporter rejects stale or unexpected outbox entries, and the API CLI reads
the manifest order rather than globbing transport residue. After generation,
inspect the exact identity and run the offline freshness gate with:

```sh
python3 scripts/oracle_loop.py identity --outbox oracle/outbox
python3 scripts/oracle_loop.py certification-check --outbox oracle/outbox
```

`docs/oracle-certification.json` records the certified writer-input, bundle,
and per-fixture SHA-256 identities. A `current` mismatch fails CI. An explicitly
`stale` record must carry a reason and passes ordinary development CI with a
prominent warning; release/conformance gates add `--require-current`.

The Boolean rung has a separate facade-only supplemental bundle:

```sh
cargo run --release -p kernel --example boolean_xt_oracle -- oracle/boolean-outbox
python3 scripts/oracle_loop.py certification-check \
  --outbox oracle/boolean-outbox \
  --record docs/oracle-boolean-certification.json
```

Its fourteen payloads cover connected block/block unite/subtract/intersect, both
bodies of a disjoint union, contained subtraction with one finite void, one
bounded-arc block/cylinder intersection, and both public-result-order bodies of
the bounded-arc planar-minus-cylinder subtraction, plus rectangular and
five-portal cap-retaining Unite/cylinder-left Subtract plus the seam-crossing
five-portal cylinder-left Subtract. Generation requires
Full-valid committed results, independent volumes, local X_T import,
byte-stable replay, and an empty output directory. The eight queued files are
`bounded_arc_plane_cylinder_intersect.x_t`,
`bounded_arc_plane_cylinder_subtract_body_0.x_t`,
`bounded_arc_plane_cylinder_subtract_body_1.x_t`,
`cap_retaining_plane_cylinder_unite.x_t`,
`cap_retaining_cylinder_minus_plane.x_t`,
`five_portal_plane_cylinder_unite.x_t`, and
`five_portal_cylinder_minus_plane.x_t`, and
`seam_crossing_five_portal_cylinder_minus_plane.x_t`; this batch is not yet
licensed-host certification. After the final queued changes settle, regenerate
the bundle, record `identity` output, and manually dispatch the full
fourteen-payload Boolean suite with that exact `bundle_sha256`; keep the record
stale until every final payload has completed import/re-export comparison.

## 2. Manual catch-up entry points

`scripts/oracle_loop.py` drives the entire loop through Onshape's REST API
using a developer API key pair — no browser, no cookie session, no manual
uploads. An authorized human or agent starts each catch-up session explicitly;
repository CI has no host credentials and never invokes `bundle`.

The shared path is the manual **Licensed-host oracle catch-up** GitHub Action
(`.github/workflows/oracle-catchup.yml`), dispatched from the default branch.
Protect its `onshape-oracle` environment and configure `ONSHAPE_ACCESS_KEY`,
`ONSHAPE_SECRET_KEY`, `ONSHAPE_DOCUMENT_ID`, `ONSHAPE_WORKSPACE_ID`, and
`ONSHAPE_ELEMENT_ID` as secrets. Choose `base` or `boolean`, optionally select
exact manifest fixture names with the locally generated bundle SHA-256, confirm
the spend, and set the request ceiling.
The action serializes runs and archives exact identities, outboxes, host re-exports,
logs, metadata, and partial result rows; it never changes a certification record
automatically. To continue a capped partial run, dispatch the remaining manifest
names with the prior artifact's `bundle_sha256`; a changed bundle fails before any
host request rather than combining evidence from different bytes.

For a local session, one-time setup is:

1. Create an API key pair at <https://dev-portal.onshape.com> (available on
   free plans) and export it in the environment:

   ```sh
   export ONSHAPE_ACCESS_KEY=... ONSHAPE_SECRET_KEY=... ONSHAPE_REQUEST_LIMIT=200
   ```

   Keys are secrets: never committed. Either export them in the environment
   or put both lines in `.env` at the repository root — the CLI loads it
   automatically, real environment variables take precedence, and `/.env` is
   gitignored.

2. Create a disposable Onshape document containing one blob element to
   receive uploads (free-plan documents are public — upload nothing
   confidential), then record its coordinates once:

   ```sh
   python3 scripts/oracle_loop.py init   # writes untracked oracle/config.json
   # fill in document_id/workspace_id/element_id from the document URL:
   #   https://cad.onshape.com/documents/{did}/w/{wid}/e/{eid}
   python3 scripts/oracle_loop.py check  # verifies the credentials
   ```

After the queued changes settle and the final batch fits the session request
cap, dispatch a certification run:

```sh
cargo run --release -p kxt --bin xt_oracle -- export oracle/outbox
python3 scripts/oracle_loop.py bundle --reexport --compare --results-rows
```

Use `--fixtures NAME.x_t ...` to run only queued or remaining names from that
bundle's manifest; unknown and duplicate names fail before credentials load.

For the supplemental Boolean matrix:

```sh
python3 scripts/oracle_loop.py bundle \
  --outbox oracle/boolean-outbox \
  --inbox oracle/inbox/onshape/boolean \
  --reexport --compare --results-rows
```

`bundle` uploads every manifest fixture by default, or only the manifest-bound
`--fixtures` selection, then waits for each translation verdict,
classifies any `failureReason` against the taxonomy in the appendix,
downloads the host's Parasolid re-export into `oracle/inbox/onshape/`, runs
`xt_oracle compare` on each outbox/inbox pair, and prints ready-to-append
`docs/oracle-results.tsv` rows stamped with the writer git revision. Exit 1
means completed host findings; exit 2 means incomplete operational evidence.
`--results-file PATH` retains completed rows even if a later request hits the cap.
`run FILE... --results-rows` does the same for individual files when
bisecting a single writer defect.

The upload/poll endpoints and synchronous Part Studio Parasolid re-export with
`includeSurfaces=true` are live-host proven by the committed 2026-07-20 rows.

## 3. Manual host import (alternative hosts)

Accessible Parasolid-backed hosts, in preference order:

1. **Solid Edge Community Edition** (free license, Windows) — a real Parasolid kernel
   with a user-facing body checker. Open each `.x_t`, then run the geometry inspector
   (`Inspect` → check/optimize tooling varies by release) and record any errors or
   repairs. Measure the body's volume (`Inspect` → `Physical Properties`; Parasolid
   files are in meters — check the displayed unit) and compare against the manifest's
   `volume_exact_m3` (the closed-form value; the neighbouring `volume_mesh_m3` is the
   chord-1e-3 tessellated value used by `compare` and sits up to ~1% below it on
   curved solids).
2. **Onshape** (free plan; Parasolid-kernel SaaS) — upload the `.x_t` to a document and
   record whether translation succeeds and whether the imported part looks correct.
   Note: free-plan documents are public.
3. **Fusion 360** (personal license) — a secondary, non-Parasolid translator; useful as
   an additional data point, not as the certification oracle.

Open-source OCCT cannot read XT (its XT translator is a commercial component), so OCCT
serves only as a geometry/mass-property differential via a host-exported STEP, or after
M6 STEP support lands here.

For each file record in `docs/oracle-results.tsv`: did import succeed, did the host
checker report anything, does the host-measured volume match the manifest.

## 4. Re-export and compare (manual flow)

Re-export each part from the host as Parasolid text (`.x_t`) into
`oracle/inbox/<host>/`, keeping the file name, then run:

```sh
cargo run --release -p kxt --bin xt_oracle -- compare \
    oracle/outbox/solid_block.x_t oracle/inbox/solid-edge/solid_block.x_t
```

`compare` re-imports both files here and diffs body kind, entity counts, geometry-class
histograms, entity tolerances, checker cleanliness, watertightness, and tessellated
volume (2e-3 relative — XT does not store edge parameter bounds, so a re-import
legitimately re-triangulates; the bound is discretization-driven). Exit code 0 means
the re-export matches; 1 means mismatches (each is
printed); 2 means the host file could not be read — which is itself a reader-gap
finding worth a fixture. Hosts typically write their own newer schema with embedded
edit scripts; the reader supports that mechanism, so a parse failure is signal, not
noise.

## 5. Record the outcome

Append one row per (host, fixture) run to `docs/oracle-results.tsv`. That file is the
committed, non-shrinking record the M3b exit gate ratchets on: 100% of the declared
Tier 1 matrix importing with zero host checker errors and surviving there-and-back
comparison. Failures stay in the table with their resolution commit; do not delete
rows. If a host rejects or repairs a file, minimize the difference, fix the writer,
regenerate the bundle, and re-run.

## Appendix: the cookie-session fallback loop (Onshape)

The API-key CLI in section 2 supersedes this flow. Use it only when no key
pair is provisioned: the committed helper
`scripts/oracle/browser_fallback.js`, pasted into the devtools console (or
injected via browser tooling) on a logged-in Onshape document page, installs
`window.__oracle` and borrows the human's session cookies. Each experiment is
then `await __oracle.run(name, base64Bytes)`. The underlying endpoints are the
same two the CLI uses:

1. `POST /api/v6/blobelements/d/{did}/w/{wid}/e/{eid}` with a multipart `file`
   field replaces a blob element's content **and starts a translation
   automatically**, returning a `translationId` (send the `XSRF-TOKEN` cookie
   value as the `X-XSRF-TOKEN` header; session cookies authenticate).
2. `GET /api/v6/translations/{translationId}` until `requestState` leaves
   `ACTIVE`; a failure carries `failureReason` (e.g. "Invalid or corrupt input
   file" = parse-level rejection vs "Imported file contains no translatable
   geometry" = parsed but semantically rejected — the distinction localizes a
   defect to framing/layout versus topology/geometry semantics).

The 2026-07-11 session drove ~15 experiments this way, bisecting from a
known-good corpus file to isolate four writer conformance rules (embedded
schema required, BODY/REGION edit scripts, USFLD_SIZE=1, no line may end with
a space). All four are now encoded and pinned in `kxt::write`.

Operational pitfalls observed:

- The `XSRF-TOKEN` cookie value can end in base64 `=` padding; extract it
  with everything after the first `=` (`raw.slice('XSRF-TOKEN='.length)`),
  never `split('=')[1]`, or the POST fails 401 with an empty body.
- Browser sessions expire between working sessions. A stale page still
  renders (free-plan documents are public) and may even show the signed-in
  avatar, but `GET /api/users/sessioninfo` returns 204 (anonymous) and every
  POST fails 401 "Unauthenticated API request". The reconnect banner
  redirects to `/signin`; a human must sign in again before the loop can
  resume.
- `USFLD_SIZE` is not a conformance requirement in itself: real V27 corpus
  files declare 1, while Onshape's own Parasolid 37 exports declare 0. Both
  are accepted; this writer emits 1 to match its V26105 declaration.

The 2026-07-11 rerun (10/13 accepted) added five host-verified writer
conventions, each isolated by diffing against a real file or by uploading
hand-patched probe variants:

- `EDGE.fin` must point at the positive-sense fin (cyl.x_t; the negative
  pick read as "no translatable geometry" on periodic-wall solids).
- `NURBS_CURVE` must declare `knot_type` 5 and `curve_form` 1, and every
  `B_CURVE` needs a `CURVE_DATA` companion (exemplar.x_t, 301/301); zeros
  read as "corrupt" on B-curve edges and killed tolerant SP chains late in
  translation.
- A sheet's shell lists its faces as `front_face`, and each face fronts
  that same shell (disk_nat.x_t).
- Every fin claiming a vertex — loop-less dummies included — must be
  reachable from `VERTEX.fin` via the `next_at_vx` chain, and a sheet
  boundary dummy fin must keep its vertex pointer (zeroing it regresses
  from "no translatable geometry" back to "corrupt").
- Wire and acorn bodies are still rejected as corrupt; no real exemplar
  exists to bisect against (acorn suspect: the body_type encoding).
