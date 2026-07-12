# External Parasolid Oracle Loop (M3b)

Self-round-trip cannot certify interchange: any convention shared by this repository's
reader and writer — the provisional v-fastest B-surface pole ordering is the standing
example — round-trips cleanly and stays invisible. The M3b exit gate therefore requires
every declared Tier 1 authoring capability to import into a licensed Parasolid host with
zero checker errors and survive a there-and-back comparison. This document is the
operating procedure for that loop. It requires a human with host access; nothing else in
the roadmap waits on it, so run it early and re-run it whenever writer capabilities grow.

## Certification invalidation rule

Licensed-host evidence certifies the exact writer behavior that produced the
tested bytes, not the repository's writer in perpetuity. A change to
`crates/kxt/src/write.rs`, writer planning/schema code, writer-reachable geometry
serialization, or any generated fixture bytes makes the corresponding prior
host evidence stale. Treat “writer bytes changed since the last oracle run” as
a standing re-test trigger even when local round-trip and checker tests pass.

After such a change:

1. regenerate the deterministic bundle and retain its manifest hashes;
2. identify every fixture whose bytes changed, conservatively treating an
   unclear impact as the full bundle;
3. re-run those bytes in a licensed Parasolid host before making a current
   writer-conformance claim; and
4. append results with the writer Git revision and bundle identity in the
   notes field of `docs/oracle-results.tsv` until those fields receive a
   versioned schema of their own.

CI does not replace the licensed host. Q8 should add a deterministic staleness
check that compares the current writer/bundle identity with the last committed
certified identity and reports certification as stale. That gate blocks a
writer-conformance claim, not unrelated kernel development.

**Current handoff state (2026-07-11, writer=2beb267):** certification is
current. The full 14-fixture loop ran through the API CLI: 11/14 imports
accepted (wire/acorn remain the known parse-level rejections), and the first
there-and-back leg in project history compared 6/9 exports clean — block,
cylinder, sphere, torus, and both sheets round-trip through Onshape's kernel
with matching topology, geometry classes, and volume. `offset_plane.x_t` was
rejected as corrupt until the writer registered each OFFSET_SURF in its basis
surface's geometric-owner ring (exemplar evidence: 44/44); it now imports.
Open findings from the compare leg: the two exactly-analytic NURBS fixtures
come back host-canonicalized (line/plane), so class-preservation needs
genuinely curved fixtures; Onshape's cone and tolerant-edge re-exports fail
our reconstruction (preserved as `*_onshape_reexport.x_t` reader-gap
fixtures); and the accepted offset sheet materializes no exportable body, so
its re-export leg is unavailable.

## 1. Generate the bundle

```sh
cargo run --release -p kxt --bin xt_oracle -- export oracle/outbox
```

This writes one `.x_t` file per declared Tier 1 writer capability plus `manifest.tsv`
(expected topology counts, enclosed volume, checker outcomes, byte count, FNV-1a hash).
Generation is deterministic — same source, same bytes — and every file is re-imported
and re-checked locally before it is written, so a host is never handed a file this
repository's own pipeline rejects. `oracle/` is gitignored transport space; the
committed record is `docs/oracle-results.tsv`.

The bundle deliberately includes `solid_block_nurbs_face.x_t` (a B_SURFACE part).
**Every host run must include it** until a licensed host confirms or corrects the
provisional B-surface pole ordering in `kxt::recon`.

## 2. Run the loop (Onshape API CLI — primary)

`scripts/oracle_loop.py` drives the entire loop through Onshape's REST API
using a developer API key pair — no browser, no cookie session, no manual
uploads — so any human, agent, or bounded CI job with the two environment
variables can certify a bundle. One-time setup:

1. Create an API key pair at <https://dev-portal.onshape.com> (available on
   free plans) and export it in the environment:

   ```sh
   export ONSHAPE_ACCESS_KEY=... ONSHAPE_SECRET_KEY=...
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

Each certification run is then:

```sh
cargo run --release -p kxt --bin xt_oracle -- export oracle/outbox
python3 scripts/oracle_loop.py bundle --reexport --compare --results-rows
```

`bundle` uploads every outbox fixture, waits for each translation verdict,
classifies any `failureReason` against the taxonomy in the appendix,
downloads the host's Parasolid re-export into `oracle/inbox/onshape/`, runs
`xt_oracle compare` on each outbox/inbox pair, and prints ready-to-append
`docs/oracle-results.tsv` rows stamped with the writer git revision. A
nonzero exit means at least one rejection or compare mismatch.
`run FILE... --results-rows` does the same for individual files when
bisecting a single writer defect.

The upload/poll endpoints are host-proven from the original loop. The
export-back pair (`partstudios .../translations` + `externaldata` download)
follows Onshape's published API but predates any live run here: confirm it on
first use and correct this document if the response shapes differ.

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
