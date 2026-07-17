# Certified bundle snapshots

Committed byte-for-byte snapshots of oracle-bundle fixtures whose SHA-256
identities are pinned in `docs/oracle-certification.json` (host run
2026-07-11, writer `2beb267`). Consumers here are import-accounting and
resource-budget tests that require *stable input bytes*, not current writer
output — so these files must never be regenerated in place. Current writer
output belongs in the gitignored `oracle/outbox/` transport directory and is
validated separately by `scripts/oracle_loop.py certification-check`.

The same convention lives in `benches/testdata/*.certified.x_t`. Adding a
snapshot here requires its SHA-256 to match an entry in the certification
record; if the writer legitimately changes a fixture, add a new snapshot with
new provenance rather than overwriting the certified bytes.

| File | Provenance |
| --- | --- |
| `solid_block_nurbs_edge.certified.x_t` | Onshape cloud-2026-07-11 accepted bundle; sha256 pinned in `docs/oracle-certification.json` |
