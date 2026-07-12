"""Offline identity and freshness checks for licensed-host oracle evidence."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .onshape import OracleError


ROOT = Path(__file__).resolve().parents[2]
CERTIFICATION_PATH = ROOT / "docs" / "oracle-certification.json"
SCHEMA_VERSION = "kernel-oracle-certification.v1"
WRITER_INPUTS = (
    "crates/kxt/src/write.rs",
    "crates/kxt/src/schema.rs",
)


def _digest_named_payloads(payloads):
    digest = hashlib.sha256()
    for name, payload in payloads:
        encoded = name.encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(len(payload).to_bytes(8, "big"))
        digest.update(payload)
    return digest.hexdigest()


def observed_identity(outbox, root=ROOT):
    """Hash the writer inputs and exact manifest-declared host payloads."""
    root = Path(root)
    outbox = Path(outbox)
    manifest_path = outbox / "manifest.tsv"
    try:
        manifest = manifest_path.read_bytes()
    except FileNotFoundError:
        raise OracleError("missing bundle manifest: {}".format(manifest_path))
    lines = manifest.decode("utf-8").splitlines()
    if not lines or not lines[0].startswith("file\tbody_kind\t"):
        raise OracleError("invalid bundle manifest header: {}".format(manifest_path))
    names = [line.partition("\t")[0] for line in lines[1:]]
    if not names or len(names) != len(set(names)):
        raise OracleError("bundle manifest fixture names are empty or duplicated")
    fixtures = {}
    bundle_payloads = [("manifest.tsv", manifest)]
    for name in names:
        if not name.endswith(".x_t") or Path(name).name != name:
            raise OracleError("invalid bundle fixture name: {!r}".format(name))
        path = outbox / name
        try:
            payload = path.read_bytes()
        except FileNotFoundError:
            raise OracleError("bundle is missing manifest fixture: {}".format(name))
        fixtures[name] = hashlib.sha256(payload).hexdigest()
        bundle_payloads.append((name, payload))
    actual = {path.name for path in outbox.glob("*.x_t")}
    if actual != set(names):
        raise OracleError("bundle fixture set does not match its manifest")
    writer_payloads = []
    for relative in WRITER_INPUTS:
        path = root / relative
        writer_payloads.append((relative, path.read_bytes()))
    return {
        "writer_inputs_sha256": _digest_named_payloads(writer_payloads),
        "bundle_sha256": _digest_named_payloads(bundle_payloads),
        "fixture_count": len(fixtures),
        "fixtures_sha256": fixtures,
    }


def validate_certification(record, observed, require_current=False):
    """Validate one record against observed bytes; return its status message."""
    if record.get("schema_version") != SCHEMA_VERSION:
        raise OracleError("unsupported oracle certification schema")
    status = record.get("status")
    if status not in ("current", "stale"):
        raise OracleError("oracle certification status must be current or stale")
    if status == "stale":
        reason = str(record.get("reason", "")).strip()
        if not reason:
            raise OracleError("stale oracle certification requires a reason")
        if require_current:
            raise OracleError("oracle certification is stale: {}".format(reason))
        return "STALE oracle certification acknowledged: {}".format(reason)

    mismatches = []
    for field in ("writer_inputs_sha256", "bundle_sha256", "fixture_count"):
        if record.get(field) != observed.get(field):
            mismatches.append(field)
    expected_fixtures = record.get("fixtures_sha256")
    if not isinstance(expected_fixtures, dict):
        raise OracleError("current oracle certification requires fixture hashes")
    actual_fixtures = observed.get("fixtures_sha256", {})
    for name in sorted(set(expected_fixtures) | set(actual_fixtures)):
        if expected_fixtures.get(name) != actual_fixtures.get(name):
            mismatches.append("fixture:" + name)
    if mismatches:
        raise OracleError(
            "oracle certification is falsely current; identity mismatches: {}".format(
                ", ".join(mismatches)
            )
        )
    return "CURRENT oracle certification matches {} host payloads".format(
        observed["fixture_count"]
    )


def check_certification(outbox, record_path=CERTIFICATION_PATH, require_current=False):
    """Load, observe, validate, and print the committed freshness state."""
    try:
        record = json.loads(Path(record_path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise OracleError("missing oracle certification record: {}".format(record_path))
    except ValueError:
        raise OracleError("oracle certification record is not valid JSON")
    observed = observed_identity(outbox)
    message = validate_certification(record, observed, require_current=require_current)
    print(message)
