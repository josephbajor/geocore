"""Offline tests for licensed-host certification freshness."""

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from oracle.certification import SCHEMA_VERSION, validate_certification  # noqa: E402
from oracle.onshape import OracleError  # noqa: E402


def identity(fixture_hash="fixture-hash", writer_hash="writer-hash"):
    return {
        "writer_inputs_sha256": writer_hash,
        "bundle_sha256": "bundle-hash",
        "fixture_count": 1,
        "fixtures_sha256": {"a.x_t": fixture_hash},
    }


def record(status="current", reason=""):
    result = {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "reason": reason,
        **identity(),
    }
    return result


class CertificationTests(unittest.TestCase):
    def test_exact_current_identity_passes(self):
        self.assertIn("CURRENT", validate_certification(record(), identity()))

    def test_current_writer_or_fixture_mismatch_fails_closed(self):
        with self.assertRaisesRegex(OracleError, "writer_inputs_sha256"):
            validate_certification(record(), identity(writer_hash="changed"))
        with self.assertRaisesRegex(OracleError, "fixture:a.x_t"):
            validate_certification(record(), identity(fixture_hash="changed"))

    def test_acknowledged_stale_state_passes_ordinary_ci_but_not_release_gate(self):
        stale = record("stale", "writer bytes changed")
        self.assertIn("STALE", validate_certification(stale, identity(writer_hash="changed")))
        with self.assertRaisesRegex(OracleError, "is stale"):
            validate_certification(stale, identity(), require_current=True)

    def test_stale_reason_and_schema_status_are_validated(self):
        with self.assertRaisesRegex(OracleError, "requires a reason"):
            validate_certification(record("stale"), identity())
        invalid_schema = record()
        invalid_schema["schema_version"] = "future"
        with self.assertRaisesRegex(OracleError, "schema"):
            validate_certification(invalid_schema, identity())
        invalid_status = record()
        invalid_status["status"] = "unknown"
        with self.assertRaisesRegex(OracleError, "status"):
            validate_certification(invalid_status, identity())


if __name__ == "__main__":
    unittest.main()
