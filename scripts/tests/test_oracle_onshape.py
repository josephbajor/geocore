"""Offline regression tests for the Onshape oracle-loop tooling."""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from oracle import onshape  # noqa: E402
from oracle.cli import bundle_paths, results_row, verdict_line  # noqa: E402


class FakeTransport:
    """Scripted transport: pops one (status, body) per request, records calls."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def request(self, method, path, body=None, content_type=None):
        self.calls.append((method, path, body, content_type))
        return self.responses.pop(0)


def json_response(payload, status=200):
    return status, json.dumps(payload).encode("utf-8")


class MultipartTests(unittest.TestCase):
    def test_encoding_is_deterministic_with_fixed_boundary(self):
        content_type, body = onshape.encode_multipart(
            "file", "a.x_t", b"PAYLOAD", boundary="b0undary"
        )
        self.assertEqual(content_type, "multipart/form-data; boundary=b0undary")
        self.assertEqual(
            body,
            b"--b0undary\r\n"
            b'Content-Disposition: form-data; name="file"; filename="a.x_t"\r\n'
            b"Content-Type: application/octet-stream\r\n"
            b"\r\n"
            b"PAYLOAD"
            b"\r\n--b0undary--\r\n",
        )

    def test_binary_payload_passes_through_unmodified(self):
        payload = bytes(range(256))
        _, body = onshape.encode_multipart("file", "b.x_t", payload, boundary="x")
        self.assertIn(payload, body)


class FailureTaxonomyTests(unittest.TestCase):
    def test_documented_reasons_classify_to_their_levels(self):
        self.assertEqual(
            onshape.classify_failure("Invalid or corrupt input file"), onshape.PARSE_LEVEL
        )
        self.assertEqual(
            onshape.classify_failure("Imported file contains no translatable geometry"),
            onshape.SEMANTIC,
        )
        self.assertEqual(
            onshape.classify_failure("part.x_t failed to translate"),
            onshape.LATE_TRANSLATION,
        )
        self.assertEqual(onshape.classify_failure(""), "")


class EnvFileTests(unittest.TestCase):
    def test_missing_file_is_a_no_op(self):
        self.assertEqual(onshape.load_env_file("/nonexistent/.env"), [])

    def test_values_load_but_never_override_the_real_environment(self):
        preset = "ORACLE_TEST_PRESET_KEY"
        fresh = "ORACLE_TEST_FRESH_KEY"
        self.addCleanup(os.environ.pop, preset, None)
        self.addCleanup(os.environ.pop, fresh, None)
        os.environ[preset] = "environment-wins"
        with tempfile.TemporaryDirectory() as tmp:
            env_file = Path(tmp) / ".env"
            env_file.write_text(
                "# comment\n"
                "\n"
                "{}=from-file\n"
                '{}="quoted-value"\n'
                "not a key value line\n".format(preset, fresh)
            )
            applied = onshape.load_env_file(env_file)
        self.assertEqual(applied, [fresh])
        self.assertEqual(os.environ[preset], "environment-wins")
        self.assertEqual(os.environ[fresh], "quoted-value")


class TransportTests(unittest.TestCase):
    def test_missing_key_pair_is_a_configuration_error(self):
        with self.assertRaises(onshape.OracleError):
            onshape.ApiKeyTransport("", "")

    def test_decode_json_raises_with_bounded_excerpt_on_http_error(self):
        with self.assertRaises(onshape.OracleError) as raised:
            onshape.decode_json(401, b"Unauthenticated API request", "blob upload")
        self.assertIn("HTTP 401", str(raised.exception))
        self.assertIn("Unauthenticated", str(raised.exception))


class ConfigTests(unittest.TestCase):
    def test_missing_file_and_placeholder_values_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "config.json"
            with self.assertRaises(onshape.OracleError):
                onshape.load_config(missing)
            missing.write_text(
                json.dumps(
                    {
                        "document_id": "<fill me>",
                        "workspace_id": "b" * 24,
                        "element_id": "c" * 24,
                    }
                )
            )
            with self.assertRaises(onshape.OracleError):
                onshape.load_config(missing)

    def test_complete_config_loads_exactly_the_required_keys(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            path.write_text(
                json.dumps(
                    {
                        "document_id": "a" * 24,
                        "workspace_id": "b" * 24,
                        "element_id": "c" * 24,
                        "extraneous": True,
                    }
                )
            )
            self.assertEqual(
                onshape.load_config(path),
                {
                    "document_id": "a" * 24,
                    "workspace_id": "b" * 24,
                    "element_id": "c" * 24,
                },
            )


class BundleManifestTests(unittest.TestCase):
    def test_manifest_order_is_authoritative(self):
        with tempfile.TemporaryDirectory() as tmp:
            outbox = Path(tmp)
            (outbox / "manifest.tsv").write_text(
                "file\tbody_kind\tprobe\n" "b.x_t\tsolid\tb\n" "a.x_t\tsheet\ta\n"
            )
            (outbox / "a.x_t").write_bytes(b"a")
            (outbox / "b.x_t").write_bytes(b"b")
            self.assertEqual(
                [path.name for path in bundle_paths(outbox)], ["b.x_t", "a.x_t"]
            )

    def test_missing_and_stale_files_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            outbox = Path(tmp)
            (outbox / "manifest.tsv").write_text(
                "file\tbody_kind\tprobe\n" "expected.x_t\tsolid\tprobe\n"
            )
            with self.assertRaises(onshape.OracleError):
                bundle_paths(outbox)
            (outbox / "expected.x_t").write_bytes(b"expected")
            (outbox / "stale.x_t").write_bytes(b"stale")
            with self.assertRaises(onshape.OracleError):
                bundle_paths(outbox)


class TranslationLoopTests(unittest.TestCase):
    CONFIG = {
        "document_id": "a" * 24,
        "workspace_id": "b" * 24,
        "element_id": "c" * 24,
    }

    def test_upload_extracts_translation_id_and_targets_the_blob_element(self):
        transport = FakeTransport([json_response({"translationId": "t123"})])
        translation_id = onshape.upload_and_translate(
            transport, self.CONFIG, "a.x_t", b"bytes"
        )
        self.assertEqual(translation_id, "t123")
        method, path, _, content_type = transport.calls[0]
        self.assertEqual(method, "POST")
        self.assertEqual(
            path,
            "/api/v6/blobelements/d/{}/w/{}/e/{}".format("a" * 24, "b" * 24, "c" * 24),
        )
        self.assertTrue(content_type.startswith("multipart/form-data"))

    def test_upload_without_translation_id_is_a_protocol_error(self):
        transport = FakeTransport([json_response({})])
        with self.assertRaises(onshape.OracleError):
            onshape.upload_and_translate(transport, self.CONFIG, "a.x_t", b"bytes")

    def test_poll_sleeps_between_active_states_and_returns_terminal(self):
        transport = FakeTransport(
            [
                json_response({"requestState": "ACTIVE"}),
                json_response({"requestState": "ACTIVE"}),
                json_response({"requestState": "DONE", "resultElementIds": ["e1"]}),
            ]
        )
        sleeps = []
        terminal = onshape.poll_translation(
            transport, "t123", attempts=5, delay=2.0, sleep=sleeps.append
        )
        self.assertEqual(terminal["requestState"], "DONE")
        self.assertEqual(sleeps, [2.0, 2.0])

    def test_poll_raises_after_the_attempt_budget(self):
        transport = FakeTransport([json_response({"requestState": "ACTIVE"})] * 3)
        with self.assertRaises(onshape.OracleError):
            onshape.poll_translation(
                transport, "t123", attempts=3, delay=0.0, sleep=lambda _: None
            )

    def test_run_fixture_carries_reason_classification_and_elements(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture = Path(tmp) / "solid_block.x_t"
            fixture.write_bytes(b"**PARASOLID")
            accepted = FakeTransport(
                [
                    json_response({"translationId": "t1"}),
                    json_response({"requestState": "DONE", "resultElementIds": ["e9"]}),
                ]
            )
            result = onshape.run_fixture(
                accepted, self.CONFIG, fixture, sleep=lambda _: None
            )
            self.assertTrue(result.accepted)
            self.assertEqual(result.result_element_ids, ["e9"])
            self.assertEqual(result.size, len(b"**PARASOLID"))

            rejected = FakeTransport(
                [
                    json_response({"translationId": "t2"}),
                    json_response(
                        {
                            "requestState": "FAILED",
                            "failureReason": "Invalid or corrupt input file",
                        }
                    ),
                ]
            )
            result = onshape.run_fixture(
                rejected, self.CONFIG, fixture, sleep=lambda _: None
            )
            self.assertFalse(result.accepted)
            self.assertEqual(result.classification, onshape.PARSE_LEVEL)


class PartStudioDiscoveryTests(unittest.TestCase):
    CONFIG = TranslationLoopTests.CONFIG

    def test_single_part_studio_is_found_among_other_elements(self):
        transport = FakeTransport(
            [
                json_response(
                    [
                        {"elementType": "BLOB", "id": "b1", "name": "x.x_t"},
                        {"elementType": "PARTSTUDIO", "id": "p1", "name": "disk_nat"},
                    ]
                )
            ]
        )
        self.assertEqual(
            onshape.find_translated_part_studio(transport, self.CONFIG), "p1"
        )

    def test_zero_or_multiple_part_studios_is_an_error(self):
        for elements in ([], [{"elementType": "PARTSTUDIO", "id": "p1"}] * 2):
            transport = FakeTransport([json_response(elements)])
            with self.assertRaises(onshape.OracleError):
                onshape.find_translated_part_studio(transport, self.CONFIG)


class ReexportTests(unittest.TestCase):
    CONFIG = TranslationLoopTests.CONFIG

    def test_reexport_uses_the_synchronous_endpoint_with_surfaces_included(self):
        transport = FakeTransport([(200, b"**PARASOLID reexport")])
        payload = onshape.reexport_element(transport, self.CONFIG, "elem1")
        self.assertEqual(payload, b"**PARASOLID reexport")
        method, path, _, _ = transport.calls[0]
        self.assertEqual(method, "GET")
        self.assertIn("/partstudios/d/{}/w/{}/e/elem1/parasolid".format("a" * 24, "b" * 24), path)
        self.assertIn("includeSurfaces=true", path)

    def test_transient_no_visible_parts_is_retried_until_regeneration_lands(self):
        transport = FakeTransport(
            [
                (400, b'{"message": "No visible parts to export"}'),
                (400, b'{"message": "No visible parts to export"}'),
                (200, b"**PARASOLID reexport"),
            ]
        )
        sleeps = []
        payload = onshape.reexport_element(
            transport, self.CONFIG, "elem1", attempts=5, delay=5.0, sleep=sleeps.append
        )
        self.assertEqual(payload, b"**PARASOLID reexport")
        self.assertEqual(sleeps, [5.0, 5.0])

    def test_persistent_no_visible_parts_exhausts_and_raises(self):
        transport = FakeTransport([(400, b'{"message": "No visible parts to export"}')] * 3)
        with self.assertRaises(onshape.OracleError) as raised:
            onshape.reexport_element(
                transport, self.CONFIG, "elem1", attempts=3, delay=0.0, sleep=lambda _: None
            )
        self.assertIn("No visible parts", str(raised.exception))

    def test_other_export_errors_raise_immediately(self):
        transport = FakeTransport([(403, b"forbidden")])
        with self.assertRaises(onshape.OracleError) as raised:
            onshape.reexport_element(transport, self.CONFIG, "elem1")
        self.assertIn("HTTP 403", str(raised.exception))
        self.assertEqual(len(transport.calls), 1)


class ReportingTests(unittest.TestCase):
    def test_results_row_matches_the_committed_tsv_shape(self):
        accepted = onshape.FixtureResult(
            name="solid_block.x_t",
            size=6300,
            state="DONE",
            reason="",
            classification="",
        )
        row = results_row(accepted, "2026-07-11", "abc1234", "yes", "yes")
        self.assertEqual(
            row.split("\t"),
            [
                "2026-07-11",
                "onshape",
                "cloud-2026-07-11",
                "solid_block.x_t",
                "yes",
                "none",
                "-",
                "yes",
                "yes",
                "writer=abc1234; accepted",
            ],
        )

    def test_rejection_row_and_line_carry_reason_and_classification(self):
        rejected = onshape.FixtureResult(
            name="acorn_point.x_t",
            size=1036,
            state="FAILED",
            reason="Invalid or corrupt input file",
            classification=onshape.PARSE_LEVEL,
        )
        row = results_row(rejected, "2026-07-11", "abc1234")
        fields = row.split("\t")
        self.assertEqual(fields[4], "no")
        self.assertIn("Invalid or corrupt input file", fields[9])
        self.assertIn("[parse-level]", fields[9])
        self.assertIn("parse-level", verdict_line(rejected))

    def test_bundle_rows_carry_the_exact_host_payload_identity(self):
        accepted = onshape.FixtureResult(
            name="solid_block.x_t",
            size=6300,
            state="DONE",
            reason="",
            classification="",
        )
        row = results_row(
            accepted,
            "2026-07-11",
            "abc1234",
            "yes",
            "yes",
            bundle_identity="0123456789abcdef",
        )
        self.assertIn("; bundle=0123456789abcdef", row.split("\t")[9])


if __name__ == "__main__":
    unittest.main()
