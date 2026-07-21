"""Offline regression tests for the Onshape oracle-loop tooling."""

import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from oracle import cli, onshape  # noqa: E402
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

    def test_request_limit_fails_before_an_over_budget_host_call(self):
        transport = onshape.ApiKeyTransport("access", "secret", request_limit=1)
        response = mock.MagicMock()
        response.__enter__.return_value.status = 200
        response.__enter__.return_value.read.return_value = b"{}"
        with mock.patch.object(onshape.urllib.request, "urlopen", return_value=response) as call:
            self.assertEqual(
                transport.request("GET", "/api/v6/users/sessioninfo"), (200, b"{}")
            )
            with self.assertRaisesRegex(onshape.OracleError, "request limit exhausted"):
                transport.request("GET", "/api/v6/users/sessioninfo")
            call.assert_called_once()
        self.assertEqual(transport.request_count, 1)

    def test_request_limit_environment_must_be_positive(self):
        self.addCleanup(os.environ.pop, onshape.ACCESS_KEY_ENV, None)
        self.addCleanup(os.environ.pop, onshape.SECRET_KEY_ENV, None)
        self.addCleanup(os.environ.pop, onshape.REQUEST_LIMIT_ENV, None)
        os.environ[onshape.ACCESS_KEY_ENV] = "access"
        os.environ[onshape.SECRET_KEY_ENV] = "secret"
        for invalid in ("0", "401", "not-a-number"):
            os.environ[onshape.REQUEST_LIMIT_ENV] = invalid
            with self.assertRaises(onshape.OracleError):
                onshape.ApiKeyTransport.from_environment()


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

    def test_selected_fixtures_are_manifest_bound_and_manifest_ordered(self):
        with tempfile.TemporaryDirectory() as tmp:
            outbox = Path(tmp)
            (outbox / "manifest.tsv").write_text(
                "file\tbody_kind\tprobe\n"
                "a.x_t\tsolid\ta\n"
                "b.x_t\tsolid\tb\n"
                "c.x_t\tsolid\tc\n"
            )
            for name in ("a.x_t", "b.x_t", "c.x_t"):
                (outbox / name).write_bytes(name.encode())
            self.assertEqual(
                [path.name for path in bundle_paths(outbox, ["c.x_t", "a.x_t"])],
                ["a.x_t", "c.x_t"],
            )
            for invalid in (["a.x_t", "a.x_t"], ["unknown.x_t"]):
                with self.assertRaises(onshape.OracleError):
                    bundle_paths(outbox, invalid)

    def test_identity_command_prints_the_exact_offline_bundle_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            outbox = Path(tmp)
            (outbox / "manifest.tsv").write_text(
                "file\tbody_kind\tprobe\nfixture.x_t\tsolid\tprobe\n"
            )
            (outbox / "fixture.x_t").write_bytes(b"fixture")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(
                    cli.command_identity(SimpleNamespace(outbox=str(outbox))), 0
                )
            identity = json.loads(output.getvalue())
            self.assertEqual(identity["fixture_count"], 1)
            self.assertEqual(set(identity["fixtures_sha256"]), {"fixture.x_t"})


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


class LiveLoopContractTests(unittest.TestCase):
    def args(self, tmp, reexport=True, compare=True):
        return SimpleNamespace(
            reexport=reexport,
            compare=compare,
            inbox=str(Path(tmp) / "inbox"),
            results_rows=False,
            results_file=None,
            request_count_file=None,
            completion_file=None,
        )

    def accepted(self):
        return onshape.FixtureResult(
            name="solid_block.x_t",
            size=12,
            state="DONE",
            reason="",
            classification="",
            result_element_ids=["element"],
        )

    def transport(self):
        return SimpleNamespace(request_count=4, request_limit=20)

    def test_compare_without_reexport_fails_before_credentials_are_loaded(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment"
        ) as transport:
            with self.assertRaisesRegex(onshape.OracleError, "requires --reexport"):
                cli.run_paths(
                    self.args(tmp, reexport=False, compare=True),
                    [Path(tmp) / "solid_block.x_t"],
                )
            transport.assert_not_called()

    def test_reexport_error_is_an_operational_failure(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli, "run_fixture", return_value=self.accepted()
        ), mock.patch.object(
            cli, "reexport_element", side_effect=onshape.OracleError("host unavailable")
        ):
            self.assertEqual(
                cli.run_paths(self.args(tmp), [Path(tmp) / "solid_block.x_t"]), 2
            )

    def test_rate_limit_stops_the_session_immediately(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli, "run_fixture", return_value=self.accepted()
        ) as fixture, mock.patch.object(
            cli, "reexport_element", side_effect=onshape.OracleError("HTTP 429 quota")
        ):
            args = self.args(tmp)
            args.results_file = str(Path(tmp) / "partial.tsv")
            with self.assertRaisesRegex(onshape.OracleError, "HTTP 429"):
                cli.run_paths(
                    args,
                    [Path(tmp) / "solid_block.x_t", Path(tmp) / "second.x_t"],
                )
            self.assertEqual(fixture.call_count, 1)
            row = Path(args.results_file).read_text(encoding="utf-8").split("\t")
            self.assertEqual(row[3], "solid_block.x_t")
            self.assertEqual(row[4], "yes")
            self.assertEqual(row[7], "error")

    def test_compare_tool_error_is_an_operational_failure(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli, "run_fixture", return_value=self.accepted()
        ), mock.patch.object(
            cli, "reexport_element", return_value=b"**PARASOLID"
        ), mock.patch.object(
            cli, "compare_files", return_value="error"
        ):
            self.assertEqual(
                cli.run_paths(self.args(tmp), [Path(tmp) / "solid_block.x_t"]), 2
            )

    def test_partial_results_survive_a_later_transport_error(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli,
            "run_fixture",
            side_effect=[self.accepted(), onshape.OracleError("request limit exhausted")],
        ):
            args = self.args(tmp, reexport=False, compare=False)
            args.results_file = str(Path(tmp) / "results.tsv")
            with self.assertRaisesRegex(onshape.OracleError, "request limit exhausted"):
                cli.run_paths(
                    args,
                    [Path(tmp) / "solid_block.x_t", Path(tmp) / "second.x_t"],
                )
            rows = Path(args.results_file).read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0].split("\t")[3], "solid_block.x_t")

    def test_request_count_survives_a_partial_run(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli, "run_fixture", side_effect=onshape.OracleError("host unavailable")
        ):
            args = self.args(tmp, reexport=False, compare=False)
            args.request_count_file = str(Path(tmp) / "requests.txt")
            with self.assertRaises(onshape.OracleError):
                cli.run_paths(args, [Path(tmp) / "solid_block.x_t"])
            self.assertEqual(Path(args.request_count_file).read_text(), "4\n")

    def test_completion_marker_distinguishes_findings_from_an_aborted_run(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli,
            "run_fixture",
            side_effect=[self.accepted(), onshape.OracleError("host unavailable")],
        ):
            args = self.args(tmp, reexport=False, compare=False)
            args.completion_file = str(Path(tmp) / "complete.txt")
            with self.assertRaises(onshape.OracleError):
                cli.run_paths(
                    args,
                    [Path(tmp) / "solid_block.x_t", Path(tmp) / "second.x_t"],
                )
            self.assertEqual(Path(args.completion_file).read_text(), "")

    def test_unexpected_exception_cannot_masquerade_as_completed_findings(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli, "run_fixture", side_effect=RuntimeError("unexpected defect")
        ):
            args = self.args(tmp, reexport=False, compare=False)
            args.completion_file = str(Path(tmp) / "complete.txt")
            Path(args.completion_file).write_text("1\n")
            with self.assertRaisesRegex(RuntimeError, "unexpected defect"):
                cli.run_paths(args, [Path(tmp) / "solid_block.x_t"])
            self.assertEqual(Path(args.completion_file).read_text(), "")

    def test_completion_marker_matches_a_finished_findings_exit(self):
        rejected = onshape.FixtureResult(
            name="wire.x_t",
            size=12,
            state="FAILED",
            reason="Invalid or corrupt input file",
            classification=onshape.PARSE_LEVEL,
        )
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            cli.ApiKeyTransport, "from_environment", return_value=self.transport()
        ), mock.patch.object(cli, "load_config", return_value={}), mock.patch.object(
            cli, "run_fixture", return_value=rejected
        ):
            args = self.args(tmp, reexport=False, compare=False)
            args.completion_file = str(Path(tmp) / "complete.txt")
            self.assertEqual(cli.run_paths(args, [Path(tmp) / "wire.x_t"]), 1)
            self.assertEqual(Path(args.completion_file).read_text(), "1\n")


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
