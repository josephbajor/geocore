"""Offline regression tests for benchmark metadata and format drift."""

import copy
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import benchmark  # noqa: E402


class BenchmarkBaselineTests(unittest.TestCase):
    """Contract validation remains independent of benchmark execution."""

    def setUp(self):
        self.example = benchmark.load_json(
            ROOT / "benches" / "baselines" / "example.synthetic.v1.json"
        )
        self.measurement = (
            ROOT / "benches" / "testdata" / "cargo-criterion.synthetic.ndjson"
        ).read_text(encoding="utf-8")

    def test_committed_contract_is_valid_offline(self):
        benchmark.validate_schema_document()
        cases = benchmark.load_cases()
        self.assertEqual(len(cases), 67)
        self.assertEqual(cases[0]["deterministic_seed"], 0x4B45524E454C0001)
        self.assertEqual(
            cases[0]["expected_result_counters"]["output_digest"],
            "142890537c90ed65",
        )
        topology = [
            case for case in cases if case["benchmark_target"] == "topology_commit"
        ]
        self.assertEqual(len(topology), 21)
        self.assertTrue(
            all(case["deterministic_seed"] == 0x51544F504F000002 for case in topology)
        )
        self.assertTrue(
            all(
                case["size_parameters"]["elements"]
                == case["size_parameters"]["bodies"]
                for case in topology
            )
        )
        self.assertTrue(
            all(
                "wrapping_sum_hex" not in case["expected_result_counters"]
                for case in topology
            )
        )
        self.assertTrue(
            all(
                "output_digest" in case["expected_result_counters"]
                for case in topology
            )
        )
        graph_build = [
            case for case in cases if case["benchmark_target"] == "graph_build"
        ]
        self.assertEqual(len(graph_build), 17)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154324147520006
                for case in graph_build
            )
        )
        self.assertEqual(
            {case["policy_values"]["shape"] for case in graph_build},
            {"independent", "chain", "fanout", "rollback-chain"},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["stable_order"]
                and "graph_digest" in case["expected_result_counters"]
                and "reverse_index_digest" in case["expected_result_counters"]
                for case in graph_build
            )
        )
        tessellation = [
            case for case in cases if case["benchmark_target"] == "body_tessellation"
        ]
        self.assertEqual(len(tessellation), 12)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154455353000003
                for case in tessellation
            )
        )
        self.assertEqual(
            {case["tolerances"]["chord_tol"] for case in tessellation},
            {1e-2, 1e-3},
        )
        self.assertTrue(
            all(
                case["fixture_version"] == "body-tessellation.v2"
                and case["policy_values"]["api"]
                == "tessellate_body_with_context"
                and case["policy_values"]["budget_profile"]
                == "body-tessellation.compatibility-v1"
                and case["policy_values"]["execution"] == "serial"
                and case["policy_values"]["policy_version"] == "v1"
                and case["policy_values"]["usage_contract"] == "q3-usage.v1"
                for case in tessellation
            )
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["usage_stage_count"] == 21
                and len(case["expected_result_counters"]["usage_consumed"]) == 21
                and len(case["expected_result_counters"]["usage_stage_digest"])
                == 16
                and case["expected_result_counters"]["limit_event_count"] == 0
                and case["expected_result_counters"][
                    "numeric_resolution_stage_count"
                ]
                == 0
                and case["expected_result_counters"]["diagnostic_count"] == 0
                and case["expected_result_counters"]["dropped_diagnostic_count"]
                == 0
                for case in tessellation
            )
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["watertight"]
                and case["expected_result_counters"]["outward"]
                and case["expected_result_counters"]["volume_within_tolerance"]
                for case in tessellation
            )
        )
        imported_nurbs = [
            case
            for case in tessellation
            if case["policy_values"].get("source_fixture")
            == "solid_block_nurbs_face.x_t@onshape-cloud-2026-07-11"
        ]
        self.assertEqual(len(imported_nurbs), 2)
        certified = benchmark.load_json(ROOT / "docs" / "oracle-certification.json")
        expected_sha256 = certified["fixtures_sha256"]["solid_block_nurbs_face.x_t"]
        fixture_bytes = (
            ROOT / "benches" / "testdata" / "solid_block_nurbs_face.certified.x_t"
        ).read_bytes()
        self.assertEqual(len(fixture_bytes), 6_488)
        self.assertEqual(hashlib.sha256(fixture_bytes).hexdigest(), expected_sha256)
        self.assertTrue(
            all(
                case["size_parameters"]["input_bytes"] == len(fixture_bytes)
                and case["policy_values"]["source_sha256"] == expected_sha256
                for case in imported_nurbs
            )
        )
        face_tessellation = [
            case for case in cases if case["benchmark_target"] == "face_tessellation"
        ]
        self.assertEqual(len(face_tessellation), 2)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154464143450007
                and case["fixture_version"] == "face-tessellation.v1"
                and case["policy_values"]["api"] == "tessellate_with_context"
                and case["policy_values"]["budget_profile"]
                == "face-tessellation.compatibility-v1"
                and case["policy_values"]["execution"] == "serial"
                and case["policy_values"]["policy_version"] == "v1"
                and case["policy_values"]["usage_contract"]
                == "q3-face-usage.v1"
                and case["expected_result_counters"]["usage_stage_count"] == 5
                and len(case["expected_result_counters"]["usage_consumed"]) == 5
                and all(
                    consumed > 0
                    for consumed in case["expected_result_counters"]["usage_consumed"]
                )
                for case in face_tessellation
            )
        )
        isolation = [
            case for case in cases if case["benchmark_target"] == "nurbs_isolation"
        ]
        self.assertEqual(len(isolation), 6)
        self.assertTrue(
            all(case["deterministic_seed"] == 0x51544E5552420004 for case in isolation)
        )
        limited = [
            case
            for case in isolation
            if case["expected_result_counters"]["limit_kind"] != "none"
        ]
        self.assertEqual(len(limited), 2)
        self.assertTrue(
            all(
                case["expected_result_counters"]["indeterminate"]
                and case["expected_result_counters"]["conservative_cover"]
                and not case["expected_result_counters"]["complete"]
                and not case["expected_result_counters"]["proven_empty"]
                for case in limited
            )
        )
        xt_io = [case for case in cases if case["benchmark_target"] == "xt_io"]
        self.assertEqual(len(xt_io), 8)
        self.assertTrue(
            all(case["deterministic_seed"] == 0x51545854494F0005 for case in xt_io)
        )
        self.assertEqual(
            {case["policy_values"]["phase"] for case in xt_io},
            {"parse-records", "full-read", "write-text", "round-trip"},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["unsupported_capabilities"] == 0
                for case in xt_io
            )
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["roundtrip_equal"]
                for case in xt_io
                if case["policy_values"]["phase"] == "round-trip"
            )
        )
        benchmark.validate_report(self.example)
        parsed = benchmark.parse_cargo_criterion(
            self.measurement, cases[0]["path"], 64
        )
        self.assertTrue(parsed["advisory_only"])
        self.assertEqual(parsed["sample_count"], 3)

    def test_missing_identity_field_fails_closed(self):
        report = copy.deepcopy(self.example)
        del report["host"]["cpu_model"]
        with self.assertRaises(benchmark.ContractError):
            benchmark.validate_report(report)

    def test_q1_case_keeps_its_target_specific_wrapping_sum_contract(self):
        case = copy.deepcopy(benchmark.load_cases()[0])
        self.assertEqual(case["benchmark_target"], "benchmark_contract")
        del case["expected_result_counters"]["wrapping_sum_hex"]
        manifest = {
            "schema_version": "kernel-benchmark-cases.v1",
            "cases": [case],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cases.json"
            path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaises(benchmark.ContractError):
                benchmark.load_cases(path)

    def test_cargo_criterion_format_drift_fails_closed(self):
        messages = [json.loads(line) for line in self.measurement.splitlines()]
        del messages[0]["typical"]["lower_bound"]
        drifted = "\n".join(json.dumps(message) for message in messages)
        with self.assertRaises(benchmark.ContractError):
            benchmark.parse_cargo_criterion(
                drifted, "harness/contract/tiny-v1/64/default-v1", 64
            )

    def test_synthetic_record_is_never_comparison_eligible(self):
        report = benchmark.record_from_text(
            self.measurement,
            "harness/contract/tiny-v1/64/default-v1",
            synthetic=True,
            smoke=True,
        )
        self.assertFalse(report["run"]["comparison_eligible"])
        comparison = benchmark.compare_identity(report, report)
        self.assertFalse(comparison["compatible"])
        self.assertEqual(comparison["mismatches"], ["run.comparison_eligible"])
        self.assertNotIn("ratio", comparison)

    def test_runner_output_round_trips_through_validation(self):
        report = benchmark.record_from_text(
            self.measurement,
            "harness/contract/tiny-v1/64/default-v1",
            synthetic=True,
            smoke=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            benchmark.write_json(path, report)
            benchmark.validate_report(benchmark.load_json(path))


if __name__ == "__main__":
    unittest.main()
