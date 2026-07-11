"""Offline regression tests for benchmark metadata and format drift."""

import copy
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
        self.assertEqual(len(cases), 22)
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
