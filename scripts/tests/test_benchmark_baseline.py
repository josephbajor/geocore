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
        self.assertEqual(len(cases), 101)
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
        graph_traversal = [
            case for case in cases if case["benchmark_target"] == "graph_traversal"
        ]
        self.assertEqual(len(graph_traversal), 8)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154325452410008
                and case["fixture_version"] == "graph-traversal.v1"
                and case["policy_values"]["membership"] == "indexed"
                and case["expected_result_counters"]["stable"]
                and len(case["expected_result_counters"]["result_digest"]) == 16
                and len(case["expected_result_counters"]["output_digest"]) == 16
                for case in graph_traversal
            )
        )
        self.assertEqual(
            {case["policy_values"]["operation"] for case in graph_traversal},
            {"dependency-closure", "dependency-path-miss"},
        )
        self.assertEqual(
            {case["policy_values"]["shape"] for case in graph_build},
            {"independent", "chain", "fanout", "rollback-chain"},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["stable_order"]
                and case["expected_result_counters"]["full_order_rebuilds"] == 0
                and "graph_digest" in case["expected_result_counters"]
                and "reverse_index_digest" in case["expected_result_counters"]
                for case in graph_build
            )
        )
        tessellation = [
            case for case in cases if case["benchmark_target"] == "body_tessellation"
        ]
        self.assertEqual(len(tessellation), 20)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154455353000003
                for case in tessellation
            )
        )
        self.assertEqual(
            {case["tolerances"]["chord_tol"] for case in tessellation},
            {1e-2, 3e-3, 1e-3, 3e-4},
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
        mixed_store = [
            case
            for case in tessellation
            if case["policy_values"].get("store_shape")
            == "block-cylinder-sphere; target=cylinder"
        ]
        self.assertEqual(len(mixed_store), 2)
        self.assertTrue(
            all(
                case["size_parameters"]["bodies"] == 3
                and case["expected_result_counters"]["source_faces"] == 3
                and case["expected_result_counters"]["source_edges"] == 2
                for case in mixed_store
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
        imported_cylinder = [
            case
            for case in tessellation
            if case["policy_values"].get("source_fixture")
            == "solid_cylinder.x_t@onshape-cloud-2026-07-11"
        ]
        self.assertEqual(len(imported_cylinder), 4)
        cylinder_sha256 = certified["fixtures_sha256"]["solid_cylinder.x_t"]
        cylinder_bytes = (
            ROOT / "oracle" / "outbox" / "solid_cylinder.x_t"
        ).read_bytes()
        self.assertEqual(len(cylinder_bytes), 2_309)
        self.assertEqual(hashlib.sha256(cylinder_bytes).hexdigest(), cylinder_sha256)
        self.assertTrue(
            all(
                case["size_parameters"]["input_bytes"] == len(cylinder_bytes)
                and case["policy_values"]["source_sha256"] == cylinder_sha256
                and case["expected_result_counters"]["source_faces"] == 3
                and case["expected_result_counters"]["source_edges"] == 2
                and case["policy_values"]["volume_ratio_floor"]
                == {
                    1e-2: 0.94,
                    3e-3: 0.98,
                    1e-3: 0.99,
                    3e-4: 0.998,
                }[case["tolerances"]["chord_tol"]]
                for case in imported_cylinder
            )
        )
        imported_tolerant = [
            case
            for case in tessellation
            if case["policy_values"].get("source_fixture")
            == "solid_block_tolerant_edge.x_t@onshape-cloud-2026-07-11"
        ]
        self.assertEqual(len(imported_tolerant), 2)
        tolerant_sha256 = certified["fixtures_sha256"]["solid_block_tolerant_edge.x_t"]
        tolerant_bytes = (
            ROOT / "oracle" / "outbox" / "solid_block_tolerant_edge.x_t"
        ).read_bytes()
        self.assertEqual(len(tolerant_bytes), 7_172)
        self.assertEqual(hashlib.sha256(tolerant_bytes).hexdigest(), tolerant_sha256)
        self.assertTrue(
            all(
                case["size_parameters"]["input_bytes"] == len(tolerant_bytes)
                and case["policy_values"]["source_sha256"] == tolerant_sha256
                and case["expected_result_counters"]["tolerant_edges"] == 1
                and case["expected_result_counters"]["pcurve_uses"] == 2
                and case["expected_result_counters"]["skipped_geometric_owners"]
                == 4
                for case in imported_tolerant
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
        curve_pair_isolation = [
            case
            for case in cases
            if case["benchmark_target"] == "curve_pair_isolation"
        ]
        self.assertEqual(len(curve_pair_isolation), 8)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154435041490009
                and case["fixture_version"] == "curve-pair-isolation.v2"
                for case in curve_pair_isolation
            )
        )
        curve_pair_limited = [
            case
            for case in curve_pair_isolation
            if case["expected_result_counters"]["limit_kind"] != "none"
        ]
        self.assertEqual(
            {
                case["expected_result_counters"]["limit_kind"]
                for case in curve_pair_limited
            },
            {"work", "candidates", "depth"},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["indeterminate"]
                and case["expected_result_counters"]["conservative_cover"]
                and not case["expected_result_counters"]["complete"]
                and not case["expected_result_counters"]["proven_empty"]
                for case in curve_pair_limited
            )
        )
        curve_pair_misses = [
            case
            for case in curve_pair_isolation
            if case["expected_result_counters"]["proven_empty"]
        ]
        self.assertEqual(len(curve_pair_misses), 2)
        self.assertTrue(
            all(case["expected_result_counters"]["complete"] for case in curve_pair_misses)
        )
        curve_pair_solve = [
            case for case in cases if case["benchmark_target"] == "curve_pair_solve"
        ]
        self.assertEqual(len(curve_pair_solve), 10)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x51544350534F000A
                and case["fixture_version"] == "curve-pair-solve.v3"
                and case["policy_values"]["policy_version"] == "v1"
                and case["policy_values"]["execution"] == "serial"
                and case["policy_values"]["api"]
                == "intersect_bounded_nurbs_nurbs_with_context"
                and case["policy_values"]["overlap_work_allowed"] >= 0
                and case["policy_values"]["overlap_items_allowed"] >= 0
                and case["expected_result_counters"]["overlap_equivalence_work"]
                >= 0
                and case["expected_result_counters"]["overlap_equivalence_items"]
                >= 0
                and len(case["expected_result_counters"]["overlap_digest"]) == 16
                and case["expected_result_counters"]["verified_witnesses"]
                for case in curve_pair_solve
            )
        )
        solve_limited = [
            case
            for case in curve_pair_solve
            if case["expected_result_counters"]["limit_kind"] != "none"
        ]
        self.assertEqual(len(solve_limited), 2)
        self.assertEqual(
            {
                case["expected_result_counters"]["limit_kind"]
                for case in solve_limited
            },
            {"seed-attempts", "overlap-work"},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["indeterminate"]
                and not case["expected_result_counters"]["complete"]
                for case in solve_limited
            )
        )
        common_refinement = [
            case
            for case in curve_pair_solve
            if "common-refinement-overlap-v1" in case["path"]
        ]
        self.assertEqual(len(common_refinement), 2)
        common_complete = next(
            case
            for case in common_refinement
            if case["expected_result_counters"]["limit_kind"] == "none"
        )
        self.assertTrue(common_complete["expected_result_counters"]["complete"])
        self.assertEqual(common_complete["expected_result_counters"]["overlaps"], 1)
        self.assertGreater(
            common_complete["expected_result_counters"]["overlap_equivalence_work"],
            0,
        )
        self.assertGreater(
            common_complete["expected_result_counters"]["overlap_equivalence_items"],
            0,
        )
        common_denied = next(
            case
            for case in common_refinement
            if case["expected_result_counters"]["limit_kind"] == "overlap-work"
        )
        self.assertEqual(
            common_denied["expected_result_counters"]["limit_attempted_consumed"],
            common_denied["expected_result_counters"]["limit_attempted_allowed"] + 1,
        )
        self.assertEqual(common_denied["expected_result_counters"]["overlaps"], 0)
        solve_miss = [
            case
            for case in curve_pair_solve
            if case["expected_result_counters"]["proven_empty"]
        ]
        self.assertEqual(len(solve_miss), 1)
        self.assertTrue(solve_miss[0]["expected_result_counters"]["complete"])
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
