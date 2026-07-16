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
        self.assertEqual(len(cases), 167)
        self.assertEqual(cases[0]["deterministic_seed"], 0x4B45524E454C0001)
        self.assertEqual(
            cases[0]["expected_result_counters"]["output_digest"],
            "142890537c90ed65",
        )
        topology = [
            case for case in cases if case["benchmark_target"] == "topology_commit"
        ]
        self.assertEqual(len(topology), 32)
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
        cohort = [
            case
            for case in topology
            if case["policy_values"]["ladder"] == "cohort"
        ]
        self.assertEqual(len(cohort), 7)
        self.assertEqual(
            {
                case["size_parameters"]["bodies"]
                for case in cohort
                if case["size_parameters"]["affected_bodies"] == 4
            },
            {4, 16, 64, 256},
        )
        self.assertEqual(
            {
                case["size_parameters"]["affected_bodies"]
                for case in cohort
                if case["size_parameters"]["bodies"] == 64
            },
            {1, 4, 16, 64},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["affected_bodies"]
                == case["expected_result_counters"]["refreshed_bodies"]
                == case["expected_result_counters"]["checked_bodies"]
                == case["size_parameters"]["affected_bodies"]
                for case in cohort
            )
        )
        affected_solids = [
            case
            for case in topology
            if case["policy_values"]["ladder"] == "affected-solid-footprint"
        ]
        self.assertEqual(len(affected_solids), 4)
        self.assertEqual(
            {case["size_parameters"]["affected_bodies"] for case in affected_solids},
            {1, 4, 16, 64},
        )
        self.assertTrue(
            all(
                case["size_parameters"]["bodies"] == 64
                and case["expected_result_counters"]["affected_bodies"]
                == case["expected_result_counters"]["refreshed_bodies"]
                == case["expected_result_counters"]["checked_bodies"]
                == case["expected_result_counters"]["mutations"]
                == case["size_parameters"]["affected_bodies"]
                and case["policy_values"]["checked_commit"] == "ordinary"
                and case["policy_values"]["mutation"]
                == "operation-owned-face-tolerance-growth"
                for case in affected_solids
            )
        )
        graph_build = [
            case for case in cases if case["benchmark_target"] == "graph_build"
        ]
        self.assertEqual(len(graph_build), 21)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154324147520007
                and case["fixture_version"] == "graph-build.v2"
                for case in graph_build
            )
        )
        diamond = [
            case
            for case in graph_build
            if case["policy_values"]["shape"] == "diamond"
        ]
        self.assertEqual(len(diamond), 4)
        self.assertEqual(
            {case["size_parameters"]["dependents"] for case in diamond},
            {1, 10, 100, 1000},
        )
        self.assertTrue(
            all(
                case["size_parameters"]["shared_sources"] == 4
                and case["expected_result_counters"]["diamond_closure_nodes"] == 6
                and case["expected_result_counters"][
                    "diamond_closure_deduplicated"
                ]
                for case in diamond
            )
        )
        graph_traversal = [
            case for case in cases if case["benchmark_target"] == "graph_traversal"
        ]
        self.assertEqual(len(graph_traversal), 10)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154325452410009
                and case["fixture_version"] == "graph-traversal.v2"
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
        diamond_traversal = [
            case
            for case in graph_traversal
            if case["policy_values"]["shape"] == "verified-intersection-diamond"
        ]
        self.assertEqual(len(diamond_traversal), 2)
        self.assertTrue(
            all(
                case["size_parameters"]["shared_bases"] == 1
                and case["expected_result_counters"]["nodes"] == 7
                and case["expected_result_counters"]["dependency_edges"] == 6
                for case in diamond_traversal
            )
        )
        diamond_closure = next(
            case
            for case in diamond_traversal
            if case["policy_values"]["operation"] == "dependency-closure"
        )
        self.assertEqual(
            diamond_closure["expected_result_counters"]["result_nodes"], 6
        )
        self.assertTrue(diamond_closure["expected_result_counters"]["reached"])
        self.assertEqual(
            {case["policy_values"]["shape"] for case in graph_build},
            {"independent", "chain", "fanout", "diamond", "rollback-chain"},
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
        self.assertEqual(len(tessellation), 32)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154455353000003
                for case in tessellation
            )
        )
        self.assertEqual(
            {case["tolerances"]["chord_tol"] for case in tessellation},
            {1e-2, 3e-3, 1e-3, 5e-4, 3e-4},
        )
        self.assertTrue(
            all(
                case["fixture_version"] == "body-tessellation.v3"
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
                case["expected_result_counters"]["manifold"]
                and case["expected_result_counters"]["boundary_matches_topology"]
                and case["expected_result_counters"]["orientation_valid"]
                and case["expected_result_counters"]["measure_within_tolerance"]
                and case["policy_values"]["incidence_proof"]
                == "directed-manifold+exact-topological-boundary"
                for case in tessellation
            )
        )
        solids = [
            case for case in tessellation if case["policy_values"]["body_kind"] == "solid"
        ]
        sheets = [
            case for case in tessellation if case["policy_values"]["body_kind"] == "sheet"
        ]
        self.assertEqual(len(solids), 24)
        self.assertEqual(len(sheets), 8)
        self.assertTrue(
            all(
                case["policy_values"]["validation"] == "closed-solid"
                and case["policy_values"]["measure"] == "signed-volume"
                and case["expected_result_counters"]["boundary_segments"] == 0
                for case in solids
            )
        )
        self.assertTrue(
            all(
                case["policy_values"]["validation"] == "oriented-sheet"
                and case["policy_values"]["measure"] == "faceted-surface-area"
                and case["expected_result_counters"]["boundary_segments"] > 0
                and case["policy_values"]["orientation_dust_threshold"]
                == "64*epsilon*exact-measure"
                for case in sheets
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
        historical_evidence = (
            "historical-host-accepted:onshape-cloud-2026-07-11"
        )
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
                and case["policy_values"]["source_evidence"]
                == historical_evidence
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
                and case["policy_values"]["source_evidence"]
                == historical_evidence
                and case["expected_result_counters"]["source_faces"] == 3
                and case["expected_result_counters"]["source_edges"] == 2
                and case["policy_values"]["measure_ratio_floor"]
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
                and case["policy_values"]["source_evidence"]
                == historical_evidence
                and case["expected_result_counters"]["tolerant_edges"] == 1
                and case["expected_result_counters"]["pcurve_uses"] == 2
                and case["expected_result_counters"]["skipped_geometric_owners"]
                == 4
                for case in imported_tolerant
            )
        )
        curved_nurbs = [
            case
            for case in tessellation
            if case["policy_values"].get("source_fixture")
            == "solid_block_curved_nurbs_face.x_t@local-import-verified-2026-07-13"
        ]
        self.assertEqual(len(curved_nurbs), 4)
        curved_bytes = (
            ROOT
            / "benches"
            / "testdata"
            / "solid_block_curved_nurbs_face.local.x_t"
        ).read_bytes()
        curved_sha256 = (
            "7fad6999a2d2bd0653a3b7558e0460e9ccfe07a43d00f249709ea7aae642829e"
        )
        self.assertEqual(len(curved_bytes), 6_785)
        self.assertEqual(hashlib.sha256(curved_bytes).hexdigest(), curved_sha256)
        self.assertEqual(
            {case["tolerances"]["chord_tol"] for case in curved_nurbs},
            {1e-2, 3e-3, 1e-3, 5e-4},
        )
        self.assertTrue(
            all(
                case["size_parameters"]["input_bytes"] == len(curved_bytes)
                and case["policy_values"]["source_sha256"] == curved_sha256
                and case["policy_values"]["source_evidence"]
                == "local-import-verified;host-certification=pending"
                and case["expected_result_counters"]["source_faces"] == 6
                and case["expected_result_counters"]["source_edges"] == 12
                and case["expected_result_counters"]["source_vertices"] == 8
                for case in curved_nurbs
            )
        )
        curved_finest = next(
            case
            for case in curved_nurbs
            if case["tolerances"]["chord_tol"] == 5e-4
        )
        self.assertEqual(
            curved_finest["policy_values"]["rejected_finer_tier"],
            "chord-3e-4;interior-refinement-passes=25;allowed=24",
        )
        plane_sheets = [
            case
            for case in tessellation
            if case["policy_values"].get("source_fixture")
            == "sheet_plane_polygon.x_t@onshape-cloud-2026-07-11"
        ]
        self.assertEqual(len(plane_sheets), 4)
        plane_sheet_bytes = (
            ROOT / "benches" / "testdata" / "sheet_plane_polygon.certified.x_t"
        ).read_bytes()
        plane_sheet_sha256 = certified["fixtures_sha256"]["sheet_plane_polygon.x_t"]
        self.assertEqual(len(plane_sheet_bytes), 3_113)
        self.assertEqual(
            hashlib.sha256(plane_sheet_bytes).hexdigest(), plane_sheet_sha256
        )
        self.assertTrue(
            all(
                case["size_parameters"]["input_bytes"] == len(plane_sheet_bytes)
                and case["policy_values"]["source_sha256"] == plane_sheet_sha256
                and case["policy_values"]["source_evidence"]
                == historical_evidence
                and case["expected_result_counters"]["source_faces"] == 1
                and case["expected_result_counters"]["source_edges"] == 6
                and case["expected_result_counters"]["source_vertices"] == 6
                and case["expected_result_counters"]["boundary_segments"] == 6
                for case in plane_sheets
            )
        )
        cylinder_sheets = [
            case
            for case in tessellation
            if case["policy_values"].get("source_fixture")
            == "sheet_cylinder_seam.x_t@onshape-cloud-2026-07-11"
        ]
        self.assertEqual(len(cylinder_sheets), 4)
        cylinder_sheet_bytes = (
            ROOT / "benches" / "testdata" / "sheet_cylinder_seam.certified.x_t"
        ).read_bytes()
        cylinder_sheet_sha256 = certified["fixtures_sha256"][
            "sheet_cylinder_seam.x_t"
        ]
        self.assertEqual(len(cylinder_sheet_bytes), 2_209)
        self.assertEqual(
            hashlib.sha256(cylinder_sheet_bytes).hexdigest(), cylinder_sheet_sha256
        )
        cylinder_sheet_boundary = {1e-2: 32, 3e-3: 32, 1e-3: 64, 3e-4: 128}
        cylinder_sheet_measure = {
            1e-2: (1.001, 1.002),
            3e-3: (0.994, 0.996),
            1e-3: (1.0, 1.001),
            3e-4: (1.0, 1.002),
        }
        self.assertTrue(
            all(
                case["size_parameters"]["input_bytes"]
                == len(cylinder_sheet_bytes)
                and case["policy_values"]["source_sha256"]
                == cylinder_sheet_sha256
                and case["policy_values"]["source_evidence"]
                == historical_evidence
                and case["expected_result_counters"]["source_faces"] == 1
                and case["expected_result_counters"]["source_edges"] == 3
                and case["expected_result_counters"]["source_vertices"] == 2
                and case["expected_result_counters"]["boundary_segments"]
                == cylinder_sheet_boundary[case["tolerances"]["chord_tol"]]
                and (
                    case["policy_values"]["measure_ratio_floor"],
                    case["policy_values"]["measure_ratio_ceiling"],
                )
                == cylinder_sheet_measure[case["tolerances"]["chord_tol"]]
                for case in cylinder_sheets
            )
        )
        face_tessellation = [
            case for case in cases if case["benchmark_target"] == "face_tessellation"
        ]
        self.assertEqual(len(face_tessellation), 18)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x5154464143450007
                and case["fixture_version"] == "face-tessellation.v2"
                and case["policy_values"]["api"] == "tessellate_with_context"
                and case["policy_values"]["budget_profile"]
                == "face-tessellation.compatibility-v1"
                and case["policy_values"]["execution"] == "serial"
                and case["policy_values"]["policy_version"] == "v1"
                and case["policy_values"]["usage_contract"]
                == "q3-face-usage.v1"
                and case["expected_result_counters"]["usage_stage_count"] == 5
                and len(case["expected_result_counters"]["usage_consumed"]) == 5
                and case["expected_result_counters"]["mesh_vertices"] > 0
                and case["expected_result_counters"]["mesh_triangles"] > 0
                and case["expected_result_counters"]["boundary_vertices"] > 0
                for case in face_tessellation
            )
        )
        self.assertEqual(
            {
                case["policy_values"]["representation"]
                for case in face_tessellation
            },
            {"plane-v2", "half-cylinder-v2", "rational-nurbs-v2"},
        )
        self.assertEqual(
            {case["policy_values"]["trim_shape"] for case in face_tessellation},
            {"outer", "one-hole", "three-holes"},
        )
        self.assertEqual(
            {case["tolerances"]["chord_tol"] for case in face_tessellation},
            {1e-2, 1e-3},
        )
        matrix = {
            (
                case["policy_values"]["representation"],
                case["policy_values"]["trim_shape"],
                case["tolerances"]["chord_tol"],
            )
            for case in face_tessellation
        }
        self.assertEqual(len(matrix), 18)
        for case in face_tessellation:
            counters = case["expected_result_counters"]
            expected_loops = {
                "outer": 1,
                "one-hole": 2,
                "three-holes": 4,
            }[case["policy_values"]["trim_shape"]]
            self.assertEqual(case["size_parameters"]["trim_loops"], expected_loops)
            self.assertEqual(counters["boundary_loops"], expected_loops)
            self.assertEqual(len(counters["boundary_loop_vertices"]), expected_loops)
            self.assertEqual(
                sum(counters["boundary_loop_vertices"]),
                counters["boundary_vertices"],
            )
            self.assertTrue(
                all(
                    counters[field]
                    for field in (
                        "positions_finite",
                        "uvs_finite",
                        "indices_valid",
                        "coordinates_aligned",
                        "triangles_oriented",
                        "positions_on_surface",
                        "triangles_follow_surface_orientation",
                        "boundary_retains_trim_vertices",
                        "parameter_area_matches_trim",
                        "model_area_finite_positive",
                    )
                )
            )
            self.assertGreater(counters["parameter_area_units"], 0)
            self.assertGreater(counters["model_area_units"], 0)
            for digest in (
                "trim_digest",
                "boundary_digest",
                "usage_digest",
                "mesh_digest",
                "output_digest",
            ):
                self.assertEqual(len(counters[digest]), 16)
        isolation = [
            case for case in cases if case["benchmark_target"] == "nurbs_isolation"
        ]
        self.assertEqual(len(isolation), 8)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x51544E5552420006
                and case["fixture_version"] == "nurbs-isolation.v3"
                for case in isolation
            )
        )
        surface_roundoff = [
            case
            for case in isolation
            if "subdivision-roundoff" in case["path"]
        ]
        self.assertEqual(len(surface_roundoff), 1)
        self.assertTrue(surface_roundoff[0]["expected_result_counters"]["complete"])
        self.assertFalse(surface_roundoff[0]["expected_result_counters"]["proven_empty"])
        limited = [
            case
            for case in isolation
            if case["expected_result_counters"]["limit_kind"] != "none"
        ]
        self.assertEqual(len(limited), 3)
        self.assertTrue(
            all(
                case["expected_result_counters"]["indeterminate"]
                and case["expected_result_counters"]["conservative_cover"]
                and not case["expected_result_counters"]["complete"]
                and not case["expected_result_counters"]["proven_empty"]
                for case in limited
            )
        )
        surface_work_low = [
            case
            for case in isolation
            if "rational-four-patch" in case["path"]
            and case["expected_result_counters"]["limit_kind"] == "work"
        ]
        self.assertEqual(len(surface_work_low), 1)
        self.assertEqual(
            surface_work_low[0]["expected_result_counters"]["limit_attempted_consumed"],
            surface_work_low[0]["expected_result_counters"]["limit_attempted_allowed"]
            + 1,
        )
        curve_pair_isolation = [
            case
            for case in cases
            if case["benchmark_target"] == "curve_pair_isolation"
        ]
        self.assertEqual(len(curve_pair_isolation), 9)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x515443504149000A
                and case["fixture_version"] == "curve-pair-isolation.v4"
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
        self.assertEqual(len(curve_pair_solve), 28)
        self.assertTrue(
            all(
                case["deterministic_seed"] == 0x51544350534F0018
                and case["fixture_version"] == "curve-pair-solve.v18"
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
        algebraic_spatial = [
            case
            for case in curve_pair_solve
            if "algebraic-spatial" in case["path"]
        ]
        self.assertEqual(len(algebraic_spatial), 1)
        self.assertTrue(algebraic_spatial[0]["expected_result_counters"]["complete"])
        self.assertEqual(
            algebraic_spatial[0]["expected_result_counters"]["root_certificates"], 1
        )
        algebraic_linear_form = [
            case
            for case in curve_pair_solve
            if "algebraic-linear-form" in case["path"]
        ]
        self.assertEqual(len(algebraic_linear_form), 1)
        self.assertTrue(
            algebraic_linear_form[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_linear_form[0]["expected_result_counters"]["root_certificates"],
            1,
        )
        algebraic_primitive_form = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-form" in case["path"]
        ]
        self.assertEqual(len(algebraic_primitive_form), 1)
        self.assertTrue(
            algebraic_primitive_form[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_primitive_form[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_three = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-three" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_three), 1)
        self.assertTrue(
            algebraic_magnitude_three[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_three[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_four = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-four" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_four), 1)
        self.assertTrue(
            algebraic_magnitude_four[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_four[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_five = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-five" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_five), 1)
        self.assertTrue(
            algebraic_magnitude_five[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_five[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_six = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-six" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_six), 1)
        self.assertTrue(
            algebraic_magnitude_six[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_six[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_seven = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-seven" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_seven), 1)
        self.assertTrue(
            algebraic_magnitude_seven[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_seven[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_eight = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-eight" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_eight), 1)
        self.assertTrue(
            algebraic_magnitude_eight[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_eight[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_nine = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-nine" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_nine), 1)
        self.assertTrue(
            algebraic_magnitude_nine[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_nine[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_ten = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-ten" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_ten), 1)
        self.assertTrue(
            algebraic_magnitude_ten[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_ten[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_eleven = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-eleven" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_eleven), 1)
        self.assertTrue(
            algebraic_magnitude_eleven[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_eleven[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        algebraic_magnitude_twelve = [
            case
            for case in curve_pair_solve
            if "algebraic-primitive-magnitude-twelve" in case["path"]
        ]
        self.assertEqual(len(algebraic_magnitude_twelve), 1)
        self.assertTrue(
            algebraic_magnitude_twelve[0]["expected_result_counters"]["complete"]
        )
        self.assertEqual(
            algebraic_magnitude_twelve[0]["expected_result_counters"][
                "root_certificates"
            ],
            1,
        )
        solve_limited = [
            case
            for case in curve_pair_solve
            if case["expected_result_counters"]["limit_kind"] != "none"
        ]
        self.assertEqual(len(solve_limited), 5)
        self.assertEqual(
            {
                case["expected_result_counters"]["limit_kind"]
                for case in solve_limited
            },
            {"seed-attempts", "overlap-work", "overlap-items"},
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
        self.assertEqual(len(common_refinement), 3)
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
        common_items_denied = next(
            case
            for case in common_refinement
            if case["expected_result_counters"]["limit_kind"] == "overlap-items"
        )
        self.assertEqual(
            common_items_denied["expected_result_counters"][
                "limit_attempted_consumed"
            ],
            common_items_denied["expected_result_counters"][
                "limit_attempted_allowed"
            ]
            + 1,
        )
        self.assertEqual(
            common_items_denied["expected_result_counters"]["overlaps"], 0
        )
        inverse_history = [
            case
            for case in curve_pair_solve
            if "inverse-history" in case["path"]
        ]
        self.assertEqual(len(inverse_history), 4)
        inverse_complete = next(
            case
            for case in inverse_history
            if "inverse-history-overlap" in case["path"]
            and case["expected_result_counters"]["limit_kind"] == "none"
        )
        inverse_altered = next(
            case
            for case in inverse_history
            if "altered-inverse-history" in case["path"]
        )
        self.assertTrue(inverse_complete["expected_result_counters"]["complete"])
        self.assertEqual(inverse_complete["expected_result_counters"]["overlaps"], 1)
        self.assertFalse(inverse_altered["expected_result_counters"]["complete"])
        self.assertEqual(inverse_altered["expected_result_counters"]["overlaps"], 0)
        self.assertEqual(inverse_altered["expected_result_counters"]["points"], 2)
        self.assertEqual(
            inverse_complete["expected_result_counters"]["overlap_equivalence_work"],
            inverse_altered["expected_result_counters"]["overlap_equivalence_work"],
        )
        self.assertEqual(
            inverse_complete["expected_result_counters"]["overlap_equivalence_items"],
            inverse_altered["expected_result_counters"]["overlap_equivalence_items"],
        )
        inverse_limits = [
            case
            for case in inverse_history
            if case["expected_result_counters"]["limit_kind"] != "none"
        ]
        self.assertEqual(len(inverse_limits), 2)
        self.assertEqual(
            {case["expected_result_counters"]["limit_kind"] for case in inverse_limits},
            {"overlap-work", "overlap-items"},
        )
        self.assertTrue(
            all(
                case["expected_result_counters"]["limit_attempted_consumed"]
                == case["expected_result_counters"]["limit_attempted_allowed"] + 1
                for case in inverse_limits
            )
        )
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
