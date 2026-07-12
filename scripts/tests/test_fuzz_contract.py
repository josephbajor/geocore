"""Bounded offline validation for the isolated fuzz target contract."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FUZZ = ROOT / "fuzz"


class FuzzContractTests(unittest.TestCase):
    """Validate pins, corpus provenance, and deterministic seed generation."""

    def setUp(self):
        self.contract = json.loads((FUZZ / "contract.json").read_text())
        self.xt_target = self.contract["targets"]["xt_read"]
        self.xt_corpus = self.contract["corpora"]["xt_read"]
        self.nurbs_target = self.contract["targets"]["nurbs_constructors"]
        self.nurbs_corpus = self.contract["corpora"]["nurbs_constructors"]

    def test_toolchain_and_runner_versions_are_exactly_pinned(self):
        cargo = (FUZZ / "Cargo.toml").read_text()
        toolchain = (FUZZ / "rust-toolchain.toml").read_text()
        root_cargo = (ROOT / "Cargo.toml").read_text()
        self.assertIn('libfuzzer-sys = { version = "=0.4.13"', cargo)
        self.assertEqual(self.contract["schema_version"], "kernel-fuzz-contract.v2")
        self.assertEqual(self.contract["toolchain"], "nightly-2026-01-22")
        self.assertIn(f'channel = "{self.contract["toolchain"]}"', toolchain)
        self.assertEqual(self.contract["cargo_fuzz_version"], "0.13.2")
        self.assertEqual(self.contract["libfuzzer_sys_version"], "0.4.13")
        self.assertIn('exclude = ["benches", "fuzz"]', root_cargo)

    def test_contract_caps_and_smoke_cli_are_synchronized(self):
        target = self.xt_target
        rust = (FUZZ / "src/xt_read.rs").read_text()
        readme = (FUZZ / "README.md").read_text()
        self.assertEqual(target["selector_bytes"], 1)
        self.assertEqual(target["max_payload_bytes"], 256 * 1024)
        self.assertEqual(target["max_import_records"], 4096)
        self.assertIn("pub const MAX_PAYLOAD_BYTES: usize = 256 * 1024;", rust)
        self.assertIn("pub const MAX_IMPORT_RECORDS: usize = 4_096;", rust)
        self.assertIn("python3 scripts/fuzz_smoke.py xt_read", readme)

    def test_checked_corpus_is_exactly_reproducible(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run(
                [
                    sys.executable,
                    str(FUZZ / "scripts/generate_xt_read_corpus.py"),
                    "--output",
                    directory,
                ],
                check=True,
                cwd=ROOT,
            )
            generated = Path(directory)
            expected_names = {entry["file"] for entry in self.xt_corpus}
            self.assertEqual(
                {path.name for path in generated.glob("*.xtseed")}, expected_names
            )
            self.assertEqual(
                {path.name for path in (FUZZ / "corpus/xt_read").glob("*.xtseed")},
                expected_names,
            )
            max_payload = self.xt_target["max_payload_bytes"]
            for entry in self.xt_corpus:
                checked = (FUZZ / "corpus/xt_read" / entry["file"]).read_bytes()
                regenerated = (generated / entry["file"]).read_bytes()
                self.assertEqual(checked, regenerated, entry["file"])
                self.assertEqual(checked[0], entry["selector"])
                self.assertLessEqual(len(checked) - 1, max_payload)
                self.assertEqual(entry["license"], "Apache-2.0")

    def test_declared_sources_and_transforms_exactly_produce_checked_seeds(self):
        for entry in self.xt_corpus:
            source = (ROOT / entry["source"]).read_bytes()
            transform = entry["transform"]
            if transform == "full":
                payload = source
            elif transform == "header-boundary":
                marker = b"**END_OF_HEADER"
                start = source.index(marker)
                payload = source[: source.index(b"\n", start) + 1]
            elif transform == "terminator-token-boundary":
                stripped = source.rstrip()
                self.assertTrue(stripped.endswith(b"1 0"))
                payload = stripped[:-1]
            elif transform == "terminator-record-boundary":
                stripped = source.rstrip()
                self.assertTrue(stripped.endswith(b"1 0"))
                payload = stripped[:-3].rstrip() + b" "
            else:
                self.fail(f"unknown corpus transform: {transform}")
            checked = (FUZZ / "corpus/xt_read" / entry["file"]).read_bytes()
            self.assertEqual(checked, bytes([entry["selector"]]) + payload)

    def test_seed_sources_are_local_and_redistributable(self):
        manifest_lines = (
            ROOT / "crates/kxt/tests/fixtures/manifest.tsv"
        ).read_text().splitlines()
        manifest = {
            columns[0]: columns
            for columns in (line.split("\t") for line in manifest_lines[1:])
        }
        for entry in self.xt_corpus:
            source = Path(entry["source"])
            self.assertFalse(source.is_absolute())
            self.assertTrue((ROOT / source).is_file())
            if source.parts[:4] == ("crates", "kxt", "tests", "fixtures"):
                self.assertEqual(manifest[source.name][4], "Apache-2.0")

    def test_nurbs_contract_caps_and_smoke_cli_are_synchronized(self):
        target = self.nurbs_target
        cargo = (FUZZ / "Cargo.toml").read_text()
        rust = (FUZZ / "src/nurbs_constructors.rs").read_text()
        readme = (FUZZ / "README.md").read_text()
        self.assertEqual(target["format_version"], "nurbs-structured.v1")
        self.assertEqual(target["header_bytes"], 7)
        self.assertIn('name = "nurbs_constructors"', cargo)
        self.assertIn('path = "fuzz_targets/nurbs_constructors.rs"', cargo)
        constants = {
            "MAX_INPUT_BYTES": ("4 * 1024", "max_input_bytes", 4 * 1024),
            "MAX_DEGREE": ("5", "max_degree", 5),
            "MAX_KNOTS_PER_DIRECTION": (
                "20",
                "max_knots_per_direction",
                20,
            ),
            "MAX_CURVE_POINTS": ("16", "max_curve_points", 16),
            "MAX_SURFACE_POINTS": ("64", "max_surface_points", 64),
            "MAX_WEIGHTS": ("64", "max_weights", 64),
        }
        for constant, (expression, contract_key, expected_value) in constants.items():
            self.assertIn(f"pub const {constant}: usize = {expression};", rust)
            self.assertEqual(target[contract_key], expected_value)
        self.assertIn("pub const MAX_ISOLATION_DEPTH: u32 = 2;", rust)
        self.assertIn(
            "pub const MAX_ISOLATION_CANDIDATE_CELLS: usize = 32;", rust
        )
        self.assertEqual(target["max_isolation_depth"], 2)
        self.assertEqual(target["max_isolation_candidate_cells"], 32)
        self.assertIn("python3 scripts/fuzz_smoke.py nurbs_constructors", readme)

    def test_nurbs_corpus_is_exactly_reproducible_and_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run(
                [
                    sys.executable,
                    str(FUZZ / "scripts/generate_nurbs_constructors_corpus.py"),
                    "--output",
                    directory,
                ],
                check=True,
                cwd=ROOT,
            )
            generated = Path(directory)
            expected_names = {entry["file"] for entry in self.nurbs_corpus}
            self.assertEqual(
                {path.name for path in generated.glob("*.nurbsseed")},
                expected_names,
            )
            self.assertEqual(
                {
                    path.name
                    for path in (FUZZ / "corpus/nurbs_constructors").glob(
                        "*.nurbsseed"
                    )
                },
                expected_names,
            )
            for entry in self.nurbs_corpus:
                checked = (
                    FUZZ / "corpus/nurbs_constructors" / entry["file"]
                ).read_bytes()
                regenerated = (generated / entry["file"]).read_bytes()
                self.assertEqual(checked, regenerated, entry["file"])
                self.assertLessEqual(len(checked), self.nurbs_target["max_input_bytes"])
                self.assertEqual(entry["source"], "generated")
                self.assertEqual(entry["license"], "Apache-2.0")

        families = [entry["family"] for entry in self.nurbs_corpus]
        expectations = [entry["expected_constructor"] for entry in self.nurbs_corpus]
        self.assertEqual(families.count("curve"), 5)
        self.assertEqual(families.count("surface"), 4)
        self.assertEqual(expectations.count("accepted"), 4)
        self.assertEqual(expectations.count("rejected"), 5)

    def test_nurbs_seed_headers_match_declared_families_and_caps(self):
        target = self.nurbs_target
        for entry in self.nurbs_corpus:
            seed = (FUZZ / "corpus/nurbs_constructors" / entry["file"]).read_bytes()
            self.assertGreaterEqual(len(seed), target["header_bytes"])
            selector, degree_u, degree_v, knots_u, knots_v, points, weights = seed[:7]
            expected_family = "surface" if selector & 1 else "curve"
            self.assertEqual(entry["family"], expected_family)
            self.assertLessEqual(degree_u, target["max_degree"])
            self.assertLessEqual(degree_v, target["max_degree"])
            self.assertLessEqual(knots_u, target["max_knots_per_direction"])
            self.assertLessEqual(knots_v, target["max_knots_per_direction"])
            point_cap = (
                target["max_surface_points"]
                if expected_family == "surface"
                else target["max_curve_points"]
            )
            self.assertLessEqual(points, point_cap)
            self.assertLessEqual(weights, target["max_weights"])
            declared_size = 7 + 8 * (5 + knots_u + knots_v + 3 * points + weights)
            self.assertLessEqual(declared_size, len(seed))

    def test_generated_crash_locations_are_ignored_and_absent(self):
        ignored = (FUZZ / ".gitignore").read_text()
        self.assertIn("/target/", ignored)
        self.assertIn("/artifacts/", ignored)
        self.assertIn("/coverage/", ignored)
        self.assertFalse((FUZZ / "artifacts").exists())
        self.assertFalse((FUZZ / "coverage").exists())


if __name__ == "__main__":
    unittest.main()
