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

    def test_toolchain_and_runner_versions_are_exactly_pinned(self):
        cargo = (FUZZ / "Cargo.toml").read_text()
        toolchain = (FUZZ / "rust-toolchain.toml").read_text()
        root_cargo = (ROOT / "Cargo.toml").read_text()
        self.assertIn('libfuzzer-sys = { version = "=0.4.13"', cargo)
        self.assertEqual(self.contract["toolchain"], "nightly-2026-01-22")
        self.assertIn(f'channel = "{self.contract["toolchain"]}"', toolchain)
        self.assertEqual(self.contract["cargo_fuzz_version"], "0.13.2")
        self.assertEqual(self.contract["libfuzzer_sys_version"], "0.4.13")
        self.assertIn('exclude = ["benches", "fuzz"]', root_cargo)

    def test_contract_caps_and_smoke_cli_are_synchronized(self):
        target = self.contract["target"]
        rust = (FUZZ / "src/xt_read.rs").read_text()
        readme = (FUZZ / "README.md").read_text()
        self.assertEqual(target["selector_bytes"], 1)
        self.assertEqual(target["max_payload_bytes"], 256 * 1024)
        self.assertEqual(target["max_import_records"], 4096)
        self.assertIn("pub const MAX_PAYLOAD_BYTES: usize = 256 * 1024;", rust)
        self.assertIn("pub const MAX_IMPORT_RECORDS: usize = 4_096;", rust)
        command = (
            "cargo fuzz run xt_read --features fuzzing corpus/xt_read -- "
            f'-seed={target["smoke_seed"]} '
            f'-max_len={target["selector_bytes"] + target["max_payload_bytes"]} '
            f'-timeout={target["timeout_seconds"]} '
            f'-rss_limit_mb={target["rss_limit_mb"]} '
            f'-max_total_time={target["smoke_seconds"]} '
            "-artifact_prefix=artifacts/xt_read/"
        )
        self.assertIn(command, readme)

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
            expected_names = {entry["file"] for entry in self.contract["corpus"]}
            self.assertEqual(
                {path.name for path in generated.glob("*.xtseed")}, expected_names
            )
            self.assertEqual(
                {path.name for path in (FUZZ / "corpus/xt_read").glob("*.xtseed")},
                expected_names,
            )
            max_payload = self.contract["target"]["max_payload_bytes"]
            for entry in self.contract["corpus"]:
                checked = (FUZZ / "corpus/xt_read" / entry["file"]).read_bytes()
                regenerated = (generated / entry["file"]).read_bytes()
                self.assertEqual(checked, regenerated, entry["file"])
                self.assertEqual(checked[0], entry["selector"])
                self.assertLessEqual(len(checked) - 1, max_payload)
                self.assertEqual(entry["license"], "Apache-2.0")

    def test_declared_sources_and_transforms_exactly_produce_checked_seeds(self):
        for entry in self.contract["corpus"]:
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
        for entry in self.contract["corpus"]:
            source = Path(entry["source"])
            self.assertFalse(source.is_absolute())
            self.assertTrue((ROOT / source).is_file())
            if source.parts[:4] == ("crates", "kxt", "tests", "fixtures"):
                self.assertEqual(manifest[source.name][4], "Apache-2.0")

    def test_generated_crash_locations_are_ignored_and_absent(self):
        ignored = (FUZZ / ".gitignore").read_text()
        self.assertIn("/target/", ignored)
        self.assertIn("/artifacts/", ignored)
        self.assertIn("/coverage/", ignored)
        self.assertFalse((FUZZ / "artifacts").exists())
        self.assertFalse((FUZZ / "coverage").exists())


if __name__ == "__main__":
    unittest.main()
