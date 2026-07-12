"""Contract tests for the bounded cross-platform fuzz runner."""

import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import fuzz_smoke  # noqa: E402


class FuzzSmokeTests(unittest.TestCase):
    def test_timeout_escalates_the_known_session_group_after_leader_exit(self):
        process = mock.Mock(pid=4242)
        with (
            mock.patch.object(fuzz_smoke, "_group_exists", return_value=True),
            mock.patch.object(fuzz_smoke, "_signal_group") as send,
            mock.patch.object(fuzz_smoke.time, "monotonic", side_effect=[0.0, 10.0]),
        ):
            fuzz_smoke._stop_process(process)
        self.assertEqual(
            send.call_args_list,
            [mock.call(4242, signal.SIGTERM), mock.call(4242, signal.SIGKILL)],
        )
        process.wait.assert_called_once_with()

    def test_child_environment_overrides_ambient_toolchain(self):
        contract = fuzz_smoke.load_contract()
        environment = fuzz_smoke.child_environment(
            contract, {"RUSTUP_TOOLCHAIN": "stable", "KEEP": "value"}
        )
        self.assertEqual(environment["RUSTUP_TOOLCHAIN"], contract["toolchain"])
        self.assertEqual(environment["KEEP"], "value")

    def test_commands_derive_every_limit_from_the_contract(self):
        contract = fuzz_smoke.load_contract()
        cases = {
            "xt_read": (262145, 5860406134146269190),
            "nurbs_constructors": (4096, 5860395143475101702),
        }
        for target, (max_len, seed) in cases.items():
            command = fuzz_smoke.build_command(
                target, Path("fresh") / target, Path("artifacts") / target, contract
            )
            rendered = " ".join(str(value) for value in command)
            self.assertIn("-max_len={}".format(max_len), rendered)
            self.assertIn("-seed={}".format(seed), rendered)
            self.assertIn("-timeout=5", rendered)
            self.assertIn("-rss_limit_mb=2048", rendered)
            self.assertIn("-max_total_time=20", rendered)

    def test_corpus_staging_is_fresh_and_disposable(self):
        with tempfile.TemporaryDirectory() as source:
            target_source = Path(source) / "xt_read"
            target_source.mkdir()
            (target_source / "seed").write_bytes(b"seed")
            with fuzz_smoke.staged_corpus("xt_read", source) as staged:
                self.assertEqual([path.name for path in staged.iterdir()], ["seed"])
                (staged / "grown").write_bytes(b"generated")
                staged_parent = staged.parent
            self.assertFalse(staged_parent.exists())
            self.assertEqual([path.name for path in target_source.iterdir()], ["seed"])


if __name__ == "__main__":
    unittest.main()
