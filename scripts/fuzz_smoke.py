#!/usr/bin/env python3
"""Run one pinned fuzz target with a fresh corpus and a hard wall deadline."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FUZZ = ROOT / "fuzz"
CONTRACT = FUZZ / "contract.json"
HARD_TIMEOUT_SECONDS = 45
KILL_GRACE_SECONDS = 5
TARGETS = ("xt_read", "nurbs_constructors")


def load_contract(path=CONTRACT):
    """Load the checked target/resource contract."""
    return json.loads(Path(path).read_text(encoding="utf-8"))


def build_command(target, corpus, artifacts, contract):
    """Build the exact cargo-fuzz command for one declared target."""
    settings = contract["targets"][target]
    max_len = (
        settings["selector_bytes"] + settings["max_payload_bytes"]
        if target == "xt_read"
        else settings["max_input_bytes"]
    )
    return [
        "cargo",
        "fuzz",
        "run",
        target,
        "--features",
        "fuzzing",
        str(corpus),
        "--",
        "-seed={}".format(settings["smoke_seed"]),
        "-max_len={}".format(max_len),
        "-timeout={}".format(settings["timeout_seconds"]),
        "-rss_limit_mb={}".format(settings["rss_limit_mb"]),
        "-max_total_time={}".format(settings["smoke_seconds"]),
        "-artifact_prefix={}/".format(artifacts),
    ]


@contextmanager
def staged_corpus(target, source_root=FUZZ / "corpus"):
    """Copy committed seeds into a new disposable directory for one run."""
    with tempfile.TemporaryDirectory(prefix="kernel-fuzz-{}-".format(target)) as tmp:
        destination = Path(tmp) / target
        shutil.copytree(Path(source_root) / target, destination)
        yield destination


def child_environment(contract, base=None):
    """Force the contract nightly even when the caller exports an override."""
    environment = dict(os.environ if base is None else base)
    environment["RUSTUP_TOOLCHAIN"] = contract["toolchain"]
    return environment


def _group_exists(process_group):
    try:
        os.killpg(process_group, 0)
        return True
    except ProcessLookupError:
        return False


def _signal_group(process_group, requested_signal):
    try:
        os.killpg(process_group, requested_signal)
    except ProcessLookupError:
        pass


def _stop_process(process):
    """Terminate, then unconditionally escalate any surviving POSIX group."""
    # start_new_session=True makes the launched PID the durable process-group
    # ID even if the cargo-fuzz leader exits before its descendants.
    process_group = process.pid
    _signal_group(process_group, signal.SIGTERM)
    deadline = time.monotonic() + KILL_GRACE_SECONDS
    while _group_exists(process_group) and time.monotonic() < deadline:
        time.sleep(0.05)
    if _group_exists(process_group):
        _signal_group(process_group, signal.SIGKILL)
    process.wait()


def run_target(target):
    """Run one target, returning its process status or 124 on hard timeout."""
    if os.name != "posix":
        raise RuntimeError("the fuzz smoke process-tree deadline requires POSIX")
    contract = load_contract()
    artifacts = FUZZ / "artifacts" / target
    artifacts.mkdir(parents=True, exist_ok=True)
    with staged_corpus(target) as corpus:
        command = build_command(target, corpus, artifacts, contract)
        process = subprocess.Popen(
            command,
            cwd=FUZZ,
            env=child_environment(contract),
            start_new_session=True,
        )
        try:
            return process.wait(timeout=HARD_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            _stop_process(process)
            print(
                "fuzz smoke hard timeout after {} seconds: {}".format(
                    HARD_TIMEOUT_SECONDS, target
                )
            )
            return 124


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", choices=TARGETS)
    args = parser.parse_args(argv)
    return run_target(args.target)


if __name__ == "__main__":
    raise SystemExit(main())
