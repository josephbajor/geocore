"""Measured-run repository, toolchain, host, and runner metadata."""

import ctypes
import os
import platform
import subprocess
from pathlib import Path

from .contract import ContractError, ROOT


CARGO_CRITERION_VERSION = "1.1.0"
CRITERION_VERSION = "0.8.2"


def command(args, cwd=ROOT):
    """Run a metadata command and return stdout or one contract failure."""
    result = subprocess.run(
        args,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise ContractError(
            "command failed: {}\n{}".format(" ".join(args), result.stderr.strip())
        )
    return result.stdout.strip()


def _cpu_model():
    system = platform.system().lower()
    if system == "darwin":
        try:
            value = command(["sysctl", "-n", "machdep.cpu.brand_string"])
            if value:
                return value
        except ContractError:
            pass
    if system == "linux":
        try:
            for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
                if line.lower().startswith("model name"):
                    return line.split(":", 1)[1].strip()
        except OSError:
            pass
    return platform.processor() or os.environ.get("PROCESSOR_IDENTIFIER") or "unknown-cpu"


def _available_memory():
    system = platform.system().lower()
    if system == "linux":
        try:
            for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) * 1024
        except (OSError, ValueError, IndexError):
            pass
    if system == "darwin":
        try:
            page_size = int(command(["sysctl", "-n", "hw.pagesize"]))
            free = 0
            for line in command(["vm_stat"]).splitlines():
                if line.startswith(("Pages free:", "Pages inactive:", "Pages speculative:")):
                    free += int(line.split(":", 1)[1].strip().rstrip("."))
            if free > 0:
                return free * page_size
        except (ContractError, ValueError):
            pass
    if system == "windows":
        class MemoryStatus(ctypes.Structure):
            _fields_ = [
                ("length", ctypes.c_ulong),
                ("memory_load", ctypes.c_ulong),
                ("total_physical", ctypes.c_ulonglong),
                ("available_physical", ctypes.c_ulonglong),
                ("total_page_file", ctypes.c_ulonglong),
                ("available_page_file", ctypes.c_ulonglong),
                ("total_virtual", ctypes.c_ulonglong),
                ("available_virtual", ctypes.c_ulonglong),
                ("available_extended_virtual", ctypes.c_ulonglong),
            ]

        status = MemoryStatus()
        status.length = ctypes.sizeof(MemoryStatus)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return int(status.available_physical)
    try:
        pages = os.sysconf("SC_AVPHYS_PAGES")
        page_size = os.sysconf("SC_PAGE_SIZE")
        return int(pages * page_size)
    except (AttributeError, OSError, ValueError):
        raise ContractError("could not determine available host memory")


def _affinity():
    if hasattr(os, "sched_getaffinity"):
        cpus = sorted(os.sched_getaffinity(0))
        return "sched_getaffinity:" + ",".join(str(cpu) for cpu in cpus)
    return "unrestricted-by-runner"


def synthetic_environment(features, smoke):
    """Return unmistakably synthetic, comparison-ineligible identity data."""
    return {
        "run": {
            "kind": "synthetic-example",
            "comparison_eligible": False,
            "note": "Synthetic parser/schema exercise only; not performance evidence.",
        },
        "repository": {
            "revision": "SYNTHETIC-NOT-A-GIT-REVISION",
            "dirty_worktree": True,
        },
        "toolchain": {
            "rustc_version": "SYNTHETIC",
            "cargo_version": "SYNTHETIC",
            "target_triple": "synthetic-unknown-none",
            "profile": "bench",
            "enabled_features": sorted(features),
        },
        "host": {
            "os": "synthetic",
            "architecture": "synthetic",
            "cpu_model": "SYNTHETIC-NOT-A-HOST",
            "logical_core_count": 1,
            "available_memory_bytes": 1,
        },
        "runner": _runner(smoke, "synthetic-unrestricted"),
    }


def measured_environment(features, smoke):
    """Capture all required comparison identity fields for a measured run."""
    rustc_verbose = command(["rustc", "--version", "--verbose"])
    target_lines = [
        line for line in rustc_verbose.splitlines() if line.startswith("host:")
    ]
    if len(target_lines) != 1:
        raise ContractError("rustc verbose output lacks one host target")
    dirty = bool(command(["git", "status", "--porcelain"]))
    note = "Measured run; advisory timing only."
    if dirty:
        note += " Dirty worktree makes this run ineligible for baseline comparison."
    return {
        "run": {
            "kind": "measured",
            "comparison_eligible": not dirty,
            "note": note,
        },
        "repository": {
            "revision": command(["git", "rev-parse", "HEAD"]),
            "dirty_worktree": dirty,
        },
        "toolchain": {
            "rustc_version": rustc_verbose.splitlines()[0],
            "cargo_version": command(["cargo", "--version"]),
            "target_triple": target_lines[0].split(":", 1)[1].strip(),
            "profile": "bench",
            "enabled_features": sorted(features),
        },
        "host": {
            "os": platform.system().lower() or "unknown-os",
            "architecture": platform.machine() or "unknown-architecture",
            "cpu_model": _cpu_model(),
            "logical_core_count": os.cpu_count() or 1,
            "available_memory_bytes": _available_memory(),
        },
        "runner": _runner(smoke, _affinity()),
    }


def _runner(smoke, affinity):
    return {
        "name": "cargo-criterion",
        "version": CARGO_CRITERION_VERSION,
        "criterion_version": CRITERION_VERSION,
        "warm_up_seconds": 0.1 if smoke else 3.0,
        "sample_size": 10 if smoke else 100,
        "measurement_seconds": 0.2 if smoke else 5.0,
        "process_affinity": affinity,
    }
