#!/usr/bin/env python3
"""Validate the supported facade package and facade-only client boundaries."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterable


KERNEL_PACKAGE_FILES = {
    ".cargo_vcs_info.json",
    "Cargo.lock",
    "Cargo.toml",
    "Cargo.toml.orig",
    "README.md",
    "src/error.rs",
    "src/id.rs",
    "src/intersection.rs",
    "src/interchange.rs",
    "src/iter.rs",
    "src/lib.rs",
    "src/operation.rs",
    "src/session.rs",
    "src/view/body.rs",
    "src/view/boundary.rs",
    "src/view/edge.rs",
    "src/view/geometry.rs",
    "src/view/mod.rs",
    "src/view/part.rs",
    "tests/lifecycle.rs",
}


class ContractError(RuntimeError):
    """A facade packaging or dependency boundary changed unexpectedly."""


def validate_package_files(paths: Iterable[str]) -> None:
    """Require the reviewed, self-contained `kernel` package inventory."""
    actual = {path.strip() for path in paths if path.strip()}
    missing = sorted(KERNEL_PACKAGE_FILES - actual)
    unexpected = sorted(actual - KERNEL_PACKAGE_FILES)
    if missing or unexpected:
        raise ContractError(
            f"kernel package inventory changed: missing={missing}, unexpected={unexpected}"
        )


def validate_facade_client(metadata: dict[str, Any]) -> None:
    """Require the lifecycle client to depend directly only on `kernel`."""
    clients = [
        package
        for package in metadata.get("packages", [])
        if package.get("name") == "kernel-lifecycle"
    ]
    if len(clients) != 1:
        raise ContractError(
            f"expected one kernel-lifecycle package, found {len(clients)}"
        )
    dependencies = clients[0].get("dependencies", [])
    normal = sorted(
        dependency.get("name")
        for dependency in dependencies
        if dependency.get("kind") is None
    )
    non_normal = sorted(
        (dependency.get("name"), dependency.get("kind"))
        for dependency in dependencies
        if dependency.get("kind") is not None
    )
    if normal != ["kernel"] or non_normal:
        raise ContractError(
            "kernel-lifecycle direct dependencies changed: "
            f"normal={normal}, non_normal={non_normal}"
        )


def main() -> int:
    """Run Cargo-backed contract checks from the repository root."""
    repository = Path(__file__).resolve().parents[1]
    package = subprocess.run(
        ["cargo", "package", "-p", "kernel", "--list", "--allow-dirty"],
        cwd=repository,
        check=True,
        capture_output=True,
        text=True,
    )
    metadata = subprocess.run(
        ["cargo", "metadata", "--format-version", "1", "--no-deps", "--locked"],
        cwd=repository,
        check=True,
        capture_output=True,
        text=True,
    )
    validate_package_files(package.stdout.splitlines())
    validate_facade_client(json.loads(metadata.stdout))
    print("kernel package inventory and facade-client dependency boundary are current")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ContractError, subprocess.CalledProcessError, json.JSONDecodeError) as error:
        print(f"package contract failed: {error}", file=sys.stderr)
        sys.exit(1)
