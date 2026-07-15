#!/usr/bin/env python3
"""Run deterministic developer test lanes with per-stage timing."""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
import time
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

# Keep this ordered tuple synchronized with the root workspace membership. It
# makes target discovery independent of filesystem iteration order. The
# standard-library TOML ratchet below fails closed if membership or package
# identity drifts.
WORKSPACE_PACKAGES = (
    ("kcore", Path("crates/kcore")),
    ("kgeom", Path("crates/kgeom")),
    ("kgraph", Path("crates/kgraph")),
    ("kernel", Path("crates/kernel")),
    ("ktopo", Path("crates/ktopo")),
    ("kops", Path("crates/kops")),
    ("kxt", Path("crates/kxt")),
    ("kernel-lifecycle", Path("examples/kernel-lifecycle")),
)

PRODUCTION_FIXTURE_MARKER = 'include_bytes!("fixtures/exemplar.x_t")'


@dataclass(frozen=True, order=True)
class IntegrationTarget:
    """One Cargo integration-test binary."""

    package: str
    target: str

    def display(self) -> str:
        """Return the stable developer-facing target identity."""
        return f"{self.package}::{self.target}"


# These are slow because each binary embeds and reconstructs the 908 KiB,
# 7,423-node production exemplar. The classification contract below rejects a
# newly added exemplar consumer until this reviewed list is updated.
EMBEDDED_EXEMPLAR_RATCHETS = tuple(
    IntegrationTarget("kxt", target)
    for target in (
        "equal_limit_intersection",
        "finite_open_cubic_dual_offset",
        "finite_open_nurbs_endpoint_roundoff",
        "finite_open_plane_nurbs_data",
        "finite_open_plane_offset_nurbs_data",
        "finite_open_seven_sample_dual_offset",
        "finite_open_two_sample_dual_offset",
        "offset_nurbs_intersection",
        "periodic_nurbs",
        "plane_sp_curve",
        "terminated_intersection",
        "zero_multiplicity_knot_padding",
    )
)

# `corpus_manifest` reads the production fixture through the manifest rather
# than embedding it, but its observed-corpus-stage contract performs the same
# production-scale reconstruction work. Keep the mechanism-specific 12-target
# marker contract above while excluding all 13 measured slow targets here.
PRODUCTION_CORPUS_RATCHETS = tuple(
    sorted(
        (IntegrationTarget("kxt", "corpus_manifest"),)
        + EMBEDDED_EXEMPLAR_RATCHETS
    )
)


class LaneContractError(RuntimeError):
    """The reviewed fast/full classification no longer matches the workspace."""


@dataclass(frozen=True)
class LaneInventory:
    """Deterministic partition of workspace integration tests."""

    all_targets: tuple[IntegrationTarget, ...]
    fast_targets: tuple[IntegrationTarget, ...]
    production_corpus_ratchets: tuple[IntegrationTarget, ...]
    embedded_exemplar_ratchets: tuple[IntegrationTarget, ...]


@dataclass(frozen=True)
class Stage:
    """One timed subprocess in a test lane."""

    label: str
    command: tuple[str, ...]


def discover_integration_targets(
    repository: Path = REPOSITORY_ROOT,
) -> tuple[IntegrationTarget, ...]:
    """Discover workspace integration-test binaries in stable order."""
    targets: list[IntegrationTarget] = []
    for package, relative_directory in WORKSPACE_PACKAGES:
        tests_directory = repository / relative_directory / "tests"
        if not tests_directory.is_dir():
            continue
        targets.extend(
            IntegrationTarget(package, source.stem)
            for source in sorted(tests_directory.glob("*.rs"))
        )
    return tuple(sorted(targets))


def validate_workspace_packages(repository: Path = REPOSITORY_ROOT) -> None:
    """Fail if the reviewed package inventory drifts from Cargo metadata."""
    workspace = tomllib.loads(
        (repository / "Cargo.toml").read_text(encoding="utf-8")
    )
    declared_members = tuple(Path(member) for member in workspace["workspace"]["members"])
    reviewed_members = tuple(path for _, path in WORKSPACE_PACKAGES)
    if len(set(declared_members)) != len(declared_members) or set(declared_members) != set(
        reviewed_members
    ):
        raise LaneContractError(
            "workspace package classification changed: "
            f"declared={[str(path) for path in declared_members]}, "
            f"reviewed={[str(path) for path in reviewed_members]}"
        )

    for reviewed_name, relative_directory in WORKSPACE_PACKAGES:
        package = tomllib.loads(
            (repository / relative_directory / "Cargo.toml").read_text(encoding="utf-8")
        )
        declared_name = package["package"]["name"]
        if declared_name != reviewed_name:
            raise LaneContractError(
                "workspace package identity changed: "
                f"path={relative_directory}, declared={declared_name}, "
                f"reviewed={reviewed_name}"
            )


def discover_production_fixture_users(
    repository: Path = REPOSITORY_ROOT,
) -> tuple[IntegrationTarget, ...]:
    """Find integration targets that embed the production X_T exemplar."""
    users: list[IntegrationTarget] = []
    for target in discover_integration_targets(repository):
        package_directory = dict(WORKSPACE_PACKAGES)[target.package]
        source = repository / package_directory / "tests" / f"{target.target}.rs"
        if PRODUCTION_FIXTURE_MARKER in source.read_text(encoding="utf-8"):
            users.append(target)
    return tuple(sorted(users))


def classify_targets(
    all_targets: Iterable[IntegrationTarget],
    production_fixture_users: Iterable[IntegrationTarget],
) -> LaneInventory:
    """Validate and partition targets into fast and production-corpus groups."""
    all_ordered = tuple(sorted(set(all_targets)))
    all_set = set(all_ordered)
    expected_slow = set(PRODUCTION_CORPUS_RATCHETS)
    expected_embedded = set(EMBEDDED_EXEMPLAR_RATCHETS)
    actual = set(production_fixture_users)

    missing_targets = sorted(expected_slow - all_set)
    unreviewed_users = sorted(actual - expected_embedded)
    stale_embedded_ratchets = sorted(expected_embedded - actual)
    if missing_targets or unreviewed_users or stale_embedded_ratchets:
        raise LaneContractError(
            "production-corpus classification changed: "
            f"missing_targets={[target.display() for target in missing_targets]}, "
            f"unreviewed_users={[target.display() for target in unreviewed_users]}, "
            "stale_embedded_ratchets="
            f"{[target.display() for target in stale_embedded_ratchets]}"
        )

    return LaneInventory(
        all_targets=all_ordered,
        fast_targets=tuple(
            target for target in all_ordered if target not in expected_slow
        ),
        production_corpus_ratchets=tuple(sorted(expected_slow)),
        embedded_exemplar_ratchets=tuple(sorted(expected_embedded)),
    )


def repository_inventory(repository: Path = REPOSITORY_ROOT) -> LaneInventory:
    """Return the validated classification for a repository checkout."""
    validate_workspace_packages(repository)
    return classify_targets(
        discover_integration_targets(repository),
        discover_production_fixture_users(repository),
    )


def format_inventory(inventory: LaneInventory) -> str:
    """Render the classification in deterministic, reviewable order."""
    kxt_fast = tuple(
        target for target in inventory.fast_targets if target.package == "kxt"
    )
    lines = [
        f"fast integration targets ({len(inventory.fast_targets)}):",
        *(f"  {target.display()}" for target in inventory.fast_targets),
        "",
        f"fast kxt targets retained ({len(kxt_fast)}):",
        *(f"  {target.display()}" for target in kxt_fast),
        "",
        "production-corpus ratchets excluded from fast "
        f"({len(inventory.production_corpus_ratchets)}):",
        *(
            f"  {target.display()}"
            for target in inventory.production_corpus_ratchets
        ),
        "",
        "embedded exemplar users within that group "
        f"({len(inventory.embedded_exemplar_ratchets)}):",
        *(
            f"  {target.display()}"
            for target in inventory.embedded_exemplar_ratchets
        ),
        "",
        f"full integration targets ({len(inventory.all_targets)}): all of the above",
    ]
    return "\n".join(lines)


def _cargo_test_base(release: bool) -> list[str]:
    command = ["cargo", "test"]
    if release:
        command.append("--release")
    return command


def _tooling_contract_stage() -> Stage:
    return Stage(
        "Python tooling contracts",
        (
            sys.executable,
            "-m",
            "unittest",
            "discover",
            "-s",
            "scripts/tests",
            "-v",
        ),
    )


def fast_stages(inventory: LaneInventory, release: bool = False) -> tuple[Stage, ...]:
    """Build fast-lane stages without selecting production-corpus ratchets."""
    base = _cargo_test_base(release)
    stages = [
        Stage(
            "workspace library and binary tests",
            tuple(base + ["--workspace", "--lib", "--bins"]),
        ),
    ]

    for package, _ in WORKSPACE_PACKAGES:
        package_targets = tuple(
            target.target
            for target in inventory.fast_targets
            if target.package == package
        )
        if not package_targets:
            continue
        command = base + ["-p", package]
        for target in package_targets:
            command.extend(("--test", target))
        stages.append(Stage(f"{package} fast integration tests", tuple(command)))

    stages.extend(
        (
            Stage(
                "workspace documentation tests",
                tuple(base + ["--workspace", "--doc"]),
            ),
            _tooling_contract_stage(),
        )
    )
    return tuple(stages)


def full_stages(release: bool = False) -> tuple[Stage, ...]:
    """Build the pre-merge lane, including all production-corpus ratchets."""
    base = _cargo_test_base(release)
    return (
        Stage(
            "all workspace targets",
            tuple(base + ["--workspace", "--all-targets"]),
        ),
        Stage(
            "workspace documentation tests",
            tuple(base + ["--workspace", "--doc"]),
        ),
        _tooling_contract_stage(),
    )


def focused_stage(
    inventory: LaneInventory,
    package: str,
    target: str | None,
    library: bool,
    test_filter: str | None,
    exact: bool,
    nocapture: bool,
    release: bool,
) -> Stage:
    """Build a single-package, single-target inner-loop invocation."""
    known_packages = {name for name, _ in WORKSPACE_PACKAGES}
    if package not in known_packages:
        raise LaneContractError(f"unknown workspace package: {package}")
    if library == (target is not None):
        raise LaneContractError("focused lane requires exactly one of --lib or --test")
    if exact and test_filter is None:
        raise LaneContractError("--exact requires --filter")

    command = _cargo_test_base(release) + ["-p", package]
    label: str
    if library:
        command.append("--lib")
        label = f"{package} library tests"
    else:
        integration_target = IntegrationTarget(package, target or "")
        if integration_target not in inventory.all_targets:
            raise LaneContractError(
                f"unknown integration target: {integration_target.display()}"
            )
        command.extend(("--test", integration_target.target))
        label = f"{integration_target.display()} focused tests"

    if test_filter is not None:
        command.append(test_filter)
    harness_arguments = []
    if exact:
        harness_arguments.append("--exact")
    if nocapture:
        harness_arguments.append("--nocapture")
    if harness_arguments:
        command.append("--")
        command.extend(harness_arguments)
    return Stage(label, tuple(command))


def run_stages(stages: Sequence[Stage], dry_run: bool = False) -> int:
    """Run stages in order, reporting command and elapsed wall time."""
    lane_started = time.monotonic()
    for index, stage in enumerate(stages, start=1):
        print(f"[test-lane] START {index}/{len(stages)} {stage.label}", flush=True)
        print(f"[test-lane] $ {shlex.join(stage.command)}", flush=True)
        if dry_run:
            print(f"[test-lane] DRY-RUN {stage.label}", flush=True)
            continue

        started = time.monotonic()
        try:
            result = subprocess.run(stage.command, cwd=REPOSITORY_ROOT, check=False)
        except KeyboardInterrupt:
            elapsed = time.monotonic() - started
            print(
                f"[test-lane] INTERRUPTED {stage.label} ({elapsed:.3f}s)", flush=True
            )
            return 130
        elapsed = time.monotonic() - started
        status = "PASS" if result.returncode == 0 else "FAIL"
        print(f"[test-lane] {status} {stage.label} ({elapsed:.3f}s)", flush=True)
        if result.returncode != 0:
            total = time.monotonic() - lane_started
            print(f"[test-lane] FAIL total ({total:.3f}s)", flush=True)
            return result.returncode

    total = time.monotonic() - lane_started
    qualifier = " DRY-RUN" if dry_run else ""
    print(f"[test-lane]{qualifier} PASS total ({total:.3f}s)", flush=True)
    return 0


def _add_execution_flags(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--release",
        action="store_true",
        help="run the selected Rust tests with Cargo's release profile",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the exact staged commands without executing them",
    )


def build_parser() -> argparse.ArgumentParser:
    """Build the developer CLI parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="lane", required=True)

    subparsers.add_parser(
        "list", help="list the reviewed fast/full integration-test classification"
    )

    fast = subparsers.add_parser(
        "fast", help="run all ordinary tests except named production-corpus ratchets"
    )
    _add_execution_flags(fast)

    full = subparsers.add_parser(
        "full", help="run every workspace target, doc test, and tooling contract"
    )
    _add_execution_flags(full)

    focused = subparsers.add_parser(
        "focused", help="run one package library or one integration-test binary"
    )
    focused.add_argument("-p", "--package", required=True)
    selection = focused.add_mutually_exclusive_group(required=True)
    selection.add_argument("--lib", action="store_true")
    selection.add_argument("-t", "--test")
    focused.add_argument("--filter", help="Cargo test-name substring filter")
    focused.add_argument(
        "--exact", action="store_true", help="require an exact harness filter match"
    )
    focused.add_argument(
        "--nocapture", action="store_true", help="show test process output"
    )
    _add_execution_flags(focused)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Validate classification and execute the requested lane."""
    arguments = build_parser().parse_args(argv)
    try:
        inventory = repository_inventory()
        if arguments.lane == "list":
            print(format_inventory(inventory))
            return 0
        if arguments.lane == "fast":
            return run_stages(
                fast_stages(inventory, release=arguments.release),
                dry_run=arguments.dry_run,
            )
        if arguments.lane == "full":
            return run_stages(
                full_stages(release=arguments.release),
                dry_run=arguments.dry_run,
            )
        if arguments.lane == "focused":
            stage = focused_stage(
                inventory,
                package=arguments.package,
                target=arguments.test,
                library=arguments.lib,
                test_filter=arguments.filter,
                exact=arguments.exact,
                nocapture=arguments.nocapture,
                release=arguments.release,
            )
            return run_stages((stage,), dry_run=arguments.dry_run)
    except LaneContractError as error:
        print(f"test-lane contract failed: {error}", file=sys.stderr)
        return 2
    raise AssertionError(f"unhandled lane: {arguments.lane}")


if __name__ == "__main__":
    sys.exit(main())
