#!/usr/bin/env python3
"""Run deterministic developer test lanes with per-stage timing."""

from __future__ import annotations

import argparse
import json
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

PRODUCTION_FIXTURE_NAME = "exemplar.x_t"


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
        "finite_open_five_sample_dual_offset",
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
# production-scale reconstruction work. Keep the mechanism-specific 13-target
# source-reference contract above while excluding all 14 measured slow targets
# here.
PRODUCTION_CORPUS_RATCHETS = tuple(
    sorted(
        (IntegrationTarget("kxt", "corpus_manifest"),)
        + EMBEDDED_EXEMPLAR_RATCHETS
    )
)

# The fast lane is intentionally a representative smoke gate rather than the
# 78-target non-corpus partition. Workspace library/binary tests already carry
# the dense unit-test surface; these integration targets protect the principal
# cross-crate, determinism, topology, completion, interchange, and facade seams.
FAST_SMOKE_TARGETS = tuple(
    sorted(
        (
            IntegrationTarget("kcore", "determinism"),
            IntegrationTarget("kcore", "roadmap_ledger"),
            IntegrationTarget("kgeom", "determinism"),
            IntegrationTarget("kgraph", "intersection_curve_certificate"),
            IntegrationTarget("kernel", "lifecycle"),
            IntegrationTarget("ktopo", "euler_transactions"),
            IntegrationTarget("ktopo", "transactions"),
            IntegrationTarget("kops", "completion"),
            IntegrationTarget("kops", "operation_intersection"),
            IntegrationTarget("kxt", "intersection_chart"),
            IntegrationTarget("kxt", "read"),
            IntegrationTarget("kxt", "write"),
            IntegrationTarget("kernel-lifecycle", "cli"),
        )
    )
)

EXPECTED_INTEGRATION_TARGET_COUNT = 92
EXPECTED_STANDARD_TARGET_COUNT = 78


class LaneContractError(RuntimeError):
    """The reviewed lane classification no longer matches the workspace."""


@dataclass(frozen=True)
class LaneInventory:
    """Deterministic partition of workspace integration tests."""

    all_targets: tuple[IntegrationTarget, ...]
    fast_smoke_targets: tuple[IntegrationTarget, ...]
    standard_targets: tuple[IntegrationTarget, ...]
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
    """Discover Cargo-authoritative integration-test binaries in stable order."""
    return tuple(discover_integration_sources(repository))


def discover_integration_sources(
    repository: Path = REPOSITORY_ROOT,
) -> dict[IntegrationTarget, Path]:
    """Map every workspace Cargo integration target to its declared source."""
    result = subprocess.run(
        ("cargo", "metadata", "--no-deps", "--format-version", "1"),
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise LaneContractError(
            "cargo metadata failed while discovering integration targets: "
            f"{result.stderr.strip()}"
        )
    try:
        metadata = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise LaneContractError(f"cargo metadata returned invalid JSON: {error}") from error

    workspace_package_ids = set(metadata["workspace_members"])
    reviewed_packages = {name for name, _ in WORKSPACE_PACKAGES}
    sources: dict[IntegrationTarget, Path] = {}
    for package in metadata["packages"]:
        if package["id"] not in workspace_package_ids:
            continue
        package_name = package["name"]
        if package_name not in reviewed_packages:
            raise LaneContractError(
                f"unreviewed workspace package in Cargo metadata: {package_name}"
            )
        for cargo_target in package["targets"]:
            if "test" not in cargo_target["kind"]:
                continue
            target = IntegrationTarget(package_name, cargo_target["name"])
            source = Path(cargo_target["src_path"]).resolve()
            if target in sources:
                raise LaneContractError(
                    f"duplicate Cargo integration target: {target.display()}"
                )
            sources[target] = source
    return dict(sorted(sources.items()))


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
    integration_sources: dict[IntegrationTarget, Path] | None = None,
) -> tuple[IntegrationTarget, ...]:
    """Find Cargo targets whose source names the production X_T exemplar."""
    sources = integration_sources or discover_integration_sources(repository)
    source_targets = {source: target for target, source in sources.items()}
    if len(source_targets) != len(sources):
        raise LaneContractError(
            "multiple Cargo integration targets share one source path; "
            "production-fixture ownership is ambiguous"
        )
    users: list[IntegrationTarget] = []
    referenced_sources: set[Path] = set()
    for _, relative_directory in WORKSPACE_PACKAGES:
        tests_directory = repository / relative_directory / "tests"
        if not tests_directory.is_dir():
            continue
        for source in tests_directory.rglob("*.rs"):
            text = source.read_text(encoding="utf-8")
            if PRODUCTION_FIXTURE_NAME in text or (
                "exemplar" in text and ".x_t" in text
            ):
                referenced_sources.add(source.resolve())

    unmapped_sources = tuple(sorted(referenced_sources - set(source_targets)))
    if unmapped_sources:
        raise LaneContractError(
            "production fixture reference is outside a declared Cargo test target: "
            f"{[str(source) for source in unmapped_sources]}"
        )
    for source in sorted(referenced_sources):
        users.append(source_targets[source])
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
    expected_fast_smoke = set(FAST_SMOKE_TARGETS)
    actual = set(production_fixture_users)

    count_drift = (
        len(all_ordered) != EXPECTED_INTEGRATION_TARGET_COUNT
        or len(all_set - expected_slow) != EXPECTED_STANDARD_TARGET_COUNT
    )
    missing_targets = sorted(expected_slow - all_set)
    missing_fast_smoke = sorted(expected_fast_smoke - all_set)
    slow_fast_smoke = sorted(expected_fast_smoke & expected_slow)
    unreviewed_users = sorted(actual - expected_embedded)
    stale_embedded_ratchets = sorted(expected_embedded - actual)
    if (
        count_drift
        or missing_targets
        or missing_fast_smoke
        or slow_fast_smoke
        or unreviewed_users
        or stale_embedded_ratchets
    ):
        raise LaneContractError(
            "production-corpus classification changed: "
            f"target_counts={len(all_ordered)}/{len(all_set - expected_slow)}, "
            "expected_counts="
            f"{EXPECTED_INTEGRATION_TARGET_COUNT}/{EXPECTED_STANDARD_TARGET_COUNT}, "
            f"missing_targets={[target.display() for target in missing_targets]}, "
            "missing_fast_smoke="
            f"{[target.display() for target in missing_fast_smoke]}, "
            f"slow_fast_smoke={[target.display() for target in slow_fast_smoke]}, "
            f"unreviewed_users={[target.display() for target in unreviewed_users]}, "
            "stale_embedded_ratchets="
            f"{[target.display() for target in stale_embedded_ratchets]}"
        )

    return LaneInventory(
        all_targets=all_ordered,
        fast_smoke_targets=tuple(sorted(expected_fast_smoke)),
        standard_targets=tuple(
            target for target in all_ordered if target not in expected_slow
        ),
        production_corpus_ratchets=tuple(sorted(expected_slow)),
        embedded_exemplar_ratchets=tuple(sorted(expected_embedded)),
    )


def repository_inventory(repository: Path = REPOSITORY_ROOT) -> LaneInventory:
    """Return the validated classification for a repository checkout."""
    validate_workspace_packages(repository)
    integration_sources = discover_integration_sources(repository)
    return classify_targets(
        integration_sources,
        discover_production_fixture_users(repository, integration_sources),
    )


def format_inventory(inventory: LaneInventory) -> str:
    """Render the classification in deterministic, reviewable order."""
    kxt_standard = tuple(
        target for target in inventory.standard_targets if target.package == "kxt"
    )
    lines = [
        f"fast smoke integration targets ({len(inventory.fast_smoke_targets)}):",
        *(f"  {target.display()}" for target in inventory.fast_smoke_targets),
        "",
        f"standard non-corpus integration targets ({len(inventory.standard_targets)}):",
        *(f"  {target.display()}" for target in inventory.standard_targets),
        "",
        f"standard kxt targets retained ({len(kxt_standard)}):",
        *(f"  {target.display()}" for target in kxt_standard),
        "",
        "production-corpus ratchets excluded from standard "
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


def _lane_contract_stage() -> Stage:
    return Stage(
        "test-lane contracts",
        (
            sys.executable,
            "-m",
            "unittest",
            "scripts.tests.test_test_lanes",
            "-v",
        ),
    )


def _workspace_and_integration_stages(
    targets: Sequence[IntegrationTarget],
    release: bool,
    lane_name: str,
) -> list[Stage]:
    """Build the shared workspace-unit plus selected-integration prefix."""
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
            for target in targets
            if target.package == package
        )
        if not package_targets:
            continue
        command = base + ["-p", package]
        for target in package_targets:
            command.extend(("--test", target))
        stages.append(Stage(f"{package} {lane_name} integration tests", tuple(command)))
    return stages


def fast_stages(inventory: LaneInventory, release: bool = False) -> tuple[Stage, ...]:
    """Build the curated inner-loop smoke gate."""
    stages = _workspace_and_integration_stages(
        inventory.fast_smoke_targets,
        release,
        "fast-smoke",
    )
    stages.append(_lane_contract_stage())
    return tuple(stages)


def standard_stages(
    inventory: LaneInventory, release: bool = False
) -> tuple[Stage, ...]:
    """Build the broad code/tooling gate without production-corpus ratchets."""
    stages = _workspace_and_integration_stages(
        inventory.standard_targets,
        release,
        "standard",
    )
    stages.append(_tooling_contract_stage())
    return tuple(stages)


def docs_stages(release: bool = False) -> tuple[Stage, ...]:
    """Build the explicit workspace documentation gate."""
    base = _cargo_test_base(release)
    return (
        Stage(
            "workspace documentation tests",
            tuple(base + ["--workspace", "--doc"]),
        ),
    )


def full_stages(release: bool = False) -> tuple[Stage, ...]:
    """Build the pre-merge lane, including all production-corpus ratchets."""
    base = _cargo_test_base(release)
    return (
        Stage(
            "all workspace targets",
            tuple(base + ["--workspace", "--all-targets"]),
        ),
        *docs_stages(release),
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
        "list", help="list the reviewed integration-target classification"
    )

    fast = subparsers.add_parser(
        "fast", help="run workspace unit tests and a curated integration smoke set"
    )
    _add_execution_flags(fast)

    standard = subparsers.add_parser(
        "standard",
        help="run every non-corpus target plus tooling contracts",
    )
    _add_execution_flags(standard)

    docs = subparsers.add_parser(
        "docs", help="run every workspace documentation test"
    )
    _add_execution_flags(docs)

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
        if arguments.lane == "standard":
            return run_stages(
                standard_stages(inventory, release=arguments.release),
                dry_run=arguments.dry_run,
            )
        if arguments.lane == "docs":
            return run_stages(
                docs_stages(release=arguments.release),
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
