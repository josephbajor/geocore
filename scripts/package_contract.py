#!/usr/bin/env python3
"""Validate the supported facade package and facade-only client boundaries."""

from __future__ import annotations

import json
import re
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
    "examples/boolean_xt_oracle.rs",
    "src/boolean/boundary_select.rs",
    "src/boolean/curved_pipeline.rs",
    "src/boolean/curved_realize.rs",
    "src/boolean/curved_source.rs",
    "src/boolean/curved_support_separation.rs",
    "src/boolean/dispatch.rs",
    "src/boolean/component_layout.rs",
    "src/boolean/components.rs",
    "src/boolean/disk_face_arrangement.rs",
    "src/boolean/extract.rs",
    "src/boolean/face_arrangement.rs",
    "src/boolean/mixed_boundary.rs",
    "src/boolean/mixed_cap_boundary.rs",
    "src/boolean/mixed_face_arrangement.rs",
    "src/boolean/mixed_periodic_arrangement.rs",
    "src/boolean/mixed_shell_components.rs",
    "src/boolean/mixed_shell_materialize.rs",
    "src/boolean/mixed_shell_materialize_tests.rs",
    "src/boolean/mixed_shell_plan.rs",
    "src/boolean/mod.rs",
    "src/boolean/parallel_cylinder_boundary.rs",
    "src/boolean/parallel_cylinder_pipeline.rs",
    "src/boolean/parallel_cylinder_relation.rs",
    "src/boolean/pipeline.rs",
    "src/boolean/planar_bsp.rs",
    "src/boolean/realize.rs",
    "src/boolean/select.rs",
    "src/classify.rs",
    "src/classify/convex.rs",
    "src/classify/curved.rs",
    "src/distance.rs",
    "src/edit.rs",
    "src/error.rs",
    "src/id.rs",
    "src/intersection.rs",
    "src/interchange.rs",
    "src/iter.rs",
    "src/lib.rs",
    "src/operation.rs",
    "src/primitive.rs",
    "src/properties.rs",
    "src/section/broad_phase.rs",
    "src/section/branch_publish.rs",
    "src/section/circle_discovery.rs",
    "src/section/circle_disk_clip.rs",
    "src/section/clip.rs",
    "src/section/closed_stitch.rs",
    "src/section/curved_clip.rs",
    "src/section/curve_publish.rs",
    "src/section/cylinder_cylinder_publish.rs",
    "src/section/disk_clip.rs",
    "src/section/disk_publish.rs",
    "src/section/mixed_stitch.rs",
    "src/section/mod.rs",
    "src/section/periodic_embedding.rs",
    "src/section/root_identity.rs",
    "src/section/ruling_clip.rs",
    "src/section/ruling_public.rs",
    "src/section/ruling_publish.rs",
    "src/section/semantic_ruling.rs",
    "src/section/stitch.rs",
    "src/section/tests.rs",
    "src/section/tests/ruling.rs",
    "src/session.rs",
    "src/tessellation.rs",
    "src/view/body.rs",
    "src/view/boundary.rs",
    "src/view/edge.rs",
    "src/view/geometry.rs",
    "src/view/mod.rs",
    "src/view/part.rs",
    "tests/lifecycle.rs",
    "tests/lifecycle/body_distance.rs",
    "tests/lifecycle/cap_crossing_secant.rs",
    "tests/lifecycle/curved_cavity.rs",
    "tests/lifecycle/curved_constructive_contact.rs",
    "tests/lifecycle/curved_cylinder_cylinder_rulings.rs",
    "tests/lifecycle/curved_inverse_cavity.rs",
    "tests/lifecycle/curved_one_port_budget.rs",
    "tests/lifecycle/curved_plane_cylinder_rulings.rs",
    "tests/lifecycle/mixed_plane_cylinder_cycles.rs",
    "tests/lifecycle/parallel_cylinder_boolean.rs",
    "tests/lifecycle/curved_support_contact.rs",
    "tests/lifecycle/curved_two_port.rs",
    "tests/lifecycle/curved_two_ring_union.rs",
    "src/boolean/axial_contact_adapter.rs",
    "src/boolean/axial_interval_sweep.rs",
    "src/boolean/curved_pipeline_bounded_skew_tests.rs",
    "src/boolean/cylinder_dispatch.rs",
    "src/boolean/cylinder_pair_boundary.rs",
    "src/boolean/mixed_periodic_arrangement/error.rs",
    "src/boolean/mixed_periodic_arrangement/face_local.rs",
    "src/boolean/mixed_periodic_arrangement/source_span.rs",
    "src/boolean/mixed_periodic_arrangement_bounded_procedural_tests.rs",
    "src/boolean/mixed_periodic_arrangement_face_local_tests.rs",
    "src/boolean/mixed_periodic_arrangement_public_section_tests.rs",
    "src/boolean/mixed_periodic_arrangement_returning_tests.rs",
    "src/boolean/mixed_shell_materialize/skew_cylinder.rs",
    "src/boolean/mixed_shell_plan/cylinder_pair.rs",
    "src/boolean/mixed_shell_plan/parallel_cylinder_lens.rs",
    "src/boolean/mixed_shell_plan/projected_source_circle.rs",
    "src/boolean/parallel_cylinder_boundary/coincident_caps.rs",
    "src/boolean/parallel_cylinder_relation/coincident_caps.rs",
    "src/boolean/parallel_cylinder_relation/common_support.rs",
    "src/boolean/parallel_cylinder_relation/internal_tangency.rs",
    "src/boolean/parallel_cylinder_relation/tests.rs",
    "src/boolean/periodic_chart.rs",
    "src/boolean/transverse_cylinder_pipeline.rs",
    "src/section/periodic_embedding/procedural.rs",
    "src/section/periodic_embedding/procedural_tests.rs",
    "src/section/periodic_embedding/test_support.rs",
    "src/section/periodic_embedding/work.rs",
    "src/section/root_identity_quartic.rs",
    "src/section/ruling_disk_clip.rs",
    "src/section/skew_cylinder_fragment.rs",
    "src/section/skew_cylinder_persistence.rs",
    "src/section/skew_cylinder_public.rs",
    "src/section/source_annulus.rs",
    "tests/lifecycle/bounded_skew_body_properties.rs",
    "tests/lifecycle/bounded_skew_contact_roots.rs",
    "tests/lifecycle/bounded_skew_xt.rs",
    "tests/lifecycle/parallel_cylinder_boolean/axial_contact_unite.rs",
    "tests/lifecycle/parallel_cylinder_boolean/coincident_cap_setops.rs",
    "tests/lifecycle/parallel_cylinder_boolean/coincident_caps.rs",
    "tests/lifecycle/parallel_cylinder_boolean/common_support_setops.rs",
    "tests/lifecycle/parallel_cylinder_boolean/internal_tangency_setops.rs",
    "tests/lifecycle/parallel_cylinder_boolean/radial_miss.rs",
    "tests/lifecycle/parallel_cylinder_boolean/radial_miss_setops.rs",
}

LIVE_SHELL_CERTIFIERS = frozenset(
    "certify_whole_closed_surface certify_sphere_cap_shell certify_cylinder_band_shell "
    "certify_cylindrical_host_shell certify_bounded_skew_lobe_shell "
    "certify_mixed_profile_prism certify_cap_reaching_cylinder_shell "
    "certify_two_host_axial_chain_shell certify_portal_cylinder_shell "
    "certify_chord_portal_shell certify_convex_cylindrical_shell "
    "certify_planar_profile_prism certify_convex_planar_shell "
    "certify_semantic_planar_shell_in_scope certify_general_planar_shell_in_scope".split()
)
SPINE_FROZEN_FILENAMES = frozenset(
    """
crates/kernel/src/boolean/mixed_shell_plan/parallel_cylinder_lens.rs crates/kernel/src/boolean/parallel_cylinder_boundary.rs
crates/kernel/src/boolean/parallel_cylinder_pipeline.rs crates/kernel/src/boolean/parallel_cylinder_relation.rs
crates/kernel/src/boolean/transverse_cylinder_pipeline.rs crates/kernel/tests/lifecycle/parallel_cylinder_boolean.rs
crates/kops/tests/parallel_cylinder_radial_relation.rs crates/ktopo/src/bounded_skew_lobe_shell_proof.rs
crates/ktopo/src/cap_reaching_cylinder_shell_proof.rs crates/ktopo/src/chord_portal_shell_proof.rs
crates/ktopo/src/convex_cylindrical_shell_proof.rs crates/ktopo/src/planar_shell_proof.rs
crates/ktopo/src/portal_cylinder_shell_proof.rs crates/ktopo/src/semantic_planar_shell_proof.rs
crates/ktopo/src/two_host_axial_chain_shell_proof.rs
""".split()
)


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


def validate_spine_freeze(
    shell_source: str, mixed_shell_source: str, paths: Iterable[str]
) -> None:
    """Reject growth of shell certification and mixed-shell planning spines."""
    cascade = shell_source[
        shell_source.index("if body_kind != BodyKind::Solid") :
        shell_source.index("\nfn indeterminate()")
    ]
    certifiers = set(re.findall(r"\bcertify_[a-z0-9_]+", cascade))
    frozen = {
        path
        for path in paths
        if Path(path).name.endswith("_shell_proof.rs")
        or Path(path).name.startswith("parallel_cylinder_")
        or Path(path).name.endswith("_cylinder_pipeline.rs")
    }
    new_certifiers = sorted(certifiers - LIVE_SHELL_CERTIFIERS)
    new_filenames = sorted(frozen - SPINE_FROZEN_FILENAMES)
    planners = set(
        re.findall(r"\bfn\s+(plan_[a-z0-9_]*mixed_shell)\b", mixed_shell_source)
    )
    admission = "SectionPlanningAdmission" in mixed_shell_source
    callback = bool(
        re.search(
            r"\b(?:impl|dyn)\s+Fn(?:Once|Mut)?\b"
            r"|\b[A-Z][A-Za-z0-9_]*\s*:\s*Fn(?:Once|Mut)?\b|[:=]\s*fn\s*\(",
            mixed_shell_source,
        )
    )
    if (
        new_certifiers
        or new_filenames
        or planners != {"plan_mixed_shell"}
        or admission
        or callback
    ):
        raise ContractError(
            "spine freeze changed: "
            f"new_certifiers={new_certifiers}, new_filenames={new_filenames}, "
            f"mixed_shell_planners={sorted(planners)}, "
            f"section_planning_admission={admission}, plan_callback={callback}"
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
    mixed_shell_paths = [
        repository / "crates/kernel/src/boolean/mixed_shell_plan.rs",
        *sorted(
            (repository / "crates/kernel/src/boolean/mixed_shell_plan").glob("*.rs")
        ),
    ]
    validate_spine_freeze(
        (repository / "crates/ktopo/src/shell_proof.rs").read_text(),
        "\n".join(
            path.read_text().split("\n#[cfg(test)]\nmod tests", 1)[0]
            for path in mixed_shell_paths
        ),
        (str(path.relative_to(repository)) for path in repository.glob("crates/**/*.rs")),
    )
    print("package, facade-client, and spine-freeze contracts are current")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ContractError, subprocess.CalledProcessError, json.JSONDecodeError) as error:
        print(f"package contract failed: {error}", file=sys.stderr)
        sys.exit(1)
