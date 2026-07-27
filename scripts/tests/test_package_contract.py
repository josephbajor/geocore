"""Tests for facade packaging and dependency-boundary contracts."""

import unittest

from scripts.package_contract import (
    ContractError,
    KERNEL_PACKAGE_FILES,
    SPINE_FROZEN_FILENAMES,
    validate_facade_client,
    validate_package_files,
    validate_spine_freeze,
)


class PackageInventoryTests(unittest.TestCase):
    def test_exact_reviewed_inventory_passes(self) -> None:
        validate_package_files(sorted(KERNEL_PACKAGE_FILES))

    def test_missing_or_unexpected_files_fail(self) -> None:
        with self.assertRaisesRegex(ContractError, "missing=.*README.md"):
            validate_package_files(KERNEL_PACKAGE_FILES - {"README.md"})
        with self.assertRaisesRegex(ContractError, "unexpected=.*raw-fixture"):
            validate_package_files(KERNEL_PACKAGE_FILES | {"raw-fixture.x_t"})


class FacadeClientDependencyTests(unittest.TestCase):
    @staticmethod
    def metadata(dependencies: list[dict[str, object]]) -> dict[str, object]:
        return {
            "packages": [
                {"name": "kernel-lifecycle", "dependencies": dependencies},
                {"name": "kernel", "dependencies": []},
            ]
        }

    def test_kernel_is_the_only_direct_dependency(self) -> None:
        validate_facade_client(self.metadata([{"name": "kernel", "kind": None}]))

    def test_lower_layer_or_development_dependency_fails(self) -> None:
        with self.assertRaisesRegex(ContractError, "normal=.*ktopo"):
            validate_facade_client(
                self.metadata(
                    [
                        {"name": "kernel", "kind": None},
                        {"name": "ktopo", "kind": None},
                    ]
                )
            )
        with self.assertRaisesRegex(ContractError, "non_normal=.*kxt"):
            validate_facade_client(
                self.metadata(
                    [
                        {"name": "kernel", "kind": None},
                        {"name": "kxt", "kind": "dev"},
                    ]
                )
            )


class SpineFreezeTests(unittest.TestCase):
    SOURCE = """fn certify_shell_impl() {
    if body_kind != BodyKind::Solid {}
    certify_whole_closed_surface();
    certify_cylindrical_host_shell();
}
fn indeterminate() {}
"""
    MIXED_SHELL_SOURCE = "pub(crate) fn plan_mixed_shell() {}\n"

    def test_reviewed_certifiers_and_filenames_pass(self) -> None:
        validate_spine_freeze(
            self.SOURCE, self.MIXED_SHELL_SOURCE, SPINE_FROZEN_FILENAMES
        )

    def test_new_certifier_or_frozen_filename_fails(self) -> None:
        source = self.SOURCE.replace(
            "\nfn indeterminate()", "\n    certify_new_shell();\nfn indeterminate()"
        )
        paths = SPINE_FROZEN_FILENAMES | {
            "crates/kernel/src/boolean/parallel_cylinder_new.rs"
        }
        with self.assertRaisesRegex(
            ContractError, "new_certifiers=.*certify_new_shell.*new_filenames="
        ):
            validate_spine_freeze(source, self.MIXED_SHELL_SOURCE, paths)

    def test_mixed_shell_admission_extra_planner_and_callback_fail(self) -> None:
        source = """enum SectionPlanningAdmission { CoincidentCaps }
fn plan_mixed_shell() {}
fn plan_internal_tangency_mixed_shell() {}
fn finish_shell(hook: impl FnOnce(&mut Vec<MixedShellFacePlan>)) {}
"""
        with self.assertRaisesRegex(
            ContractError,
            "mixed_shell_planners=.*plan_internal_tangency_mixed_shell"
            ".*section_planning_admission=True.*plan_callback=True",
        ):
            validate_spine_freeze(self.SOURCE, source, SPINE_FROZEN_FILENAMES)

    def test_missing_mixed_shell_planner_fails(self) -> None:
        with self.assertRaisesRegex(ContractError, "mixed_shell_planners=\\[\\]"):
            validate_spine_freeze(self.SOURCE, "", SPINE_FROZEN_FILENAMES)


if __name__ == "__main__":
    unittest.main()
