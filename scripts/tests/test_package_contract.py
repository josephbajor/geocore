"""Tests for facade packaging and dependency-boundary contracts."""

import unittest

from scripts.package_contract import (
    ContractError,
    KERNEL_PACKAGE_FILES,
    validate_facade_client,
    validate_package_files,
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


if __name__ == "__main__":
    unittest.main()
