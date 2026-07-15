"""Contract tests for deterministic developer test lanes."""

import unittest

from scripts.test_lanes import (
    EMBEDDED_EXEMPLAR_RATCHETS,
    PRODUCTION_CORPUS_RATCHETS,
    IntegrationTarget,
    LaneContractError,
    classify_targets,
    fast_stages,
    focused_stage,
    format_inventory,
    repository_inventory,
    validate_workspace_packages,
)


class ClassificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.inventory = repository_inventory()

    def test_production_fixture_consumers_are_exact_reviewed_embedded_group(
        self,
    ) -> None:
        self.assertEqual(
            self.inventory.embedded_exemplar_ratchets,
            tuple(sorted(EMBEDDED_EXEMPLAR_RATCHETS)),
        )
        self.assertEqual(len(self.inventory.embedded_exemplar_ratchets), 12)

    def test_slow_group_adds_the_manifest_driven_corpus_ratchet(self) -> None:
        self.assertEqual(
            self.inventory.production_corpus_ratchets,
            tuple(sorted(PRODUCTION_CORPUS_RATCHETS)),
        )
        self.assertEqual(len(self.inventory.production_corpus_ratchets), 13)
        self.assertIn(
            IntegrationTarget("kxt", "corpus_manifest"),
            self.inventory.production_corpus_ratchets,
        )

    def test_fast_and_production_groups_are_an_exact_partition(self) -> None:
        partition = set(self.inventory.fast_targets) | set(
            self.inventory.production_corpus_ratchets
        )
        self.assertEqual(partition, set(self.inventory.all_targets))
        self.assertFalse(
            set(self.inventory.fast_targets)
            & set(self.inventory.production_corpus_ratchets)
        )

    def test_workspace_package_inventory_matches_cargo(self) -> None:
        validate_workspace_packages()

    def test_fast_lane_retains_lightweight_kxt_targets(self) -> None:
        retained = {
            target.target
            for target in self.inventory.fast_targets
            if target.package == "kxt"
        }
        self.assertTrue(
            {
                "import_tess",
                "inspect_cli",
                "intersection_chart",
                "offset_surface",
                "oracle_cli",
                "read",
                "write",
            }.issubset(retained)
        )

    def test_unreviewed_production_fixture_user_fails_closed(self) -> None:
        new_target = IntegrationTarget("kxt", "new_production_ratchet")
        all_targets = self.inventory.all_targets + (new_target,)
        fixture_users = self.inventory.embedded_exemplar_ratchets + (new_target,)
        with self.assertRaisesRegex(LaneContractError, "unreviewed_users"):
            classify_targets(all_targets, fixture_users)

    def test_listing_is_stable_and_names_both_groups(self) -> None:
        first = format_inventory(self.inventory)
        self.assertEqual(first, format_inventory(self.inventory))
        self.assertIn("fast kxt targets retained (7):", first)
        self.assertIn("kxt::read", first)
        self.assertIn("production-corpus ratchets excluded from fast (13):", first)
        self.assertIn("kxt::corpus_manifest", first)
        self.assertIn("embedded exemplar users within that group (12):", first)
        self.assertIn("kxt::finite_open_seven_sample_dual_offset", first)


class CommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.inventory = repository_inventory()

    def test_fast_commands_never_select_a_production_ratchet(self) -> None:
        commands = tuple(stage.command for stage in fast_stages(self.inventory))
        flattened = {argument for command in commands for argument in command}
        for target in PRODUCTION_CORPUS_RATCHETS:
            self.assertNotIn(target.target, flattened)
        self.assertIn("read", flattened)
        self.assertIn("write", flattened)

    def test_focused_target_builds_one_cargo_invocation(self) -> None:
        stage = focused_stage(
            self.inventory,
            package="kxt",
            target="read",
            library=False,
            test_filter="hand_authored_block_text_reconstructs_checker_clean",
            exact=True,
            nocapture=True,
            release=False,
        )
        self.assertEqual(
            stage.command,
            (
                "cargo",
                "test",
                "-p",
                "kxt",
                "--test",
                "read",
                "hand_authored_block_text_reconstructs_checker_clean",
                "--",
                "--exact",
                "--nocapture",
            ),
        )

    def test_focused_target_rejects_typos_before_cargo(self) -> None:
        with self.assertRaisesRegex(LaneContractError, "unknown integration target"):
            focused_stage(
                self.inventory,
                package="kxt",
                target="raed",
                library=False,
                test_filter=None,
                exact=False,
                nocapture=False,
                release=False,
            )


if __name__ == "__main__":
    unittest.main()
