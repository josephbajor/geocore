"""Contract tests for deterministic developer test lanes."""

import unittest

from scripts.test_lanes import (
    EMBEDDED_EXEMPLAR_RATCHETS,
    EXPECTED_INTEGRATION_TARGET_COUNT,
    EXPECTED_STANDARD_TARGET_COUNT,
    FAST_SMOKE_TARGETS,
    PRODUCTION_CORPUS_RATCHETS,
    IntegrationTarget,
    LaneContractError,
    classify_targets,
    docs_stages,
    fast_stages,
    focused_stage,
    format_inventory,
    full_stages,
    repository_inventory,
    standard_stages,
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
        self.assertEqual(len(self.inventory.embedded_exemplar_ratchets), 13)

    def test_slow_group_adds_the_manifest_driven_corpus_ratchet(self) -> None:
        self.assertEqual(
            self.inventory.production_corpus_ratchets,
            tuple(sorted(PRODUCTION_CORPUS_RATCHETS)),
        )
        self.assertEqual(len(self.inventory.production_corpus_ratchets), 14)
        self.assertIn(
            IntegrationTarget("kxt", "corpus_manifest"),
            self.inventory.production_corpus_ratchets,
        )

    def test_standard_and_production_groups_are_an_exact_partition(self) -> None:
        partition = set(self.inventory.standard_targets) | set(
            self.inventory.production_corpus_ratchets
        )
        self.assertEqual(partition, set(self.inventory.all_targets))
        self.assertFalse(
            set(self.inventory.standard_targets)
            & set(self.inventory.production_corpus_ratchets)
        )

    def test_workspace_package_inventory_matches_cargo(self) -> None:
        validate_workspace_packages()

    def test_standard_lane_retains_lightweight_kxt_targets(self) -> None:
        retained = {
            target.target
            for target in self.inventory.standard_targets
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

    def test_fast_smoke_group_is_exact_reviewed_standard_subset(self) -> None:
        self.assertEqual(
            self.inventory.fast_smoke_targets,
            tuple(sorted(FAST_SMOKE_TARGETS)),
        )
        self.assertTrue(
            set(self.inventory.fast_smoke_targets).issubset(
                self.inventory.standard_targets
            )
        )
        self.assertEqual(len(self.inventory.fast_smoke_targets), 13)
        self.assertEqual(
            len(self.inventory.standard_targets), EXPECTED_STANDARD_TARGET_COUNT
        )
        self.assertEqual(
            len(self.inventory.all_targets), EXPECTED_INTEGRATION_TARGET_COUNT
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
        self.assertIn("fast smoke integration targets (13):", first)
        self.assertIn("standard kxt targets retained (7):", first)
        self.assertIn("kxt::read", first)
        self.assertIn(
            "production-corpus ratchets excluded from standard (14):", first
        )
        self.assertIn("kxt::corpus_manifest", first)
        self.assertIn("embedded exemplar users within that group (13):", first)
        self.assertIn("kxt::finite_open_five_sample_dual_offset", first)
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
        self.assertNotIn("--doc", flattened)
        self.assertIn("unittest", flattened)
        self.assertEqual(
            commands[0],
            ("cargo", "test", "--workspace", "--lib", "--bins"),
        )
        self.assertEqual(
            commands[-1][1:],
            ("-m", "unittest", "scripts.tests.test_test_lanes", "-v"),
        )
        self.assertEqual(
            {
                IntegrationTarget(package, command[index + 1])
                for command in commands
                for index, argument in enumerate(command)
                if argument == "--test"
                for package in (command[command.index("-p") + 1],)
            },
            set(self.inventory.fast_smoke_targets),
        )

    def test_standard_commands_select_every_non_corpus_target_and_tooling(self) -> None:
        commands = tuple(stage.command for stage in standard_stages(self.inventory))
        flattened = {argument for command in commands for argument in command}
        selected = {
            IntegrationTarget(package, command[index + 1])
            for command in commands
            if "-p" in command
            for package in (command[command.index("-p") + 1],)
            for index, argument in enumerate(command)
            if argument == "--test"
        }
        self.assertEqual(selected, set(self.inventory.standard_targets))
        for target in PRODUCTION_CORPUS_RATCHETS:
            self.assertNotIn(target, selected)
        self.assertNotIn("--doc", flattened)
        self.assertIn("unittest", flattened)
        self.assertEqual(
            commands[0],
            ("cargo", "test", "--workspace", "--lib", "--bins"),
        )

    def test_docs_commands_select_only_workspace_documentation(self) -> None:
        commands = tuple(stage.command for stage in docs_stages())
        self.assertEqual(
            commands,
            (("cargo", "test", "--workspace", "--doc"),),
        )

    def test_full_commands_preserve_all_targets_docs_and_tooling(self) -> None:
        commands = tuple(stage.command for stage in full_stages())
        self.assertEqual(
            commands[0], ("cargo", "test", "--workspace", "--all-targets")
        )
        self.assertEqual(
            commands[1], ("cargo", "test", "--workspace", "--doc")
        )
        self.assertEqual(commands[2][1:4], ("-m", "unittest", "discover"))

    def test_release_reaches_every_cargo_stage_in_every_lane(self) -> None:
        lane_stages = (
            fast_stages(self.inventory, release=True),
            standard_stages(self.inventory, release=True),
            docs_stages(release=True),
            full_stages(release=True),
        )
        for stages in lane_stages:
            for stage in stages:
                if stage.command[0] == "cargo":
                    self.assertEqual(stage.command[:3], ("cargo", "test", "--release"))

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
