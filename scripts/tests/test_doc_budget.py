"""Tests for the ORCHESTRATION.md R4 documentation-budget contract."""

import tempfile
import unittest
from pathlib import Path

from scripts.doc_budget import (
    BUDGETS,
    ContractError,
    audit_repository,
    line_budget_violations,
    table_cell_violations,
    tsv_field_violations,
)


class LineBudgetTests(unittest.TestCase):
    def test_file_at_budget_passes(self) -> None:
        self.assertEqual(line_budget_violations("f.md", "x\n" * 200, 200), [])

    def test_file_over_budget_reports_actual_and_budget(self) -> None:
        (violation,) = line_budget_violations("f.md", "x\n" * 201, 200)
        self.assertIn("f.md", violation)
        self.assertIn("201 lines", violation)
        self.assertIn("budget of 200", violation)

    def test_final_line_without_newline_counts(self) -> None:
        self.assertEqual(line_budget_violations("f.md", "a\nb\nc", 3), [])
        self.assertEqual(len(line_budget_violations("f.md", "a\nb\nc\nd", 3)), 1)


class TableCellTests(unittest.TestCase):
    def test_cell_at_budget_passes(self) -> None:
        row = f"| {'a' * 400} |"
        self.assertEqual(table_cell_violations("d.md", row, 400), [])

    def test_cell_over_budget_names_line_column_and_counts(self) -> None:
        row = f"| ok | {'a' * 401} |"
        (violation,) = table_cell_violations("d.md", f"intro\n{row}", 400)
        self.assertIn("d.md:2", violation)
        self.assertIn("cell 2", violation)
        self.assertIn("401 characters", violation)
        self.assertIn("budget of 400", violation)

    def test_non_table_and_separator_lines_are_ignored(self) -> None:
        prose = "This is a plain sentence with a | pipe but no leading pipe."
        separator = "| --- | --- |"
        long_prose = "prose " * 200  # >400 chars, but not a table row
        text = "\n".join([prose, separator, long_prose])
        self.assertEqual(table_cell_violations("d.md", text, 400), [])

    def test_fenced_code_block_rows_are_ignored(self) -> None:
        fenced = "\n".join(["```", f"| {'a' * 500} |", "```"])
        self.assertEqual(table_cell_violations("d.md", fenced, 400), [])

    def test_row_without_trailing_pipe_is_measured(self) -> None:
        row = f"| {'a' * 401}"
        (violation,) = table_cell_violations("d.md", row, 400)
        self.assertIn("cell 1", violation)


class TsvFieldTests(unittest.TestCase):
    def test_field_at_budget_passes(self) -> None:
        line = "\t".join(["ok", "a" * 400])
        self.assertEqual(tsv_field_violations("t.tsv", line, 400), [])

    def test_field_over_budget_names_line_field_and_counts(self) -> None:
        line = "\t".join(["ok", "a" * 401])
        (violation,) = tsv_field_violations("t.tsv", f"header\n{line}", 400)
        self.assertIn("t.tsv:2", violation)
        self.assertIn("field 2", violation)
        self.assertIn("401 characters", violation)
        self.assertIn("budget of 400", violation)


class AuditRepositoryTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self._tmp.name)
        (self.repo / "docs" / "projects").mkdir(parents=True)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def write(self, rel: str, text: str) -> None:
        path = self.repo / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def build_compliant_repo(self) -> None:
        self.write("ORCHESTRATION.md", "line\n" * 200)
        self.write("README.md", "line\n" * 120)
        self.write("docs/kernel-roadmap.md", "line\n" * 500)
        self.write("docs/kernel-spec.md", "line\n" * 50)
        self.write("docs/projects/alpha.md", "line\n" * 300)
        self.write("docs/projects/beta.md", "line\n" * 10)
        self.write("docs/kernel-support.tsv", "cap\tstatus\tevidence\n")

    def test_compliant_repo_has_no_violations(self) -> None:
        self.build_compliant_repo()
        self.assertEqual(audit_repository(self.repo), [])

    def test_each_line_budget_is_enforced(self) -> None:
        self.build_compliant_repo()
        self.write("ORCHESTRATION.md", "line\n" * 201)
        self.write("README.md", "line\n" * 121)
        self.write("docs/kernel-roadmap.md", "line\n" * 501)
        self.write("docs/projects/alpha.md", "line\n" * 301)
        violations = audit_repository(self.repo)
        joined = "\n".join(violations)
        self.assertIn("ORCHESTRATION.md: 201 lines", joined)
        self.assertIn("README.md: 121 lines", joined)
        self.assertIn("docs/kernel-roadmap.md: 501 lines", joined)
        self.assertIn("docs/projects/alpha.md: 301 lines", joined)
        self.assertEqual(len(violations), 4)

    def test_markdown_cell_budget_spans_all_target_files(self) -> None:
        self.build_compliant_repo()
        over = f"| {'a' * 401} |"
        self.write("ORCHESTRATION.md", "\n".join(["intro", over]))
        self.write("README.md", "\n".join(["intro", over]))
        self.write("docs/kernel-spec.md", "\n".join(["intro", over]))
        self.write("docs/projects/beta.md", "\n".join(["intro", over]))
        violations = audit_repository(self.repo)
        labels = {v.split(":", 1)[0] for v in violations}
        self.assertEqual(
            labels,
            {
                "ORCHESTRATION.md",
                "README.md",
                "docs/kernel-spec.md",
                "docs/projects/beta.md",
            },
        )

    def test_tsv_field_budget_is_enforced(self) -> None:
        self.build_compliant_repo()
        self.write(
            "docs/kernel-support.tsv",
            "cap\tstatus\tevidence\ncap\tstatus\t" + "a" * 401,
        )
        violations = audit_repository(self.repo)
        self.assertEqual(len(violations), 1)
        self.assertIn("docs/kernel-support.tsv:2 field 3", violations[0])

    def test_all_violations_are_reported_together(self) -> None:
        self.build_compliant_repo()
        self.write("ORCHESTRATION.md", "line\n" * 201)
        self.write("docs/kernel-support.tsv", "cap\t" + "a" * 401)
        violations = audit_repository(self.repo)
        self.assertEqual(len(violations), 2)


class ContractSurfaceTests(unittest.TestCase):
    def test_contract_error_is_runtime_error(self) -> None:
        self.assertTrue(issubclass(ContractError, RuntimeError))

    def test_budgets_are_a_single_reviewable_dict(self) -> None:
        self.assertIsInstance(BUDGETS, dict)
        self.assertEqual(BUDGETS["max_lines"]["ORCHESTRATION.md"], 200)
        self.assertEqual(BUDGETS["max_lines"]["docs/kernel-roadmap.md"], 500)
        self.assertEqual(BUDGETS["max_lines"]["docs/projects/*.md"], 300)
        self.assertEqual(BUDGETS["max_lines"]["README.md"], 120)
        self.assertEqual(BUDGETS["max_cell_chars"], 400)


if __name__ == "__main__":
    unittest.main()
