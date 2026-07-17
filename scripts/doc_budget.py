#!/usr/bin/env python3
"""Enforce ORCHESTRATION.md rule R4: documentation line and cell budgets."""

from __future__ import annotations

import sys
from pathlib import Path


# Every R4 budget lives in this single dict so the limits are easy to review.
# `max_lines` maps a repo-relative path or glob to a per-file line ceiling;
# `max_cell_chars` bounds each delimited cell in the markdown tables and the
# tab-separated ledger; `markdown_table_globs` and `tsv_field_files` name the
# files those cell ceilings apply to.
BUDGETS = {
    "max_lines": {
        "ORCHESTRATION.md": 200,
        "docs/kernel-roadmap.md": 500,
        "docs/projects/*.md": 300,
        "README.md": 120,
    },
    "max_cell_chars": 400,
    "markdown_table_globs": (
        "ORCHESTRATION.md",
        "README.md",
        "docs/*.md",
        "docs/projects/*.md",
    ),
    "tsv_field_files": ("docs/kernel-support.tsv",),
}


class ContractError(RuntimeError):
    """A documentation file grew past its R4 budget."""


def line_budget_violations(label: str, text: str, budget: int) -> list[str]:
    """Return a violation if `text` has more lines than `budget` allows."""
    count = len(text.splitlines())
    if count > budget:
        return [f"{label}: {count} lines exceeds line budget of {budget}"]
    return []


def table_cell_violations(label: str, text: str, budget: int) -> list[str]:
    """Return a violation per markdown table cell longer than `budget` chars."""
    violations = []
    in_fence = False
    for number, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            in_fence = not in_fence
            continue
        if in_fence or not stripped.startswith("|"):
            continue
        body = stripped[1:]
        if body.endswith("|"):
            body = body[:-1]
        for column, cell in enumerate(body.split("|"), start=1):
            # Measure the cell's content, not the conventional `| x |` padding.
            content = cell.strip()
            if len(content) > budget:
                violations.append(
                    f"{label}:{number} cell {column}: "
                    f"{len(content)} characters exceeds cell budget of {budget}"
                )
    return violations


def tsv_field_violations(label: str, text: str, budget: int) -> list[str]:
    """Return a violation per tab-separated field longer than `budget` chars."""
    violations = []
    for number, line in enumerate(text.splitlines(), start=1):
        for column, field in enumerate(line.split("\t"), start=1):
            if len(field) > budget:
                violations.append(
                    f"{label}:{number} field {column}: "
                    f"{len(field)} characters exceeds cell budget of {budget}"
                )
    return violations


def audit_repository(repository: Path) -> list[str]:
    """Return every R4 budget violation across the repository's docs."""
    violations = []

    seen_line: set[Path] = set()
    for pattern, budget in BUDGETS["max_lines"].items():
        for path in sorted(repository.glob(pattern)):
            if not path.is_file() or path in seen_line:
                continue
            seen_line.add(path)
            label = path.relative_to(repository).as_posix()
            text = path.read_text(encoding="utf-8")
            violations += line_budget_violations(label, text, budget)

    cell_budget = BUDGETS["max_cell_chars"]

    seen_cell: set[Path] = set()
    for pattern in BUDGETS["markdown_table_globs"]:
        for path in sorted(repository.glob(pattern)):
            if not path.is_file() or path in seen_cell:
                continue
            seen_cell.add(path)
            label = path.relative_to(repository).as_posix()
            text = path.read_text(encoding="utf-8")
            violations += table_cell_violations(label, text, cell_budget)

    for pattern in BUDGETS["tsv_field_files"]:
        for path in sorted(repository.glob(pattern)):
            if not path.is_file():
                continue
            label = path.relative_to(repository).as_posix()
            text = path.read_text(encoding="utf-8")
            violations += tsv_field_violations(label, text, cell_budget)

    return violations


def main() -> int:
    """Enforce the R4 documentation budgets from the repository root."""
    repository = Path(__file__).resolve().parents[1]
    violations = audit_repository(repository)
    if violations:
        joined = "\n  ".join(violations)
        raise ContractError(
            "documentation budgets (ORCHESTRATION.md R4) exceeded:\n  " + joined
        )
    print("documentation budgets (ORCHESTRATION.md R4) are within limits")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ContractError as error:
        print(f"doc budget contract failed: {error}", file=sys.stderr)
        sys.exit(1)
