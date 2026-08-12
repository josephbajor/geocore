#!/usr/bin/env python3
"""Audit the persistent fail-closed frontier census.

The census is deliberately hand-classified, but its source inventory is not.
This script discovers the refusal-shaped Rust and ledger declarations that
must be acknowledged by docs/projects/refusal-frontier.md.  Each discovered
key must appear exactly once as either ``frontier-key`` or ``contract-key``.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CENSUS = ROOT / "docs/projects/refusal-frontier.md"

ENUM_NAME_PATTERN = re.compile(r"\benum\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{")
CONST_PATTERN = re.compile(
    r"\b(?:pub(?:\([^)]*\))?\s+)?const\s+"
    r"([A-Z][A-Z0-9_]*(?:INCOMPLETE|INDETERMINATE|UNSUPPORTED)[A-Z0-9_]*)\s*:"
)
MARKER_PATTERN = re.compile(r"<!--\s*(frontier|contract)-key:\s*([^>]+?)\s*-->")
SECTION_GAP_PATTERN = re.compile(r"\b(?:pub\(crate\)\s+)?const\s+(GAP_[A-Z0-9_]+)\s*:")
VARIANT_PATTERN = re.compile(r"^\s*([A-Z][A-Za-z0-9_]*)\b")

FULL_ENUMS = {
    "BooleanOperandUnsupportedReason",
    "BooleanOperandProofFailure",
    "BooleanRefusal",
    "IncompleteCause",
    "SectionPeriodicEmbeddingGap",
    "XtCapability",
}

SELECTED_ENUMS = {
    "CheckOutcome",
    "ClosedStitchCompletion",
    "Completion",
    "IntersectionCompletion",
    "IntersectionError",
    "RulingCertificationOutcome",
    "SectionCompletion",
    "TessellationError",
    "XtError",
}

SELECTED_VARIANT = re.compile(r"^(?:Incomplete|Indeterminate|Unsupported)")
LEDGER_PATTERN = re.compile(
    r"\b(?:refus\w*|unsupported|incomplete|indeterminate|missing|unavailable|"
    r"gap|proof|evidence|certif\w*|replay)\b",
    re.IGNORECASE,
)


@dataclass(frozen=True, order=True)
class InventoryItem:
    key: str
    source: str


def strip_comments_and_strings(source: str) -> str:
    """Preserve layout while blanking Rust comments and string literals."""

    output = list(source)
    index = 0
    block_depth = 0
    in_string = False
    in_char = False
    escaped = False
    while index < len(source):
        pair = source[index : index + 2]
        if block_depth:
            output[index] = " " if source[index] != "\n" else "\n"
            if pair == "/*":
                output[index + 1] = " "
                block_depth += 1
                index += 2
                continue
            if pair == "*/":
                output[index + 1] = " "
                block_depth -= 1
                index += 2
                continue
            index += 1
            continue
        if in_string or in_char:
            delimiter = '"' if in_string else "'"
            output[index] = " " if source[index] != "\n" else "\n"
            if escaped:
                escaped = False
            elif source[index] == "\\":
                escaped = True
            elif source[index] == delimiter:
                in_string = False
                in_char = False
            index += 1
            continue
        if pair == "//":
            end = source.find("\n", index)
            if end < 0:
                end = len(source)
            output[index:end] = " " * (end - index)
            index = end
            continue
        if pair == "/*":
            output[index : index + 2] = [" ", " "]
            block_depth = 1
            index += 2
            continue
        if source[index] == '"':
            output[index] = " "
            in_string = True
        elif source[index] == "'" and not _looks_like_lifetime(source, index):
            output[index] = " "
            in_char = True
        index += 1
    return "".join(output)


def _looks_like_lifetime(source: str, index: int) -> bool:
    if index + 1 >= len(source) or not (source[index + 1].isalpha() or source[index + 1] == "_"):
        return False
    cursor = index + 2
    while cursor < len(source) and (source[cursor].isalnum() or source[cursor] == "_"):
        cursor += 1
    return cursor >= len(source) or source[cursor] != "'"


def matching_brace(source: str, opening: int) -> int:
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    raise ValueError(f"unclosed enum brace at byte {opening}")


def enum_variants(body: str) -> list[str]:
    variants: list[str] = []
    depth = 0
    for line in body.splitlines():
        if depth == 0:
            match = VARIANT_PATTERN.match(line)
            if match and match.group(1) not in {"pub", "where"}:
                variants.append(match.group(1))
        depth += line.count("{") + line.count("(") + line.count("[")
        depth -= line.count("}") + line.count(")") + line.count("]")
    return variants


def rust_inventory() -> set[InventoryItem]:
    items: set[InventoryItem] = set()
    for path in sorted((ROOT / "crates").glob("*/src/**/*.rs")):
        raw = path.read_text(encoding="utf-8")
        source = strip_comments_and_strings(raw)
        relative = path.relative_to(ROOT).as_posix()
        for name in CONST_PATTERN.findall(source):
            items.add(InventoryItem(f"diagnostic::{name}", relative))
        if relative == "crates/kernel/src/section/mod.rs":
            for name in SECTION_GAP_PATTERN.findall(source):
                items.add(InventoryItem(f"section-gap::{name}", relative))
        for match in ENUM_NAME_PATTERN.finditer(source):
            enum = match.group(1)
            opening = source.find("{", match.start())
            closing = matching_brace(source, opening)
            variants = enum_variants(source[opening + 1 : closing])
            include_all = enum.endswith(("Refusal", "Gap")) or enum in FULL_ENUMS
            if not include_all and enum not in SELECTED_ENUMS:
                variants = [variant for variant in variants if SELECTED_VARIANT.match(variant)]
            for variant in variants:
                if include_all or SELECTED_VARIANT.match(variant):
                    items.add(InventoryItem(f"variant::{enum}::{variant}", relative))
    return items


def ledger_inventory() -> set[InventoryItem]:
    items: set[InventoryItem] = set()
    path = ROOT / "docs/kernel-support.tsv"
    rows = path.read_text(encoding="utf-8").splitlines()
    for line_number, row in enumerate(rows[1:], start=2):
        fields = row.split("\t")
        if len(fields) != 7:
            raise ValueError(f"{path}:{line_number}: expected seven TSV fields")
        capability, next_step = fields[0], fields[6]
        if LEDGER_PATTERN.search(next_step):
            items.add(
                InventoryItem(
                    f"ledger::{capability}",
                    f"docs/kernel-support.tsv:{line_number}",
                )
            )
    return items


def inventory() -> set[InventoryItem]:
    return rust_inventory() | ledger_inventory()


def census_markers() -> tuple[dict[str, str], list[str]]:
    if not CENSUS.exists():
        return {}, [f"missing census: {CENSUS.relative_to(ROOT)}"]
    markers: dict[str, str] = {}
    errors: list[str] = []
    for kind, raw_key in MARKER_PATTERN.findall(CENSUS.read_text(encoding="utf-8")):
        key = raw_key.strip()
        if key in markers:
            errors.append(f"duplicate census key: {key}")
        markers[key] = kind
    return markers, errors


def audit() -> list[str]:
    discovered = inventory()
    by_key = {item.key: item for item in discovered}
    markers, errors = census_markers()
    for key in sorted(by_key.keys() - markers.keys()):
        errors.append(f"missing: {key} ({by_key[key].source})")
    for key in sorted(markers.keys() - by_key.keys()):
        errors.append(f"stale: {key}")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="print discovered keys")
    args = parser.parse_args(argv)
    if args.list:
        for item in sorted(inventory()):
            print(f"{item.key}\t{item.source}")
        return 0
    errors = audit()
    if errors:
        print("refusal frontier census audit failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("refusal frontier census covers every audited source declaration")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
