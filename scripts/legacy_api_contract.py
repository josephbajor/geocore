#!/usr/bin/env python3
"""Forbid new production use of equivalence-proven legacy kernel APIs."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Mapping


SOURCE_ROOTS = (
    Path("crates/kgeom/src"),
    Path("crates/kops/src"),
    Path("crates/ktopo/src"),
    Path("crates/kxt/src"),
)
LEGACY_BODY_TESSELLATION = re.compile(r"\btessellate_body\b")
LEGACY_FACE_TESSELLATION = re.compile(r"\btessellate\b")
LEGACY_SURFACE_PROJECTION = re.compile(r"\bproject_to_surface\b")
LEGACY_CURVE_PROJECTION = re.compile(r"\bproject_to_curve\b")
TEST_MODULE = re.compile(
    r"#\s*\[\s*cfg\s*\(\s*test\s*\)\s*\]\s*"
    r"(?:#\s*\[[^\]]*\]\s*)*"
    r"mod\s+[A-Za-z_][A-Za-z0-9_]*\s*\{",
    re.MULTILINE,
)
BODY_TESSELLATION_DEFINITION = Path("crates/ktopo/src/btess.rs")
FACE_TESSELLATION_DEFINITION = Path("crates/kgeom/src/tess.rs")
SURFACE_PROJECTION_DEFINITION = Path("crates/kgeom/src/project.rs")
CURVE_PROJECTION_DEFINITION = Path("crates/kgeom/src/project.rs")
SURFACE_POINT_COMPATIBILITY = Path("crates/kgeom/src/surface_point.rs")


class ContractError(RuntimeError):
    """A closed internal legacy-use boundary was crossed."""


def _blank(masked: list[str], start: int, end: int) -> None:
    for index in range(start, end):
        if masked[index] != "\n":
            masked[index] = " "


def _char_literal_end(source: str, start: int) -> int | None:
    """Return one Rust character literal's end without mistaking lifetimes."""
    value = start + 1
    if value >= len(source) or source[value] == "\n":
        return None
    if source[value] != "\\":
        return value + 2 if value + 1 < len(source) and source[value + 1] == "'" else None

    value += 1
    if value >= len(source):
        return None
    if source[value] == "u" and value + 1 < len(source) and source[value + 1] == "{":
        closing_escape = source.find("}", value + 2)
        if closing_escape == -1:
            return None
        closing_quote = closing_escape + 1
    elif source[value] == "x":
        closing_quote = value + 3
    else:
        closing_quote = value + 1
    if closing_quote < len(source) and source[closing_quote] == "'":
        return closing_quote + 1
    return None


def _lexical_code(source: str) -> str:
    """Mask Rust comments and literals, preserving offsets and newlines."""
    masked = list(source)
    index = 0
    while index < len(source):
        if source.startswith("//", index):
            end = source.find("\n", index + 2)
            if end == -1:
                end = len(source)
            _blank(masked, index, end)
            index = end
            continue

        if source.startswith("/*", index):
            depth = 1
            end = index + 2
            while end < len(source) and depth:
                if source.startswith("/*", end):
                    depth += 1
                    end += 2
                elif source.startswith("*/", end):
                    depth -= 1
                    end += 2
                else:
                    end += 1
            _blank(masked, index, end)
            index = end
            continue

        if source[index] == "r":
            delimiter = index + 1
            while delimiter < len(source) and source[delimiter] == "#":
                delimiter += 1
            if delimiter < len(source) and source[delimiter] == '"':
                hashes = delimiter - index - 1
                terminator = '"' + "#" * hashes
                closing = source.find(terminator, delimiter + 1)
                end = len(source) if closing == -1 else closing + len(terminator)
                _blank(masked, index, end)
                index = end
                continue

        if source[index] == '"':
            end = index + 1
            escaped = False
            while end < len(source):
                char = source[end]
                end += 1
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    break
            _blank(masked, index, end)
            index = end
            continue

        if source[index] == "'" and (end := _char_literal_end(source, index)) is not None:
            _blank(masked, index, end)
            index = end
            continue

        index += 1
    return "".join(masked)


def _without_test_modules(source: str) -> str:
    """Mask explicit cfg(test) modules while retaining source line numbers."""
    code = _lexical_code(source)
    masked = list(code)
    cursor = 0
    while match := TEST_MODULE.search(code, cursor):
        opening = code.find("{", match.start(), match.end())
        depth = 0
        closing = None
        for index in range(opening, len(code)):
            if code[index] == "{":
                depth += 1
            elif code[index] == "}":
                depth -= 1
                if depth == 0:
                    closing = index + 1
                    break
        if closing is None:
            raise ContractError("unterminated #[cfg(test)] module in audited Rust source")
        _blank(masked, match.start(), closing)
        cursor = closing
    return "".join(masked)


def _function_ranges(source: str, names: frozenset[str]) -> list[tuple[int, int]]:
    """Return complete lexical ranges for explicitly named Rust functions."""
    ranges = []
    function = re.compile(
        r"\b(?:pub(?:\s*\([^)]*\))?\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)\b[^{]*\{",
        re.MULTILINE,
    )
    for match in function.finditer(source):
        if match.group(1) not in names:
            continue
        opening = source.find("{", match.start(), match.end())
        depth = 0
        for index in range(opening, len(source)):
            if source[index] == "{":
                depth += 1
            elif source[index] == "}":
                depth -= 1
                if depth == 0:
                    ranges.append((match.start(), index + 1))
                    break
        else:
            raise ContractError(
                f"unterminated allowed compatibility function {match.group(1)}"
            )
    return ranges


def _find_legacy_uses(
    sources: Mapping[Path, str],
    symbol: re.Pattern[str],
    definition_path: Path,
    allowed_functions: Mapping[Path, frozenset[str]] | None = None,
) -> list[str]:
    """Return forbidden production references for one legacy symbol."""
    violations = []
    for path in sorted(sources, key=lambda item: item.as_posix()):
        source = _without_test_modules(sources[path])
        allowed_ranges = _function_ranges(
            source,
            (allowed_functions or {}).get(path, frozenset()),
        )
        for match in symbol.finditer(source):
            if any(start <= match.start() < end for start, end in allowed_ranges):
                continue
            line_start = source.rfind("\n", 0, match.start()) + 1
            line_end = source.find("\n", match.end())
            if line_end == -1:
                line_end = len(source)
            line = source[line_start:line_end]
            if path == definition_path and re.search(
                rf"\bpub\s+fn\s+{re.escape(match.group())}\b", line
            ):
                continue
            line_number = source.count("\n", 0, match.start()) + 1
            violations.append(f"{path.as_posix()}:{line_number}")
    return violations


def find_legacy_body_tessellation_uses(sources: Mapping[Path, str]) -> list[str]:
    """Return forbidden production body-wrapper references."""
    return _find_legacy_uses(
        sources,
        LEGACY_BODY_TESSELLATION,
        BODY_TESSELLATION_DEFINITION,
    )


def find_legacy_face_tessellation_uses(sources: Mapping[Path, str]) -> list[str]:
    """Return forbidden production standalone-face-wrapper references."""
    return _find_legacy_uses(
        sources,
        LEGACY_FACE_TESSELLATION,
        FACE_TESSELLATION_DEFINITION,
    )


def find_legacy_surface_projection_uses(sources: Mapping[Path, str]) -> list[str]:
    """Return forbidden production references to the legacy surface projector."""
    return _find_legacy_uses(
        sources,
        LEGACY_SURFACE_PROJECTION,
        SURFACE_PROJECTION_DEFINITION,
        {
            SURFACE_POINT_COMPATIBILITY: frozenset(
                {"invert_surface_point", "distance_to_surface"}
            )
        },
    )


def find_legacy_curve_projection_uses(sources: Mapping[Path, str]) -> list[str]:
    """Return forbidden production references to the legacy curve projector."""
    return _find_legacy_uses(
        sources,
        LEGACY_CURVE_PROJECTION,
        CURVE_PROJECTION_DEFINITION,
    )


def audit_repository(repository: Path) -> list[str]:
    """Audit the crate production trees governed by this ratchet."""
    sources = {}
    for root in SOURCE_ROOTS:
        for path in sorted((repository / root).rglob("*.rs")):
            sources[path.relative_to(repository)] = path.read_text(encoding="utf-8")
    return sorted(
        find_legacy_body_tessellation_uses(sources)
        + find_legacy_face_tessellation_uses(sources)
        + find_legacy_surface_projection_uses(sources)
        + find_legacy_curve_projection_uses(sources)
    )


def main() -> int:
    repository = Path(__file__).resolve().parents[1]
    violations = audit_repository(repository)
    if violations:
        joined = "\n  ".join(violations)
        raise ContractError(
            "equivalence-proven legacy APIs are closed to new production callers; "
            "use the contextual or in-scope entry points:\n  "
            f"{joined}"
        )
    print("legacy API production-use ratchets are closed")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ContractError as error:
        print(f"legacy API contract failed: {error}", file=sys.stderr)
        sys.exit(1)
