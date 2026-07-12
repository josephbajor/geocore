#!/usr/bin/env python3
"""Generate the checked xt_read seed corpus from local licensed fixtures."""

from __future__ import annotations

import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT / "fuzz" / "corpus" / "xt_read"


def after_header(payload: bytes) -> bytes:
    marker = b"**END_OF_HEADER"
    start = payload.index(marker)
    line_end = payload.index(b"\n", start)
    return payload[: line_end + 1]


def without_terminator_pointer(payload: bytes) -> bytes:
    stripped = payload.rstrip()
    if not stripped.endswith(b"1 0"):
        raise ValueError("fixture does not end in the reviewed X_T terminator")
    return stripped[:-1]


def without_terminator_record(payload: bytes) -> bytes:
    stripped = payload.rstrip()
    if not stripped.endswith(b"1 0"):
        raise ValueError("fixture does not end in the reviewed X_T terminator")
    return stripped[:-3].rstrip() + b" "


def corpus_entries() -> dict[str, bytes]:
    block = (ROOT / "crates/kxt/tests/fixtures/block.x_t").read_bytes()
    offset = (ROOT / "crates/kxt/tests/fixtures/offset_plane.x_t").read_bytes()
    minimal = (ROOT / "fuzz/fixtures/minimal_valid.x_t").read_bytes()
    return {
        "parse-minimal-valid.xtseed": bytes([0]) + minimal,
        "parse-block.xtseed": bytes([0]) + block,
        "import-block.xtseed": bytes([1]) + block,
        "import-offset-plane.xtseed": bytes([1]) + offset,
        "parse-header-boundary.xtseed": bytes([0]) + after_header(block),
        "parse-token-boundary.xtseed": bytes([0])
        + without_terminator_pointer(block),
        "import-record-boundary.xtseed": bytes([1])
        + without_terminator_record(block),
    }


def write_corpus(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    expected = corpus_entries()
    for stale in output.glob("*.xtseed"):
        if stale.name not in expected:
            stale.unlink()
    for name, data in expected.items():
        (output / name).write_bytes(data)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    write_corpus(args.output)


if __name__ == "__main__":
    main()
