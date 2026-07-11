"""Stable Python API for the kernel benchmark contract tooling."""

from .baseline import compare_identity, record_from_text, write_json
from .contract import (
    BENCHES,
    ContractError,
    find_case,
    load_cases,
    load_json,
    parse_cargo_criterion,
    validate_report,
    validate_schema_document,
)

__all__ = [
    "BENCHES",
    "ContractError",
    "compare_identity",
    "find_case",
    "load_cases",
    "load_json",
    "parse_cargo_criterion",
    "record_from_text",
    "validate_report",
    "validate_schema_document",
    "write_json",
]
