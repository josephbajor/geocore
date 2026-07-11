"""Compose baseline reports and check identity compatibility."""

import copy
import json
from pathlib import Path

from .contract import (
    SCHEMA_VERSION,
    _integer,
    find_case,
    parse_cargo_criterion,
    validate_report,
)
from .environment import measured_environment, synthetic_environment


IDENTITY_PATHS = [
    "schema_version",
    "case",
    "repository",
    "toolchain",
    "host",
    "runner",
    "result_counters",
    "measurement.source_format",
    "measurement.unit",
]


def record_from_text(text, case_path, synthetic=False, smoke=False, features=()):
    """Compose and validate one report from cargo-criterion JSON lines."""
    case = find_case(case_path)
    elements = case["size_parameters"].get("elements")
    _integer(elements, "case.size_parameters.elements", minimum=1)
    measurement = parse_cargo_criterion(text, case_path, elements)
    environment = (
        synthetic_environment(features, smoke)
        if synthetic
        else measured_environment(features, smoke)
    )
    report = {
        "schema_version": SCHEMA_VERSION,
        "run": environment["run"],
        "case": {
            "path": case["path"],
            "fixture_version": case["fixture_version"],
            "deterministic_seed": case["deterministic_seed"],
            "size_parameters": copy.deepcopy(case["size_parameters"]),
            "tolerances": copy.deepcopy(case["tolerances"]),
            "policy_values": copy.deepcopy(case["policy_values"]),
        },
        "repository": environment["repository"],
        "toolchain": environment["toolchain"],
        "host": environment["host"],
        "runner": environment["runner"],
        "result_counters": copy.deepcopy(case["expected_result_counters"]),
        "measurement": measurement,
    }
    return validate_report(report)


def _at(document, dotted):
    value = document
    for segment in dotted.split("."):
        value = value[segment]
    return value


def compare_identity(left, right):
    """Establish identity compatibility without comparing timing values."""
    validate_report(left, "left")
    validate_report(right, "right")
    mismatches = []
    if not left["run"]["comparison_eligible"] or not right["run"]["comparison_eligible"]:
        mismatches.append("run.comparison_eligible")
    for path in IDENTITY_PATHS:
        if _at(left, path) != _at(right, path):
            mismatches.append(path)
    return {
        "compatible": not mismatches,
        "mismatches": mismatches,
        "judgement": "identity-only; no performance pass/fail is computed at Q1",
    }


def write_json(path, value):
    """Write one stable, reviewed-size JSON report."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
