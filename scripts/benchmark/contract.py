"""Strict benchmark manifest, report, and cargo-criterion contracts."""

import json
import math
import re
from pathlib import Path


SCHEMA_VERSION = "kernel-benchmark-baseline.v1"
CASES_VERSION = "kernel-benchmark-cases.v1"
SOURCE_FORMAT = "cargo-criterion-json.v1"
CASE_PATH = re.compile(r"^[a-z0-9-]+(?:/[a-z0-9-]+){4}$")
HEX64 = re.compile(r"^[0-9a-f]{16}$")
ROOT = Path(__file__).resolve().parents[2]
BENCHES = ROOT / "benches"


class ContractError(ValueError):
    """Closed failure for malformed manifests, runner messages, or reports."""


def load_json(path):
    """Load JSON or raise one typed contract failure."""
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError("{}: invalid JSON: {}".format(path, error)) from error
    return value


def _object(value, path, keys):
    if not isinstance(value, dict):
        raise ContractError("{} must be an object".format(path))
    missing = sorted(set(keys) - set(value))
    extra = sorted(set(value) - set(keys))
    if missing or extra:
        raise ContractError(
            "{} fields differ: missing={}, extra={}".format(path, missing, extra)
        )
    return value


def _string(value, path):
    if not isinstance(value, str) or not value:
        raise ContractError("{} must be a non-empty string".format(path))
    return value


def _bool(value, path):
    if not isinstance(value, bool):
        raise ContractError("{} must be a boolean".format(path))
    return value


def _integer(value, path, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ContractError("{} must be an integer >= {}".format(path, minimum))
    return value


def _number(value, path, minimum=0.0, exclusive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError("{} must be a finite number".format(path))
    number = float(value)
    if not math.isfinite(number) or (number <= minimum if exclusive else number < minimum):
        operator = ">" if exclusive else ">="
        raise ContractError("{} must be finite and {} {}".format(path, operator, minimum))
    return number


def _json_values(value, path):
    if isinstance(value, dict):
        for key, child in value.items():
            _string(key, path + ".<key>")
            _json_values(child, path + "." + key)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _json_values(child, "{}[{}]".format(path, index))
    elif isinstance(value, float) and not math.isfinite(value):
        raise ContractError("{} must not contain non-finite numbers".format(path))
    elif value is not None and not isinstance(value, (str, int, float, bool)):
        raise ContractError("{} contains a non-JSON value".format(path))


def validate_case(case, path="case", include_target=False):
    """Validate a report case or the richer committed case entry."""
    keys = [
        "path",
        "fixture_version",
        "deterministic_seed",
        "size_parameters",
        "tolerances",
        "policy_values",
    ]
    if include_target:
        keys.extend(["benchmark_target", "expected_result_counters"])
    _object(case, path, keys)
    case_path = _string(case["path"], path + ".path")
    if CASE_PATH.fullmatch(case_path) is None:
        raise ContractError(
            "{}.path must use subsystem/operation/fixture/scale/policy".format(path)
        )
    _string(case["fixture_version"], path + ".fixture_version")
    _integer(case["deterministic_seed"], path + ".deterministic_seed")
    for field in ("size_parameters", "tolerances", "policy_values"):
        if not isinstance(case[field], dict):
            raise ContractError("{}.{} must be an object".format(path, field))
        _json_values(case[field], path + "." + field)
    if include_target:
        _string(case["benchmark_target"], path + ".benchmark_target")
        counters = case["expected_result_counters"]
        if not isinstance(counters, dict) or not counters:
            raise ContractError("{}.expected_result_counters must be non-empty".format(path))
        _json_values(counters, path + ".expected_result_counters")
        for key in ("wrapping_sum_hex", "output_digest"):
            if HEX64.fullmatch(str(counters.get(key, ""))) is None:
                raise ContractError(
                    "{}.expected_result_counters.{} must be hex64".format(path, key)
                )


def load_cases(path=BENCHES / "cases.json"):
    """Load and validate the committed case registry."""
    manifest = load_json(path)
    _object(manifest, "cases manifest", ["schema_version", "cases"])
    if manifest["schema_version"] != CASES_VERSION:
        raise ContractError("unsupported cases schema")
    if not isinstance(manifest["cases"], list) or not manifest["cases"]:
        raise ContractError("cases must be a non-empty list")
    seen = set()
    for index, case in enumerate(manifest["cases"]):
        validate_case(case, "cases[{}]".format(index), include_target=True)
        if case["path"] in seen:
            raise ContractError("duplicate benchmark case path: {}".format(case["path"]))
        seen.add(case["path"])
    return manifest["cases"]


def find_case(path):
    """Resolve exactly one registered benchmark case."""
    matches = [case for case in load_cases() if case["path"] == path]
    if len(matches) != 1:
        raise ContractError("unknown benchmark case: {}".format(path))
    return matches[0]


def validate_schema_document(path=BENCHES / "baselines" / "schema.json"):
    """Validate the committed schema's own closed root contract."""
    schema = load_json(path)
    _object(
        schema,
        "schema",
        ["$schema", "$id", "title", "type", "additionalProperties", "required", "properties"],
    )
    if schema["$schema"] != "https://json-schema.org/draft/2020-12/schema":
        raise ContractError("schema must declare JSON Schema draft 2020-12")
    if schema["type"] != "object" or schema["additionalProperties"] is not False:
        raise ContractError("baseline schema root must be a closed object")
    required = {
        "schema_version", "run", "case", "repository", "toolchain", "host",
        "runner", "result_counters", "measurement",
    }
    if set(schema["required"]) != required or set(schema["properties"]) != required:
        raise ContractError("schema root fields do not match the v1 contract")
    return schema


def validate_report(report, path="report"):
    """Validate every required v1 baseline identity and measurement field."""
    top = [
        "schema_version", "run", "case", "repository", "toolchain", "host",
        "runner", "result_counters", "measurement",
    ]
    _object(report, path, top)
    if report["schema_version"] != SCHEMA_VERSION:
        raise ContractError("{}.schema_version is unsupported".format(path))

    run = _object(report["run"], path + ".run", ["kind", "comparison_eligible", "note"])
    if run["kind"] not in ("measured", "synthetic-example"):
        raise ContractError("{}.run.kind is unsupported".format(path))
    _bool(run["comparison_eligible"], path + ".run.comparison_eligible")
    _string(run["note"], path + ".run.note")
    if run["kind"] == "synthetic-example" and run["comparison_eligible"]:
        raise ContractError("synthetic examples can never be comparison eligible")

    validate_case(report["case"], path + ".case")
    repository = _object(
        report["repository"], path + ".repository", ["revision", "dirty_worktree"]
    )
    _string(repository["revision"], path + ".repository.revision")
    _bool(repository["dirty_worktree"], path + ".repository.dirty_worktree")

    toolchain = _object(
        report["toolchain"], path + ".toolchain",
        ["rustc_version", "cargo_version", "target_triple", "profile", "enabled_features"],
    )
    for field in ("rustc_version", "cargo_version", "target_triple", "profile"):
        _string(toolchain[field], path + ".toolchain." + field)
    if not isinstance(toolchain["enabled_features"], list) or not all(
        isinstance(feature, str) for feature in toolchain["enabled_features"]
    ):
        raise ContractError("{}.toolchain.enabled_features must be strings".format(path))

    host = _object(
        report["host"], path + ".host",
        ["os", "architecture", "cpu_model", "logical_core_count", "available_memory_bytes"],
    )
    for field in ("os", "architecture", "cpu_model"):
        _string(host[field], path + ".host." + field)
    _integer(host["logical_core_count"], path + ".host.logical_core_count", minimum=1)
    _integer(host["available_memory_bytes"], path + ".host.available_memory_bytes", minimum=1)

    runner = _object(
        report["runner"], path + ".runner",
        ["name", "version", "criterion_version", "warm_up_seconds", "sample_size", "measurement_seconds", "process_affinity"],
    )
    if runner["name"] != "cargo-criterion":
        raise ContractError("{}.runner.name must be cargo-criterion".format(path))
    for field in ("version", "criterion_version", "process_affinity"):
        _string(runner[field], path + ".runner." + field)
    _number(runner["warm_up_seconds"], path + ".runner.warm_up_seconds")
    _integer(runner["sample_size"], path + ".runner.sample_size", minimum=1)
    _number(
        runner["measurement_seconds"], path + ".runner.measurement_seconds", exclusive=True
    )

    counters = report["result_counters"]
    if not isinstance(counters, dict) or not counters:
        raise ContractError("{}.result_counters must be non-empty".format(path))
    _json_values(counters, path + ".result_counters")

    measurement = _object(
        report["measurement"], path + ".measurement",
        ["source_format", "unit", "estimate", "lower_bound", "upper_bound", "sample_count", "total_iterations", "advisory_only"],
    )
    if measurement["source_format"] != SOURCE_FORMAT:
        raise ContractError("{}.measurement.source_format drifted".format(path))
    _string(measurement["unit"], path + ".measurement.unit")
    lower = _number(measurement["lower_bound"], path + ".measurement.lower_bound")
    estimate = _number(measurement["estimate"], path + ".measurement.estimate")
    upper = _number(measurement["upper_bound"], path + ".measurement.upper_bound")
    if not lower <= estimate <= upper:
        raise ContractError("{}.measurement confidence interval is unordered".format(path))
    _integer(measurement["sample_count"], path + ".measurement.sample_count", minimum=1)
    _integer(
        measurement["total_iterations"], path + ".measurement.total_iterations", minimum=1
    )
    if measurement["advisory_only"] is not True:
        raise ContractError("{}.measurement must remain advisory-only".format(path))
    return report


def parse_cargo_criterion(text, case_path, expected_elements):
    """Parse the documented cargo-criterion JSON-lines format, failing closed."""
    messages = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            message = json.loads(line)
        except json.JSONDecodeError as error:
            raise ContractError(
                "cargo-criterion line {} is not JSON; runner format may have drifted".format(
                    line_number
                )
            ) from error
        if not isinstance(message, dict) or not isinstance(message.get("reason"), str):
            raise ContractError("cargo-criterion line {} lacks a reason".format(line_number))
        messages.append(message)
    matches = [
        message for message in messages
        if message.get("reason") == "benchmark-complete" and message.get("id") == case_path
    ]
    if len(matches) != 1:
        raise ContractError(
            "expected exactly one benchmark-complete message for {}, found {}".format(
                case_path, len(matches)
            )
        )
    message = matches[0]
    required = {
        "reason", "id", "report_directory", "iteration_count", "measured_values",
        "unit", "throughput", "typical", "mean", "median", "median_abs_dev",
    }
    missing = sorted(required - set(message))
    if missing:
        raise ContractError("cargo-criterion benchmark message is missing {}".format(missing))
    _string(message["report_directory"], "cargo-criterion.report_directory")
    unit = _string(message["unit"], "cargo-criterion.unit")
    iterations = message["iteration_count"]
    values = message["measured_values"]
    if not isinstance(iterations, list) or not iterations:
        raise ContractError("cargo-criterion iteration_count must be non-empty")
    if not isinstance(values, list) or len(values) != len(iterations):
        raise ContractError("cargo-criterion measured_values length drifted")
    for index, value in enumerate(iterations):
        _integer(value, "cargo-criterion.iteration_count[{}]".format(index), minimum=1)
    for index, value in enumerate(values):
        _number(value, "cargo-criterion.measured_values[{}]".format(index))
    typical = _object(
        message["typical"], "cargo-criterion.typical",
        ["estimate", "lower_bound", "upper_bound", "unit"],
    )
    if _string(typical["unit"], "cargo-criterion.typical.unit") != unit:
        raise ContractError("cargo-criterion typical unit does not match samples")
    lower = _number(typical["lower_bound"], "cargo-criterion.typical.lower_bound")
    estimate = _number(typical["estimate"], "cargo-criterion.typical.estimate")
    upper = _number(typical["upper_bound"], "cargo-criterion.typical.upper_bound")
    if not lower <= estimate <= upper:
        raise ContractError("cargo-criterion typical interval is unordered")
    throughput = message["throughput"]
    expected = {"per_iteration": expected_elements, "unit": "elements"}
    if not isinstance(throughput, list) or expected not in throughput:
        raise ContractError("cargo-criterion throughput does not match fixture elements")
    return {
        "source_format": SOURCE_FORMAT,
        "unit": unit,
        "estimate": estimate,
        "lower_bound": lower,
        "upper_bound": upper,
        "sample_count": len(iterations),
        "total_iterations": sum(iterations),
        "advisory_only": True,
    }
