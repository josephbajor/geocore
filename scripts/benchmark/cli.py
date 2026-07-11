"""Command-line and process orchestration for benchmark baselines."""

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from .baseline import compare_identity, record_from_text, write_json
from .contract import (
    BENCHES,
    ROOT,
    ContractError,
    find_case,
    load_cases,
    load_json,
    validate_report,
    validate_schema_document,
)
from .environment import CARGO_CRITERION_VERSION, command


DEFAULT_CASE = "harness/contract/tiny-v1/64/default-v1"


def command_validate(args):
    """Validate schema, case registry, and one or more reports."""
    validate_schema_document()
    load_cases()
    paths = args.paths or [BENCHES / "baselines" / "example.synthetic.v1.json"]
    for path in paths:
        validate_report(load_json(path), str(path))
    print("validated schema, cases, and {} report(s)".format(len(paths)))


def command_record(args):
    """Assemble a report from an existing cargo-criterion stream."""
    text = Path(args.measurement).read_text(encoding="utf-8")
    report = record_from_text(
        text,
        args.case,
        synthetic=args.synthetic_example,
        smoke=args.smoke,
        features=args.features,
    )
    write_json(args.output, report)
    print(args.output)


def command_compare(args):
    """Print only identity compatibility, never a timing judgement."""
    result = compare_identity(load_json(args.left), load_json(args.right))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["compatible"] else 2


def command_run(args):
    """Run the exact cargo-criterion contract and record one case."""
    version = command(["cargo", "criterion", "--version"])
    if CARGO_CRITERION_VERSION not in version:
        raise ContractError(
            "cargo-criterion version drift: expected {}, got {}".format(
                CARGO_CRITERION_VERSION, version
            )
        )
    case = find_case(args.case)
    invocation = [
        "cargo",
        "criterion",
        "--manifest-path",
        str(BENCHES / "Cargo.toml"),
        "--message-format=json",
        "--bench",
        case["benchmark_target"],
        "--",
        case["path"],
    ]
    environment = os.environ.copy()
    if args.smoke:
        environment["KERNEL_BENCH_SMOKE"] = "1"
    result = subprocess.run(
        invocation,
        cwd=str(ROOT),
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
        check=False,
    )
    if result.returncode != 0:
        raise ContractError(
            "cargo-criterion failed with status {}".format(result.returncode)
        )
    report = record_from_text(
        result.stdout,
        args.case,
        synthetic=False,
        smoke=args.smoke,
        features=args.features,
    )
    write_json(args.output, report)
    print(args.output)


def command_smoke(args):
    """Exercise schema, parser, composition, and output fully offline."""
    validate_schema_document()
    load_cases()
    validate_report(load_json(BENCHES / "baselines" / "example.synthetic.v1.json"))
    text = (BENCHES / "testdata" / "cargo-criterion.synthetic.ndjson").read_text(
        encoding="utf-8"
    )
    with tempfile.TemporaryDirectory() as directory:
        report = record_from_text(text, args.case, synthetic=True, smoke=True)
        output = Path(directory) / "synthetic-report.json"
        write_json(output, report)
        validate_report(load_json(output))
    print("benchmark contract smoke passed without timing or network")


def parser():
    """Construct the stable Q1 command-line interface."""
    result = argparse.ArgumentParser(
        description=(
            "Validate, record, and identity-check kernel benchmark baselines. "
            "Timing is always advisory."
        )
    )
    commands = result.add_subparsers(dest="command", required=True)

    validate = commands.add_parser(
        "validate", help="validate schema, case manifest, and reports"
    )
    validate.add_argument("paths", nargs="*", type=Path)
    validate.set_defaults(function=command_validate)

    record = commands.add_parser(
        "record", help="assemble a baseline from cargo-criterion JSON lines"
    )
    record.add_argument("--measurement", required=True, type=Path)
    record.add_argument("--output", required=True, type=Path)
    record.add_argument("--case", default=DEFAULT_CASE)
    record.add_argument("--features", nargs="*", default=[])
    record.add_argument("--smoke", action="store_true")
    record.add_argument("--synthetic-example", action="store_true")
    record.set_defaults(function=command_record)

    compare = commands.add_parser(
        "compare", help="check identity compatibility without timing judgement"
    )
    compare.add_argument("left", type=Path)
    compare.add_argument("right", type=Path)
    compare.set_defaults(function=command_compare)

    run = commands.add_parser(
        "run", help="run pinned cargo-criterion and record one case"
    )
    run.add_argument("--output", required=True, type=Path)
    run.add_argument("--case", default=DEFAULT_CASE)
    run.add_argument("--features", nargs="*", default=[])
    run.add_argument("--smoke", action="store_true")
    run.set_defaults(function=command_run)

    smoke = commands.add_parser("smoke", help="offline schema/parser smoke test")
    smoke.add_argument("--case", default=DEFAULT_CASE)
    smoke.set_defaults(function=command_smoke)
    return result


def main(argv=None):
    """Execute the CLI and collapse contract failures to exit status one."""
    args = parser().parse_args(argv)
    try:
        status = args.function(args)
    except (ContractError, OSError) as error:
        print("benchmark contract error: {}".format(error), file=sys.stderr)
        return 1
    return status or 0
