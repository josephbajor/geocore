"""Command-line orchestration for the automated Onshape oracle loop."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from .onshape import (
    CONFIG_PATH,
    ApiKeyTransport,
    OracleError,
    find_translated_part_studio,
    load_config,
    load_env_file,
    reexport_element,
    run_fixture,
    session_info,
)

ROOT = Path(__file__).resolve().parents[2]

CONFIG_TEMPLATE = {
    "document_id": "<24-hex id from the Onshape document URL: /documents/{id}/...>",
    "workspace_id": "<24-hex id from .../w/{id}/...>",
    "element_id": "<24-hex id of the blob element receiving uploads: .../e/{id}>",
}

DEFAULT_OUTBOX = ROOT / "oracle" / "outbox"
DEFAULT_INBOX = ROOT / "oracle" / "inbox" / "onshape"


def writer_identity():
    """Best-effort short git revision for oracle-results.tsv notes."""
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "-"
    return revision or "-"


def results_row(
    result,
    date,
    revision,
    reexport_ok="-",
    compare_ok="-",
    bundle_identity=None,
):
    """Format one docs/oracle-results.tsv row for a fixture verdict."""
    note = "writer={}".format(revision)
    if result.accepted:
        note += "; accepted"
    else:
        note += "; {}".format(result.reason or "rejected without a reason")
        if result.classification:
            note += " [{}]".format(result.classification)
    if bundle_identity:
        note += "; bundle={}".format(bundle_identity)
    return "\t".join(
        [
            date,
            "onshape",
            "cloud-{}".format(date),
            result.name,
            "yes" if result.accepted else "no",
            "none" if result.accepted else "-",
            "-",
            reexport_ok,
            compare_ok,
            note,
        ]
    )


def verdict_line(result):
    """One human-readable status line per fixture."""
    if result.accepted:
        return "{:<36} {:>7}B  accepted".format(result.name, result.size)
    return "{:<36} {:>7}B  {}: {} [{}]".format(
        result.name, result.size, result.state, result.reason, result.classification
    )


def compare_files(outbox_file, inbox_file):
    """Run xt_oracle compare; returns yes/no/error per its exit-code contract."""
    completed = subprocess.run(
        [
            "cargo",
            "run",
            "--release",
            "-p",
            "kxt",
            "--bin",
            "xt_oracle",
            "--",
            "compare",
            str(outbox_file),
            str(inbox_file),
        ],
        cwd=ROOT,
    )
    if completed.returncode == 0:
        return "yes"
    if completed.returncode == 1:
        return "no"
    return "error"


def request_budget_exhausted(error):
    """Whether an operational error requires the catch-up session to stop."""
    message = str(error)
    return "request limit exhausted" in message or "HTTP 429" in message


def command_check(_args):
    """Verify API-key credentials against the live host."""
    info = session_info(ApiKeyTransport.from_environment())
    name = info.get("name") or info.get("id") or "unknown session"
    print("authenticated: {}".format(name))
    return 0


def command_init(_args):
    """Write the untracked config template if it does not exist yet."""
    path = ROOT / CONFIG_PATH
    if path.exists():
        print("{} already exists; not overwriting".format(path))
        return 0
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(CONFIG_TEMPLATE, indent=2) + "\n", encoding="utf-8")
    print("wrote {}; fill in the ids from the Onshape document URL".format(path))
    return 0


def run_paths(args, paths, bundle_identity=None):
    """Upload each path, optionally re-export/compare, and report."""
    if args.compare and not args.reexport:
        raise OracleError("--compare requires --reexport")
    results_file = Path(args.results_file) if args.results_file else None
    request_count_file = (
        Path(args.request_count_file) if args.request_count_file else None
    )
    completion_file = Path(args.completion_file) if args.completion_file else None
    for output_file in (results_file, request_count_file, completion_file):
        if output_file is not None:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            output_file.write_text("", encoding="utf-8")
    transport = ApiKeyTransport.from_environment()
    config = load_config(ROOT / CONFIG_PATH)
    date = time.strftime("%Y-%m-%d")
    revision = writer_identity()
    rows = []
    findings = 0
    errors = 0
    imports_accepted = 0
    compares_clean = 0
    try:
        for path in paths:
            result = run_fixture(transport, config, path)
            reexport_ok = "-"
            compare_ok = "-"
            stop_error = None
            if result.accepted and args.reexport:
                # Must run before the next upload: each translation rewrites the
                # document's single derived part studio.
                try:
                    element_id = (
                        result.result_element_ids[0]
                        if result.result_element_ids
                        else find_translated_part_studio(transport, config)
                    )
                    inbox = Path(args.inbox)
                    inbox.mkdir(parents=True, exist_ok=True)
                    inbox_file = inbox / result.name
                    inbox_file.write_bytes(reexport_element(transport, config, element_id))
                    reexport_ok = "yes"
                    if args.compare:
                        compare_ok = compare_files(path, inbox_file)
                except OracleError as error:
                    print("  re-export of {} failed: {}".format(result.name, error))
                    reexport_ok = "error"
                    if request_budget_exhausted(error):
                        stop_error = error
            print(verdict_line(result))
            rows.append(
                results_row(
                    result,
                    date,
                    revision,
                    reexport_ok,
                    compare_ok,
                    bundle_identity=bundle_identity,
                )
            )
            if results_file is not None:
                results_file.write_text("\n".join(rows) + "\n", encoding="utf-8")
            imports_accepted += int(result.accepted)
            compares_clean += int(compare_ok == "yes")
            if not result.accepted or (args.compare and compare_ok == "no"):
                findings += 1
            if args.reexport and result.accepted and reexport_ok != "yes":
                errors += 1
            if args.compare and result.accepted and compare_ok == "error":
                errors += 1
            if stop_error is not None:
                raise stop_error
    finally:
        limit = transport.request_limit if transport.request_limit is not None else "unbounded"
        print("\nOnshape API requests: {} / {}".format(transport.request_count, limit))
        if request_count_file is not None:
            request_count_file.parent.mkdir(parents=True, exist_ok=True)
            request_count_file.write_text(str(transport.request_count) + "\n", encoding="utf-8")
    if args.results_rows:
        print("\nappend to docs/oracle-results.tsv:")
        for row in rows:
            print(row)
    print("\nimports: {}/{} accepted".format(imports_accepted, len(paths)))
    if args.reexport and args.compare:
        print("there-and-back: {}/{} compared clean".format(compares_clean, imports_accepted))
    exit_code = 2 if errors else (1 if findings else 0)
    if completion_file is not None:
        completion_file.parent.mkdir(parents=True, exist_ok=True)
        completion_file.write_text(str(exit_code) + "\n", encoding="utf-8")
    return exit_code


def command_run(args):
    """Upload explicitly named fixture files."""
    return run_paths(args, [Path(p) for p in args.files])


def bundle_paths(outbox, selected_names=None):
    """Return exact manifest paths, optionally filtered by validated names."""
    outbox = Path(outbox)
    manifest_path = outbox / "manifest.tsv"
    try:
        lines = manifest_path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        raise OracleError("missing bundle manifest: {}".format(manifest_path))
    if not lines or not lines[0].startswith("file\tbody_kind\t"):
        raise OracleError("invalid bundle manifest header: {}".format(manifest_path))
    names = []
    for line in lines[1:]:
        name = line.partition("\t")[0]
        if not name.endswith(".x_t") or Path(name).name != name or name in names:
            raise OracleError("invalid or duplicate bundle fixture name: {!r}".format(name))
        names.append(name)
    if not names:
        raise OracleError("bundle manifest contains no fixtures: {}".format(manifest_path))
    paths = [outbox / name for name in names]
    missing = [path.name for path in paths if not path.is_file()]
    if missing:
        raise OracleError("bundle is missing manifest fixtures: {}".format(", ".join(missing)))
    actual = {path.name for path in outbox.glob("*.x_t")}
    expected = set(names)
    if actual != expected:
        raise OracleError(
            "bundle contains stale or unexpected fixtures: expected {}, found {}".format(
                sorted(expected), sorted(actual)
            )
        )
    if selected_names:
        if len(selected_names) != len(set(selected_names)):
            raise OracleError("selected bundle fixtures must not be duplicated")
        unknown = sorted(set(selected_names) - expected)
        if unknown:
            raise OracleError(
                "selected fixtures are not in the bundle manifest: {}".format(
                    ", ".join(unknown)
                )
            )
        selected = set(selected_names)
        paths = [path for path in paths if path.name in selected]
    return paths


def command_bundle(args):
    """Upload the full generated bundle or its manifest-bound selection."""
    from .certification import observed_identity

    paths = bundle_paths(args.outbox, args.fixtures)
    identity = observed_identity(args.outbox)["bundle_sha256"]
    return run_paths(args, paths, bundle_identity=identity)


def command_certification_check(args):
    """Verify the committed licensed-host evidence against current bytes."""
    from .certification import check_certification

    check_certification(
        args.outbox,
        record_path=args.record,
        require_current=args.require_current,
    )
    return 0


def command_identity(args):
    """Print the exact offline writer and bundle identity as JSON."""
    from .certification import observed_identity

    print(json.dumps(observed_identity(args.outbox), indent=2))
    return 0


def build_parser():
    parser = argparse.ArgumentParser(
        prog="oracle_loop",
        description="Automated Onshape oracle loop (docs/oracle-loop.md)",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("check", help="verify API-key credentials").set_defaults(
        func=command_check
    )
    commands.add_parser("init", help="write the oracle/config.json template").set_defaults(
        func=command_init
    )

    def add_loop_arguments(sub):
        sub.add_argument("--reexport", action="store_true", help="download host re-exports")
        sub.add_argument("--compare", action="store_true", help="run xt_oracle compare")
        sub.add_argument("--inbox", default=str(DEFAULT_INBOX), help="re-export directory")
        sub.add_argument(
            "--results-rows",
            action="store_true",
            help="print docs/oracle-results.tsv rows for the run",
        )
        sub.add_argument(
            "--results-file",
            help="write completed docs/oracle-results.tsv rows, including partial runs",
        )
        sub.add_argument(
            "--request-count-file",
            help="write the number of attempted host requests for budget accounting",
        )
        sub.add_argument(
            "--completion-file",
            help="write the final CLI status only after the fixture loop completes",
        )

    run = commands.add_parser("run", help="upload named .x_t files")
    run.add_argument("files", nargs="+")
    add_loop_arguments(run)
    run.set_defaults(func=command_run)

    bundle = commands.add_parser("bundle", help="upload the generated outbox bundle")
    bundle.add_argument("--outbox", default=str(DEFAULT_OUTBOX))
    bundle.add_argument(
        "--fixtures",
        nargs="+",
        help="upload only these exact manifest fixture names, in manifest order",
    )
    add_loop_arguments(bundle)
    bundle.set_defaults(func=command_bundle)

    certification = commands.add_parser(
        "certification-check",
        help="compare current writer/bundle bytes with committed host evidence",
    )
    certification.add_argument("--outbox", default=str(DEFAULT_OUTBOX))
    certification.add_argument(
        "--record", default=str(ROOT / "docs" / "oracle-certification.json")
    )
    certification.add_argument("--require-current", action="store_true")
    certification.set_defaults(func=command_certification_check)

    identity = commands.add_parser(
        "identity", help="print the offline identity of a generated oracle bundle"
    )
    identity.add_argument("--outbox", default=str(DEFAULT_OUTBOX))
    identity.set_defaults(func=command_identity)

    return parser


def main(argv=None):
    load_env_file(ROOT / ".env")
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except OracleError as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2
