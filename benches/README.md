# Kernel benchmark harness

This isolated package owns benchmark-only dependencies and deterministic
fixtures. It is excluded from the normal Cargo workspace so `cargo test
--workspace` and the kernel MSRV contract do not depend on the benchmark
runner lifecycle.

## Contract

Every case is registered in `cases.json` with a five-segment path:

```text
<subsystem>/<operation>/<fixture>/<scale>/<policy>
```

Fixture construction, seeds, expected counters, and invariant checks are
deterministic. Criterion measures elapsed time, but timing never establishes
correctness. The runner always marks measurements advisory.

Criterion is pinned to `0.8.2`. Machine-readable measured runs use
`cargo-criterion 1.1.0` because its JSON-lines output is a documented external
tool contract. Install that exact runner when recording a real measurement:

```sh
cargo install --locked --version 1.1.0 cargo-criterion
```

The package has its own reviewed `Cargo.lock`; do not add benchmark
dependencies to the root workspace lockfile.

## Offline validation

These commands require no benchmark execution or network:

```sh
python3 scripts/benchmark_baseline.py smoke
python3 scripts/benchmark_baseline.py validate
python3 -m unittest discover -s scripts/tests -p 'test_benchmark_baseline.py' -v
```

They validate the case registry, JSON Schema root contract, the complete
checked-in synthetic report, strict cargo-criterion parsing, runner-output
round trips, and fail-closed behavior for missing identity fields or format
drift.

The checked-in `example.synthetic.v1.json` and synthetic JSON-lines fixture
are schema/parser examples only. They are explicitly comparison-ineligible and
contain no performance claim.

## Compile and smoke the Rust target

```sh
cargo test --manifest-path benches/Cargo.toml
cargo bench --manifest-path benches/Cargo.toml --bench benchmark_contract --no-run
```

The Q1 target verifies the result digest before measurement and again in every
timed iteration. Q2-Q5 add their subsystem fixtures as independent changes.

## Record a measured run

Write generated reports outside the repository or below ignored
`benches/results/`:

```sh
python3 scripts/benchmark_baseline.py run \
  --smoke \
  --output benches/results/q1-smoke.json
python3 scripts/benchmark_baseline.py validate benches/results/q1-smoke.json
```

The report includes repository state, compiler/Cargo/target/profile/features,
OS/architecture/CPU/cores/available memory, runner configuration and affinity,
fixture identity and policy, result counters, and Criterion's typical estimate.
A dirty worktree is recorded but is not comparison-eligible.

## Compare identity

```sh
python3 scripts/benchmark_baseline.py compare baseline.json candidate.json
```

At Q1 this command only determines whether comparison identities match. It
refuses compatibility when either input is ineligible or schema, repository,
host, compiler, runner, fixture, policy, result counters, or measurement units
differ. It does not calculate ratios, thresholds, or a performance pass/fail
result.

Compact reviewed baselines belong under `baselines/<host>/<revision>.json`.
Raw cargo-criterion streams and generated reports are artifacts and must not be
committed.
