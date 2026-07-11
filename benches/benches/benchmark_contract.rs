//! Q1 smoke benchmark proving the benchmark contract and runner connection.

use core::time::Duration;
use std::hint::black_box;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use kernel_benchmarks::{
    CONTRACT_CASE_PATH, CONTRACT_ELEMENTS, CONTRACT_SEED, TinyFixture, TinyResult,
    validate_case_path,
};

fn configuration() -> Criterion {
    let smoke = std::env::var_os("KERNEL_BENCH_SMOKE").is_some();
    Criterion::default()
        .sample_size(if smoke { 10 } else { 100 })
        .warm_up_time(Duration::from_millis(if smoke { 100 } else { 3_000 }))
        .measurement_time(Duration::from_millis(if smoke { 200 } else { 5_000 }))
        .without_plots()
}

fn benchmark_contract(criterion: &mut Criterion) {
    validate_case_path(CONTRACT_CASE_PATH).expect("committed case path must be canonical");
    let fixture = TinyFixture::new(CONTRACT_SEED, CONTRACT_ELEMENTS);
    let expected = fixture.execute();
    assert_eq!(
        expected,
        TinyResult {
            elements: CONTRACT_ELEMENTS,
            sum: 0xbabf_ef09_cc07_d280,
            digest: 0x1428_9053_7c90_ed65,
        },
        "fixture counters changed; update the reviewed contract intentionally"
    );

    let mut group = criterion.benchmark_group("harness/contract/tiny-v1/64");
    group.throughput(Throughput::Elements(CONTRACT_ELEMENTS as u64));
    group.bench_function("default-v1", |bencher| {
        bencher.iter(|| {
            let result = black_box(&fixture).execute();
            assert_eq!(result, expected, "a timed iteration violated its invariant");
            black_box(result)
        });
    });
    group.finish();
}

criterion_group! {
    name = contract;
    config = configuration();
    targets = benchmark_contract
}
criterion_main!(contract);
