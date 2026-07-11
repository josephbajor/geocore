//! Q4 contextual NURBS implicit-isolation benchmark ladder.

use core::time::Duration;
use std::hint::black_box;
use std::sync::OnceLock;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use kernel_benchmarks::nurbs_isolation::{CASES, fixture, verify};

fn configuration() -> Criterion {
    let smoke = std::env::var_os("KERNEL_BENCH_SMOKE").is_some();
    Criterion::default()
        .sample_size(if smoke { 10 } else { 100 })
        .warm_up_time(Duration::from_millis(if smoke { 100 } else { 3_000 }))
        .measurement_time(Duration::from_millis(if smoke { 200 } else { 5_000 }))
        .without_plots()
}

fn nurbs_isolation(criterion: &mut Criterion) {
    for case in CASES {
        let mut group = criterion.benchmark_group(case.path.rsplit_once('/').unwrap().0);
        group.throughput(Throughput::Elements(case.expected_patches as u64));
        let prepared = OnceLock::new();
        let expected = OnceLock::new();
        group.bench_function(case.path.rsplit_once('/').unwrap().1, move |bencher| {
            let fixture = prepared.get_or_init(|| fixture(case));
            let expected = expected.get_or_init(|| {
                let first = fixture.measure_once(case).1;
                let repeated = fixture.measure_once(case).1;
                assert_eq!(first, repeated, "Q4 preflight repeatability drift");
                first
            });
            verify(case, *expected);
            bencher.iter_custom(|iterations| {
                let mut measured = Duration::ZERO;
                for _ in 0..iterations {
                    let (elapsed, evidence) = black_box(fixture).measure_once(case);
                    verify(case, evidence);
                    black_box(evidence);
                    measured += elapsed;
                }
                measured
            });
        });
        group.finish();
    }
}

criterion_group! {
    name = q4;
    config = configuration();
    targets = nurbs_isolation
}
criterion_main!(q4);
