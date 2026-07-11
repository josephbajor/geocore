//! Q2 checked-commit and index-refresh ladder.

use core::time::Duration;
use std::hint::black_box;
use std::sync::OnceLock;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use kernel_benchmarks::topology::{CASES, Ladder, fixture, verify, verify_full_rebuild};

fn configuration() -> Criterion {
    let smoke = std::env::var_os("KERNEL_BENCH_SMOKE").is_some();
    Criterion::default()
        .sample_size(if smoke { 10 } else { 100 })
        .warm_up_time(Duration::from_millis(if smoke { 100 } else { 3_000 }))
        .measurement_time(Duration::from_millis(if smoke { 200 } else { 5_000 }))
        .without_plots()
}

fn topology_commit(criterion: &mut Criterion) {
    for case in CASES {
        let mut group = criterion.benchmark_group(case.path.rsplit_once('/').unwrap().0);
        group.throughput(Throughput::Elements(case.bodies as u64));
        let prepared = OnceLock::new();
        let expected = OnceLock::new();
        group.bench_function(case.path.rsplit_once('/').unwrap().1, move |bencher| {
            let fixture = prepared.get_or_init(|| fixture(case));
            if case.ladder == Ladder::FullRebuild {
                verify_full_rebuild(case, fixture.measure_prepared_full_rebuild().1);
                bencher.iter_custom(|iterations| {
                    let mut measured = Duration::ZERO;
                    for _ in 0..iterations {
                        let (elapsed, audit) =
                            black_box(fixture).measure_prepared_full_rebuild();
                        verify_full_rebuild(case, audit);
                        black_box(audit);
                        measured += elapsed;
                    }
                    measured
                });
            } else {
                let expected = expected.get_or_init(|| fixture.measure_once(case.ladder).1);
                verify(case, expected);
                bencher.iter_custom(|iterations| {
                    let mut measured = Duration::ZERO;
                    for _ in 0..iterations {
                        let (elapsed, result) =
                            black_box(fixture).measure_once(case.ladder);
                        verify(case, &result);
                        black_box(result);
                        measured += elapsed;
                    }
                    measured
                });
            }
        });
        group.finish();
    }
}

criterion_group! {
    name = q2;
    config = configuration();
    targets = topology_commit
}
criterion_main!(q2);
