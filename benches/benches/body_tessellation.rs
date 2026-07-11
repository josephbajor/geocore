//! Q3 analytic body-tessellation benchmark ladder.

use core::time::Duration;
use std::hint::black_box;
use std::sync::OnceLock;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use kernel_benchmarks::body_tessellation::{CASES, fixture, verify};

fn configuration() -> Criterion {
    let smoke = std::env::var_os("KERNEL_BENCH_SMOKE").is_some();
    Criterion::default()
        .sample_size(if smoke { 10 } else { 100 })
        .warm_up_time(Duration::from_millis(if smoke { 100 } else { 3_000 }))
        .measurement_time(Duration::from_millis(if smoke { 200 } else { 5_000 }))
        .without_plots()
}

fn body_tessellation(criterion: &mut Criterion) {
    for case in CASES {
        let mut group = criterion.benchmark_group(case.path.rsplit_once('/').unwrap().0);
        group.throughput(Throughput::Elements(1));
        let prepared = OnceLock::new();
        let expected = OnceLock::new();
        group.bench_function(case.path.rsplit_once('/').unwrap().1, move |bencher| {
            let fixture = prepared.get_or_init(|| fixture(case));
            let expected = expected.get_or_init(|| {
                let first = fixture.evidence(&fixture.tessellate(case.chord_tol));
                let repeated = fixture.evidence(&fixture.tessellate(case.chord_tol));
                assert_eq!(first, repeated, "Q3 preflight repeatability drift");
                first
            });
            verify(case, *expected);
            bencher.iter_custom(|iterations| {
                let mut measured = Duration::ZERO;
                for _ in 0..iterations {
                    let started = std::time::Instant::now();
                    let mesh = fixture.tessellate(black_box(case.chord_tol));
                    measured += started.elapsed();
                    let evidence = fixture.evidence(&mesh);
                    verify(case, evidence);
                    black_box(mesh);
                }
                measured
            });
        });
        group.finish();
    }
}

criterion_group! {
    name = q3;
    config = configuration();
    targets = body_tessellation
}
criterion_main!(q3);
