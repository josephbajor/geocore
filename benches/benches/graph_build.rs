//! Q2a geometry-graph construction and reverse-dependency ladder.

use core::time::Duration;
use std::hint::black_box;
use std::sync::OnceLock;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use kernel_benchmarks::graph_build::{CASES, GraphBuildFixture, verify};

fn configuration() -> Criterion {
    let smoke = std::env::var_os("KERNEL_BENCH_SMOKE").is_some();
    Criterion::default()
        .sample_size(if smoke { 10 } else { 100 })
        .warm_up_time(Duration::from_millis(if smoke { 100 } else { 3_000 }))
        .measurement_time(Duration::from_millis(if smoke { 200 } else { 5_000 }))
        .without_plots()
}

fn graph_build(criterion: &mut Criterion) {
    for case in CASES {
        let mut group = criterion.benchmark_group(case.path.rsplit_once('/').unwrap().0);
        group.throughput(Throughput::Elements(case.scale as u64));
        let prepared = OnceLock::new();
        group.bench_function(case.path.rsplit_once('/').unwrap().1, move |bencher| {
            let fixture = prepared.get_or_init(|| GraphBuildFixture::new(case));
            verify(case, fixture.measure_once().1);
            bencher.iter_custom(|iterations| {
                let mut measured = Duration::ZERO;
                for _ in 0..iterations {
                    let (elapsed, result) = black_box(fixture).measure_once();
                    verify(case, result);
                    black_box(result);
                    measured += elapsed;
                }
                measured
            });
        });
        group.finish();
    }
}

criterion_group! {
    name = q2a;
    config = configuration();
    targets = graph_build
}
criterion_main!(q2a);
