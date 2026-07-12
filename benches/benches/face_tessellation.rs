//! Q3 contextual standalone face-tessellation benchmark ladder.

use core::time::Duration;
use std::hint::black_box;
use std::sync::OnceLock;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use kcore::operation::OperationContext;
use kcore::tolerance::Tolerances;
use kernel_benchmarks::face_tessellation::{
    CASES, FaceTessellationRun, compatibility_session, fixture, tessellation_options, verify,
};

fn configuration() -> Criterion {
    let smoke = std::env::var_os("KERNEL_BENCH_SMOKE").is_some();
    Criterion::default()
        .sample_size(if smoke { 10 } else { 100 })
        .warm_up_time(Duration::from_millis(if smoke { 100 } else { 3_000 }))
        .measurement_time(Duration::from_millis(if smoke { 200 } else { 5_000 }))
        .without_plots()
}

fn face_tessellation(criterion: &mut Criterion) {
    for case in CASES {
        let mut group = criterion.benchmark_group(case.path.rsplit_once('/').unwrap().0);
        group.throughput(Throughput::Elements(1));
        let prepared = OnceLock::new();
        let expected = OnceLock::new();
        group.bench_function(case.path.rsplit_once('/').unwrap().1, move |bencher| {
            let fixture = prepared.get_or_init(fixture);
            let face = fixture.trimmed();
            let session = compatibility_session();
            let context = OperationContext::new(&session, Tolerances::default())
                .expect("reviewed Q3 face tolerances satisfy compatibility-v1 precision");
            let options = tessellation_options(case.chord_tol);
            let expected = expected.get_or_init(|| {
                let first = fixture.tessellate(case.chord_tol, &context);
                let repeated = fixture.tessellate(case.chord_tol, &context);
                assert_eq!(first.mesh, repeated.mesh, "Q3 face preflight mesh drift");
                assert_eq!(
                    first.report, repeated.report,
                    "Q3 face preflight report drift"
                );
                fixture.evidence(&first)
            });
            verify(case, *expected);
            bencher.iter_custom(|iterations| {
                let mut measured = Duration::ZERO;
                for _ in 0..iterations {
                    let options = black_box(&options);
                    let started = std::time::Instant::now();
                    let outcome = fixture.tessellate_outcome(&face, options, &context);
                    measured += started.elapsed();
                    let run = FaceTessellationRun::from_outcome(
                        outcome.expect("reviewed Q3 face policy must be valid"),
                    );
                    let evidence = fixture.evidence(&run);
                    verify(case, evidence);
                    black_box(run);
                }
                measured
            });
        });
        group.finish();
    }
}

criterion_group! {
    name = q3_face;
    config = configuration();
    targets = face_tessellation
}
criterion_main!(q3_face);
