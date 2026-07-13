//! Public regression for source-provenance bounds after rounded NURBS subdivision.

use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::{
    CurvePairProjectionPlane, CurvePairRootCertificate, NurbsCurve, NurbsCurvePairBudgetProfile,
    certify_curve_pair_unique_root, isolate_curve_pair_candidates_in_scope,
};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const X: f64 = 9_007_199_254_740_991.0;
const CUBIC_Z: [i64; 4] = [
    9_007_199_254_740_360,
    9_007_199_254_740_978,
    9_007_199_254_741_648,
    9_007_199_254_739_690,
];

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn adversary() -> (NurbsCurve, NurbsCurve) {
    let cubic = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, CUBIC_Z[0] as f64),
            Point3::new(-1.0 / 3.0, 0.0, CUBIC_Z[1] as f64),
            Point3::new(1.0 / 3.0, 0.0, CUBIC_Z[2] as f64),
            Point3::new(1.0, 0.0, CUBIC_Z[3] as f64),
        ],
        None,
    )
    .unwrap();
    let line = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, -1.0, X), Point3::new(0.0, 1.0, X)],
        None,
    )
    .unwrap();
    (cubic, line)
}

fn certify(first: &NurbsCurve, second: &NurbsCurve) -> CurvePairRootCertificate {
    certify_curve_pair_unique_root(first, full_range(), second, full_range())
        .unwrap()
        .expect("the exact source midpoint contact and P-matrix prove one root")
}

fn isolate(first: &NurbsCurve, second: &NurbsCurve) -> kgeom::nurbs::CurvePairIsolation {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    isolate_curve_pair_candidates_in_scope(
        first,
        full_range(),
        second,
        full_range(),
        0.0,
        1,
        &mut scope,
    )
    .unwrap()
}

fn ordered_ranges(
    isolation: &kgeom::nurbs::CurvePairIsolation,
    swapped: bool,
) -> Vec<(ParamRange, ParamRange)> {
    let mut ranges = isolation
        .candidates()
        .iter()
        .map(|cell| {
            if swapped {
                (cell.second_range(), cell.first_range())
            } else {
                (cell.first_range(), cell.second_range())
            }
        })
        .collect::<Vec<_>>();
    ranges.sort_by(|a, b| {
        a.0.lo
            .total_cmp(&b.0.lo)
            .then(a.0.hi.total_cmp(&b.0.hi))
            .then(a.1.lo.total_cmp(&b.1.lo))
            .then(a.1.hi.total_cmp(&b.1.hi))
    });
    ranges
}

#[test]
fn rounded_depth_one_endpoint_cannot_erase_exact_source_midpoint_contact() {
    let (cubic, line) = adversary();

    // For a cubic Bezier at t=1/2, the exact numerator is
    // p0 + 3*p1 + 3*p2 + p3. Integer arithmetic independently establishes
    // that the source point is exactly (0, 0, X), matching the line midpoint.
    let z_numerator = i128::from(CUBIC_Z[0])
        + 3 * i128::from(CUBIC_Z[1])
        + 3 * i128::from(CUBIC_Z[2])
        + i128::from(CUBIC_Z[3]);
    assert_eq!(z_numerator, 8 * i128::from(X as i64));
    assert_eq!(-1.0 + 3.0 * (-1.0 / 3.0) + 3.0 * (1.0 / 3.0) + 1.0, 0.0);

    let certificate = certify(&cubic, &line);
    assert_eq!(certificate.first_range(), full_range());
    assert_eq!(certificate.second_range(), full_range());
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);

    let (low, high) = cubic.split_at(0.5).unwrap();
    assert_eq!(low.points().last().unwrap().z, X - 1.0);
    assert_eq!(high.points().first().unwrap().z, X - 1.0);

    let forward = isolate(&cubic, &line);
    assert!(!forward.candidates().is_empty());
    assert!(!forward.is_proven_empty());
    assert!(forward.candidates().iter().all(|cell| cell.depth() == 1));

    let repeated = isolate(&cubic, &line);
    assert_eq!(repeated, forward);

    let swapped_certificate = certify(&line, &cubic);
    assert_eq!(swapped_certificate.first_range(), full_range());
    assert_eq!(swapped_certificate.second_range(), full_range());
    assert_eq!(
        swapped_certificate.projection_plane(),
        certificate.projection_plane()
    );

    let swapped = isolate(&line, &cubic);
    assert!(!swapped.is_proven_empty());
    assert_eq!(
        ordered_ranges(&swapped, true),
        ordered_ranges(&forward, false)
    );
}
