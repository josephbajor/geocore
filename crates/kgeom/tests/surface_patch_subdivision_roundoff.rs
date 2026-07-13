//! Public regression for source-provenance bounds after rounded surface subdivision.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationContext, OperationReport,
    OperationScope, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::nurbs::{
    ImplicitPatchIsolation, NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, NurbsSurface, NurbsSurfaceBvh,
};
use kgeom::surface::{Dir, Plane};
use kgeom::vec::{Point3, Vec3};

const CONTACT_Z: f64 = 9_007_199_254_740_991.0;
const CUBIC_Z: [i64; 4] = [
    9_007_199_254_740_360,
    9_007_199_254_740_978,
    9_007_199_254_741_648,
    9_007_199_254_739_690,
];
const SOURCE_BUILD_WORK: u64 = 7; // one tensor slot * (6 scans + centered point)
const SOURCE_CHILD_WORK: u64 = 29; // one subdivision + four source ranges
const EXACT_CONTEXT_WORK: u64 = SOURCE_BUILD_WORK + SOURCE_CHILD_WORK;

fn adversary() -> NurbsSurface {
    let xs = [-1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0];
    let mut points = Vec::with_capacity(8);
    for (x, z) in xs.into_iter().zip(CUBIC_Z) {
        points.push(Point3::new(x, -1.0, z as f64));
        points.push(Point3::new(x, 1.0, z as f64));
    }
    NurbsSurface::new(
        3,
        1,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        points,
        None,
    )
    .unwrap()
}

fn contextual_isolation(
    surface: &NurbsSurface,
    plane: &Plane,
    work_allowed: u64,
) -> (ImplicitPatchIsolation, OperationReport) {
    let budget = BudgetPlan::new([
        LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            work_allowed,
        ),
        LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            8,
        ),
        LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            1,
        ),
    ])
    .unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(budget);
    let mut scope = OperationScope::new(&context);
    let hierarchy = NurbsSurfaceBvh::build_in_scope(surface, &mut scope).unwrap();
    let isolation = hierarchy
        .isolate_implicit_candidates_in_scope(plane, 0.0, 1, &mut scope)
        .unwrap();
    let (result, report) = scope.finish(Ok(isolation)).into_parts();
    (result.unwrap(), report)
}

#[test]
fn rounded_depth_one_patch_hulls_cannot_erase_exact_source_contact() {
    let surface = adversary();

    // At u = 1/2 the exact cubic Bezier numerator is p0+3p1+3p2+p3.
    // Integer arithmetic establishes z = CONTACT_Z independently of the
    // floating evaluator; the x numerator is identically zero. Extrusion in
    // v therefore gives a complete exact contact line with this plane.
    let z_numerator = i128::from(CUBIC_Z[0])
        + 3 * i128::from(CUBIC_Z[1])
        + 3 * i128::from(CUBIC_Z[2])
        + i128::from(CUBIC_Z[3]);
    assert_eq!(z_numerator, 8 * i128::from(CONTACT_Z as i64));
    assert_eq!(-1.0 + 3.0 * (-1.0 / 3.0) + 3.0 * (1.0 / 3.0) + 1.0, 0.0);

    let (low, high) = surface.split_at(Dir::U, 0.5).unwrap();
    assert!(
        low.points()
            .iter()
            .chain(high.points())
            .all(|point| point.z < CONTACT_Z),
        "the rounded child control hulls deliberately lose the exact source contact"
    );
    assert_eq!(low.points()[low.points().len() - 2].z, CONTACT_Z - 1.0);
    assert_eq!(high.points()[0].z, CONTACT_Z - 1.0);

    let plane = Plane::new(
        Frame::from_z(Point3::new(0.0, 0.0, CONTACT_Z), Vec3::new(0.0, 0.0, 1.0)).unwrap(),
    );
    let hierarchy = NurbsSurfaceBvh::build(&surface).unwrap();
    let isolation = hierarchy
        .isolate_implicit_candidates(&plane, 0.0, 1, 8)
        .unwrap();
    let exact_contact = Point3::new(0.0, 0.0, CONTACT_Z);

    assert!(isolation.is_complete());
    assert!(!isolation.is_proven_empty());
    assert!(isolation.candidates().iter().all(|cell| cell.depth() == 1));
    assert!(isolation.candidates().iter().any(|cell| {
        let range = cell.parameter_range();
        range[0].contains(0.5) && range[1].contains(0.5) && cell.bounds().contains(exact_contact)
    }));

    let repeated = hierarchy
        .isolate_implicit_candidates(&plane, 0.0, 1, 8)
        .unwrap();
    assert_eq!(repeated, isolation);

    let (contextual, exact_report) = contextual_isolation(&surface, &plane, EXACT_CONTEXT_WORK);
    assert_eq!(contextual, isolation);
    let work_usage = exact_report
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS)
        .unwrap();
    assert_eq!(work_usage.consumed, EXACT_CONTEXT_WORK);

    let (denied, denied_report) = contextual_isolation(&surface, &plane, EXACT_CONTEXT_WORK - 1);
    let denied_snapshot = LimitSnapshot {
        stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
        resource: ResourceKind::Work,
        consumed: EXACT_CONTEXT_WORK,
        allowed: EXACT_CONTEXT_WORK - 1,
    };
    assert_eq!(denied.limits().subdivision_work(), Some(denied_snapshot));
    assert_eq!(denied_report.limit_events(), &[denied_snapshot]);
    assert_eq!(denied.candidates().len(), 1);
    assert_eq!(denied.candidates()[0].depth(), 0);
    assert!(denied.candidates()[0].bounds().contains(exact_contact));
    assert_eq!(
        denied_report
            .usage()
            .iter()
            .find(|snapshot| snapshot.stage == NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS)
            .unwrap()
            .consumed,
        SOURCE_BUILD_WORK,
        "denied child scan retains the parent without partially consuming Work"
    );
}
