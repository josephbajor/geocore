//! General bounded curve/curve dispatch over supported analytic classes.

use std::error::Error as _;

use kcore::error::{ClassifiedError, Error, ErrorClass};
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
    SessionPolicy,
};
use kcore::proof::Completion;
use kcore::tolerance::Tolerances;
use kgeom::aabb::Aabb3;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::project::ProjectionBudgetProfile;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    CURVE_CURVE_CLASS_PAIR, ContactKind, CurveClass, IntersectionError, ParamOrientation,
    UNSUPPORTED_CLASS_PAIR, intersect_bounded_curves, intersect_bounded_curves_in_scope,
    intersect_bounded_curves_with_context,
};

fn line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

fn full_range(curve: &dyn Curve) -> ParamRange {
    curve.param_range()
}

struct UnsupportedCurve;

impl Curve for UnsupportedCurve {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, _t: f64, _order: usize) -> kgeom::curve::CurveDerivs {
        kgeom::curve::CurveDerivs::default()
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::new(0.0, 1.0)
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, _range: ParamRange) -> Aabb3 {
        Aabb3::from_points(&[Point3::new(0.0, 0.0, 0.0)])
    }
}

#[test]
fn dispatches_line_line_and_line_ellipse() {
    let a = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let b = line([0.5, -1.0, 0.0], [0.0, 1.0, 0.0]);
    let hit = intersect_bounded_curves(
        &a,
        ParamRange::new(0.0, 1.0),
        &b,
        ParamRange::new(0.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].point, Point3::new(0.5, 0.0, 0.0));

    let ellipse = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let hit = intersect_bounded_curves(
        &a,
        ParamRange::new(-4.0, 4.0),
        &ellipse,
        full_range(&ellipse),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
}

#[test]
fn dispatches_line_nurbs_both_orders() {
    let line = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        None,
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &line,
        ParamRange::new(-2.0, 2.0),
        &curve,
        full_range(&curve),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].t_a.abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);

    let reversed = intersect_bounded_curves(
        &curve,
        full_range(&curve),
        &line,
        ParamRange::new(-2.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(reversed.points.len(), 1);
    assert_eq!(reversed.completion(), hit.completion());
    assert!((reversed.points[0].t_a - 0.5).abs() < 1e-8);
    assert!(reversed.points[0].t_b.abs() < 1e-8);

    let contained = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &contained,
        full_range(&contained),
        &line,
        ParamRange::new(1.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!((hit.overlaps[0].a.lo - 1.0 / 3.0).abs() < 1e-8);
    assert!((hit.overlaps[0].a.hi - 2.0 / 3.0).abs() < 1e-8);
    assert_eq!(hit.overlaps[0].b, ParamRange::new(1.0, 2.0));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);
    assert_eq!(
        hit.completion().indeterminate_reason(),
        Some("fixed-grid line/NURBS candidate discovery does not prove complete coverage")
    );
}

#[test]
fn dispatches_circle_nurbs_both_orders() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &circle,
        full_range(&circle),
        &curve,
        full_range(&curve),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
    assert!(hit.points[0].t_a.abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.75).abs() < 1e-8);
    assert!((hit.points[1].t_a - core::f64::consts::PI).abs() < 1e-8);
    assert!((hit.points[1].t_b - 0.25).abs() < 1e-8);

    let reversed = intersect_bounded_curves(
        &curve,
        full_range(&curve),
        &circle,
        full_range(&circle),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(reversed.points.len(), 2);
    assert!((reversed.points[0].t_a - 0.25).abs() < 1e-8);
    assert!((reversed.points[0].t_b - core::f64::consts::PI).abs() < 1e-8);
    assert!((reversed.points[1].t_a - 0.75).abs() < 1e-8);
    assert!(reversed.points[1].t_b.abs() < 1e-8);

    let quarter = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &quarter,
        full_range(&quarter),
        &circle,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].a.lo.abs() < 1e-8);
    assert!((hit.overlaps[0].a.hi - 0.5).abs() < 1e-8);
    assert_eq!(
        hit.overlaps[0].b,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4)
    );
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);
}

#[test]
fn dispatches_ellipse_nurbs_both_orders() {
    let ellipse = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-3.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &ellipse,
        full_range(&ellipse),
        &curve,
        full_range(&curve),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
    assert!(hit.points[0].t_a.abs() < 1e-8);
    assert!((hit.points[0].t_b - 5.0 / 6.0).abs() < 1e-8);
    assert!((hit.points[1].t_a - core::f64::consts::PI).abs() < 1e-8);
    assert!((hit.points[1].t_b - 1.0 / 6.0).abs() < 1e-8);

    let reversed = intersect_bounded_curves(
        &curve,
        full_range(&curve),
        &ellipse,
        full_range(&ellipse),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(reversed.points.len(), 2);
    assert!((reversed.points[0].t_a - 1.0 / 6.0).abs() < 1e-8);
    assert!((reversed.points[0].t_b - core::f64::consts::PI).abs() < 1e-8);
    assert!((reversed.points[1].t_a - 5.0 / 6.0).abs() < 1e-8);
    assert!(reversed.points[1].t_b.abs() < 1e-8);

    let quarter = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &quarter,
        full_range(&quarter),
        &ellipse,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].a.lo.abs() < 1e-8);
    assert!((hit.overlaps[0].a.hi - 0.5).abs() < 1e-8);
    assert_eq!(
        hit.overlaps[0].b,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4)
    );
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);
}

#[test]
fn dispatches_nurbs_nurbs() {
    let a = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        None,
    )
    .unwrap();
    let b = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &a,
        full_range(&a),
        &b,
        full_range(&b),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!((hit.points[0].t_a - 0.5).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);

    let contained = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let reversed = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let hit = intersect_bounded_curves(
        &contained,
        full_range(&contained),
        &reversed,
        full_range(&reversed),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);
}

#[test]
fn reversed_dispatch_recanonicalizes_in_first_curve_order() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let line = line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_curves(
        &circle,
        full_range(&circle),
        &line,
        ParamRange::new(0.0, 4.0),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.completion(), Completion::Complete);
    assert_eq!(hit.points.len(), 2);
    assert!(hit.points[0].t_a.abs() < 1e-12);
    assert!((hit.points[0].t_b - 3.0).abs() < 1e-12);
    assert!((hit.points[1].t_a - core::f64::consts::PI).abs() < 1e-12);
    assert!((hit.points[1].t_b - 1.0).abs() < 1e-12);
}

#[test]
fn reversed_dispatch_preserves_complete_miss_evidence() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let line = line([-2.0, 2.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_curves(
        &circle,
        full_range(&circle),
        &line,
        ParamRange::new(0.0, 4.0),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.completion(), Completion::Complete);
    assert!(hit.is_proven_empty());
}

#[test]
fn dispatches_circle_ellipse_and_ellipse_ellipse() {
    let circle = Circle::new(Frame::world(), 2.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let hit = intersect_bounded_curves(
        &circle,
        full_range(&circle),
        &ellipse,
        full_range(&ellipse),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 4);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );

    let other = Ellipse::new(Frame::world(), 2.0, 1.5).unwrap();
    let hit = intersect_bounded_curves(
        &ellipse,
        full_range(&ellipse),
        &other,
        full_range(&other),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 4);
}

#[test]
fn contextual_dispatch_reuses_one_scope_and_preserves_projection_limits() {
    let a = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let b = Ellipse::new(Frame::world(), 2.0, 1.5).unwrap();
    let range = ParamRange::new(0.0, core::f64::consts::TAU);
    let tolerances = Tolerances::default();
    let legacy = intersect_bounded_curves(&a, range, &b, range, tolerances).unwrap();

    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let contextual = intersect_bounded_curves_with_context(&a, range, &b, range, &context);
    assert_eq!(contextual.result(), Ok(&legacy));
    let queries = contextual
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == kgeom::project::CURVE_PROJECTION_QUERIES)
        .unwrap();
    assert!(queries.consumed > 1);

    let composed = context
        .clone()
        .with_family_budget_defaults(ProjectionBudgetProfile::curve_aggregate_compatibility());
    let mut scope = OperationScope::new(&composed);
    let scoped = intersect_bounded_curves_in_scope(&a, range, &b, range, &mut scope);
    let scoped = scope.finish_typed(scoped);
    assert_eq!(scoped, contextual);

    let allowed = queries.consumed - 1;
    let limited = BudgetPlan::new([LimitSpec::new(
        kgeom::project::CURVE_PROJECTION_QUERIES,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    let limited_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(limited);
    let outcome = intersect_bounded_curves_with_context(&a, range, &b, range, &limited_context);
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, kgeom::project::CURVE_PROJECTION_QUERIES);
    assert_eq!(
        (crossing.consumed, crossing.allowed),
        (queries.consumed, allowed)
    );
    assert_eq!(outcome.report().limit_events(), &[crossing]);
}

#[test]
fn unsupported_curve_class_is_explicit_error() {
    let a = UnsupportedCurve;
    let b = line([0.5, -1.0, 0.0], [0.0, 1.0, 0.0]);
    let err = intersect_bounded_curves(
        &a,
        full_range(&a),
        &b,
        full_range(&b),
        Tolerances::default(),
    )
    .unwrap_err();

    assert_eq!(
        err,
        IntersectionError::UnsupportedCurvePair {
            class_a: None,
            class_b: Some(CurveClass::Line.key()),
        }
    );
    assert_eq!(err.class(), ErrorClass::Unsupported);
    assert_eq!(err.code(), UNSUPPORTED_CLASS_PAIR);
    assert_eq!(err.capability(), Some(CURVE_CURVE_CLASS_PAIR));
    assert!(err.source().is_none());

    let classified: &dyn ClassifiedError = &err;
    assert_eq!(classified.class(), ErrorClass::Unsupported);
}

#[test]
fn supported_dispatch_preserves_kernel_error_classification_and_source() {
    let a = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let b = line([0.0, 1.0, 0.0], [1.0, 0.0, 0.0]);
    let invalid_range = ParamRange { lo: 1.0, hi: 0.0 };
    let err = intersect_bounded_curves(
        &a,
        invalid_range,
        &b,
        ParamRange::new(0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();

    let kernel = Error::InvalidGeometry {
        reason: "line intersection requires a finite non-reversed range",
    };
    assert_eq!(err, IntersectionError::Kernel(kernel.clone()));
    assert_eq!(err.class(), kernel.class());
    assert_eq!(err.code(), kernel.code());
    assert_eq!(err.capability(), kernel.capability());
    assert_eq!(err.limit(), kernel.limit());
    assert_eq!(
        err.source()
            .and_then(|source| source.downcast_ref::<Error>()),
        Some(&kernel)
    );
}
