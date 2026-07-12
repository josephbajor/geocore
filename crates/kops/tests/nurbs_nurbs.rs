//! Bounded NURBS/NURBS curve intersections.

use kcore::operation::{
    BudgetPlan, ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
    SessionPrecision,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kops::intersect::{
    ContactKind, ParamOrientation, intersect_bounded_nurbs_nurbs,
    intersect_bounded_nurbs_nurbs_with_context,
};

fn line_nurbs(start: Point3, end: Point3) -> NurbsCurve {
    line_nurbs_with_domain(start, end, 1.0)
}

fn line_nurbs_with_domain(start: Point3, end: Point3, hi: f64) -> NurbsCurve {
    NurbsCurve::new(1, vec![0.0, 0.0, hi, hi], vec![start, end], None).unwrap()
}

fn tangent_parabola() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap()
}

fn tangent_parabola_at_with_domain(vertex_parameter: f64, hi: f64) -> NurbsCurve {
    let q = vertex_parameter;
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, hi, hi, hi],
        vec![
            Point3::new(-1.0, q * q, 0.0),
            Point3::new(0.0, q * q - q, 0.0),
            Point3::new(1.0, (1.0 - q) * (1.0 - q), 0.0),
        ],
        None,
    )
    .unwrap()
}

#[test]
fn nurbs_nurbs_crossing_tangent_and_range_filtering() {
    let diagonal = line_nurbs(Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0));
    let horizontal = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let hit = intersect_bounded_nurbs_nurbs(
        &diagonal,
        diagonal.param_range(),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_a - 0.5).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);

    let range_miss = intersect_bounded_nurbs_nurbs(
        &diagonal,
        ParamRange::new(0.0, 0.49),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let tangent = tangent_parabola();
    let hit = intersect_bounded_nurbs_nurbs(
        &tangent,
        tangent.param_range(),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_a - 0.5).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);
}

#[test]
fn nurbs_nurbs_reports_simple_contained_overlaps() {
    let a = line_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let b = line_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let hit = intersect_bounded_nurbs_nurbs(
        &a,
        a.param_range(),
        &b,
        b.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);

    let reversed = line_nurbs(Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    let hit = intersect_bounded_nurbs_nurbs(
        &a,
        a.param_range(),
        &reversed,
        reversed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);
}

#[test]
fn contextual_v1_entry_is_exactly_legacy_compatible() {
    let a = tangent_parabola();
    let b = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let session = SessionPolicy::v1();
    for tolerances in [
        Tolerances::default(),
        Tolerances::with_linear(1.0e-5).unwrap(),
    ] {
        let legacy =
            intersect_bounded_nurbs_nurbs(&a, a.param_range(), &b, b.param_range(), tolerances);
        let context = OperationContext::new(&session, tolerances).unwrap();
        let contextual = intersect_bounded_nurbs_nurbs_with_context(
            &a,
            a.param_range(),
            &b,
            b.param_range(),
            &context,
        );
        assert_eq!(contextual.result(), legacy.as_ref());
        assert_eq!(contextual.report().policy_version(), PolicyVersion::V1);
        assert!(contextual.report().usage().is_empty());
        assert!(contextual.report().limit_events().is_empty());
        assert!(contextual.report().diagnostics().is_empty());
    }

    let invalid_range = ParamRange { lo: 0.75, hi: 0.25 };
    let legacy = intersect_bounded_nurbs_nurbs(
        &a,
        invalid_range,
        &b,
        b.param_range(),
        Tolerances::default(),
    );
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let contextual = intersect_bounded_nurbs_nurbs_with_context(
        &a,
        invalid_range,
        &b,
        b.param_range(),
        &context,
    );
    assert_eq!(contextual.result(), legacy.as_ref());
}

#[test]
fn nurbs_nurbs_is_stable_under_small_and_large_parameter_reparameterization() {
    let session = SessionPolicy::v1();
    for parameter_scale in [1.0e-13, 1.0, 1.0e13] {
        let diagonal = line_nurbs_with_domain(
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            parameter_scale,
        );
        let horizontal = line_nurbs_with_domain(
            Point3::new(-2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            parameter_scale,
        );
        let tolerances = Tolerances::default();
        let legacy = intersect_bounded_nurbs_nurbs(
            &diagonal,
            diagonal.param_range(),
            &horizontal,
            horizontal.param_range(),
            tolerances,
        )
        .unwrap();
        assert_eq!(
            legacy.points.len(),
            1,
            "parameter scale {parameter_scale:e}"
        );
        assert!(legacy.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) <= 1.0e-8);
        assert!((legacy.points[0].t_a / parameter_scale - 0.5).abs() <= 1.0e-8);
        assert!((legacy.points[0].t_b / parameter_scale - 0.5).abs() <= 1.0e-8);

        let context = OperationContext::new(&session, tolerances).unwrap();
        let contextual = intersect_bounded_nurbs_nurbs_with_context(
            &diagonal,
            diagonal.param_range(),
            &horizontal,
            horizontal.param_range(),
            &context,
        );
        assert_eq!(contextual.result(), Ok(&legacy));
    }
}

#[test]
fn coarse_custom_progress_policy_can_stop_but_cannot_accept_a_contact() {
    let diagonal = line_nurbs(Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0));
    let horizontal = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let tolerances = Tolerances::default();

    let default = intersect_bounded_nurbs_nurbs(
        &diagonal,
        diagonal.param_range(),
        &horizontal,
        horizontal.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(default.points.len(), 1);

    let numerical = NumericalPolicy::try_new(32.0, 1.0e16, 128.0 * f64::EPSILON).unwrap();
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        numerical,
        ExecutionPolicy::Available,
        BudgetPlan::empty(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, tolerances).unwrap();
    let stopped = intersect_bounded_nurbs_nurbs_with_context(
        &diagonal,
        diagonal.param_range(),
        &horizontal,
        horizontal.param_range(),
        &context,
    );
    let stopped = stopped.result().unwrap();
    assert!(stopped.points.is_empty());
    assert!(stopped.overlaps.is_empty());
    assert!(!stopped.is_complete());
}

#[test]
fn normalized_gradient_stop_is_reparameterization_swap_and_context_stable() {
    let q = 0.371_234;
    let expected_point = Point3::new(2.0 * q - 1.0, 0.0, 0.0);
    let expected_horizontal_parameter = (2.0 * q + 1.0) / 4.0;
    let tolerances = Tolerances::default();
    let session = SessionPolicy::v1();

    for parameter_scale in [1.0e-6, 1.0, 1.0e3] {
        let parabola = tangent_parabola_at_with_domain(q, parameter_scale);
        let horizontal = line_nurbs_with_domain(
            Point3::new(-2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            parameter_scale,
        );
        let forward = intersect_bounded_nurbs_nurbs(
            &parabola,
            parabola.param_range(),
            &horizontal,
            horizontal.param_range(),
            tolerances,
        )
        .unwrap();
        let swapped = intersect_bounded_nurbs_nurbs(
            &horizontal,
            horizontal.param_range(),
            &parabola,
            parabola.param_range(),
            tolerances,
        )
        .unwrap();
        assert_eq!(
            forward.points.len(),
            1,
            "parameter scale {parameter_scale:e}: {:?}",
            forward.points
        );
        assert_eq!(
            swapped.points.len(),
            1,
            "parameter scale {parameter_scale:e}"
        );
        assert_eq!(forward.points[0].kind, ContactKind::Tangent);
        assert_eq!(swapped.points[0].kind, ContactKind::Tangent);
        assert!(
            forward.points[0].point.dist(expected_point) <= tolerances.linear().sqrt(),
            "parameter scale {parameter_scale:e}: {:?} != {expected_point:?}",
            forward.points[0]
        );
        assert!(
            swapped.points[0].point.dist(expected_point) <= tolerances.linear().sqrt(),
            "parameter scale {parameter_scale:e}: {:?} != {expected_point:?}",
            swapped.points[0]
        );
        assert!((forward.points[0].t_a / parameter_scale - q).abs() <= 1.0e-4);
        assert!(
            (forward.points[0].t_b / parameter_scale - expected_horizontal_parameter).abs()
                <= 1.0e-4
        );
        assert_eq!(forward.points[0].t_a, swapped.points[0].t_b);
        assert_eq!(forward.points[0].t_b, swapped.points[0].t_a);
        assert_eq!(forward.points[0].point, swapped.points[0].point);
        assert_eq!(forward.points[0].residual, swapped.points[0].residual);

        if parameter_scale == 1.0 {
            let context = OperationContext::new(&session, tolerances).unwrap();
            let contextual = intersect_bounded_nurbs_nurbs_with_context(
                &parabola,
                parabola.param_range(),
                &horizontal,
                horizontal.param_range(),
                &context,
            );
            assert_eq!(contextual.result(), Ok(&forward));
        }
    }
}

#[test]
fn coarse_custom_gradient_policy_can_stop_but_cannot_accept_a_contact() {
    let q = 0.371_234;
    let parabola = tangent_parabola_at_with_domain(q, 1.0);
    let horizontal = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let tolerances = Tolerances::default();
    let default = intersect_bounded_nurbs_nurbs(
        &parabola,
        parabola.param_range(),
        &horizontal,
        horizontal.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(default.points.len(), 1);

    let numerical = NumericalPolicy::try_new(1.0e16, 64.0, 128.0 * f64::EPSILON).unwrap();
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        numerical,
        ExecutionPolicy::Available,
        BudgetPlan::empty(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, tolerances).unwrap();
    let stopped = intersect_bounded_nurbs_nurbs_with_context(
        &parabola,
        parabola.param_range(),
        &horizontal,
        horizontal.param_range(),
        &context,
    );
    let stopped = stopped.result().unwrap();
    assert!(stopped.points.is_empty());
    assert!(stopped.overlaps.is_empty());
    assert!(!stopped.is_complete());
}
