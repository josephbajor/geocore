//! Bounded NURBS/NURBS curve intersections.

use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticKind, DiagnosticLevel, ExecutionPolicy, LimitSpec,
    NumericalPolicy, OperationContext, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision,
};
use kcore::proof::{IncompleteCause, IncompleteEvidence};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kops::intersect::{
    ContactKind, NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE,
    NURBS_CURVE_PAIR_ISOLATION_SUBDIVISION_LIMIT, NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION,
    NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE, NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE_LIMIT,
    NURBS_CURVE_PAIR_POLISH_FALLBACK, NURBS_CURVE_PAIR_POLISH_STATIONARY,
    NURBS_CURVE_PAIR_SEED_ATTEMPTS, NURBS_CURVE_PAIR_SEED_LIMIT, ParamOrientation,
    intersect_bounded_curves_with_context, intersect_bounded_nurbs_nurbs,
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
    assert!(hit.is_complete());
    assert!(hit.incomplete_evidence().is_empty());
    assert_eq!(hit.root_certificates().len(), 1);
    assert!(
        hit.root_certificates()
            .iter()
            .all(|certificate| certificate.determinant_lower_bound() > 0.0)
    );
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
    assert!(range_miss.is_proven_empty());
    assert!(range_miss.incomplete_evidence().is_empty());

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
    assert!(!hit.is_complete());
    assert!(hit.root_certificates().is_empty());
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_a - 0.5).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);
}

#[test]
fn cell_local_discovery_retains_multiple_roots_and_verified_witnesses() {
    let arch = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
        None,
    )
    .unwrap();
    let line = line_nurbs(Point3::new(-2.0, 0.5, 0.0), Point3::new(2.0, 0.5, 0.0));
    let tolerances = Tolerances::default();
    let forward = intersect_bounded_nurbs_nurbs(
        &arch,
        arch.param_range(),
        &line,
        line.param_range(),
        tolerances,
    )
    .unwrap();
    let swapped = intersect_bounded_nurbs_nurbs(
        &line,
        line.param_range(),
        &arch,
        arch.param_range(),
        tolerances,
    )
    .unwrap();

    assert_eq!(forward.points.len(), 2, "{:?}", forward.points);
    assert_eq!(swapped.points.len(), 2);
    assert!(forward.is_complete());
    assert!(forward.incomplete_evidence().is_empty());
    assert_eq!(forward.root_certificates().len(), 2);
    assert_eq!(forward.clone().swapped(), swapped);
    for (point, reversed) in forward.points.iter().zip(&swapped.points) {
        assert!(arch.param_range().contains(point.t_a));
        assert!(line.param_range().contains(point.t_b));
        assert!(point.residual <= tolerances.linear());
        assert_eq!(point.t_a, reversed.t_b);
        assert_eq!(point.t_b, reversed.t_a);
        assert_eq!(point.point, reversed.point);
        assert_eq!(point.residual, reversed.residual);
    }
}

#[test]
fn joined_components_complete_rational_boundary_roots_under_swap() {
    let rational = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        Some(vec![1.0, 1.5]),
    )
    .unwrap();
    let horizontal = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let forward = intersect_bounded_nurbs_nurbs(
        &rational,
        rational.param_range(),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_nurbs_nurbs(
        &horizontal,
        horizontal.param_range(),
        &rational,
        rational.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(forward.is_complete());
    assert_eq!(forward.points.len(), 1);
    assert_eq!(forward.root_certificates().len(), 1);
    assert!(forward.incomplete_evidence().is_empty());
    assert_eq!(forward.clone().swapped(), reversed);
}

#[test]
fn exact_spatial_corner_certificate_completes_the_non_coplanar_solve() {
    let first = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(2.0, 2.0, 0.25),
        ],
        None,
    )
    .unwrap();
    let second = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-2.0, 2.0, 0.75),
        ],
        None,
    )
    .unwrap();

    let forward = intersect_bounded_nurbs_nurbs(
        &first,
        first.param_range(),
        &second,
        second.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_nurbs_nurbs(
        &second,
        second.param_range(),
        &first,
        first.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(forward.is_complete());
    assert_eq!(forward.points.len(), 1);
    assert_eq!(forward.root_certificates().len(), 1);
    assert!(forward.incomplete_evidence().is_empty());
    assert_eq!(forward.clone().swapped(), reversed);
}

#[test]
fn exact_spatial_interior_knot_certificate_completes_the_non_coplanar_solve() {
    let first = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-2.0, -2.0, -1.0),
            Point3::new(-1.0, -1.0, 2.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, -2.0),
            Point3::new(2.0, 2.0, 1.0),
        ],
        Some(vec![1.0, 1.001, 1.002, 1.001, 1.0]),
    )
    .unwrap();
    let second = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-2.0, 2.0, 3.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, -1.0, 4.0),
            Point3::new(2.0, -2.0, -3.0),
        ],
        Some(vec![1.002, 1.001, 1.0, 1.001, 1.002]),
    )
    .unwrap();

    assert_eq!(first.eval(0.5), Point3::new(0.0, 0.0, 0.0));
    assert_eq!(second.eval(0.5), Point3::new(0.0, 0.0, 0.0));
    let forward = intersect_bounded_nurbs_nurbs(
        &first,
        first.param_range(),
        &second,
        second.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_nurbs_nurbs(
        &second,
        second.param_range(),
        &first,
        first.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(forward.is_complete(), "{forward:?}");
    assert_eq!(forward.points.len(), 1);
    assert_eq!(forward.root_certificates().len(), 1);
    assert_eq!(forward.points[0].point, Point3::new(0.0, 0.0, 0.0));
    assert_eq!(forward.clone().swapped(), reversed);
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
    assert!(hit.is_complete());
    assert!(hit.incomplete_evidence().is_empty());
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
    assert!(hit.is_complete());
    assert!(hit.incomplete_evidence().is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);

    let parameter_scale = 1.0e13;
    let scaled_a = line_nurbs_with_domain(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        parameter_scale,
    );
    let scaled_b = line_nurbs_with_domain(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        parameter_scale,
    );
    let scaled = intersect_bounded_nurbs_nurbs(
        &scaled_a,
        scaled_a.param_range(),
        &scaled_b,
        scaled_b.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(scaled.points.is_empty());
    assert!(scaled.is_complete());
    assert!(scaled.incomplete_evidence().is_empty());
    assert_eq!(scaled.overlaps.len(), 1);
    assert_eq!(scaled.overlaps[0].a, ParamRange::new(0.0, parameter_scale));
    assert_eq!(scaled.overlaps[0].b, ParamRange::new(0.0, parameter_scale));
    assert_eq!(scaled.overlaps[0].orientation, ParamOrientation::Same);

    let subrange = ParamRange::new(0.25 * parameter_scale, 0.75 * parameter_scale);
    let subrange_overlap = intersect_bounded_nurbs_nurbs(
        &scaled_a,
        subrange,
        &scaled_b,
        subrange,
        Tolerances::default(),
    )
    .unwrap();
    assert!(subrange_overlap.is_complete());
    assert_eq!(subrange_overlap.overlaps[0].a, subrange);
    assert_eq!(subrange_overlap.overlaps[0].b, subrange);

    let near = line_nurbs(Point3::new(0.0, 0.5e-8, 0.0), Point3::new(3.0, 0.5e-8, 0.0));
    let sampled = intersect_bounded_nurbs_nurbs(
        &a,
        a.param_range(),
        &near,
        near.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(sampled.overlaps.len(), 1);
    assert!(!sampled.is_complete());
    assert_eq!(sampled.incomplete_evidence().len(), 1);
}

#[test]
fn exact_reversed_overlap_handles_asymmetric_knots() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(3.0, 0.5, 0.0),
    ];
    let forward = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.25, 1.0, 1.0, 1.0],
        points.clone(),
        Some(vec![1.0, 0.75, 1.25, 2.0]),
    )
    .unwrap();
    let reversed = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.75, 1.0, 1.0, 1.0],
        points.into_iter().rev().collect(),
        Some(vec![2.0, 1.25, 0.75, 1.0]),
    )
    .unwrap();
    let result = intersect_bounded_nurbs_nurbs(
        &forward,
        forward.param_range(),
        &reversed,
        reversed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(result.is_complete());
    assert!(result.points.is_empty());
    assert_eq!(result.overlaps.len(), 1);
    assert_eq!(result.overlaps[0].orientation, ParamOrientation::Reversed);
    assert_eq!(result.clone().swapped(), result);

    let lo = 1.0e308;
    let hi = 1.1e308;
    let forward = NurbsCurve::new(
        1,
        vec![lo, lo, hi, hi],
        vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let reversed = NurbsCurve::new(
        1,
        vec![lo, lo, hi, hi],
        vec![Point3::new(1.0, 0.0, 0.0), Point3::new(-1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let extreme = intersect_bounded_nurbs_nurbs(
        &forward,
        forward.param_range(),
        &reversed,
        reversed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(extreme.is_complete());
    assert_eq!(extreme.overlaps[0].orientation, ParamOrientation::Reversed);
}

#[test]
fn exact_affine_overlap_clips_ranges_and_accepts_global_weight_scaling() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let unit = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points.clone(),
        Some(vec![1.0, 1.5, 2.0]),
    )
    .unwrap();
    let shifted = NurbsCurve::new(
        2,
        vec![2.0, 2.0, 2.0, 6.0, 6.0, 6.0],
        points,
        Some(vec![2.0, 3.0, 4.0]),
    )
    .unwrap();
    let shifted_reversed = NurbsCurve::new(
        2,
        vec![2.0, 2.0, 2.0, 6.0, 6.0, 6.0],
        shifted.points().iter().copied().rev().collect(),
        Some(vec![4.0, 3.0, 2.0]),
    )
    .unwrap();

    let result = intersect_bounded_nurbs_nurbs(
        &unit,
        ParamRange::new(0.25, 0.75),
        &shifted,
        ParamRange::new(2.0, 4.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(result.is_complete());
    assert!(result.points.is_empty());
    assert!(result.incomplete_evidence().is_empty());
    assert_eq!(result.overlaps.len(), 1);
    assert_eq!(result.overlaps[0].a, ParamRange::new(0.25, 0.5));
    assert_eq!(result.overlaps[0].b, ParamRange::new(3.0, 4.0));
    assert_eq!(result.overlaps[0].orientation, ParamOrientation::Same);

    let swapped = intersect_bounded_nurbs_nurbs(
        &shifted,
        ParamRange::new(2.0, 4.0),
        &unit,
        ParamRange::new(0.25, 0.75),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(result.swapped(), swapped);

    let reversed = intersect_bounded_nurbs_nurbs(
        &unit,
        ParamRange::new(0.25, 0.75),
        &shifted_reversed,
        ParamRange::new(2.0, 4.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(reversed.is_complete());
    assert_eq!(reversed.overlaps[0].a, ParamRange::new(0.5, 0.75));
    assert_eq!(reversed.overlaps[0].b, ParamRange::new(3.0, 4.0));
    assert_eq!(reversed.overlaps[0].orientation, ParamOrientation::Reversed);
}

#[test]
fn exact_knot_refinement_equivalence_preserves_complete_clipped_overlaps() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let coarse = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points.clone(),
        Some(vec![1.0, 1.5, 2.0]),
    )
    .unwrap();
    let refined = coarse.with_knot_inserted(0.5, 1).unwrap();
    let same = intersect_bounded_nurbs_nurbs(
        &coarse,
        ParamRange::new(0.25, 0.75),
        &refined,
        ParamRange::new(0.5, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(same.is_complete());
    assert_eq!(same.overlaps[0].a, ParamRange::new(0.5, 0.75));
    assert_eq!(same.overlaps[0].b, ParamRange::new(0.5, 0.75));
    assert_eq!(same.overlaps[0].orientation, ParamOrientation::Same);

    let refined_twice = coarse.with_knots_refined(&[0.25, 0.75]).unwrap();
    let repeated = intersect_bounded_nurbs_nurbs(
        &coarse,
        coarse.param_range(),
        &refined_twice,
        refined_twice.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(repeated.is_complete());
    assert_eq!(repeated.overlaps[0].a, coarse.param_range());
    assert_eq!(repeated.overlaps[0].b, coarse.param_range());

    let shifted = NurbsCurve::new(
        2,
        vec![2.0, 2.0, 2.0, 6.0, 6.0, 6.0],
        points.clone(),
        Some(vec![2.0, 3.0, 4.0]),
    )
    .unwrap()
    .with_knot_inserted(4.0, 1)
    .unwrap();
    let affine = intersect_bounded_nurbs_nurbs(
        &coarse,
        ParamRange::new(0.25, 0.75),
        &shifted,
        ParamRange::new(2.0, 4.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(affine.is_complete());
    assert_eq!(affine.overlaps[0].a, ParamRange::new(0.25, 0.5));
    assert_eq!(affine.overlaps[0].b, ParamRange::new(3.0, 4.0));
    let affine_swapped = intersect_bounded_nurbs_nurbs(
        &shifted,
        ParamRange::new(2.0, 4.0),
        &coarse,
        ParamRange::new(0.25, 0.75),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(affine.clone().swapped(), affine_swapped);

    let reversed = NurbsCurve::new(
        2,
        vec![2.0, 2.0, 2.0, 6.0, 6.0, 6.0],
        points.into_iter().rev().collect(),
        Some(vec![4.0, 3.0, 2.0]),
    )
    .unwrap()
    .with_knot_inserted(5.0, 1)
    .unwrap();
    let reversed = intersect_bounded_nurbs_nurbs(
        &coarse,
        ParamRange::new(0.125, 0.75),
        &reversed,
        ParamRange::new(2.0, 5.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(reversed.is_complete());
    assert_eq!(reversed.overlaps[0].a, ParamRange::new(0.25, 0.75));
    assert_eq!(reversed.overlaps[0].b, ParamRange::new(3.0, 5.0));
    assert_eq!(reversed.overlaps[0].orientation, ParamOrientation::Reversed);
}

#[test]
fn checked_inverse_refinement_recovers_histories_but_rejects_altered_data() {
    let coarse = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
        Some(vec![1.0, 1.5, 2.0]),
    )
    .unwrap();
    let refined = coarse.with_knot_inserted(0.5, 1).unwrap();
    let mut changed_points = refined.points().to_vec();
    changed_points[1].y += 1.0e-4;
    let changed = NurbsCurve::new(
        refined.degree(),
        refined.knots().as_slice().to_vec(),
        changed_points,
        refined.weights().map(<[f64]>::to_vec),
    )
    .unwrap();

    let result = intersect_bounded_nurbs_nurbs(
        &coarse,
        coarse.param_range(),
        &changed,
        changed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(!result.is_complete());
    assert!(result.overlaps.is_empty());

    let left_history = coarse.with_knot_inserted(0.25, 1).unwrap();
    let right_history = coarse.with_knot_inserted(0.75, 1).unwrap();
    let recovered = intersect_bounded_nurbs_nurbs(
        &left_history,
        left_history.param_range(),
        &right_history,
        right_history.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(recovered.is_complete());
    assert_eq!(recovered.overlaps.len(), 1);
    assert_eq!(recovered.overlaps[0].a, coarse.param_range());
    assert_eq!(recovered.overlaps[0].b, coarse.param_range());
}

#[test]
fn overlap_equivalence_work_has_exact_admission_boundaries() {
    let coarse = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
        Some(vec![1.0, 1.5, 2.0]),
    )
    .unwrap();
    let refined = coarse.with_knots_refined(&[0.25, 0.75]).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let baseline = intersect_bounded_nurbs_nurbs_with_context(
        &coarse,
        coarse.param_range(),
        &refined,
        refined.param_range(),
        &context,
    );
    assert!(baseline.result().unwrap().is_complete());
    let work = baseline
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE
                && usage.resource == ResourceKind::Work
        })
        .unwrap()
        .consumed;
    let items = baseline
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE
                && usage.resource == ResourceKind::Items
        })
        .unwrap()
        .consumed;
    assert!(work > 0 && items > 0);

    let exact = BudgetPlan::new([
        LimitSpec::new(
            NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            work,
        ),
        LimitSpec::new(
            NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
            ResourceKind::Items,
            AccountingMode::Cumulative,
            items,
        ),
    ])
    .unwrap();
    let admitted = intersect_bounded_nurbs_nurbs_with_context(
        &coarse,
        coarse.param_range(),
        &refined,
        refined.param_range(),
        &context.clone().with_budget_overrides(exact),
    );
    assert_eq!(admitted.result(), baseline.result());
    assert!(admitted.report().limit_events().is_empty());

    let denied = BudgetPlan::new([LimitSpec::new(
        NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        work - 1,
    )])
    .unwrap();
    let denied = intersect_bounded_nurbs_nurbs_with_context(
        &coarse,
        coarse.param_range(),
        &refined,
        refined.param_range(),
        &context.clone().with_budget_overrides(denied),
    );
    let denied_result = denied.result().unwrap();
    assert!(denied_result.is_empty());
    assert!(!denied_result.is_complete());
    assert_eq!(denied_result.incomplete_evidence().len(), 1);
    assert_eq!(
        denied_result.incomplete_evidence()[0].code,
        NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE_LIMIT
    );
    assert_eq!(denied.report().limit_events().len(), 1);
    let crossing = denied.report().limit_events()[0];
    assert_eq!(crossing.stage, NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE);
    assert_eq!(crossing.resource, ResourceKind::Work);
    assert_eq!((crossing.consumed, crossing.allowed), (work, work - 1));

    let item_denied = BudgetPlan::new([LimitSpec::new(
        NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
        ResourceKind::Items,
        AccountingMode::Cumulative,
        items - 1,
    )])
    .unwrap();
    let item_denied = intersect_bounded_nurbs_nurbs_with_context(
        &coarse,
        coarse.param_range(),
        &refined,
        refined.param_range(),
        &context.with_budget_overrides(item_denied),
    );
    assert!(!item_denied.result().unwrap().is_complete());
    assert_eq!(item_denied.report().limit_events().len(), 1);
    let crossing = item_denied.report().limit_events()[0];
    assert_eq!(crossing.stage, NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE);
    assert_eq!(crossing.resource, ResourceKind::Items);
    assert_eq!((crossing.consumed, crossing.allowed), (items, items - 1));
}

#[test]
fn exact_reversed_overlap_clips_to_the_shared_parameter_extent() {
    let forward = line_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0));
    let reversed = line_nurbs(Point3::new(4.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    let result = intersect_bounded_nurbs_nurbs(
        &forward,
        ParamRange::new(0.125, 0.75),
        &reversed,
        ParamRange::new(0.375, 0.875),
        Tolerances::default(),
    )
    .unwrap();

    assert!(result.is_complete());
    assert_eq!(result.overlaps.len(), 1);
    assert_eq!(result.overlaps[0].a, ParamRange::new(0.125, 0.625));
    assert_eq!(result.overlaps[0].b, ParamRange::new(0.375, 0.875));
    assert_eq!(result.overlaps[0].orientation, ParamOrientation::Reversed);
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
        assert_eq!(contextual.report().usage().len(), 6);
        assert!(
            contextual
                .report()
                .usage()
                .iter()
                .find(|usage| usage.stage == NURBS_CURVE_PAIR_SEED_ATTEMPTS)
                .unwrap()
                .consumed
                > 0
        );
        assert!(
            contextual
                .report()
                .usage()
                .iter()
                .any(|usage| usage.consumed > 0)
        );
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
fn cell_local_seed_budget_has_exact_boundaries_and_never_grants_completion() {
    let a = tangent_parabola();
    let b = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let tolerances = Tolerances::default();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let default = intersect_bounded_nurbs_nurbs_with_context(
        &a,
        a.param_range(),
        &b,
        b.param_range(),
        &context,
    );
    let reviewed = default.result().unwrap();
    assert_eq!(reviewed.points.len(), 1);
    assert!(!reviewed.is_complete());
    let used = default
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == NURBS_CURVE_PAIR_SEED_ATTEMPTS)
        .unwrap()
        .consumed;
    assert!(used > 0);

    let exact = BudgetPlan::new([LimitSpec::new(
        NURBS_CURVE_PAIR_SEED_ATTEMPTS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        used,
    )])
    .unwrap();
    let exact_context = context.clone().with_budget_overrides(exact);
    let exact = intersect_bounded_nurbs_nurbs_with_context(
        &a,
        a.param_range(),
        &b,
        b.param_range(),
        &exact_context,
    );
    assert_eq!(exact.result(), Ok(reviewed));
    assert!(exact.report().limit_events().is_empty());

    let denied = BudgetPlan::new([LimitSpec::new(
        NURBS_CURVE_PAIR_SEED_ATTEMPTS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        0,
    )])
    .unwrap();
    let denied_context = context.with_budget_overrides(denied);
    let denied = intersect_bounded_nurbs_nurbs_with_context(
        &a,
        a.param_range(),
        &b,
        b.param_range(),
        &denied_context,
    );
    let result = denied.result().unwrap();
    assert!(result.is_empty());
    assert!(!result.is_complete());
    assert_eq!(denied.report().limit_events().len(), 1);
    let crossing = denied.report().limit_events()[0];
    assert_eq!(crossing.stage, NURBS_CURVE_PAIR_SEED_ATTEMPTS);
    assert_eq!((crossing.consumed, crossing.allowed), (1, 0));
    assert_eq!(result.incomplete_evidence().len(), 2);
    assert_eq!(
        result.incomplete_evidence()[0].code,
        NURBS_CURVE_PAIR_SEED_LIMIT
    );
    assert_eq!(
        result.incomplete_evidence()[0].cause,
        IncompleteCause::Limit { snapshot: crossing }
    );
    assert_eq!(
        result.incomplete_evidence()[1].code,
        NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE
    );
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
        assert_eq!(legacy.points[0].kind, ContactKind::Transverse);
        assert!(legacy.is_complete());
        assert_eq!(legacy.root_certificates().len(), 1);
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

        let swapped = intersect_bounded_nurbs_nurbs(
            &horizontal,
            horizontal.param_range(),
            &diagonal,
            diagonal.param_range(),
            tolerances,
        )
        .unwrap();
        assert_eq!(swapped.points.len(), 1);
        assert_eq!(swapped.points[0].kind, ContactKind::Transverse);
        assert_eq!(legacy.points[0].t_a, swapped.points[0].t_b);
        assert_eq!(legacy.points[0].t_b, swapped.points[0].t_a);
        assert_eq!(legacy.points[0].point, swapped.points[0].point);
        assert_eq!(legacy.points[0].residual, swapped.points[0].residual);
        assert_eq!(legacy.incomplete_evidence(), swapped.incomplete_evidence());

        let swapped_contextual = intersect_bounded_nurbs_nurbs_with_context(
            &horizontal,
            horizontal.param_range(),
            &diagonal,
            diagonal.param_range(),
            &context,
        );
        assert_eq!(swapped_contextual.result(), Ok(&swapped));
    }
}

#[test]
fn contact_classification_is_stable_under_model_scale_translation_and_near_miss() {
    let parameter_scale = 1.0e13;
    let tolerances = Tolerances::default();
    let session = SessionPolicy::v1();
    for model_scale in [1.0e-6, 1.0, 1.0e2] {
        let origin = Point3::new(7.0, -3.0, 2.0);
        let diagonal = line_nurbs_with_domain(
            Point3::new(origin.x - model_scale, origin.y - model_scale, origin.z),
            Point3::new(origin.x + model_scale, origin.y + model_scale, origin.z),
            parameter_scale,
        );
        let horizontal = line_nurbs_with_domain(
            Point3::new(origin.x - 2.0 * model_scale, origin.y, origin.z),
            Point3::new(origin.x + 2.0 * model_scale, origin.y, origin.z),
            parameter_scale,
        );
        let forward = intersect_bounded_nurbs_nurbs(
            &diagonal,
            diagonal.param_range(),
            &horizontal,
            horizontal.param_range(),
            tolerances,
        )
        .unwrap();
        assert_eq!(forward.points.len(), 1, "model scale {model_scale:e}");
        assert_eq!(forward.points[0].kind, ContactKind::Transverse);
        assert!(forward.points[0].point.dist(origin) <= tolerances.linear());

        let context = OperationContext::new(&session, tolerances).unwrap();
        let contextual = intersect_bounded_nurbs_nurbs_with_context(
            &diagonal,
            diagonal.param_range(),
            &horizontal,
            horizontal.param_range(),
            &context,
        );
        assert_eq!(contextual.result(), Ok(&forward));

        let near_miss = line_nurbs_with_domain(
            Point3::new(
                origin.x - 2.0 * model_scale,
                origin.y,
                origin.z + 2.0 * tolerances.linear(),
            ),
            Point3::new(
                origin.x + 2.0 * model_scale,
                origin.y,
                origin.z + 2.0 * tolerances.linear(),
            ),
            parameter_scale,
        );
        let miss = intersect_bounded_nurbs_nurbs(
            &diagonal,
            diagonal.param_range(),
            &near_miss,
            near_miss.param_range(),
            tolerances,
        )
        .unwrap();
        assert!(miss.points.is_empty(), "model scale {model_scale:e}");
        assert!(miss.is_proven_empty(), "model scale {model_scale:e}");
    }
}

#[test]
fn control_hull_exclusion_keeps_the_tolerance_boundary_inclusive() {
    let tolerances = Tolerances::default();
    let base = line_nurbs(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let boundary = line_nurbs(
        Point3::new(-1.0, 0.0, tolerances.linear()),
        Point3::new(1.0, 0.0, tolerances.linear()),
    );
    let result = intersect_bounded_nurbs_nurbs(
        &base,
        base.param_range(),
        &boundary,
        boundary.param_range(),
        tolerances,
    )
    .unwrap();

    assert!(!result.is_proven_empty());
    assert!(!result.is_complete());
}

#[test]
fn adaptive_control_hull_cover_proves_hidden_miss_and_limits_remain_indeterminate() {
    let arch = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
        None,
    )
    .unwrap();
    let separated = line_nurbs(Point3::new(-1.0, 1.5, 0.0), Point3::new(1.0, 1.5, 0.0));
    assert!(
        arch.bounding_box(arch.param_range())
            .intersects(separated.bounding_box(separated.param_range()))
    );
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let forward = intersect_bounded_nurbs_nurbs_with_context(
        &arch,
        arch.param_range(),
        &separated,
        separated.param_range(),
        &context,
    );
    assert!(forward.result().unwrap().is_proven_empty());
    let reversed = intersect_bounded_nurbs_nurbs_with_context(
        &separated,
        separated.param_range(),
        &arch,
        arch.param_range(),
        &context,
    );
    assert!(reversed.result().unwrap().is_proven_empty());

    let work = *forward
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == kgeom::nurbs::NURBS_CURVE_PAIR_SUBDIVISIONS)
        .unwrap();
    assert!(work.consumed > 1);
    let allowed = work.consumed - 1;
    let limited = BudgetPlan::new([LimitSpec::new(
        kgeom::nurbs::NURBS_CURVE_PAIR_SUBDIVISIONS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    let limited_context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(limited);
    let limited = intersect_bounded_nurbs_nurbs_with_context(
        &arch,
        arch.param_range(),
        &separated,
        separated.param_range(),
        &limited_context,
    );
    let result = limited.result().unwrap();
    assert!(result.is_empty());
    assert!(!result.is_complete());
    let crossing = *limited.report().limit_events().last().unwrap();
    assert_eq!(crossing.stage, kgeom::nurbs::NURBS_CURVE_PAIR_SUBDIVISIONS);
    assert_eq!(
        (crossing.consumed, crossing.allowed),
        (work.consumed, allowed)
    );
    assert_eq!(result.incomplete_evidence().len(), 2);
    assert_eq!(
        result.incomplete_evidence()[0],
        IncompleteEvidence {
            code: NURBS_CURVE_PAIR_ISOLATION_SUBDIVISION_LIMIT,
            stage: crossing.stage,
            cause: IncompleteCause::Limit { snapshot: crossing },
            message: "NURBS curve-pair isolation subdivision limit reached",
        }
    );
    assert_eq!(
        result.incomplete_evidence()[1].code,
        NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE
    );
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
    let stopped_result = stopped.result().unwrap();
    assert!(stopped_result.points.is_empty());
    assert!(stopped_result.overlaps.is_empty());
    assert!(!stopped_result.is_complete());
    let generic = intersect_bounded_curves_with_context(
        &diagonal,
        diagonal.param_range(),
        &horizontal,
        horizontal.param_range(),
        &context,
    );
    assert_eq!(generic.result(), Ok(stopped_result));
}

#[test]
fn tangent_end_to_end_is_stable_at_the_small_parameter_extreme_and_under_swap() {
    let q = 0.371_234;
    let expected_point = Point3::new(2.0 * q - 1.0, 0.0, 0.0);
    let expected_horizontal_parameter = (2.0 * q + 1.0) / 4.0;
    let tolerances = Tolerances::default();
    let parameter_scale = 1.0e-13;
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
    assert_eq!(forward.points.len(), 1, "{:?}", forward.points);
    assert_eq!(swapped.points.len(), 1);
    assert_eq!(forward.points[0].kind, ContactKind::Tangent);
    assert_eq!(swapped.points[0].kind, ContactKind::Tangent);
    assert!(forward.points[0].point.dist(expected_point) <= tolerances.linear().sqrt());
    assert!(swapped.points[0].point.dist(expected_point) <= tolerances.linear().sqrt());
    assert!((forward.points[0].t_a / parameter_scale - q).abs() <= 1.0e-4);
    assert!(
        (forward.points[0].t_b / parameter_scale - expected_horizontal_parameter).abs() <= 1.0e-4
    );
    assert_eq!(forward.points[0].t_a, swapped.points[0].t_b);
    assert_eq!(forward.points[0].t_b, swapped.points[0].t_a);
    assert_eq!(forward.points[0].point, swapped.points[0].point);
    assert_eq!(forward.points[0].residual, swapped.points[0].residual);
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
    let context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_diagnostics(DiagnosticLevel::Summary, 32);
    let stopped = intersect_bounded_nurbs_nurbs_with_context(
        &parabola,
        parabola.param_range(),
        &horizontal,
        horizontal.param_range(),
        &context,
    );
    assert!(stopped.report().diagnostics().iter().any(|diagnostic| {
        diagnostic.code == NURBS_CURVE_PAIR_POLISH_STATIONARY
            && diagnostic.kind == DiagnosticKind::ProofIncomplete
    }));
    let stopped = stopped.result().unwrap();
    assert!(stopped.points.is_empty());
    assert!(stopped.overlaps.is_empty());
    assert!(!stopped.is_complete());
}

#[test]
fn near_tangent_fallback_and_failed_stationarity_are_reported_without_acceptance() {
    let tolerances = Tolerances::default();
    let offset = tolerances.linear() * (1.0 + 1.0e-6);
    let near_tangent = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.25 + offset, 0.0),
            Point3::new(0.0, -0.25 + offset, 0.0),
            Point3::new(1.0, 0.25 + offset, 0.0),
        ],
        None,
    )
    .unwrap();
    let horizontal = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let session = SessionPolicy::v1();
    let retain_root = BudgetPlan::new([LimitSpec::new(
        kgeom::nurbs::NURBS_CURVE_PAIR_DEPTH,
        ResourceKind::Depth,
        AccountingMode::HighWater,
        0,
    )])
    .unwrap();
    let context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(retain_root)
        .with_diagnostics(DiagnosticLevel::Summary, 32);
    let outcome = intersect_bounded_nurbs_nurbs_with_context(
        &near_tangent,
        near_tangent.param_range(),
        &horizontal,
        horizontal.param_range(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert!(result.is_empty());
    assert!(!result.is_complete());
    assert!(outcome.report().diagnostics().iter().any(|diagnostic| {
        diagnostic.code == NURBS_CURVE_PAIR_POLISH_FALLBACK
            && diagnostic.kind == DiagnosticKind::FallbackSelected
    }));
    assert!(outcome.report().diagnostics().iter().any(|diagnostic| {
        diagnostic.code == NURBS_CURVE_PAIR_POLISH_STATIONARY
            && diagnostic.kind == DiagnosticKind::ProofIncomplete
    }));
    assert!(outcome.report().diagnostics().iter().any(|diagnostic| {
        diagnostic.code == NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION
            && diagnostic.kind == DiagnosticKind::NumericResolution
    }));
    assert_eq!(
        outcome.report().numeric_resolution_stages(),
        &[NURBS_CURVE_PAIR_SEED_ATTEMPTS]
    );
}
