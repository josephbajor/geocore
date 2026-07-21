use super::*;
use crate::geom::Curve2dGeom;
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::vec::Vec2;

fn line(origin: [f64; 2], direction: [f64; 2]) -> Curve2dGeom {
    Curve2dGeom::Line(
        Line2d::new(
            Point2::new(origin[0], origin[1]),
            Vec2::new(direction[0], direction[1]),
        )
        .unwrap(),
    )
}

fn circle(center: [f64; 2], radius: f64) -> Curve2dGeom {
    Curve2dGeom::Circle(
        Circle2d::new(
            Point2::new(center[0], center[1]),
            radius,
            Vec2::new(1.0, 0.0),
        )
        .unwrap(),
    )
}

#[test]
fn certified_near_line_circle_join_is_local_and_does_not_hide_second_root() {
    let epsilon = 5.0e-13;
    let diameter = line([-1.0, 0.0], [1.0, 0.0]);
    let unit_circle = circle([0.0, 0.0], 1.0);
    let line_use = BoundedPcurveSpan::new(&diameter, 1.0, 2.0 + epsilon, Point2::default());
    let quarter_arc = BoundedPcurveSpan::new(
        &unit_circle,
        0.0,
        0.5 * core::f64::consts::PI,
        Point2::default(),
    );
    let join = CertifiedSharedJoin {
        left_parameter: 2.0 + epsilon,
        right_parameter: 0.0,
        evidence: CertifiedBoundedLoopJoin::new(2.0e-12).unwrap(),
    };

    assert_eq!(
        line_circle_relation(line_use, quarter_arc, &[(2.0 + epsilon, 0.0)], &[join]).unwrap(),
        PairRelation::Disjoint
    );
    assert_eq!(
        line_circle_relation(line_use, quarter_arc, &[(2.0 + epsilon, 0.0)], &[]).unwrap(),
        PairRelation::Indeterminate
    );

    let semicircle =
        BoundedPcurveSpan::new(&unit_circle, 0.0, core::f64::consts::PI, Point2::default());
    let full_diameter = BoundedPcurveSpan::new(&diameter, 0.0, 2.0 + epsilon, Point2::default());
    assert_ne!(
        line_circle_relation(full_diameter, semicircle, &[(2.0 + epsilon, 0.0)], &[join],).unwrap(),
        PairRelation::Disjoint
    );

    // One topology endpoint cannot discharge both roots of a near-tangent
    // carrier pair, even when a deliberately oversized mutation token
    // contains both parameter pairs.
    let near_tangent = line([0.999_999, -1.0], [0.0, 1.0]);
    let tangent_window = BoundedPcurveSpan::new(&near_tangent, 0.99, 1.01, Point2::default());
    let arc_window = BoundedPcurveSpan::new(&unit_circle, -0.01, 0.01, Point2::default());
    let tangent_join = CertifiedSharedJoin {
        left_parameter: 1.0,
        right_parameter: 0.0,
        evidence: CertifiedBoundedLoopJoin::new(0.1).unwrap(),
    };
    assert_eq!(
        line_circle_relation(tangent_window, arc_window, &[(1.0, 0.0)], &[tangent_join],).unwrap(),
        PairRelation::ForbiddenIntersection
    );
}

#[test]
fn circle_join_radius_is_scaled_by_parameter_space_speed() {
    let carrier = line([0.0, 0.0], [1.0, 0.0]);
    let unit_circle = circle([0.0, 0.0], 1.0);
    let wide_circle = circle([0.0, 0.0], 10.0);
    let line_span = BoundedPcurveSpan::new(&carrier, -1.0, 1.0, Point2::default());
    let unit_span = BoundedPcurveSpan::new(&unit_circle, -1.0, 1.0, Point2::default());
    let wide_span = BoundedPcurveSpan::new(&wide_circle, -1.0, 1.0, Point2::default());
    let join = CertifiedSharedJoin {
        left_parameter: 0.0,
        right_parameter: 0.0,
        evidence: CertifiedBoundedLoopJoin::new(0.1).unwrap(),
    };
    let root = Interval::point(0.05);

    assert!(certified_join_confines_roots(
        line_span, root, unit_span, root, join,
    ));
    assert!(!certified_join_confines_roots(
        line_span, root, wide_span, root, join,
    ));
}
