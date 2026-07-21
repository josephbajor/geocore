use super::*;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::surface::Cylinder;
use kgeom::vec::Vec2;

fn seam_rectangle_curves() -> ([Curve2dGeom; 4], f64) {
    let width = core::f64::consts::TAU + 0.1 - 6.1;
    (
        [
            Curve2dGeom::Line(Line2d::new(Point2::new(6.1, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
            Curve2dGeom::Line(Line2d::new(Point2::new(0.1, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
            Curve2dGeom::Line(Line2d::new(Point2::new(0.1, 1.0), Vec2::new(-1.0, 0.0)).unwrap()),
            Curve2dGeom::Line(Line2d::new(Point2::new(6.1, 1.0), Vec2::new(0.0, -1.0)).unwrap()),
        ],
        width,
    )
}

fn seam_rectangle_spans<'a>(
    curves: &'a [Curve2dGeom; 4],
    width: f64,
    phases: [i64; 4],
) -> Vec<BoundedLoopSpan<'a, usize>> {
    let ends = [width, 1.0, width, 1.0];
    (0..4)
        .map(|index| {
            BoundedLoopSpan::new(
                BoundedPcurveSpan::new(
                    &curves[index],
                    0.0,
                    ends[index],
                    Point2::new(phases[index] as f64 * core::f64::consts::TAU, 0.0),
                ),
                index,
                (index + 1) % 4,
            )
        })
        .collect()
}

fn cylinder_surface() -> SurfaceGeom {
    SurfaceGeom::Cylinder(Cylinder::new(Frame::world(), 1.5).unwrap())
}

#[test]
fn cycle_lift_unwraps_seam_crossing_and_is_phase_invariant() {
    let surface = cylinder_surface();
    let periods = surface.as_leaf_surface().unwrap().periodicity();
    let (curves, width) = seam_rectangle_curves();

    for phases in [[0, 0, 0, 0], [7, 7, 7, 7], [-9, -9, -9, -9], [3, -2, 4, 0]] {
        let mut spans = seam_rectangle_spans(&curves, width, phases);
        assert_eq!(
            certify_bounded_chart_lifts(&surface, periods, &mut spans),
            Ok(())
        );
        assert_eq!(
            certify_bounded_loop_simplicity(&spans),
            BoundedLoopSimplicity::Certified
        );
        let geometry = spans
            .iter()
            .copied()
            .map(BoundedLoopSpan::geometry)
            .collect::<Vec<_>>();
        assert!(matches!(
            certify_signed_line_integral(&geometry),
            SignedLineIntegralProof::Certified(proof)
                if proof.orientation() == Orientation::Positive
        ));
    }
}

#[test]
fn cycle_lift_preserves_reversed_seam_loop_orientation() {
    let surface = cylinder_surface();
    let periods = surface.as_leaf_surface().unwrap().periodicity();
    let (curves, width) = seam_rectangle_curves();
    let order = [3, 2, 1, 0];
    let starts = [1.0, width, 1.0, width];
    let mut spans = order
        .into_iter()
        .enumerate()
        .map(|(index, curve)| {
            BoundedLoopSpan::new(
                BoundedPcurveSpan::new(&curves[curve], starts[index], 0.0, Point2::default()),
                index,
                (index + 1) % 4,
            )
        })
        .collect::<Vec<_>>();

    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut spans),
        Ok(())
    );
    assert_eq!(
        certify_bounded_loop_simplicity(&spans),
        BoundedLoopSimplicity::Certified
    );
    let geometry = spans
        .iter()
        .copied()
        .map(BoundedLoopSpan::geometry)
        .collect::<Vec<_>>();
    assert!(matches!(
        certify_signed_line_integral(&geometry),
        SignedLineIntegralProof::Certified(proof)
            if proof.orientation() == Orientation::Negative
    ));
}

fn shift_span(spans: &mut [BoundedLoopSpan<'_, usize>], index: usize, shift: Point2) {
    let geometry = spans[index].geometry();
    let offset = geometry.chart_offset();
    spans[index] = spans[index].with_geometry(
        geometry.with_chart_offset(Point2::new(offset.x + shift.x, offset.y + shift.y)),
    );
}

#[test]
fn periodic_cycle_lift_tamper_table_fails_closed() {
    let surface = cylinder_surface();
    let periods = surface.as_leaf_surface().unwrap().periodicity();
    let (curves, width) = seam_rectangle_curves();

    let mut half_period = seam_rectangle_spans(&curves, width, [0; 4]);
    shift_span(
        &mut half_period,
        1,
        Point2::new(0.5 * core::f64::consts::TAU, 0.0),
    );
    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut half_period),
        Err(BoundedChartLiftGap::AmbiguousPeriodLift {
            span_index: 0,
            direction: 0,
        })
    );

    let mut negative_half_period = seam_rectangle_spans(&curves, width, [0; 4]);
    shift_span(
        &mut negative_half_period,
        1,
        Point2::new(1.5 * core::f64::consts::TAU, 0.0),
    );
    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut negative_half_period),
        Err(BoundedChartLiftGap::AmbiguousPeriodLift {
            span_index: 0,
            direction: 0,
        })
    );

    let mut fractional_period = seam_rectangle_spans(&curves, width, [0; 4]);
    shift_span(
        &mut fractional_period,
        1,
        Point2::new(0.25 * core::f64::consts::TAU, 0.0),
    );
    let offsets_before_failure = fractional_period
        .iter()
        .map(|span| span.geometry().chart_offset())
        .collect::<Vec<_>>();
    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut fractional_period),
        Err(BoundedChartLiftGap::ModelJoinMismatch { span_index: 0 })
    );
    assert!(
        fractional_period
            .iter()
            .zip(offsets_before_failure)
            .all(|(span, before)| point2_bits_equal(span.geometry().chart_offset(), before)),
        "a late model-join failure must not commit any proof-local chart shift",
    );

    let mut nonperiodic_shift = seam_rectangle_spans(&curves, width, [0; 4]);
    shift_span(&mut nonperiodic_shift, 1, Point2::new(0.0, 0.25));
    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut nonperiodic_shift),
        Err(BoundedChartLiftGap::ModelJoinMismatch { span_index: 0 })
    );

    let mut huge_shift = seam_rectangle_spans(&curves, width, [0; 4]);
    shift_span(&mut huge_shift, 1, Point2::new(f64::MAX, 0.0));
    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut huge_shift),
        Err(BoundedChartLiftGap::PeriodLiftOverflow {
            span_index: 0,
            direction: 0,
        })
    );

    let mut broken_topology = seam_rectangle_spans(&curves, width, [0; 4]);
    let geometry = broken_topology[1].geometry();
    broken_topology[1] = BoundedLoopSpan::new(geometry, usize::MAX, 2);
    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut broken_topology),
        Err(BoundedChartLiftGap::TopologyDiscontinuity { span_index: 0 })
    );

    let mut invalid_period = seam_rectangle_spans(&curves, width, [0; 4]);
    assert_eq!(
        certify_bounded_chart_lifts(&surface, [Some(0.0), None], &mut invalid_period),
        Err(BoundedChartLiftGap::InvalidPeriod { direction: 0 })
    );
    assert_eq!(
        certify_bounded_chart_lifts(&surface, [Some(f64::NAN), None], &mut invalid_period,),
        Err(BoundedChartLiftGap::InvalidPeriod { direction: 0 })
    );
}

#[test]
fn periodic_cycle_lift_rejects_nonzero_total_winding() {
    let surface = cylinder_surface();
    let periods = surface.as_leaf_surface().unwrap().periodicity();
    let curves = [
        Curve2dGeom::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
        Curve2dGeom::Line(
            Line2d::new(Point2::new(core::f64::consts::PI, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
        ),
        Curve2dGeom::Line(
            Line2d::new(Point2::new(core::f64::consts::PI, 1.0), Vec2::new(1.0, 0.0)).unwrap(),
        ),
        Curve2dGeom::Line(
            Line2d::new(
                Point2::new(core::f64::consts::TAU, 1.0),
                Vec2::new(0.0, -1.0),
            )
            .unwrap(),
        ),
    ];
    let ends = [core::f64::consts::PI, 1.0, core::f64::consts::PI, 1.0];
    let mut spans = (0..4)
        .map(|index| {
            BoundedLoopSpan::new(
                BoundedPcurveSpan::new(&curves[index], 0.0, ends[index], Point2::default()),
                index,
                (index + 1) % 4,
            )
        })
        .collect::<Vec<_>>();

    assert_eq!(
        certify_bounded_chart_lifts(&surface, periods, &mut spans),
        Err(BoundedChartLiftGap::NonzeroWinding { direction: 0 })
    );
}
