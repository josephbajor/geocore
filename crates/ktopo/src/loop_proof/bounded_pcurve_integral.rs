//! Certified signed line integrals for bounded analytic pcurve spans.
//!
//! This module proves the sign of
//! `sum integral(x dy - y dx)` for finite [`Line2d`](kgeom::curve2d::Line2d)
//! and [`Circle2d`](kgeom::curve2d::Circle2d) spans. It deliberately does not
//! call that sign a loop orientation: topology-owned closure, periodic chart
//! continuity, and pairwise simplicity are independent obligations that the
//! loop checker must establish before consuming this primitive.

use crate::geom::Curve2dGeom;
use kcore::interval::Interval;
use kcore::math;
use kcore::predicates::Orientation;
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::vec::Point2;

/// One bounded traversal of an authored pcurve in a selected chart.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BoundedPcurveSpan<'a> {
    curve: &'a Curve2dGeom,
    start: f64,
    end: f64,
    chart_offset: Point2,
}

impl<'a> BoundedPcurveSpan<'a> {
    /// Describe traversal from `start` to `end` after an exact chart
    /// translation. Reversed traversal is represented by `end < start`.
    pub(crate) const fn new(
        curve: &'a Curve2dGeom,
        start: f64,
        end: f64,
        chart_offset: Point2,
    ) -> Self {
        Self {
            curve,
            start,
            end,
            chart_offset,
        }
    }

    pub(crate) const fn curve(self) -> &'a Curve2dGeom {
        self.curve
    }

    pub(crate) const fn start(self) -> f64 {
        self.start
    }

    pub(crate) const fn end(self) -> f64 {
        self.end
    }

    pub(crate) const fn chart_offset(self) -> Point2 {
        self.chart_offset
    }

    /// Return the same authored traversal in a different proof-local chart.
    ///
    /// This never changes the stored pcurve use. The enclosing loop proof may
    /// use it only after certifying that `chart_offset` differs by whole
    /// periods of the owning surface.
    pub(super) const fn with_chart_offset(mut self, chart_offset: Point2) -> Self {
        self.chart_offset = chart_offset;
        self
    }
}

/// Why a signed line-integral proof failed closed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SignedLineIntegralGap {
    /// No spans were supplied.
    Empty,
    /// A parameter or chart translation is non-finite.
    NonFiniteInput { span_index: usize },
    /// A span has equal start and end parameters.
    DegenerateSpan { span_index: usize },
    /// The pcurve class is outside the bounded Line2d/Circle2d proof slice.
    UnsupportedCurve { span_index: usize },
    /// Outward interval arithmetic became non-finite.
    NonFiniteArithmetic { span_index: usize },
    /// The final enclosure contains zero.
    UnresolvedSign,
}

/// A strictly one-signed enclosure of `sum integral(x dy - y dx)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedSignedLineIntegral {
    orientation: Orientation,
    enclosure: Interval,
}

impl CertifiedSignedLineIntegral {
    /// Certified sign of the line integral.
    pub(crate) const fn orientation(self) -> Orientation {
        self.orientation
    }
}

/// Result of attempting to certify a bounded analytic signed line integral.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum SignedLineIntegralProof {
    /// The complete integral has a strict certified sign.
    Certified(CertifiedSignedLineIntegral),
    /// The proof is unsupported or inconclusive.
    Indeterminate(SignedLineIntegralGap),
}

/// Certify the sign of `sum integral(x dy - y dx)` over bounded analytic
/// pcurve spans.
///
/// Each term is integrated from the authored analytic coefficients. Circle
/// endpoints use deterministic `<1 ulp` trigonometry widened outward before
/// arithmetic. No samples, proximity tolerance, or rounded polygon stand-in
/// participate in the proof.
pub(crate) fn certify_signed_line_integral(
    spans: &[BoundedPcurveSpan<'_>],
) -> SignedLineIntegralProof {
    if spans.is_empty() {
        return SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::Empty);
    }

    let mut sum = Interval::point(0.0);
    for (span_index, span) in spans.iter().enumerate() {
        let term = match span_integral(*span, span_index) {
            Ok(term) => term,
            Err(gap) => return SignedLineIntegralProof::Indeterminate(gap),
        };
        sum = sum + term;
        if !finite_interval(sum) {
            return SignedLineIntegralProof::Indeterminate(
                SignedLineIntegralGap::NonFiniteArithmetic { span_index },
            );
        }
    }

    let orientation = match sum.sign() {
        Some(1) => Orientation::Positive,
        Some(-1) => Orientation::Negative,
        _ => {
            return SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::UnresolvedSign);
        }
    };
    SignedLineIntegralProof::Certified(CertifiedSignedLineIntegral {
        orientation,
        enclosure: sum,
    })
}

/// Enclose one authored analytic span's directed chart integral.
///
/// Representation-specific loop theorems may combine this exact
/// Line2d/Circle2d term with independently sealed procedural terms. The
/// caller still owns topology closure, simplicity, and final sign proof.
pub(crate) fn certify_bounded_pcurve_span_integral(
    span: BoundedPcurveSpan<'_>,
) -> Option<Interval> {
    span_integral(span, 0).ok()
}

fn span_integral(
    span: BoundedPcurveSpan<'_>,
    span_index: usize,
) -> core::result::Result<Interval, SignedLineIntegralGap> {
    if !span.start.is_finite() || !span.end.is_finite() || !finite_point(span.chart_offset) {
        return Err(SignedLineIntegralGap::NonFiniteInput { span_index });
    }
    if span.start == span.end {
        return Err(SignedLineIntegralGap::DegenerateSpan { span_index });
    }

    let delta = Interval::point(span.end) - Interval::point(span.start);
    let integral = match span.curve {
        Curve2dGeom::Line(line) => line_integral(*line, span.chart_offset, delta),
        Curve2dGeom::Circle(circle) => {
            circle_integral(*circle, span.chart_offset, span.start, span.end, delta)
        }
        _ => return Err(SignedLineIntegralGap::UnsupportedCurve { span_index }),
    };
    if finite_interval(integral) {
        Ok(integral)
    } else {
        Err(SignedLineIntegralGap::NonFiniteArithmetic { span_index })
    }
}

fn line_integral(line: Line2d, chart_offset: Point2, delta: Interval) -> Interval {
    let origin = translated_point(line.origin(), chart_offset);
    let direction = point_interval(line.dir());
    cross(origin, direction) * delta
}

fn circle_integral(
    circle: Circle2d,
    chart_offset: Point2,
    start: f64,
    end: f64,
    delta: Interval,
) -> Interval {
    let center = translated_point(circle.center(), chart_offset);
    let radius = Interval::point(circle.radius());
    let x = point_interval(circle.x_dir());
    let y = [-x[1], x[0]];
    let cosine_axis = scale(x, radius);
    let sine_axis = scale(y, radius);
    let (start_sine, start_cosine) = trig_point(start);
    let (end_sine, end_cosine) = trig_point(end);
    let displacement = add(
        scale(cosine_axis, end_cosine - start_cosine),
        scale(sine_axis, end_sine - start_sine),
    );
    cross(center, displacement) + cross(cosine_axis, sine_axis) * delta
}

fn trig_point(parameter: f64) -> (Interval, Interval) {
    let (sine, cosine) = math::sincos(parameter);
    (
        Interval::new(sine.next_down(), sine.next_up()),
        Interval::new(cosine.next_down(), cosine.next_up()),
    )
}

fn point_interval(point: Point2) -> [Interval; 2] {
    [Interval::point(point.x), Interval::point(point.y)]
}

fn translated_point(point: Point2, offset: Point2) -> [Interval; 2] {
    [
        Interval::point(point.x) + Interval::point(offset.x),
        Interval::point(point.y) + Interval::point(offset.y),
    ]
}

fn add(left: [Interval; 2], right: [Interval; 2]) -> [Interval; 2] {
    [left[0] + right[0], left[1] + right[1]]
}

fn scale(vector: [Interval; 2], factor: Interval) -> [Interval; 2] {
    [vector[0] * factor, vector[1] * factor]
}

fn cross(left: [Interval; 2], right: [Interval; 2]) -> Interval {
    left[0] * right[1] - left[1] * right[0]
}

fn finite_interval(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite() && interval.lo() <= interval.hi()
}

fn finite_point(point: Point2) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::curve2d::{Circle2d, Line2d, NurbsCurve2d};
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

    fn certified(spans: &[BoundedPcurveSpan<'_>]) -> (Orientation, Interval) {
        let SignedLineIntegralProof::Certified(proof) = certify_signed_line_integral(spans) else {
            panic!("expected a certified signed integral");
        };
        (proof.orientation(), proof.enclosure)
    }

    #[test]
    fn line_spans_enclose_the_independent_shoelace_area() {
        let bottom = line([0.0, 0.0], [1.0, 0.0]);
        let right = line([4.0, 0.0], [0.0, 1.0]);
        let diagonal = line([4.0, 3.0], [-4.0, -3.0]);
        let spans = [
            BoundedPcurveSpan::new(&bottom, 0.0, 4.0, Point2::default()),
            BoundedPcurveSpan::new(&right, 0.0, 3.0, Point2::default()),
            BoundedPcurveSpan::new(&diagonal, 0.0, 5.0, Point2::default()),
        ];
        let (orientation, enclosure) = certified(&spans);
        assert_eq!(orientation, Orientation::Positive);
        assert!(enclosure.contains(12.0));

        let reversed = [
            BoundedPcurveSpan::new(&diagonal, 5.0, 0.0, Point2::default()),
            BoundedPcurveSpan::new(&right, 3.0, 0.0, Point2::default()),
            BoundedPcurveSpan::new(&bottom, 4.0, 0.0, Point2::default()),
        ];
        let (orientation, enclosure) = certified(&reversed);
        assert_eq!(orientation, Orientation::Negative);
        assert!(enclosure.contains(-12.0));
    }

    #[test]
    fn mixed_arc_line_integral_matches_a_semidisk_and_is_translation_invariant() {
        let radius = 3.0;
        let expected = core::f64::consts::PI * radius * radius;
        for center in [[0.0, 0.0], [13.0, -7.0], [-1024.0, 2048.0]] {
            let diameter = line([center[0] - radius, center[1]], [1.0, 0.0]);
            let boundary_arc = circle(center, radius);
            let spans = [
                BoundedPcurveSpan::new(&diameter, 0.0, 2.0 * radius, Point2::default()),
                BoundedPcurveSpan::new(
                    &boundary_arc,
                    0.0,
                    core::f64::consts::PI,
                    Point2::default(),
                ),
            ];
            let (orientation, enclosure) = certified(&spans);
            assert_eq!(orientation, Orientation::Positive);
            assert!(enclosure.contains(expected));
        }
    }

    #[test]
    fn arc_decomposition_and_chart_translation_preserve_the_certified_sign() {
        let diameter = line([-2.0, 0.0], [1.0, 0.0]);
        let boundary_arc = circle([0.0, 0.0], 2.0);
        let shift = Point2::new(5.0, -11.0);
        let split = core::f64::consts::PI / 3.0;
        let spans = [
            BoundedPcurveSpan::new(&diameter, 0.0, 4.0, shift),
            BoundedPcurveSpan::new(&boundary_arc, 0.0, split, shift),
            BoundedPcurveSpan::new(&boundary_arc, split, core::f64::consts::PI, shift),
        ];
        let (orientation, enclosure) = certified(&spans);
        assert_eq!(orientation, Orientation::Positive);
        assert!(enclosure.contains(4.0 * core::f64::consts::PI));
    }

    #[test]
    fn unsupported_and_invalid_inputs_fail_with_stable_gaps() {
        assert_eq!(
            certify_signed_line_integral(&[]),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::Empty)
        );

        let supported = line([0.0, 0.0], [1.0, 0.0]);
        assert_eq!(
            certify_signed_line_integral(&[BoundedPcurveSpan::new(
                &supported,
                1.0,
                1.0,
                Point2::default(),
            )]),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::DegenerateSpan {
                span_index: 0
            })
        );
        assert_eq!(
            certify_signed_line_integral(&[BoundedPcurveSpan::new(
                &supported,
                f64::NAN,
                1.0,
                Point2::default(),
            )]),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::NonFiniteInput {
                span_index: 0
            })
        );

        let unsupported = Curve2dGeom::Nurbs(
            NurbsCurve2d::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                None,
            )
            .unwrap(),
        );
        assert_eq!(
            certify_signed_line_integral(&[BoundedPcurveSpan::new(
                &unsupported,
                0.0,
                1.0,
                Point2::default(),
            )]),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::UnsupportedCurve {
                span_index: 0
            })
        );
    }

    #[test]
    fn cancellation_underflow_and_overflow_never_guess_a_sign() {
        let trace = line([0.0, 0.0], [1.0, 0.0]);
        let cancellation = [
            BoundedPcurveSpan::new(&trace, 0.0, 1.0, Point2::default()),
            BoundedPcurveSpan::new(&trace, 1.0, 0.0, Point2::default()),
        ];
        assert_eq!(
            certify_signed_line_integral(&cancellation),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::UnresolvedSign)
        );

        let microscopic = circle([0.0, 0.0], 1.0e-200);
        assert_eq!(
            certify_signed_line_integral(&[BoundedPcurveSpan::new(
                &microscopic,
                0.0,
                core::f64::consts::TAU,
                Point2::default(),
            )]),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::UnresolvedSign)
        );

        assert_eq!(
            certify_signed_line_integral(&[BoundedPcurveSpan::new(
                &trace,
                0.0,
                1.0,
                Point2::new(f64::MAX, f64::MAX),
            )]),
            SignedLineIntegralProof::Indeterminate(SignedLineIntegralGap::NonFiniteArithmetic {
                span_index: 0
            })
        );
    }
}
