//! Exact, fail-closed Cylinder/Cylinder ruling trace certification.
//!
//! This proof family is operation-local. It admits only finite line branches
//! on cylinders with exactly parallel or antiparallel axes. Outward
//! interval separation proofs exclude tangent, coincident, nested, and
//! disjoint circle pairs before paired whole-range residual certification.

use kcore::interval::Interval;
use kcore::predicates::{Orientation, orient3d};
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;

use crate::plane_cylinder_ruling::{
    CylinderRulingTrace, affine_residual_bound, cylinder_ruling_coefficients, finite_cylinder,
    finite_interval, finite_line, line_coefficients,
};
use crate::{AffineParamMap1d, IntersectionCertificateError, PairedTrace};

/// Whole-interval paired residual proof for one finite Cylinder/Cylinder ruling.
///
/// Private fields bind the canonical line carrier and both source-ordered
/// constant-longitude cylinder traces to their outward residual bounds. The
/// cylinder axes are exact-predicate parallel or antiparallel, and their transverse
/// circle pair has an outward proof of strict secancy. Persistence requires a
/// separate descriptor contract.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairedCylinderCylinderRulingResidualCertificate {
    carrier: Line,
    carrier_range: ParamRange,
    traces: [CylinderRulingTrace; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
}

impl PairedCylinderCylinderRulingResidualCertificate {
    /// Verified model-space line carrier.
    pub const fn carrier(self) -> Line {
        self.carrier
    }

    /// Finite positive-length carrier interval covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified cylinder traces in source operand order.
    pub const fn traces(self) -> [CylinderRulingTrace; 2] {
        self.traces
    }

    /// Carrier-to-pcurve parameter maps in source operand order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        [
            self.traces[0].parameter_map(),
            self.traces[1].parameter_map(),
        ]
    }

    /// Conservative whole-range residual bounds in source operand order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance against which both traces were certified.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }
}

/// Certify a finite, strictly transverse Cylinder/Cylinder ruling.
///
/// Family admission is exact and fail-closed. Exact `orient3d` determinants
/// must prove both cylinder axes parallel and the carrier parallel to each
/// axis. Outward interval arithmetic over the complete radial axis separation
/// must prove both strict circle-secant inequalities
/// `|radius_a - radius_b| < distance < radius_a + radius_b`. Each lifted
/// constant-longitude trace is affine in the carrier parameter, so one
/// interval evaluation bounds its residual over the complete retained range
/// without sampling.
pub fn certify_paired_cylinder_cylinder_ruling_residuals(
    carrier: Line,
    carrier_range: ParamRange,
    traces: [CylinderRulingTrace; 2],
    tolerance: f64,
) -> Result<PairedCylinderCylinderRulingResidualCertificate, IntersectionCertificateError> {
    if !carrier_range.is_finite() || carrier_range.lo >= carrier_range.hi {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !finite_line(carrier) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let cylinders = [traces[0].surface(), traces[1].surface()];
    if !cylinders.into_iter().all(finite_cylinder) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    certify_strict_ruling_family(carrier, cylinders)?;

    let carrier_coefficients =
        line_coefficients(carrier).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let mut residual_bounds = [0.0; 2];
    for (index, trace) in traces.into_iter().enumerate() {
        let trace_id = paired_trace(index);
        let lifted = cylinder_ruling_coefficients(trace, trace_id)?;
        let bound = affine_residual_bound(carrier_coefficients, lifted, carrier_range)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace: trace_id })?;
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace: trace_id,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(PairedCylinderCylinderRulingResidualCertificate {
        carrier,
        carrier_range,
        traces,
        residual_bounds,
        tolerance,
    })
}

const fn paired_trace(index: usize) -> PairedTrace {
    if index == 0 {
        PairedTrace::First
    } else {
        PairedTrace::Second
    }
}

fn certify_strict_ruling_family(
    carrier: Line,
    cylinders: [Cylinder; 2],
) -> Result<(), IntersectionCertificateError> {
    let axes = [cylinders[0].frame().z(), cylinders[1].frame().z()];
    if !vectors_are_exactly_parallel(axes[0], axes[1]) {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Cylinder/Cylinder ruling requires exact-predicate parallel or antiparallel axes",
            },
        );
    }
    if !axes
        .into_iter()
        .all(|axis| vectors_are_exactly_parallel(carrier.dir(), axis))
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Cylinder/Cylinder ruling carrier must share both signed cylinder axes",
            },
        );
    }

    let [inner_clearance, outer_clearance] = strict_secant_clearances(cylinders)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    if inner_clearance.lo() <= 0.0 || outer_clearance.lo() <= 0.0 {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Cylinder/Cylinder ruling requires a proven strict transverse circle secant",
            },
        );
    }
    Ok(())
}

fn vectors_are_exactly_parallel(first: kgeom::vec::Vec3, second: kgeom::vec::Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

/// Return lower-bound proof intervals for
/// `distance² - (radius_a - radius_b)²` and
/// `(radius_a + radius_b)² - distance²`.
fn strict_secant_clearances(cylinders: [Cylinder; 2]) -> Option<[Interval; 2]> {
    let first = cylinders[0];
    let second = cylinders[1];
    let offset = interval_vector_difference(
        second.frame().origin().to_array(),
        first.frame().origin().to_array(),
    )?;
    let first_distance =
        radial_distance_squared(offset, first.frame().z().to_array().map(Interval::point))?;
    let second_distance =
        radial_distance_squared(offset, second.frame().z().to_array().map(Interval::point))?;
    // Exact parallelism makes these two expressions the same mathematical
    // axis distance. Intersecting their independent outward evaluations is
    // therefore sound and makes the proof independent of source order.
    let distance_squared = intersect_intervals(first_distance, second_distance)?;

    let radius_a = Interval::point(first.radius());
    let radius_b = Interval::point(second.radius());
    let radius_difference = finite_interval(radius_a - radius_b)?;
    let radius_sum = finite_interval(radius_a + radius_b)?;
    Some([
        finite_interval(distance_squared - radius_difference.square())?,
        finite_interval(radius_sum.square() - distance_squared)?,
    ])
}

/// Squared distance from `offset` to the line spanned by `axis`.
///
/// The stored frame axis is normalized only to floating-point accuracy. The
/// denominator is therefore part of the certified metric rather than an
/// implicit exact-unit assumption.
fn radial_distance_squared(offset: [Interval; 3], axis: [Interval; 3]) -> Option<Interval> {
    let cross = interval_cross(offset, axis)?;
    let mut cross_squared = Interval::point(0.0);
    let mut axis_squared = Interval::point(0.0);
    for component in cross {
        cross_squared = finite_interval(cross_squared + component.square())?;
    }
    for component in axis {
        axis_squared = finite_interval(axis_squared + component.square())?;
    }
    if axis_squared.lo() <= 0.0 {
        return None;
    }
    finite_interval(cross_squared.checked_div(axis_squared)?)
}

fn intersect_intervals(first: Interval, second: Interval) -> Option<Interval> {
    let lo = first.lo().max(second.lo());
    let hi = first.hi().min(second.hi());
    (lo <= hi).then(|| Interval::new(lo, hi))
}

fn interval_vector_difference(point: [f64; 3], origin: [f64; 3]) -> Option<[Interval; 3]> {
    let mut difference = [Interval::point(0.0); 3];
    for axis in 0..3 {
        difference[axis] =
            finite_interval(Interval::point(point[axis]) - Interval::point(origin[axis]))?;
    }
    Some(difference)
}

fn interval_cross(lhs: [Interval; 3], rhs: [Interval; 3]) -> Option<[Interval; 3]> {
    Some([
        finite_interval(finite_interval(lhs[1] * rhs[2])? - finite_interval(lhs[2] * rhs[1])?)?,
        finite_interval(finite_interval(lhs[2] * rhs[0])? - finite_interval(lhs[0] * rhs[2])?)?,
        finite_interval(finite_interval(lhs[0] * rhs[1])? - finite_interval(lhs[1] * rhs[0])?)?,
    ])
}

#[cfg(test)]
mod tests {
    use kcore::math::atan2;
    use kgeom::curve::{Curve, Line};
    use kgeom::curve2d::{Curve2d, Line2d};
    use kgeom::frame::Frame;
    use kgeom::surface::{Cylinder, Surface};
    use kgeom::vec::{Point3, Vec2, Vec3};

    use super::*;

    fn trace_for(cylinder: Cylinder, carrier: Line) -> CylinderRulingTrace {
        let local_origin = cylinder.frame().to_local(carrier.origin());
        let longitude = atan2(local_origin.y, local_origin.x);
        let pcurve = Line2d::new(Vec2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap();
        let parameter_map =
            AffineParamMap1d::new(carrier.dir().dot(cylinder.frame().z()), local_origin.z).unwrap();
        CylinderRulingTrace::new(cylinder, pcurve, parameter_map)
    }

    fn strict_secant_fixture(frame: Frame, reverse_second_axis: bool) -> ([Cylinder; 2], Line) {
        let first = Cylinder::new(frame, 1.0).unwrap();
        let second_origin = frame.origin() + frame.x();
        let second_frame = if reverse_second_axis {
            Frame::new(second_origin, -frame.z(), frame.x()).unwrap()
        } else {
            frame.with_origin(second_origin)
        };
        let second = Cylinder::new(second_frame, 1.0).unwrap();
        let radial_height = (3.0_f64 / 4.0).sqrt();
        let point = frame.origin() + frame.x() * 0.5 + frame.y() * radial_height;
        let carrier = Line::new(point, frame.z()).unwrap();
        ([first, second], carrier)
    }

    #[test]
    fn certifies_whole_range_with_repeat_swap_and_signed_axis_parity() {
        let range = ParamRange::new(-2.25, 3.5);
        for reverse_second_axis in [false, true] {
            let (cylinders, carrier) = strict_secant_fixture(Frame::world(), reverse_second_axis);
            let traces = cylinders.map(|cylinder| trace_for(cylinder, carrier));
            let certificate =
                certify_paired_cylinder_cylinder_ruling_residuals(carrier, range, traces, 1.0e-12)
                    .unwrap();
            let repeated =
                certify_paired_cylinder_cylinder_ruling_residuals(carrier, range, traces, 1.0e-12)
                    .unwrap();
            let swapped = certify_paired_cylinder_cylinder_ruling_residuals(
                carrier,
                range,
                [traces[1], traces[0]],
                1.0e-12,
            )
            .unwrap();

            assert_eq!(certificate, repeated);
            assert_eq!(certificate.carrier(), carrier);
            assert_eq!(certificate.carrier_range(), range);
            assert_eq!(certificate.traces(), traces);
            assert_eq!(
                certificate.parameter_maps(),
                traces.map(|trace| trace.parameter_map())
            );
            assert_eq!(
                swapped.residual_bounds(),
                [
                    certificate.residual_bounds()[1],
                    certificate.residual_bounds()[0],
                ]
            );
            assert_eq!(certificate.tolerance(), 1.0e-12);

            for parameter in [range.lo, 0.625, range.hi] {
                let point = carrier.eval(parameter);
                for (index, trace) in certificate.traces().into_iter().enumerate() {
                    let uv = trace.pcurve().eval(trace.parameter_map().map(parameter));
                    let lifted = trace.surface().eval([uv.x, uv.y]);
                    let observed = point.dist(lifted);
                    assert!(observed <= certificate.residual_bounds()[index]);
                }
            }
        }
    }

    #[test]
    fn certifies_oblique_rigid_copy_without_configuration_cases() {
        let frame = Frame::new(
            Point3::new(7.0, -3.0, 2.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let (cylinders, carrier) = strict_secant_fixture(frame, false);
        let traces = cylinders.map(|cylinder| trace_for(cylinder, carrier));
        let certificate = certify_paired_cylinder_cylinder_ruling_residuals(
            carrier,
            ParamRange::new(-1.75, 4.25),
            traces,
            1.0e-11,
        )
        .unwrap();

        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );
    }

    #[test]
    fn exact_parallel_theorem_is_not_limited_to_bit_identical_vectors() {
        let first = Vec3::new(1.0, -2.0, 3.0);
        let parallel = first * 4.0;
        let near_parallel = Vec3::new(4.0, -8.0, 12.0_f64.next_up());

        assert_ne!(first, parallel);
        assert!(vectors_are_exactly_parallel(first, parallel));
        assert!(vectors_are_exactly_parallel(first, -parallel));
        assert!(!vectors_are_exactly_parallel(first, near_parallel));
    }

    #[test]
    fn radial_axis_metric_normalizes_nonunit_axes() {
        let offset = [3.0, 4.0, 7.0].map(Interval::point);
        for axis in [[0.0, 0.0, 2.0], [0.0, 0.0, -7.0]] {
            let distance = radial_distance_squared(offset, axis.map(Interval::point)).unwrap();
            assert!(distance.contains(25.0));
        }
    }

    #[test]
    fn near_boundary_clearances_are_swap_stable_and_fail_closed() {
        let frame = Frame::new(
            Point3::new(5.0, -3.0, 2.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let epsilon = 1.0e-12;
        let clearances = |distance: f64, radii: [f64; 2]| {
            let first = Cylinder::new(frame, radii[0]).unwrap();
            let second = Cylinder::new(
                frame.with_origin(frame.origin() + frame.x() * distance),
                radii[1],
            )
            .unwrap();
            let forward = strict_secant_clearances([first, second]).unwrap();
            let reversed = strict_secant_clearances([second, first]).unwrap();
            assert_eq!(forward, reversed);
            forward
        };

        let external_tangent = clearances(2.0, [1.0, 1.0]);
        assert!(external_tangent[1].contains_zero());
        assert!(clearances(2.0 - epsilon, [1.0, 1.0])[1].lo() > 0.0);
        assert!(clearances(2.0 + epsilon, [1.0, 1.0])[1].hi() < 0.0);

        let internal_tangent = clearances(1.0, [1.5, 0.5]);
        assert!(internal_tangent[0].contains_zero());
        assert!(clearances(1.0 + epsilon, [1.5, 0.5])[0].lo() > 0.0);
        assert!(clearances(1.0 - epsilon, [1.5, 0.5])[0].hi() < 0.0);
    }

    #[test]
    fn refuses_nonparallel_and_non_strict_circle_families() {
        let world = Frame::world();
        let first = Cylinder::new(world, 1.0).unwrap();
        let families = [
            Cylinder::new(world.with_origin(Point3::new(2.0, 0.0, 0.0)), 1.0).unwrap(),
            Cylinder::new(world.with_origin(Point3::new(3.0, 0.0, 0.0)), 1.0).unwrap(),
            Cylinder::new(world.with_origin(Point3::new(0.25, 0.0, 0.0)), 0.5).unwrap(),
            Cylinder::new(world.with_origin(Point3::new(0.0, 0.0, 2.0)), 1.0).unwrap(),
        ];
        let carrier = Line::new(Point3::new(0.5, (3.0_f64 / 4.0).sqrt(), 0.0), world.z()).unwrap();
        for second in families {
            let traces = [trace_for(first, carrier), trace_for(second, carrier)];
            assert!(matches!(
                certify_paired_cylinder_cylinder_ruling_residuals(
                    carrier,
                    ParamRange::new(-1.0, 1.0),
                    traces,
                    1.0e6,
                ),
                Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
            ));
        }

        let skew_frame = Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0e-12, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let skew = Cylinder::new(skew_frame, 1.0).unwrap();
        assert!(matches!(
            certify_paired_cylinder_cylinder_ruling_residuals(
                carrier,
                ParamRange::new(-1.0, 1.0),
                [trace_for(first, carrier), trace_for(skew, carrier)],
                1.0e6,
            ),
            Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
        ));
    }

    #[test]
    fn fails_closed_for_carrier_trace_range_map_and_residual_errors() {
        let (cylinders, carrier) = strict_secant_fixture(Frame::world(), false);
        let traces = cylinders.map(|cylinder| trace_for(cylinder, carrier));
        let range = ParamRange::new(-1.0, 1.0);

        for invalid_range in [
            ParamRange { lo: 1.0, hi: -1.0 },
            ParamRange::new(0.0, 0.0),
            ParamRange::unbounded(),
        ] {
            assert!(matches!(
                certify_paired_cylinder_cylinder_ruling_residuals(
                    carrier,
                    invalid_range,
                    traces,
                    1.0e-12,
                ),
                Err(IntersectionCertificateError::InvalidCarrierRange)
            ));
        }
        for invalid_tolerance in [-1.0, f64::NAN, f64::INFINITY] {
            assert!(matches!(
                certify_paired_cylinder_cylinder_ruling_residuals(
                    carrier,
                    range,
                    traces,
                    invalid_tolerance,
                ),
                Err(IntersectionCertificateError::InvalidTolerance)
            ));
        }

        let altered_carrier = Line::new(carrier.origin(), Vec3::new(0.0, 1.0e-12, 1.0)).unwrap();
        assert!(matches!(
            certify_paired_cylinder_cylinder_ruling_residuals(
                altered_carrier,
                range,
                cylinders.map(|cylinder| trace_for(cylinder, altered_carrier)),
                1.0e6,
            ),
            Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
        ));

        let mut nonconstant = traces;
        nonconstant[0] = CylinderRulingTrace::new(
            cylinders[0],
            Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0e-12, 1.0)).unwrap(),
            AffineParamMap1d::new(1.0, 0.0).unwrap(),
        );
        assert!(matches!(
            certify_paired_cylinder_cylinder_ruling_residuals(carrier, range, nonconstant, 1.0e6,),
            Err(IntersectionCertificateError::UnsupportedTraceParameterization { .. })
        ));

        let mut overflowing_map = traces;
        overflowing_map[0] = CylinderRulingTrace::new(
            cylinders[0],
            traces[0].pcurve(),
            AffineParamMap1d::new(f64::MAX, 0.0).unwrap(),
        );
        assert!(
            certify_paired_cylinder_cylinder_ruling_residuals(
                carrier,
                ParamRange::new(-2.0, 2.0),
                overflowing_map,
                f64::MAX,
            )
            .is_err()
        );

        let mut displaced_trace = traces;
        displaced_trace[0] = CylinderRulingTrace::new(
            cylinders[0],
            Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
            traces[0].parameter_map(),
        );
        assert!(matches!(
            certify_paired_cylinder_cylinder_ruling_residuals(
                carrier,
                range,
                displaced_trace,
                1.0e-12,
            ),
            Err(IntersectionCertificateError::ResidualExceedsTolerance { .. })
        ));

        let nonfinite_carrier =
            Line::new(Point3::new(f64::NAN, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(matches!(
            certify_paired_cylinder_cylinder_ruling_residuals(
                nonfinite_carrier,
                range,
                traces,
                1.0e-12,
            ),
            Err(IntersectionCertificateError::NonFiniteGeometry)
        ));
    }
}
