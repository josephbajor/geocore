//! Whole-period Plane/Cylinder circle trace certification.
//!
//! This proof family is deliberately separate from the persistent
//! intersection descriptor machinery. It certifies one operation-local,
//! graph-identified branch without pretending that a periodic carrier has
//! two distinct geometric endpoints. Persistence can be added only after its
//! copy/interchange contract is defined.

use kcore::interval::Interval;
use kcore::math;
use kgeom::curve::Circle;
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Vec2, Vec3};

use super::{AffineParamMap1d, IntersectionCertificateError, PairedTrace, PlaneCircleTrace};

/// Exact longitude/height trace of a whole-period circle on a cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderLongitudeTrace {
    surface: Cylinder,
    pcurve: Line2d,
    parameter_map: AffineParamMap1d,
}

impl CylinderLongitudeTrace {
    /// Construct an ordered cylinder trace candidate.
    pub const fn new(surface: Cylinder, pcurve: Line2d, parameter_map: AffineParamMap1d) -> Self {
        Self {
            surface,
            pcurve,
            parameter_map,
        }
    }

    /// Exact cylinder field.
    pub const fn surface(self) -> Cylinder {
        self.surface
    }

    /// Affine longitude/height pcurve.
    pub const fn pcurve(self) -> Line2d {
        self.pcurve
    }

    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

/// One operand-ordered trace of a whole-period Plane/Cylinder circle proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlaneCylinderCircleTrace {
    /// Plane trace carried by a parameter-space circle.
    Plane(PlaneCircleTrace),
    /// Cylinder trace carried by affine longitude at constant height.
    Cylinder(CylinderLongitudeTrace),
}

impl PlaneCylinderCircleTrace {
    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        match self {
            Self::Plane(trace) => trace.parameter_map(),
            Self::Cylinder(trace) => trace.parameter_map(),
        }
    }
}

/// Whole-interval paired residual proof for one full Plane/Cylinder circle.
///
/// Private fields bind the carrier, its complete period, and both ordered
/// source traces to the residual bounds. Only
/// [`certify_paired_plane_cylinder_circle_residuals`] can mint this value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairedPlaneCylinderCircleResidualCertificate {
    carrier: Circle,
    carrier_range: ParamRange,
    traces: [PlaneCylinderCircleTrace; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
}

impl PairedPlaneCylinderCircleResidualCertificate {
    /// Verified model-space circle carrier.
    pub const fn carrier(self) -> Circle {
        self.carrier
    }

    /// Complete one-period carrier interval covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified traces in source operand order.
    pub const fn traces(self) -> [PlaneCylinderCircleTrace; 2] {
        self.traces
    }

    /// Carrier-to-pcurve parameter maps in source operand order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        [
            self.traces[0].parameter_map(),
            self.traces[1].parameter_map(),
        ]
    }

    /// Conservative whole-period residual bounds in source operand order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance against which both traces were certified.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }
}

/// Certify a complete periodic Plane/Cylinder circle with paired pcurves.
///
/// The plane pcurve is a circle evaluated through a unit-speed affine angle
/// map. The cylinder pcurve is affine longitude at constant height, also at
/// unit angular speed. Expanding both lifts into constant/cosine/sine
/// coefficients gives an outward-rounded whole-period residual bound; no
/// samples or endpoint coincidence tests are used as proof evidence.
pub fn certify_paired_plane_cylinder_circle_residuals(
    carrier: Circle,
    carrier_range: ParamRange,
    traces: [PlaneCylinderCircleTrace; 2],
    tolerance: f64,
) -> Result<PairedPlaneCylinderCircleResidualCertificate, IntersectionCertificateError> {
    if !carrier_range.is_finite()
        || carrier_range.lo > carrier_range.hi
        || carrier_range.width() != core::f64::consts::TAU
    {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !finite_circle(carrier) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    if !matches!(
        traces,
        [
            PlaneCylinderCircleTrace::Plane(_),
            PlaneCylinderCircleTrace::Cylinder(_)
        ] | [
            PlaneCylinderCircleTrace::Cylinder(_),
            PlaneCylinderCircleTrace::Plane(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }

    let carrier_coefficients =
        circle_coefficients(carrier).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let mut residual_bounds = [0.0; 2];
    for (index, trace) in traces.into_iter().enumerate() {
        let trace_id = if index == 0 {
            PairedTrace::First
        } else {
            PairedTrace::Second
        };
        let lifted = match trace {
            PlaneCylinderCircleTrace::Plane(trace) => plane_trace_coefficients(trace, trace_id)?,
            PlaneCylinderCircleTrace::Cylinder(trace) => {
                cylinder_trace_coefficients(trace, trace_id)?
            }
        };
        let bound = harmonic_residual_bound(carrier_coefficients, lifted)
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

    Ok(PairedPlaneCylinderCircleResidualCertificate {
        carrier,
        carrier_range,
        traces,
        residual_bounds,
        tolerance,
    })
}

type HarmonicCoefficients = [[Interval; 3]; 3];

fn plane_trace_coefficients(
    trace: PlaneCircleTrace,
    trace_id: PairedTrace,
) -> Result<HarmonicCoefficients, IntersectionCertificateError> {
    let plane = trace.surface();
    let pcurve = trace.pcurve();
    let map = trace.parameter_map();
    if !matches!(map.scale(), -1.0 | 1.0) {
        return Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: trace_id,
                reason: "plane-circle trace angle map must have unit magnitude",
            },
        );
    }
    if !finite_plane(plane) || !finite_circle2d(pcurve) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let frame = plane.frame();
    let center = pcurve.center();
    let x = pcurve.x_dir();
    let y = x.perp();
    let (phase_sin, phase_cos) = math::sincos(map.offset());
    let orientation = map.scale();
    let cosine_uv = x * phase_cos + y * phase_sin;
    let sine_uv = (y * phase_cos - x * phase_sin) * orientation;
    let center3 = frame.origin() + frame.x() * center.x + frame.y() * center.y;
    let cosine3 = (frame.x() * cosine_uv.x + frame.y() * cosine_uv.y) * pcurve.radius();
    let sine3 = (frame.x() * sine_uv.x + frame.y() * sine_uv.y) * pcurve.radius();
    coefficients(center3, cosine3, sine3).ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn cylinder_trace_coefficients(
    trace: CylinderLongitudeTrace,
    trace_id: PairedTrace,
) -> Result<HarmonicCoefficients, IntersectionCertificateError> {
    let cylinder = trace.surface();
    let pcurve = trace.pcurve();
    let map = trace.parameter_map();
    if !finite_cylinder(cylinder) || !finite_vec2(pcurve.origin()) || !finite_vec2(pcurve.dir()) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let angular_rate = pcurve.dir().x * map.scale();
    let height_rate = pcurve.dir().y * map.scale();
    if !matches!(angular_rate, -1.0 | 1.0) || height_rate != 0.0 {
        return Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: trace_id,
                reason: "cylinder-circle trace must have unit longitude speed and constant height",
            },
        );
    }
    let phase = pcurve.origin().x + pcurve.dir().x * map.offset();
    let height = pcurve.origin().y + pcurve.dir().y * map.offset();
    let (phase_sin, phase_cos) = math::sincos(phase);
    let frame = cylinder.frame();
    let radial_cosine = frame.x() * phase_cos + frame.y() * phase_sin;
    let radial_sine = (frame.y() * phase_cos - frame.x() * phase_sin) * angular_rate;
    let center = frame.origin() + frame.z() * height;
    coefficients(
        center,
        radial_cosine * cylinder.radius(),
        radial_sine * cylinder.radius(),
    )
    .ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn coefficients(center: Vec3, cosine: Vec3, sine: Vec3) -> Option<HarmonicCoefficients> {
    if !finite_vec3(center) || !finite_vec3(cosine) || !finite_vec3(sine) {
        return None;
    }
    let arrays = [center.to_array(), cosine.to_array(), sine.to_array()];
    Some(arrays.map(|coefficient| coefficient.map(Interval::point)))
}

fn circle_coefficients(carrier: Circle) -> Option<HarmonicCoefficients> {
    let frame = carrier.frame();
    coefficients(
        frame.origin(),
        frame.x() * carrier.radius(),
        frame.y() * carrier.radius(),
    )
}

fn harmonic_residual_bound(
    carrier: HarmonicCoefficients,
    lifted: HarmonicCoefficients,
) -> Option<f64> {
    let harmonic = Interval::new(-1.0, 1.0);
    let mut squared_norm = Interval::point(0.0);
    for axis in 0..3 {
        let residual = finite_interval(
            finite_interval(carrier[0][axis] - lifted[0][axis])?
                + finite_interval(finite_interval(carrier[1][axis] - lifted[1][axis])? * harmonic)?
                + finite_interval(finite_interval(carrier[2][axis] - lifted[2][axis])? * harmonic)?,
        )?;
        squared_norm = finite_interval(squared_norm + finite_interval(residual.square())?)?;
    }
    Some(finite_interval(squared_norm.sqrt()?)?.hi())
}

fn finite_interval(value: Interval) -> Option<Interval> {
    (value.lo().is_finite() && value.hi().is_finite()).then_some(value)
}

fn finite_vec2(value: Vec2) -> bool {
    value.x.is_finite() && value.y.is_finite()
}

fn finite_vec3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn finite_plane(surface: Plane) -> bool {
    finite_vec3(surface.frame().origin())
        && finite_vec3(surface.frame().x())
        && finite_vec3(surface.frame().y())
        && finite_vec3(surface.frame().z())
}

fn finite_cylinder(surface: Cylinder) -> bool {
    surface.radius().is_finite()
        && surface.radius() > 0.0
        && finite_vec3(surface.frame().origin())
        && finite_vec3(surface.frame().x())
        && finite_vec3(surface.frame().y())
        && finite_vec3(surface.frame().z())
}

fn finite_circle(carrier: Circle) -> bool {
    carrier.radius().is_finite()
        && carrier.radius() > 0.0
        && finite_vec3(carrier.frame().origin())
        && finite_vec3(carrier.frame().x())
        && finite_vec3(carrier.frame().y())
        && finite_vec3(carrier.frame().z())
}

fn finite_circle2d(pcurve: Circle2d) -> bool {
    pcurve.radius().is_finite()
        && pcurve.radius() > 0.0
        && finite_vec2(pcurve.center())
        && finite_vec2(pcurve.x_dir())
}

#[cfg(test)]
mod tests {
    use kgeom::curve::Curve;
    use kgeom::curve2d::Curve2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Surface;
    use kgeom::vec::{Point3, Vec3};

    use super::*;

    fn traces(plane: Plane, cylinder: Cylinder) -> [PlaneCylinderCircleTrace; 2] {
        let plane_pcurve = Circle2d::new(Vec2::new(0.0, 0.0), 2.0, Vec2::new(1.0, 0.0)).unwrap();
        let cylinder_pcurve = Line2d::new(Vec2::new(0.0, 1.0), Vec2::new(1.0, 0.0)).unwrap();
        let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
        [
            PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, identity)),
            PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
                cylinder,
                cylinder_pcurve,
                identity,
            )),
        ]
    }

    #[test]
    fn certifies_both_lifts_over_one_complete_period() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)));
        let carrier = Circle::new(*plane.frame(), 2.0).unwrap();
        let certificate = certify_paired_plane_cylinder_circle_residuals(
            carrier,
            carrier.param_range(),
            traces(plane, cylinder),
            1e-9,
        )
        .unwrap();

        assert_eq!(certificate.carrier(), carrier);
        assert_eq!(certificate.carrier_range(), carrier.param_range());
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );
        for parameter in [0.0, 0.37, 2.4, core::f64::consts::TAU] {
            let point = carrier.eval(parameter);
            for (trace, map) in certificate
                .traces()
                .into_iter()
                .zip(certificate.parameter_maps())
            {
                match trace {
                    PlaneCylinderCircleTrace::Plane(trace) => {
                        let uv = trace.pcurve().eval(map.map(parameter));
                        assert!(point.dist(trace.surface().eval([uv.x, uv.y])) <= 1e-9);
                    }
                    PlaneCylinderCircleTrace::Cylinder(trace) => {
                        let uv = trace.pcurve().eval(map.map(parameter));
                        assert!(point.dist(trace.surface().eval([uv.x, uv.y])) <= 1e-9);
                    }
                }
            }
        }
    }

    #[test]
    fn refuses_a_partial_period() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)));
        let carrier = Circle::new(*plane.frame(), 2.0).unwrap();
        assert_eq!(
            certify_paired_plane_cylinder_circle_residuals(
                carrier,
                ParamRange::new(0.0, core::f64::consts::PI),
                traces(plane, cylinder),
                1e-9,
            ),
            Err(IntersectionCertificateError::InvalidCarrierRange)
        );
    }

    #[test]
    fn refuses_nonconstant_cylinder_height() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)));
        let carrier = Circle::new(*plane.frame(), 2.0).unwrap();
        let mut bad_traces = traces(plane, cylinder);
        bad_traces[1] = PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
            cylinder,
            Line2d::new(Vec2::new(0.0, 1.0), Vec2::new(1.0, 1.0)).unwrap(),
            AffineParamMap1d::new(1.0, 0.0).unwrap(),
        ));
        assert!(matches!(
            certify_paired_plane_cylinder_circle_residuals(
                carrier,
                carrier.param_range(),
                bad_traces,
                1e-9,
            ),
            Err(IntersectionCertificateError::UnsupportedTraceParameterization { .. })
        ));
    }

    #[test]
    fn anti_aligned_plane_uses_reversed_circle_parameter() {
        let plane_frame = Frame::new(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let plane = Plane::new(plane_frame);
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let carrier =
            Circle::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)), 2.0).unwrap();
        let plane_pcurve = Circle2d::new(Vec2::new(0.0, 0.0), 2.0, Vec2::new(1.0, 0.0)).unwrap();
        let cylinder_pcurve = Line2d::new(Vec2::new(0.0, 1.0), Vec2::new(1.0, 0.0)).unwrap();
        let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
        let reverse = AffineParamMap1d::new(-1.0, 0.0).unwrap();
        let certificate = certify_paired_plane_cylinder_circle_residuals(
            carrier,
            carrier.param_range(),
            [
                PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(
                    plane,
                    plane_pcurve,
                    reverse,
                )),
                PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
                    cylinder,
                    cylinder_pcurve,
                    identity,
                )),
            ],
            1e-9,
        )
        .unwrap();
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(f64::is_finite)
        );
    }

    #[test]
    fn translated_arbitrarily_tilted_frame_certifies_without_world_axis_cases() {
        let cylinder_frame = Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let cylinder = Cylinder::new(cylinder_frame, 1.25).unwrap();
        let center = cylinder_frame.origin() + cylinder_frame.z() * 0.8;
        let plane = Plane::new(Frame::new(center, cylinder_frame.z(), cylinder_frame.y()).unwrap());
        let carrier = Circle::new(
            Frame::new(center, cylinder_frame.z(), cylinder_frame.x()).unwrap(),
            cylinder.radius(),
        )
        .unwrap();
        let local_center = plane.frame().to_local(center);
        let plane_pcurve = Circle2d::new(
            Vec2::new(local_center.x, local_center.y),
            cylinder.radius(),
            Vec2::new(
                cylinder_frame.x().dot(plane.frame().x()),
                cylinder_frame.x().dot(plane.frame().y()),
            ),
        )
        .unwrap();
        let cylinder_pcurve = Line2d::new(Vec2::new(0.0, 0.8), Vec2::new(1.0, 0.0)).unwrap();
        let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
        let certificate = certify_paired_plane_cylinder_circle_residuals(
            carrier,
            carrier.param_range(),
            [
                PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(
                    plane,
                    plane_pcurve,
                    identity,
                )),
                PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
                    cylinder,
                    cylinder_pcurve,
                    identity,
                )),
            ],
            1e-9,
        )
        .unwrap();
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );
    }
}
