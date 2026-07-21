//! Exact, fail-closed Plane/Cylinder ruling trace certification.
//!
//! This proof family is operation-local. It admits only finite, strictly
//! transverse secants whose line carrier shares the signed cylinder axis.

use kcore::interval::Interval;
use kcore::math;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Vec2, Vec3};

use crate::{AffineParamMap1d, IntersectionCertificateError, PairedTrace};

/// Affine line trace on the plane side of a cylinder ruling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneRulingTrace {
    surface: Plane,
    pcurve: Line2d,
    parameter_map: AffineParamMap1d,
}

impl PlaneRulingTrace {
    /// Construct an ordered plane ruling trace candidate.
    pub const fn new(surface: Plane, pcurve: Line2d, parameter_map: AffineParamMap1d) -> Self {
        Self {
            surface,
            pcurve,
            parameter_map,
        }
    }

    /// Exact plane field.
    pub const fn surface(self) -> Plane {
        self.surface
    }

    /// Affine plane pcurve.
    pub const fn pcurve(self) -> Line2d {
        self.pcurve
    }

    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

/// Constant-longitude, affine-height trace of a cylinder ruling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderRulingTrace {
    surface: Cylinder,
    pcurve: Line2d,
    parameter_map: AffineParamMap1d,
}

impl CylinderRulingTrace {
    /// Construct an ordered cylinder ruling trace candidate.
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

    /// Constant-longitude, affine-height pcurve.
    pub const fn pcurve(self) -> Line2d {
        self.pcurve
    }

    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

/// One operand-ordered trace of a Plane/Cylinder ruling proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlaneCylinderRulingTrace {
    /// Plane trace carried by an affine parameter-space line.
    Plane(PlaneRulingTrace),
    /// Cylinder trace carried by constant longitude and affine height.
    Cylinder(CylinderRulingTrace),
}

impl PlaneCylinderRulingTrace {
    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        match self {
            Self::Plane(trace) => trace.parameter_map(),
            Self::Cylinder(trace) => trace.parameter_map(),
        }
    }
}

/// Whole-interval paired residual proof for one finite Plane/Cylinder ruling.
///
/// The private fields bind the canonical line carrier and both source-ordered
/// traces to their outward interval residual bounds. This certificate is
/// operation-local: persistence requires a separate descriptor contract.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairedPlaneCylinderRulingResidualCertificate {
    carrier: Line,
    carrier_range: ParamRange,
    traces: [PlaneCylinderRulingTrace; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
}

impl PairedPlaneCylinderRulingResidualCertificate {
    /// Verified model-space line carrier.
    pub const fn carrier(self) -> Line {
        self.carrier
    }

    /// Finite positive-length carrier interval covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified traces in source operand order.
    pub const fn traces(self) -> [PlaneCylinderRulingTrace; 2] {
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

/// Certify a finite, strictly transverse Plane/Cylinder ruling.
///
/// Family admission is exact and fail-closed. A plane normal shared with a
/// signed cylinder radial frame axis proves that the cylinder axis lies in
/// the plane; otherwise the dyadic sign of `normal · axis` must be exactly
/// zero. An outward interval discriminant must then prove a strict secant,
/// excluding tangent and numerically near-parallel candidates. Both lifted
/// traces are affine in the carrier parameter. Their residual coefficients
/// are formed before one outward interval evaluation over the complete range,
/// preserving shared-parameter correlation without sampling.
pub fn certify_paired_plane_cylinder_ruling_residuals(
    carrier: Line,
    carrier_range: ParamRange,
    traces: [PlaneCylinderRulingTrace; 2],
    tolerance: f64,
) -> Result<PairedPlaneCylinderRulingResidualCertificate, IntersectionCertificateError> {
    if !carrier_range.is_finite() || carrier_range.lo >= carrier_range.hi {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !finite_line(carrier) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let (plane, cylinder) = match traces {
        [
            PlaneCylinderRulingTrace::Plane(plane),
            PlaneCylinderRulingTrace::Cylinder(cylinder),
        ]
        | [
            PlaneCylinderRulingTrace::Cylinder(cylinder),
            PlaneCylinderRulingTrace::Plane(plane),
        ] => (plane.surface(), cylinder.surface()),
        _ => return Err(IntersectionCertificateError::InvalidTraceFamily),
    };
    if !finite_plane(plane) || !finite_cylinder(cylinder) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    certify_strict_ruling_family(carrier, plane, cylinder)?;

    let carrier_coefficients =
        line_coefficients(carrier).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let mut residual_bounds = [0.0; 2];
    for (index, trace) in traces.into_iter().enumerate() {
        let trace_id = if index == 0 {
            PairedTrace::First
        } else {
            PairedTrace::Second
        };
        let lifted = match trace {
            PlaneCylinderRulingTrace::Plane(trace) => plane_ruling_coefficients(trace, trace_id)?,
            PlaneCylinderRulingTrace::Cylinder(trace) => {
                cylinder_ruling_coefficients(trace, trace_id)?
            }
        };
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

    Ok(PairedPlaneCylinderRulingResidualCertificate {
        carrier,
        carrier_range,
        traces,
        residual_bounds,
        tolerance,
    })
}
type AffineCoefficients = [[Interval; 3]; 2];

fn certify_strict_ruling_family(
    carrier: Line,
    plane: Plane,
    cylinder: Cylinder,
) -> Result<(), IntersectionCertificateError> {
    let normal = plane.frame().z();
    let frame = cylinder.frame();
    if !shares_signed_cylinder_axis(carrier, cylinder) {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Plane/Cylinder ruling carrier must share the signed cylinder axis",
            },
        );
    }
    let shared_radial_axis = [frame.x(), frame.y()]
        .into_iter()
        .any(|axis| normal == axis || normal == -axis);
    let zero = [0.0; 3];
    let exact_parallel = shared_radial_axis
        || affine_dot3(normal.to_array(), frame.z().to_array(), zero, 0.0)
            .is_some_and(|dot| dot.sign() == Orientation::Zero);
    if !exact_parallel {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Plane/Cylinder ruling requires an exact plane-normal/cylinder-axis zero",
            },
        );
    }

    let discriminant = ruling_discriminant(plane, cylinder)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    if discriminant.lo() <= 0.0 {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Plane/Cylinder ruling requires a proven strict transverse secant",
            },
        );
    }
    Ok(())
}

fn shares_signed_cylinder_axis(carrier: Line, cylinder: Cylinder) -> bool {
    [cylinder.frame().z(), -cylinder.frame().z()]
        .into_iter()
        .any(|axis| {
            let Ok(first) = Line::new(cylinder.frame().origin(), axis) else {
                return false;
            };
            if carrier.dir() == first.dir() {
                return true;
            }
            Line::new(cylinder.frame().origin(), first.dir())
                .is_ok_and(|second| carrier.dir() == second.dir())
        })
}

fn ruling_discriminant(plane: Plane, cylinder: Cylinder) -> Option<Interval> {
    let normal = plane.frame().z().to_array().map(Interval::point);
    let frame = cylinder.frame();
    let offset = interval_dot_difference(
        normal,
        frame.origin().to_array().map(Interval::point),
        plane.frame().origin().to_array().map(Interval::point),
    )?;
    let nx = interval_dot(normal, frame.x().to_array().map(Interval::point))?;
    let ny = interval_dot(normal, frame.y().to_array().map(Interval::point))?;
    let radial_squared = finite_interval(
        Interval::point(cylinder.radius()).square() * finite_interval(nx.square() + ny.square())?,
    )?;
    finite_interval(radial_squared - offset.square())
}

fn plane_ruling_coefficients(
    trace: PlaneRulingTrace,
    _trace_id: PairedTrace,
) -> Result<AffineCoefficients, IntersectionCertificateError> {
    let plane = trace.surface();
    let pcurve = trace.pcurve();
    let map = trace.parameter_map();
    if !finite_plane(plane) || !finite_vec2(pcurve.origin()) || !finite_vec2(pcurve.dir()) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let map_offset = Interval::point(map.offset());
    let map_scale = Interval::point(map.scale());
    let pcurve_origin = pcurve.origin();
    let pcurve_direction = pcurve.dir();
    let uv_constant = [
        finite_interval(
            Interval::point(pcurve_origin.x)
                + finite_interval(Interval::point(pcurve_direction.x) * map_offset)
                    .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
        finite_interval(
            Interval::point(pcurve_origin.y)
                + finite_interval(Interval::point(pcurve_direction.y) * map_offset)
                    .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
    ];
    let uv_direction = [
        finite_interval(Interval::point(pcurve_direction.x) * map_scale)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
        finite_interval(Interval::point(pcurve_direction.y) * map_scale)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
    ];
    let frame = plane.frame();
    let surface_origin = frame.origin().to_array();
    let surface_u = frame.x().to_array();
    let surface_v = frame.y().to_array();
    let mut coefficients = [[Interval::point(0.0); 3]; 2];
    for axis in 0..3 {
        let lifted_origin_u = checked_interval_product(surface_u[axis], uv_constant[0])?;
        let lifted_origin_v = checked_interval_product(surface_v[axis], uv_constant[1])?;
        coefficients[0][axis] = finite_interval(
            finite_interval(Interval::point(surface_origin[axis]) + lifted_origin_u)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?
                + lifted_origin_v,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let lifted_direction_u = checked_interval_product(surface_u[axis], uv_direction[0])?;
        let lifted_direction_v = checked_interval_product(surface_v[axis], uv_direction[1])?;
        coefficients[1][axis] = finite_interval(lifted_direction_u + lifted_direction_v)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    }
    Ok(coefficients)
}

fn cylinder_ruling_coefficients(
    trace: CylinderRulingTrace,
    trace_id: PairedTrace,
) -> Result<AffineCoefficients, IntersectionCertificateError> {
    let cylinder = trace.surface();
    let pcurve = trace.pcurve();
    let map = trace.parameter_map();
    if !finite_cylinder(cylinder) || !finite_vec2(pcurve.origin()) || !finite_vec2(pcurve.dir()) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    if pcurve.dir().x != 0.0 {
        return Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: trace_id,
                reason: "cylinder ruling trace must have constant longitude and affine height",
            },
        );
    }
    let longitude = pcurve.origin().x;
    if !longitude.is_finite() {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let (longitude_sin, longitude_cos) = math::sincos(longitude);
    let longitude_sin =
        primitive_interval(longitude_sin).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let longitude_cos =
        primitive_interval(longitude_cos).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let map_offset = Interval::point(map.offset());
    let map_scale = Interval::point(map.scale());
    let height_origin = finite_interval(
        Interval::point(pcurve.origin().y)
            + finite_interval(Interval::point(pcurve.dir().y) * map_offset)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
    )
    .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let height_direction = finite_interval(Interval::point(pcurve.dir().y) * map_scale)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let frame = cylinder.frame();
    let surface_origin = frame.origin().to_array();
    let surface_x = frame.x().to_array();
    let surface_y = frame.y().to_array();
    let surface_z = frame.z().to_array();
    let radius = Interval::point(cylinder.radius());
    let mut coefficients = [[Interval::point(0.0); 3]; 2];
    for axis in 0..3 {
        let radial_x = checked_interval_product(surface_x[axis], longitude_cos)?;
        let radial_y = checked_interval_product(surface_y[axis], longitude_sin)?;
        let radial = finite_interval(radial_x + radial_y)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let radial_offset = finite_interval(radial * radius)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let axial_offset = checked_interval_product(surface_z[axis], height_origin)?;
        coefficients[0][axis] = finite_interval(
            finite_interval(Interval::point(surface_origin[axis]) + radial_offset)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?
                + axial_offset,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        coefficients[1][axis] = checked_interval_product(surface_z[axis], height_direction)?;
    }
    Ok(coefficients)
}

fn checked_interval_product(
    scalar: f64,
    interval: Interval,
) -> Result<Interval, IntersectionCertificateError> {
    finite_interval(Interval::point(scalar) * interval)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn primitive_interval(value: f64) -> Option<Interval> {
    value
        .is_finite()
        .then(|| Interval::new(value.next_down(), value.next_up()))
}

fn line_coefficients(carrier: Line) -> Option<AffineCoefficients> {
    if !finite_line(carrier) {
        return None;
    }
    Some([
        carrier.origin().to_array().map(Interval::point),
        carrier.dir().to_array().map(Interval::point),
    ])
}

fn affine_residual_bound(
    carrier: AffineCoefficients,
    lifted: AffineCoefficients,
    range: ParamRange,
) -> Option<f64> {
    let parameter = finite_interval(Interval::new(range.lo, range.hi))?;
    let mut squared_norm = Interval::point(0.0);
    for axis in 0..3 {
        let constant = finite_interval(carrier[0][axis] - lifted[0][axis])?;
        let direction = finite_interval(carrier[1][axis] - lifted[1][axis])?;
        let residual = finite_interval(constant + finite_interval(direction * parameter)?)?;
        squared_norm = finite_interval(squared_norm + residual.square())?;
    }
    finite_interval(squared_norm.sqrt()?).map(Interval::hi)
}

fn interval_dot(lhs: [Interval; 3], rhs: [Interval; 3]) -> Option<Interval> {
    let mut dot = Interval::point(0.0);
    for axis in 0..3 {
        dot = finite_interval(dot + finite_interval(lhs[axis] * rhs[axis])?)?;
    }
    Some(dot)
}

fn interval_dot_difference(
    normal: [Interval; 3],
    point: [Interval; 3],
    origin: [Interval; 3],
) -> Option<Interval> {
    let mut dot = Interval::point(0.0);
    for axis in 0..3 {
        dot = finite_interval(
            dot + finite_interval(normal[axis] * finite_interval(point[axis] - origin[axis])?)?,
        )?;
    }
    Some(dot)
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

fn finite_line(carrier: Line) -> bool {
    finite_vec3(carrier.origin()) && finite_vec3(carrier.dir())
}

#[cfg(test)]
mod tests {
    use kgeom::curve::{Curve, Line};
    use kgeom::curve2d::{Curve2d, Line2d};
    use kgeom::frame::Frame;
    use kgeom::surface::{Cylinder, Plane, Surface};
    use kgeom::vec::{Point3, Vec2, Vec3};

    use super::*;

    fn ruling_traces(
        plane: Plane,
        cylinder: Cylinder,
        carrier: Line,
        longitude: f64,
    ) -> [PlaneCylinderRulingTrace; 2] {
        let local_origin = plane.frame().to_local(carrier.origin());
        let uv_direction = Vec2::new(
            carrier.dir().dot(plane.frame().x()),
            carrier.dir().dot(plane.frame().y()),
        );
        let plane_pcurve =
            Line2d::new(Vec2::new(local_origin.x, local_origin.y), uv_direction).unwrap();
        let plane_map = AffineParamMap1d::new(uv_direction.norm(), 0.0).unwrap();
        let cylinder_pcurve = Line2d::new(Vec2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap();
        let cylinder_map = AffineParamMap1d::new(
            carrier.dir().dot(cylinder.frame().z()),
            (carrier.origin() - cylinder.frame().origin()).dot(cylinder.frame().z()),
        )
        .unwrap();
        [
            PlaneCylinderRulingTrace::Plane(PlaneRulingTrace::new(plane, plane_pcurve, plane_map)),
            PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
                cylinder,
                cylinder_pcurve,
                cylinder_map,
            )),
        ]
    }

    fn world_ruling(direction: Vec3) -> (Plane, Cylinder, Line) {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let plane = Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                direction,
            )
            .unwrap(),
        );
        let carrier = Line::new(Point3::new(0.0, 2.0, 0.0), direction).unwrap();
        (plane, cylinder, carrier)
    }

    #[test]
    fn certifies_world_ruling_and_operand_swap_with_range_parity() {
        let (plane, cylinder, carrier) = world_ruling(Vec3::new(0.0, 0.0, 1.0));
        let range = ParamRange::new(-1.25, 2.5);
        let traces = ruling_traces(plane, cylinder, carrier, core::f64::consts::FRAC_PI_2);
        let certificate =
            certify_paired_plane_cylinder_ruling_residuals(carrier, range, traces, 1e-9).unwrap();
        let swapped = certify_paired_plane_cylinder_ruling_residuals(
            carrier,
            range,
            [traces[1], traces[0]],
            1e-9,
        )
        .unwrap();

        assert_eq!(certificate.carrier(), carrier);
        assert_eq!(certificate.carrier_range(), range);
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
        for parameter in [range.lo, 0.375, range.hi] {
            let point = carrier.eval(parameter);
            for (trace_index, (trace, map)) in certificate
                .traces()
                .into_iter()
                .zip(certificate.parameter_maps())
                .enumerate()
            {
                let lifted = match trace {
                    PlaneCylinderRulingTrace::Plane(trace) => {
                        let uv = trace.pcurve().eval(map.map(parameter));
                        trace.surface().eval([uv.x, uv.y])
                    }
                    PlaneCylinderRulingTrace::Cylinder(trace) => {
                        let uv = trace.pcurve().eval(map.map(parameter));
                        trace.surface().eval([uv.x, uv.y])
                    }
                };
                let observed = point.dist(lifted);
                assert!(observed <= 1e-9);
                assert!(
                    observed <= certificate.residual_bounds()[trace_index],
                    "observed residual {observed:e} escaped published bound {:e}",
                    certificate.residual_bounds()[trace_index],
                );
            }
        }
    }

    #[test]
    fn certifies_true_oblique_frame_and_signed_axis_reversal() {
        let frame = Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let cylinder = Cylinder::new(frame, 1.25).unwrap();
        let plane = Plane::new(Frame::new(frame.origin(), frame.x(), frame.z()).unwrap());
        let carrier =
            Line::new(frame.origin() + frame.y() * cylinder.radius(), -frame.z()).unwrap();
        let range = ParamRange::new(-3.0, 0.75);
        let certificate = certify_paired_plane_cylinder_ruling_residuals(
            carrier,
            range,
            ruling_traces(plane, cylinder, carrier, core::f64::consts::FRAC_PI_2),
            1e-9,
        )
        .unwrap();
        assert_eq!(certificate.carrier(), carrier);
        assert!(certificate.carrier().dir().dot(frame.z()) < 0.0);
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );
    }

    #[test]
    fn refuses_tangent_and_near_parallel_families() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let tangent_plane = Plane::new(
            Frame::new(
                Point3::new(2.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        );
        let tangent_carrier =
            Line::new(Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(matches!(
            certify_paired_plane_cylinder_ruling_residuals(
                tangent_carrier,
                ParamRange::new(-1.0, 1.0),
                ruling_traces(tangent_plane, cylinder, tangent_carrier, 0.0),
                1.0,
            ),
            Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
        ));

        let near_plane = Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 1e-12),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let near_carrier = Line::new(Point3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(matches!(
            certify_paired_plane_cylinder_ruling_residuals(
                near_carrier,
                ParamRange::new(-1.0, 1.0),
                ruling_traces(
                    near_plane,
                    cylinder,
                    near_carrier,
                    core::f64::consts::FRAC_PI_2,
                ),
                1.0,
            ),
            Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
        ));
    }

    #[test]
    fn refuses_altered_carrier_and_nonconstant_longitude_independent_of_tolerance() {
        let (plane, cylinder, carrier) = world_ruling(Vec3::new(0.0, 0.0, 1.0));
        let altered = Line::new(carrier.origin(), Vec3::new(0.0, 1e-12, 1.0)).unwrap();
        assert!(matches!(
            certify_paired_plane_cylinder_ruling_residuals(
                altered,
                ParamRange::new(-1.0, 1.0),
                ruling_traces(plane, cylinder, altered, core::f64::consts::FRAC_PI_2,),
                1.0e6,
            ),
            Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
        ));

        let mut traces = ruling_traces(plane, cylinder, carrier, core::f64::consts::FRAC_PI_2);
        traces[1] = PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
            cylinder,
            Line2d::new(
                Vec2::new(core::f64::consts::FRAC_PI_2, 0.0),
                Vec2::new(1e-12, 1.0),
            )
            .unwrap(),
            AffineParamMap1d::new(1.0, 0.0).unwrap(),
        ));
        assert!(matches!(
            certify_paired_plane_cylinder_ruling_residuals(
                carrier,
                ParamRange::new(-1.0, 1.0),
                traces,
                1.0e6,
            ),
            Err(IntersectionCertificateError::UnsupportedTraceParameterization { .. })
        ));

        let mut underflow_traces =
            ruling_traces(plane, cylinder, carrier, core::f64::consts::FRAC_PI_2);
        underflow_traces[1] = PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
            cylinder,
            Line2d::new(
                Vec2::new(core::f64::consts::FRAC_PI_2, 0.0),
                Vec2::new(f64::MIN_POSITIVE, 1.0),
            )
            .unwrap(),
            AffineParamMap1d::new(f64::MIN_POSITIVE, 0.0).unwrap(),
        ));
        assert!(matches!(
            certify_paired_plane_cylinder_ruling_residuals(
                carrier,
                ParamRange::new(-1.0, 1.0),
                underflow_traces,
                1.0e6,
            ),
            Err(IntersectionCertificateError::UnsupportedTraceParameterization { .. })
        ));
    }
}
