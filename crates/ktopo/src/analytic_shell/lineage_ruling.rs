//! Source-lineage fallback for representation-independent cylinder rulings.
//!
//! The graph-level Plane/Cylinder ruling proof deliberately requires an exact
//! dyadic `plane_normal . cylinder_axis == 0`.  A rigid all-nonzero frame can
//! lose that representation equality even when the result plane is copied
//! verbatim from a topology-owned extrusion face.  This module admits only
//! that one typed refusal and replaces only its family witness. The witness
//! can be either two distinct whole-fin source-face lines sharing the signed
//! cylinder axis or a complete topology-owned circular cylinder cap whose
//! source axis is semantically perpendicular to the opposing cylinder axis.
//! Strict secancy and both complete-range residual obligations remain
//! geometric.

mod circular_cap;

use crate::entity::{EdgeId, EntityRef, FaceId, FinId};
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::store::Store;
use kcore::interval::Interval;
use kcore::math;
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, CylinderRulingTrace, IntersectionCertificateError, PairedTrace,
    PlaneCylinderRulingTrace, PlaneRulingTrace,
};

const EXACT_PLANE_AXIS_ZERO_REASON: &str =
    "Plane/Cylinder ruling requires an exact plane-normal/cylinder-axis zero";

/// Topology-owned authority for the missing exact Plane/Cylinder ruling-family
/// relation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceLineagePlaneCylinderRulingFamilyWitness {
    /// Two distinct whole-fin source lines share the opposing cylinder's
    /// signed axis.
    ParallelSourceLines {
        /// Distinct source edges in deterministic face traversal order.
        edges: [EdgeId; 2],
        /// Conservative complete-fin residual bounds against signed-axis
        /// lines.
        axis_residual_bounds: [f64; 2],
    },
    /// A complete planar cap ring is manifold-paired with a source Cylinder
    /// whose axis is semantically perpendicular to the opposing cylinder.
    CircularCylinderCap {
        /// Endpoint-free cap ring edge.
        cap_edge: EdgeId,
        /// The ring's fin on the planar source face.
        cap_fin: FinId,
        /// The paired ring fin on the source Cylinder face.
        source_cylinder_fin: FinId,
        /// The source Cylinder face owning the paired fin.
        source_cylinder_face: FaceId,
    },
}

/// Whole-range Plane/Cylinder ruling proof whose family witness comes from
/// topology-owned complete-fin source topology rather than a rounded frame
/// dot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SourceLineagePlaneCylinderRulingResidualCertificate {
    carrier: Line,
    carrier_range: ParamRange,
    traces: [PlaneCylinderRulingTrace; 2],
    source_plane_face: FaceId,
    source_family_witness: SourceLineagePlaneCylinderRulingFamilyWitness,
    residual_bounds: [f64; 2],
    tolerance: f64,
}

impl SourceLineagePlaneCylinderRulingResidualCertificate {
    /// Verified model-space line carrier.
    pub const fn carrier(self) -> Line {
        self.carrier
    }

    /// Finite positive-length carrier interval covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified result traces in face-use order.
    pub const fn traces(self) -> [PlaneCylinderRulingTrace; 2] {
        self.traces
    }

    /// Lineaged planar source face that supplied the family witness.
    pub const fn source_plane_face(self) -> FaceId {
        self.source_plane_face
    }

    /// Topology-owned source-family authority used by this proof.
    pub const fn source_family_witness(self) -> SourceLineagePlaneCylinderRulingFamilyWitness {
        self.source_family_witness
    }

    /// Conservative whole-range residual bounds in face-use order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance against which both traces were certified.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }

    /// Carrier-to-pcurve parameter maps in face-use order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        [
            self.traces[0].parameter_map(),
            self.traces[1].parameter_map(),
        ]
    }
}

pub(super) fn is_exact_plane_axis_zero_refusal(error: &IntersectionCertificateError) -> bool {
    matches!(
        error,
        IntersectionCertificateError::UnsupportedCarrierParameterization { reason }
            if *reason == EXACT_PLANE_AXIS_ZERO_REASON
    )
}

pub(super) fn certify_source_lineage_ruling_residuals(
    store: &Store,
    carrier: Line,
    carrier_range: ParamRange,
    traces: [PlaneCylinderRulingTrace; 2],
    plane_source: Option<EntityRef>,
    tolerance: f64,
) -> Option<Result<SourceLineagePlaneCylinderRulingResidualCertificate, IntersectionCertificateError>>
{
    if !carrier_range.is_finite() || carrier_range.lo >= carrier_range.hi {
        return Some(Err(IntersectionCertificateError::InvalidCarrierRange));
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Some(Err(IntersectionCertificateError::InvalidTolerance));
    }
    if !finite_line(carrier) {
        return Some(Err(IntersectionCertificateError::NonFiniteGeometry));
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
        _ => {
            return Some(Err(IntersectionCertificateError::InvalidTraceFamily));
        }
    };
    if !finite_plane(plane) || !finite_cylinder(cylinder) {
        return Some(Err(IntersectionCertificateError::NonFiniteGeometry));
    }
    if !shares_signed_axis(carrier, cylinder) {
        return Some(Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Plane/Cylinder ruling carrier must share the signed cylinder axis",
            },
        ));
    }

    let (source_plane_face, source_family_witness) =
        source_axis_witness(store, plane_source, plane, cylinder, tolerance)?;

    let Some(discriminant) = ruling_discriminant(plane, cylinder) else {
        return Some(Err(IntersectionCertificateError::NonFiniteGeometry));
    };
    if discriminant.lo() <= 0.0 {
        return Some(Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Plane/Cylinder ruling requires a proven strict transverse secant",
            },
        ));
    }

    let Some(carrier_coefficients) = line_coefficients(carrier) else {
        return Some(Err(IntersectionCertificateError::NonFiniteGeometry));
    };
    let mut residual_bounds = [0.0; 2];
    for (index, trace) in traces.into_iter().enumerate() {
        let trace_id = trace_id(index);
        let lifted = match trace {
            PlaneCylinderRulingTrace::Plane(trace) => plane_ruling_coefficients(trace),
            PlaneCylinderRulingTrace::Cylinder(trace) => {
                cylinder_ruling_coefficients(trace, trace_id)
            }
        };
        let lifted = match lifted {
            Ok(lifted) => lifted,
            Err(error) => return Some(Err(error)),
        };
        let Some(bound) = affine_residual_bound(carrier_coefficients, lifted, carrier_range) else {
            return Some(Err(IntersectionCertificateError::NonFiniteResidualBound {
                trace: trace_id,
            }));
        };
        if bound > tolerance {
            return Some(Err(
                IntersectionCertificateError::ResidualExceedsTolerance {
                    trace: trace_id,
                    residual_bound: bound,
                    tolerance,
                },
            ));
        }
        residual_bounds[index] = bound;
    }

    Some(Ok(SourceLineagePlaneCylinderRulingResidualCertificate {
        carrier,
        carrier_range,
        traces,
        source_plane_face,
        source_family_witness,
        residual_bounds,
        tolerance,
    }))
}

fn source_axis_witness(
    store: &Store,
    source: Option<EntityRef>,
    result_plane: Plane,
    cylinder: Cylinder,
    tolerance: f64,
) -> Option<(FaceId, SourceLineagePlaneCylinderRulingFamilyWitness)> {
    let EntityRef::Face(face_id) = source? else {
        return None;
    };
    let face = store.get(face_id).ok()?;
    let SurfaceGeom::Plane(source_plane) = store.get(face.surface).ok()? else {
        return None;
    };
    if *source_plane != result_plane {
        return None;
    }

    if let Some((edges, axis_residual_bounds)) =
        source_line_axis_witness(store, face_id, cylinder, tolerance)
    {
        return Some((
            face_id,
            SourceLineagePlaneCylinderRulingFamilyWitness::ParallelSourceLines {
                edges,
                axis_residual_bounds,
            },
        ));
    }

    circular_cap::certify_circular_cap_source(
        store,
        face_id,
        result_plane,
        cylinder.frame().z(),
        tolerance,
    )
    .map(|witness| {
        (
            face_id,
            SourceLineagePlaneCylinderRulingFamilyWitness::CircularCylinderCap {
                cap_edge: witness.cap_edge,
                cap_fin: witness.cap_fin,
                source_cylinder_fin: witness.source_cylinder_fin,
                source_cylinder_face: witness.source_cylinder_face,
            },
        )
    })
}

fn source_line_axis_witness(
    store: &Store,
    face_id: FaceId,
    cylinder: Cylinder,
    tolerance: f64,
) -> Option<([EdgeId; 2], [f64; 2])> {
    let face = store.get(face_id).ok()?;
    let axis = cylinder.frame().z();
    let mut lines = Vec::<(EdgeId, Line, f64)>::new();
    for &loop_id in &face.loops {
        let loop_ = store.get(loop_id).ok()?;
        for &fin_id in &loop_.fins {
            let fin = store.get(fin_id).ok()?;
            let edge = store.get(fin.edge).ok()?;
            let Some(curve_id) = edge.curve else {
                continue;
            };
            let CurveGeom::Line(line) = store.get(curve_id).ok()? else {
                continue;
            };
            let Some(bounds) = edge.bounds else {
                continue;
            };
            let range = ParamRange::new(bounds.0, bounds.1);
            let Some(axis_residual) = source_line_axis_residual(*line, range, axis) else {
                continue;
            };
            if axis_residual > tolerance
                || certify_whole_fin_incidence(store, face_id, loop_id, fin_id, tolerance)
                    != WholeFinIncidence::Certified
            {
                continue;
            }
            if !lines.iter().any(|(edge, _, _)| *edge == fin.edge) {
                lines.push((fin.edge, *line, axis_residual));
            }
        }
    }

    for first in 0..lines.len() {
        for second in (first + 1)..lines.len() {
            if distinct_parallel_lines(lines[first].1, lines[second].1, axis, tolerance) {
                return Some((
                    [lines[first].0, lines[second].0],
                    [lines[first].2, lines[second].2],
                ));
            }
        }
    }
    None
}

fn shares_signed_axis(line: Line, cylinder: Cylinder) -> bool {
    shares_signed_direction(line, cylinder.frame().z())
}

fn shares_signed_direction(line: Line, direction: Vec3) -> bool {
    line.dir() == direction || line.dir() == -direction
}

fn source_line_axis_residual(line: Line, range: ParamRange, axis: Vec3) -> Option<f64> {
    if !range.is_finite() || range.lo >= range.hi {
        return None;
    }
    let line_direction = line.dir().to_array().map(Interval::point);
    let axis_direction = axis.to_array().map(Interval::point);
    let alignment = interval_dot(line_direction, axis_direction)?;
    let signed_axis = if alignment.lo() > 0.0 {
        axis_direction
    } else if alignment.hi() < 0.0 {
        axis_direction.map(|component| -component)
    } else {
        return None;
    };
    let parameter = finite_interval(Interval::new(range.lo, range.hi))?;
    let mut squared = Interval::point(0.0);
    for coordinate in 0..3 {
        let delta = finite_interval(line_direction[coordinate] - signed_axis[coordinate])?;
        let residual = finite_interval(delta * parameter)?;
        squared = finite_interval(squared + residual.square())?;
    }
    finite_interval(squared.sqrt()?).map(Interval::hi)
}

fn distinct_parallel_lines(first: Line, second: Line, axis: Vec3, tolerance: f64) -> bool {
    let offset = second.origin() - first.origin();
    let offset = offset.to_array().map(Interval::point);
    let axis = axis.to_array().map(Interval::point);
    let cross = [
        offset[1] * axis[2] - offset[2] * axis[1],
        offset[2] * axis[0] - offset[0] * axis[2],
        offset[0] * axis[1] - offset[1] * axis[0],
    ];
    let squared = cross
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square());
    let admitted_tubes = Interval::point(2.0 * tolerance).square();
    squared.lo().is_finite()
        && admitted_tubes.hi().is_finite()
        && squared.lo() > admitted_tubes.hi()
}

type AffineCoefficients = [[Interval; 3]; 2];

fn plane_ruling_coefficients(
    trace: PlaneRulingTrace,
) -> Result<AffineCoefficients, IntersectionCertificateError> {
    let plane = trace.surface();
    let pcurve = trace.pcurve();
    let map = trace.parameter_map();
    if !finite_plane(plane) || !finite_vec2(pcurve.origin()) || !finite_vec2(pcurve.dir()) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let uv_constant = affine_line_constant(pcurve, map)?;
    let uv_direction = affine_line_direction(pcurve, map)?;
    let frame = plane.frame();
    let surface_origin = frame.origin().to_array();
    let surface_u = frame.x().to_array();
    let surface_v = frame.y().to_array();
    let mut coefficients = [[Interval::point(0.0); 3]; 2];
    for axis in 0..3 {
        coefficients[0][axis] = checked_sum3(
            Interval::point(surface_origin[axis]),
            checked_interval_product(surface_u[axis], uv_constant[0])?,
            checked_interval_product(surface_v[axis], uv_constant[1])?,
        )?;
        coefficients[1][axis] = checked_sum(
            checked_interval_product(surface_u[axis], uv_direction[0])?,
            checked_interval_product(surface_v[axis], uv_direction[1])?,
        )?;
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
    let height_origin = checked_sum(
        Interval::point(pcurve.origin().y),
        checked_interval_product(pcurve.dir().y, Interval::point(map.offset()))?,
    )?;
    let height_direction = checked_interval_product(pcurve.dir().y, Interval::point(map.scale()))?;
    let frame = cylinder.frame();
    let surface_origin = frame.origin().to_array();
    let surface_x = frame.x().to_array();
    let surface_y = frame.y().to_array();
    let surface_z = frame.z().to_array();
    let radius = Interval::point(cylinder.radius());
    let mut coefficients = [[Interval::point(0.0); 3]; 2];
    for axis in 0..3 {
        let radial = checked_sum(
            checked_interval_product(surface_x[axis], longitude_cos)?,
            checked_interval_product(surface_y[axis], longitude_sin)?,
        )?;
        let radial_offset = finite_interval(radial * radius)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let axial_offset = checked_interval_product(surface_z[axis], height_origin)?;
        coefficients[0][axis] = checked_sum3(
            Interval::point(surface_origin[axis]),
            radial_offset,
            axial_offset,
        )?;
        coefficients[1][axis] = checked_interval_product(surface_z[axis], height_direction)?;
    }
    Ok(coefficients)
}

fn affine_line_constant(
    pcurve: Line2d,
    map: AffineParamMap1d,
) -> Result<[Interval; 2], IntersectionCertificateError> {
    let map_offset = Interval::point(map.offset());
    Ok([
        checked_sum(
            Interval::point(pcurve.origin().x),
            checked_interval_product(pcurve.dir().x, map_offset)?,
        )?,
        checked_sum(
            Interval::point(pcurve.origin().y),
            checked_interval_product(pcurve.dir().y, map_offset)?,
        )?,
    ])
}

fn affine_line_direction(
    pcurve: Line2d,
    map: AffineParamMap1d,
) -> Result<[Interval; 2], IntersectionCertificateError> {
    let scale = Interval::point(map.scale());
    Ok([
        checked_interval_product(pcurve.dir().x, scale)?,
        checked_interval_product(pcurve.dir().y, scale)?,
    ])
}

fn line_coefficients(carrier: Line) -> Option<AffineCoefficients> {
    finite_line(carrier).then(|| {
        [
            carrier.origin().to_array().map(Interval::point),
            carrier.dir().to_array().map(Interval::point),
        ]
    })
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

fn checked_interval_product(
    scalar: f64,
    interval: Interval,
) -> Result<Interval, IntersectionCertificateError> {
    finite_interval(Interval::point(scalar) * interval)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn checked_sum(
    first: Interval,
    second: Interval,
) -> Result<Interval, IntersectionCertificateError> {
    finite_interval(first + second).ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn checked_sum3(
    first: Interval,
    second: Interval,
    third: Interval,
) -> Result<Interval, IntersectionCertificateError> {
    checked_sum(checked_sum(first, second)?, third)
}

fn primitive_interval(value: f64) -> Option<Interval> {
    value
        .is_finite()
        .then(|| Interval::new(value.next_down(), value.next_up()))
}

fn finite_interval(value: Interval) -> Option<Interval> {
    (value.lo().is_finite() && value.hi().is_finite()).then_some(value)
}

const fn trace_id(index: usize) -> PairedTrace {
    if index == 0 {
        PairedTrace::First
    } else {
        PairedTrace::Second
    }
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
    use super::*;
    use crate::entity::EntityRef;
    use crate::profile::PlanarProfile;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point2, Point3};

    const TOLERANCE: f64 = 1.0e-9;

    struct ObliqueRulingFixture {
        store: Store,
        source_face: FaceId,
        plane: Plane,
        cylinder: Cylinder,
        carrier: Line,
        range: ParamRange,
        traces: [PlaneCylinderRulingTrace; 2],
        placement: Frame,
    }

    struct ObliqueCapRulingFixture {
        store: Store,
        source_face: FaceId,
        source_loop: crate::entity::LoopId,
        cap_edge: EdgeId,
        cap_fin: FinId,
        source_cylinder_fin: FinId,
        source_cylinder_face: FaceId,
        source_cylinder: Cylinder,
        plane: Plane,
        cylinder: Cylinder,
        carrier: Line,
        range: ParamRange,
        traces: [PlaneCylinderRulingTrace; 2],
        placement: Frame,
    }

    fn oblique_ruling_fixture() -> ObliqueRulingFixture {
        let placement = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap();
        let polygon = [
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, 0.0),
            Point2::new(-1.0, 1.0),
        ];
        let profile = PlanarProfile::from_polygon(placement, &polygon).unwrap();
        let mut store = Store::new();
        let body = crate::make::extrude_profile(&mut store, &profile, 2.0).unwrap();
        let source_face = store.faces_of_body(body).unwrap()[2];
        let face = store.get(source_face).unwrap();
        let SurfaceGeom::Plane(plane) = store.get(face.surface).unwrap() else {
            panic!("polygon extrusion side was not planar");
        };
        let plane = *plane;
        let cylinder = Cylinder::new(placement, 0.75).unwrap();

        let segment = polygon[1] - polygon[0];
        let quadratic_a = segment.dot(segment);
        let quadratic_b = 2.0 * polygon[0].dot(segment);
        let quadratic_c = polygon[0].dot(polygon[0]) - cylinder.radius().powi(2);
        let discriminant = quadratic_b * quadratic_b - 4.0 * quadratic_a * quadratic_c;
        let parameter = (-quadratic_b + discriminant.sqrt()) / (2.0 * quadratic_a);
        let local = polygon[0] + segment * parameter;
        let carrier_origin = placement.point_at(local.x, local.y, 0.0);
        let carrier = Line::new(carrier_origin, placement.z()).unwrap();
        let range = ParamRange::new(0.0, 2.0);

        let plane_local = plane.frame().to_local(carrier_origin);
        let plane_direction = Vec2::new(
            carrier.dir().dot(plane.frame().x()),
            carrier.dir().dot(plane.frame().y()),
        );
        let plane_pcurve =
            Line2d::new(Point2::new(plane_local.x, plane_local.y), plane_direction).unwrap();
        let plane_map = AffineParamMap1d::new(plane_direction.norm(), 0.0).unwrap();

        let cylinder_local = placement.to_local(carrier_origin);
        let longitude = math::atan2(cylinder_local.y, cylinder_local.x);
        let cylinder_pcurve = Line2d::new(
            Point2::new(longitude, cylinder_local.z),
            Vec2::new(0.0, 1.0),
        )
        .unwrap();
        let cylinder_map = AffineParamMap1d::new(
            carrier.dir().dot(placement.z()),
            (carrier.origin() - placement.origin()).dot(placement.z()),
        )
        .unwrap();
        let traces = [
            PlaneCylinderRulingTrace::Plane(PlaneRulingTrace::new(plane, plane_pcurve, plane_map)),
            PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
                cylinder,
                cylinder_pcurve,
                cylinder_map,
            )),
        ];

        ObliqueRulingFixture {
            store,
            source_face,
            plane,
            cylinder,
            carrier,
            range,
            traces,
            placement,
        }
    }

    fn oblique_cap_ruling_fixture() -> ObliqueCapRulingFixture {
        let placement = Frame::new(
            Point3::new(2.5, -1.75, 0.625),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap();
        let mut store = Store::new();
        let body = crate::make::cylinder(&mut store, &placement, 1.0, 0.1).unwrap();
        let source_face = store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|face_id| {
                matches!(
                    store.get(store.get(*face_id).unwrap().surface).unwrap(),
                    SurfaceGeom::Plane(_)
                )
            })
            .unwrap();
        let source_face_data = store.get(source_face).unwrap();
        let [source_loop] = source_face_data.loops.as_slice() else {
            panic!("cylinder cap must own one ring loop");
        };
        let source_loop = *source_loop;
        let [cap_fin] = store.get(source_loop).unwrap().fins.as_slice() else {
            panic!("cylinder cap ring loop must own one fin");
        };
        let cap_fin = *cap_fin;
        let cap_fin_data = store.get(cap_fin).unwrap();
        let cap_edge = cap_fin_data.edge;
        let edge = store.get(cap_edge).unwrap();
        let [first_fin, second_fin] = edge.fins.as_slice() else {
            panic!("cylinder cap ring must have two manifold fins");
        };
        let source_cylinder_fin = if *first_fin == cap_fin {
            *second_fin
        } else if *second_fin == cap_fin {
            *first_fin
        } else {
            panic!("cap fin must be owned by its ring edge");
        };
        let source_cylinder_loop = store.get(source_cylinder_fin).unwrap().parent;
        let source_cylinder_face = store.get(source_cylinder_loop).unwrap().face;
        let SurfaceGeom::Cylinder(source_cylinder) = store
            .get(store.get(source_cylinder_face).unwrap().surface)
            .unwrap()
        else {
            panic!("paired ring fin must belong to the cylinder side");
        };
        let source_cylinder = *source_cylinder;
        let SurfaceGeom::Plane(plane) = store.get(source_face_data.surface).unwrap() else {
            unreachable!();
        };
        let plane = *plane;

        let opposing_frame = Frame::new(
            plane.frame().origin() - placement.x(),
            placement.x(),
            placement.y(),
        )
        .unwrap();
        let cylinder = Cylinder::new(opposing_frame, 0.5).unwrap();
        assert_eq!(cylinder.frame().z(), source_cylinder.frame().x());
        let carrier_origin = cylinder.frame().origin() + cylinder.frame().x() * cylinder.radius();
        let carrier = Line::new(carrier_origin, cylinder.frame().z()).unwrap();
        let range = ParamRange::new(0.0, 2.0);

        let plane_local = plane.frame().to_local(carrier_origin);
        let plane_direction = Vec2::new(
            carrier.dir().dot(plane.frame().x()),
            carrier.dir().dot(plane.frame().y()),
        );
        let plane_pcurve =
            Line2d::new(Point2::new(plane_local.x, plane_local.y), plane_direction).unwrap();
        let plane_map = AffineParamMap1d::new(plane_direction.norm(), 0.0).unwrap();

        let cylinder_local = cylinder.frame().to_local(carrier_origin);
        let cylinder_pcurve = Line2d::new(
            Point2::new(
                math::atan2(cylinder_local.y, cylinder_local.x),
                cylinder_local.z,
            ),
            Vec2::new(0.0, 1.0),
        )
        .unwrap();
        let cylinder_map = AffineParamMap1d::new(
            carrier.dir().dot(cylinder.frame().z()),
            (carrier.origin() - cylinder.frame().origin()).dot(cylinder.frame().z()),
        )
        .unwrap();
        let traces = [
            PlaneCylinderRulingTrace::Plane(PlaneRulingTrace::new(plane, plane_pcurve, plane_map)),
            PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
                cylinder,
                cylinder_pcurve,
                cylinder_map,
            )),
        ];

        ObliqueCapRulingFixture {
            store,
            source_face,
            source_loop,
            cap_edge,
            cap_fin,
            source_cylinder_fin,
            source_cylinder_face,
            source_cylinder,
            plane,
            cylinder,
            carrier,
            range,
            traces,
            placement,
        }
    }

    #[test]
    fn exact_zero_refusal_uses_two_whole_fin_source_axis_residual_witnesses() {
        let fixture = oblique_ruling_fixture();
        let error = kgraph::certify_paired_plane_cylinder_ruling_residuals(
            fixture.carrier,
            fixture.range,
            fixture.traces,
            TOLERANCE,
        )
        .unwrap_err();
        assert!(is_exact_plane_axis_zero_refusal(&error), "{error:?}");

        let Some(Ok(certificate)) = certify_source_lineage_ruling_residuals(
            &fixture.store,
            fixture.carrier,
            fixture.range,
            fixture.traces,
            Some(EntityRef::Face(fixture.source_face)),
            TOLERANCE,
        ) else {
            panic!("topology-owned oblique source ruling was not certified");
        };
        assert_eq!(certificate.source_plane_face(), fixture.source_face);
        let SourceLineagePlaneCylinderRulingFamilyWitness::ParallelSourceLines {
            edges,
            axis_residual_bounds,
        } = certificate.source_family_witness()
        else {
            panic!("polygon side must retain the parallel-line family witness");
        };
        assert_ne!(edges[0], edges[1]);
        assert!(
            axis_residual_bounds
                .into_iter()
                .all(|bound| bound <= TOLERANCE)
        );
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= TOLERANCE)
        );
    }

    #[test]
    fn oblique_circular_cap_uses_deterministic_whole_fin_source_witness_under_trace_swap() {
        let fixture = oblique_cap_ruling_fixture();
        for traces in [fixture.traces, [fixture.traces[1], fixture.traces[0]]] {
            let error = kgraph::certify_paired_plane_cylinder_ruling_residuals(
                fixture.carrier,
                fixture.range,
                traces,
                TOLERANCE,
            )
            .unwrap_err();
            assert!(is_exact_plane_axis_zero_refusal(&error), "{error:?}");

            let Some(Ok(certificate)) = certify_source_lineage_ruling_residuals(
                &fixture.store,
                fixture.carrier,
                fixture.range,
                traces,
                Some(EntityRef::Face(fixture.source_face)),
                TOLERANCE,
            ) else {
                panic!("topology-owned oblique circular cap was not certified");
            };
            assert_eq!(certificate.source_plane_face(), fixture.source_face);
            assert_eq!(
                certificate.source_family_witness(),
                SourceLineagePlaneCylinderRulingFamilyWitness::CircularCylinderCap {
                    cap_edge: fixture.cap_edge,
                    cap_fin: fixture.cap_fin,
                    source_cylinder_fin: fixture.source_cylinder_fin,
                    source_cylinder_face: fixture.source_cylinder_face,
                }
            );
            assert!(
                certificate
                    .residual_bounds()
                    .into_iter()
                    .all(|bound| bound <= TOLERANCE)
            );
        }
    }

    #[test]
    fn circular_cap_source_witness_refuses_topology_and_metadata_tampers() {
        let fixture = oblique_cap_ruling_fixture();
        let mut cases = Vec::<(&str, Store)>::new();

        let mut duplicate_cap_fin = fixture.store.clone();
        duplicate_cap_fin
            .get_mut(fixture.source_loop)
            .unwrap()
            .fins
            .push(fixture.cap_fin);
        cases.push(("duplicate cap-fin ownership", duplicate_cap_fin));

        let mut missing_cap_winding = fixture.store.clone();
        let cap_use = missing_cap_winding
            .get(fixture.cap_fin)
            .unwrap()
            .pcurve
            .unwrap();
        missing_cap_winding.get_mut(fixture.cap_fin).unwrap().pcurve = Some(
            crate::entity::FinPcurve::new(
                cap_use.curve(),
                cap_use.range(),
                cap_use.edge_to_pcurve(),
            )
            .unwrap(),
        );
        cases.push(("missing cap closure winding", missing_cap_winding));

        let mut bounded_ring = fixture.store.clone();
        bounded_ring.get_mut(fixture.cap_edge).unwrap().bounds =
            Some((0.0, core::f64::consts::TAU));
        cases.push(("bounded endpoint-free ring", bounded_ring));

        let mut duplicate_side_face = fixture.store.clone();
        let shell = duplicate_side_face
            .get(fixture.source_cylinder_face)
            .unwrap()
            .shell;
        duplicate_side_face
            .get_mut(shell)
            .unwrap()
            .faces
            .push(fixture.source_cylinder_face);
        cases.push(("duplicate side-face ownership", duplicate_side_face));

        let mut broken_side_pcurve = fixture.store.clone();
        let side_use = broken_side_pcurve
            .get(fixture.source_cylinder_fin)
            .unwrap()
            .pcurve
            .unwrap();
        broken_side_pcurve
            .get_mut(fixture.source_cylinder_fin)
            .unwrap()
            .pcurve = Some(
            crate::entity::FinPcurve::new(
                side_use.curve(),
                side_use.range(),
                side_use.edge_to_pcurve(),
            )
            .unwrap(),
        );
        cases.push(("missing side closure winding", broken_side_pcurve));

        for (case, store) in cases {
            assert!(
                source_axis_witness(
                    &store,
                    Some(EntityRef::Face(fixture.source_face)),
                    fixture.plane,
                    fixture.cylinder,
                    TOLERANCE,
                )
                .is_none(),
                "{case} must fail closed"
            );
        }

        let tilted_axis =
            fixture.source_cylinder.frame().x() + fixture.source_cylinder.frame().z() * 1.0e-4;
        let tilted_frame = Frame::new(
            fixture.cylinder.frame().origin(),
            tilted_axis,
            fixture.placement.y(),
        )
        .unwrap();
        let tilted_cylinder = Cylinder::new(tilted_frame, fixture.cylinder.radius()).unwrap();
        assert!(
            source_axis_witness(
                &fixture.store,
                Some(EntityRef::Face(fixture.source_face)),
                fixture.plane,
                tilted_cylinder,
                TOLERANCE,
            )
            .is_none(),
            "a nonperpendicular opposing axis must fail closed"
        );
    }

    #[test]
    fn tilted_axis_beyond_tolerance_cannot_use_source_lineage_witness() {
        let fixture = oblique_ruling_fixture();
        let tilted_axis = fixture.placement.z() + fixture.plane.frame().z() * 1.0e-4;
        let tilted_frame = Frame::new(
            fixture.placement.origin(),
            tilted_axis,
            fixture.placement.x(),
        )
        .unwrap();
        let tilted_cylinder = Cylinder::new(tilted_frame, fixture.cylinder.radius()).unwrap();

        assert!(
            source_axis_witness(
                &fixture.store,
                Some(EntityRef::Face(fixture.source_face)),
                fixture.plane,
                tilted_cylinder,
                TOLERANCE,
            )
            .is_none(),
            "a visibly tilted axis must exceed the complete source-fin residual budget"
        );
    }
}
