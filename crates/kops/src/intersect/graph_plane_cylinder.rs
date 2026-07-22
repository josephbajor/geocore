//! Promotion of Plane/Cylinder branches into paired traces.
//!
//! The lower analytic solver owns geometric discovery and finite-window
//! clipping. This adapter builds graph-ready pcurves and graph-owned residual
//! proofs for complete-period circles and finite ruling segments. Open arcs
//! and oblique ellipses remain unsupported here. Rulings require exact
//! parallel-family admission and a strict secant proof.

use kgeom::curve::{Circle, Line};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::Vec2;
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, CurveDescriptor, CylinderLongitudeTrace,
    CylinderRulingTrace, PlaneCircleTrace, PlaneCylinderCircleTrace, PlaneCylinderRulingTrace,
    PlaneRulingTrace, certify_paired_plane_cylinder_circle_residuals,
    certify_paired_plane_cylinder_ruling_residuals,
};

use super::error::IntersectionError;
use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult, IntersectionBranchCertificate,
    IntersectionBranchTopology, VerifiedBranchPayload, source_window_parameter_representative,
};
use super::result::{ContactKind, SurfaceSurfaceCurve};

pub(super) fn canonical_line(
    line: Line,
    range: ParamRange,
) -> kcore::error::Result<(Line, ParamRange)> {
    let direction = line.dir();
    let reversed = direction.x < 0.0
        || (direction.x == 0.0 && direction.y < 0.0)
        || (direction.x == 0.0 && direction.y == 0.0 && direction.z < 0.0);
    if reversed {
        Ok((
            Line::new(line.origin(), -direction)?,
            ParamRange::new(-range.hi, -range.lo),
        ))
    } else {
        Ok((line, range))
    }
}

pub(super) fn plane_pcurve(
    carrier: Line,
    surface: Plane,
) -> GraphSurfaceIntersectionResult<(Line2d, AffineParamMap1d)> {
    let frame = surface.frame();
    let local_origin = frame.to_local(carrier.origin());
    let uv_direction = Vec2::new(carrier.dir().dot(frame.x()), carrier.dir().dot(frame.y()));
    let scale = uv_direction.norm();
    let pcurve = Line2d::new(Vec2::new(local_origin.x, local_origin.y), uv_direction)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let map = AffineParamMap1d::new(scale, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    Ok((pcurve, map))
}

/// Promote one finite exact-family, strictly transverse cylinder ruling.
pub(super) fn build_verified_plane_cylinder_ruling_branch(
    raw_carrier: Line,
    raw_branch: &SurfaceSurfaceCurve,
    plane: Plane,
    cylinder: Cylinder,
    plane_first: bool,
    tolerance: f64,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    if raw_branch.kind != ContactKind::Transverse {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            kgraph::IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "Plane/Cylinder ruling promotion requires a transverse branch",
            },
        ));
    }
    let (carrier, carrier_range) = canonical_line(raw_carrier, raw_branch.curve_range)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let (plane_pcurve, plane_map) = plane_pcurve(carrier, plane)?;
    let longitude = if plane_first {
        raw_branch.uv_b_start[0]
    } else {
        raw_branch.uv_a_start[0]
    };
    let frame = cylinder.frame();
    let height_offset = (carrier.origin() - frame.origin()).dot(frame.z());
    let height_rate = carrier.dir().dot(frame.z());
    let cylinder_pcurve = Line2d::new(Vec2::new(longitude, 0.0), Vec2::new(0.0, 1.0))
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let cylinder_map = AffineParamMap1d::new(height_rate, height_offset)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    let plane_trace =
        PlaneCylinderRulingTrace::Plane(PlaneRulingTrace::new(plane, plane_pcurve, plane_map));
    let cylinder_trace = PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
        cylinder,
        cylinder_pcurve,
        cylinder_map,
    ));
    let (pcurves, parameter_maps, traces) = if plane_first {
        (
            [
                Curve2dDescriptor::Line(plane_pcurve),
                Curve2dDescriptor::Line(cylinder_pcurve),
            ],
            [plane_map, cylinder_map],
            [plane_trace, cylinder_trace],
        )
    } else {
        (
            [
                Curve2dDescriptor::Line(cylinder_pcurve),
                Curve2dDescriptor::Line(plane_pcurve),
            ],
            [cylinder_map, plane_map],
            [cylinder_trace, plane_trace],
        )
    };
    let certificate =
        certify_paired_plane_cylinder_ruling_residuals(carrier, carrier_range, traces, tolerance)
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;

    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::Line(carrier),
        carrier_range,
        topology: IntersectionBranchTopology::Open,
        pcurves,
        parameter_maps,
        certificate: IntersectionBranchCertificate::PlaneCylinderRuling(Box::new(certificate)),
    })
}

/// Promote a raw circle branch after proving that it covers one full period.
pub(super) fn build_verified_plane_cylinder_circle_branch(
    raw_carrier: Circle,
    raw_branch: &SurfaceSurfaceCurve,
    plane: Plane,
    cylinder: Cylinder,
    cylinder_range: [ParamRange; 2],
    plane_first: bool,
    tolerance: f64,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    if raw_branch.curve_range.width() != core::f64::consts::TAU {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            kgraph::IntersectionCertificateError::InvalidCarrierRange,
        ));
    }

    // Canonicalize the spatial parameter to the cylinder's own positive
    // longitude. The lower solver's plane-oriented circle may run in the
    // opposite direction when the plane normal is anti-aligned.
    let height =
        (raw_carrier.frame().origin() - cylinder.frame().origin()).dot(cylinder.frame().z());
    let center = cylinder.frame().origin() + cylinder.frame().z() * height;
    let carrier = Circle::new(
        Frame::new(center, cylinder.frame().z(), cylinder.frame().x())
            .map_err(IntersectionError::from)
            .map_err(GraphSurfaceIntersectionError::Intersection)?,
        cylinder.radius(),
    )
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;

    let plane_center = plane.frame().to_local(center);
    let cylinder_x = cylinder.frame().x();
    let plane_pcurve = Circle2d::new(
        Vec2::new(plane_center.x, plane_center.y),
        cylinder.radius(),
        Vec2::new(
            cylinder_x.dot(plane.frame().x()),
            cylinder_x.dot(plane.frame().y()),
        ),
    )
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let plane_orientation = if plane.frame().z().dot(cylinder.frame().z()) > 0.0 {
        1.0
    } else {
        -1.0
    };
    let plane_map = AffineParamMap1d::new(plane_orientation, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;

    // Retain the bounded solver's fitted source-chart height. Recomputing it
    // from the rounded carrier center erases exact cap-boundary provenance
    // after a rigid translation. The paired whole-period residual proof below
    // remains the stored-surface guard for this semantic chart coefficient.
    let cylinder_parameters = if plane_first {
        raw_branch.uv_b_start
    } else {
        raw_branch.uv_a_start
    };
    let cylinder_height = source_window_parameter_representative(
        cylinder_parameters[1],
        cylinder_range[1],
        tolerance,
    )
    .ok_or(GraphSurfaceIntersectionError::BranchCertificate(
        kgraph::IntersectionCertificateError::InvalidTraceFamily,
    ))?;
    let cylinder_pcurve = Line2d::new(Vec2::new(0.0, cylinder_height), Vec2::new(1.0, 0.0))
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let cylinder_map =
        AffineParamMap1d::new(1.0, cylinder_parameters[0] - raw_branch.curve_range.lo)
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;

    let plane_trace =
        PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, plane_map));
    let cylinder_trace = PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
        cylinder,
        cylinder_pcurve,
        cylinder_map,
    ));
    let (pcurves, parameter_maps, traces) = if plane_first {
        (
            [
                Curve2dDescriptor::Circle(plane_pcurve),
                Curve2dDescriptor::Line(cylinder_pcurve),
            ],
            [plane_map, cylinder_map],
            [plane_trace, cylinder_trace],
        )
    } else {
        (
            [
                Curve2dDescriptor::Line(cylinder_pcurve),
                Curve2dDescriptor::Circle(plane_pcurve),
            ],
            [cylinder_map, plane_map],
            [cylinder_trace, plane_trace],
        )
    };
    let certificate = certify_paired_plane_cylinder_circle_residuals(
        carrier,
        raw_branch.curve_range,
        traces,
        tolerance,
    )
    .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;

    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::Circle(carrier),
        carrier_range: raw_branch.curve_range,
        topology: IntersectionBranchTopology::Closed,
        pcurves,
        parameter_maps,
        certificate: IntersectionBranchCertificate::PlaneCylinderCircle(Box::new(certificate)),
    })
}
