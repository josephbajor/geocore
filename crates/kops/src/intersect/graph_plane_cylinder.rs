//! Promotion of complete Plane/Cylinder circle branches into paired traces.
//!
//! The lower analytic solver owns geometric discovery and finite-window
//! clipping. This adapter accepts only a complete carrier period and builds
//! graph-ready pcurves plus the graph-owned whole-period residual proof. Open
//! arcs, oblique ellipses, and cylinder rulings remain unsupported here.

use kgeom::curve::Circle;
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::Vec2;
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, CurveDescriptor, CylinderLongitudeTrace, PlaneCircleTrace,
    PlaneCylinderCircleTrace, certify_paired_plane_cylinder_circle_residuals,
};

use super::error::IntersectionError;
use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult, IntersectionBranchCertificate,
    IntersectionBranchTopology, VerifiedBranchPayload,
};
use super::result::SurfaceSurfaceCurve;

/// Promote a raw circle branch after proving that it covers one full period.
pub(super) fn build_verified_plane_cylinder_circle_branch(
    raw_carrier: Circle,
    raw_branch: &SurfaceSurfaceCurve,
    plane: Plane,
    cylinder: Cylinder,
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

    let cylinder_start = if plane_first {
        raw_branch.uv_b_start[0]
    } else {
        raw_branch.uv_a_start[0]
    };
    let cylinder_pcurve = Line2d::new(Vec2::new(0.0, height), Vec2::new(1.0, 0.0))
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let cylinder_map = AffineParamMap1d::new(1.0, cylinder_start - raw_branch.curve_range.lo)
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
