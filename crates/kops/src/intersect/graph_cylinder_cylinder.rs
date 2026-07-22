//! Promotion of strict parallel Cylinder/Cylinder secants into paired rulings.
//!
//! The lower analytic solver owns finite-window discovery. This adapter admits
//! only its complete strict-secant result: exactly two transverse ruling-line
//! branches on exactly parallel or antiparallel cylinder axes. Tangencies,
//! misses, coincident regions, skew axes, and every incomplete family remain
//! explicit typed gaps rather than being promoted as certified branch graphs.

use kcore::predicates::{Orientation, orient3d};
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Vec2;
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, CurveDescriptor, CylinderRulingTrace,
    certify_paired_cylinder_cylinder_ruling_residuals,
};

use super::cylinder_cylinder::{compare_cylinder_windows, intersect_bounded_cylinders};
use super::error::IntersectionError;
use super::graph_plane_cylinder::canonical_line;
use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult, IntersectionBranchCertificate,
    IntersectionBranchTopology, VerifiedBranchPayload,
};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};

const STRICT_SECANT_REASON: &str = "Cylinder/Cylinder graph promotion requires exactly two transverse rulings on exact parallel axes";

fn unsupported() -> GraphSurfaceIntersectionError {
    GraphSurfaceIntersectionError::BranchCertificate(
        kgraph::IntersectionCertificateError::UnsupportedCarrierParameterization {
            reason: STRICT_SECANT_REASON,
        },
    )
}

/// Require exact parallel or antiparallel source axes before lower dispatch.
pub(super) fn require_exact_parallel_cylinder_axes(
    cylinders: [Cylinder; 2],
) -> GraphSurfaceIntersectionResult<()> {
    let first = cylinders[0].frame().z();
    let second = cylinders[1].frame().z();
    let directly_identical = first == second || first == -second;
    let exactly_parallel = directly_identical
        || [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            .into_iter()
            .all(|axis| {
                orient3d(first.to_array(), second.to_array(), axis, [0.0; 3]) == Orientation::Zero
            });
    if exactly_parallel {
        Ok(())
    } else {
        Err(unsupported())
    }
}

/// Discover the strict result from one deterministic source order.
///
/// The lower solver anchors each ruling carrier to its first cylinder. Sorting
/// the complete cylinder/window values before dispatch makes that harmless:
/// the same geometric query receives the same raw carriers under operand swap,
/// while `swapped` restores the caller's pcurve provenance afterward.
pub(super) fn intersect_strict_parallel_cylinders(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    tolerances: kcore::tolerance::Tolerances,
) -> GraphSurfaceIntersectionResult<SurfaceSurfaceIntersections> {
    require_exact_parallel_cylinder_axes(cylinders)?;
    let reversed =
        compare_cylinder_windows(&cylinders[0], ranges[0], &cylinders[1], ranges[1]).is_gt();
    let result = if reversed {
        intersect_bounded_cylinders(
            &cylinders[1],
            ranges[1],
            &cylinders[0],
            ranges[0],
            tolerances,
        )
        .map(SurfaceSurfaceIntersections::swapped)
    } else {
        intersect_bounded_cylinders(
            &cylinders[0],
            ranges[0],
            &cylinders[1],
            ranges[1],
            tolerances,
        )
    }
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;
    require_strict_two_ruling_result(&result)?;
    Ok(result)
}

/// Admit only a complete, positive-length strict two-ruling solver result.
pub(super) fn require_strict_two_ruling_result(
    result: &SurfaceSurfaceIntersections,
) -> GraphSurfaceIntersectionResult<()> {
    let strict = result.is_complete()
        && result.incomplete_evidence().is_empty()
        && result.points.is_empty()
        && result.regions.is_empty()
        && result.curves.len() == 2
        && result.curves.iter().all(|branch| {
            branch.kind == ContactKind::Transverse
                && branch.curve_range.is_finite()
                && branch.curve_range.lo < branch.curve_range.hi
                && matches!(branch.curve, SurfaceIntersectionCurve::Line(_))
        });
    if strict { Ok(()) } else { Err(unsupported()) }
}

/// Promote one finite, proof-certified Cylinder/Cylinder ruling.
pub(super) fn build_verified_cylinder_cylinder_ruling_branch(
    raw_carrier: Line,
    raw_branch: &SurfaceSurfaceCurve,
    cylinders: [Cylinder; 2],
    tolerance: f64,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    if raw_branch.kind != ContactKind::Transverse {
        return Err(unsupported());
    }
    let (carrier, carrier_range) = canonical_line(raw_carrier, raw_branch.curve_range)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let longitudes = [raw_branch.uv_a_start[0], raw_branch.uv_b_start[0]];
    let lifted = [0, 1]
        .map(|operand| cylinder_ruling_trace(carrier, cylinders[operand], longitudes[operand]));
    let [first, second] = lifted;
    let (first_pcurve, first_map, first_trace) = first?;
    let (second_pcurve, second_map, second_trace) = second?;
    let pcurves = [
        Curve2dDescriptor::Line(first_pcurve),
        Curve2dDescriptor::Line(second_pcurve),
    ];
    let parameter_maps = [first_map, second_map];
    let certificate = certify_paired_cylinder_cylinder_ruling_residuals(
        carrier,
        carrier_range,
        [first_trace, second_trace],
        tolerance,
    )
    .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;

    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::Line(carrier),
        carrier_range,
        topology: IntersectionBranchTopology::Open,
        pcurves,
        parameter_maps,
        certificate: IntersectionBranchCertificate::CylinderCylinderRuling(Box::new(certificate)),
    })
}

fn cylinder_ruling_trace(
    carrier: Line,
    cylinder: Cylinder,
    longitude: f64,
) -> GraphSurfaceIntersectionResult<(Line2d, AffineParamMap1d, CylinderRulingTrace)> {
    let frame = cylinder.frame();
    let height_offset = (carrier.origin() - frame.origin()).dot(frame.z());
    let height_rate = carrier.dir().dot(frame.z());
    let pcurve = Line2d::new(Vec2::new(longitude, 0.0), Vec2::new(0.0, 1.0))
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let parameter_map = AffineParamMap1d::new(height_rate, height_offset)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    Ok((
        pcurve,
        parameter_map,
        CylinderRulingTrace::new(cylinder, pcurve, parameter_map),
    ))
}
