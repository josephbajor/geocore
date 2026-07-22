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
    IntersectionBranchTopology, VerifiedBranchPayload, source_window_parameter_representative,
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
    surface_ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    if raw_branch.kind != ContactKind::Transverse {
        return Err(unsupported());
    }
    let raw_direction = raw_carrier.dir();
    let reversed = raw_direction.x < 0.0
        || (raw_direction.x == 0.0 && raw_direction.y < 0.0)
        || (raw_direction.x == 0.0 && raw_direction.y == 0.0 && raw_direction.z < 0.0);
    let (carrier, carrier_range) = canonical_line(raw_carrier, raw_branch.curve_range)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    // Preserve the bounded solver's source-chart endpoint values. They carry
    // its source-window fitting provenance; recomputing the affine map from a
    // rounded model-space dot product loses exact boundary coefficients under
    // rigid translation. The paired whole-range residual certificate below
    // still validates the resulting trace against the stored cylinder.
    let mut endpoint_parameters = if reversed {
        [
            [raw_branch.uv_a_end, raw_branch.uv_a_start],
            [raw_branch.uv_b_end, raw_branch.uv_b_start],
        ]
    } else {
        [
            [raw_branch.uv_a_start, raw_branch.uv_a_end],
            [raw_branch.uv_b_start, raw_branch.uv_b_end],
        ]
    };
    for operand in 0..2 {
        for endpoint in 0..2 {
            endpoint_parameters[operand][endpoint][1] = source_window_parameter_representative(
                endpoint_parameters[operand][endpoint][1],
                surface_ranges[operand][1],
                tolerance,
            )
            .ok_or_else(unsupported)?;
        }
    }
    let chart_origin_endpoint =
        common_source_boundary_endpoint(endpoint_parameters, surface_ranges).unwrap_or(0);
    let anchor = usize::from(
        compare_cylinder_windows(
            &cylinders[0],
            surface_ranges[0],
            &cylinders[1],
            surface_ranges[1],
        )
        .is_gt(),
    );
    let (carrier, carrier_range) = normalize_carrier_parameter(
        carrier,
        carrier_range,
        endpoint_parameters[anchor],
        chart_origin_endpoint,
    )?;
    let lifted = [0, 1].map(|operand| {
        cylinder_ruling_trace(
            carrier_range,
            cylinders[operand],
            endpoint_parameters[operand],
            chart_origin_endpoint,
        )
    });
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

/// Find a graph endpoint represented by an exact height-window boundary on
/// both source cylinders.
///
/// Endpoint parameters have already passed the bounded solver's source-window
/// corridor and been replaced by their exact source coefficients. Selecting
/// the first common endpoint is therefore deterministic and source-order
/// independent. This only chooses a semantic chart origin; the paired
/// whole-range residual proof remains the geometric authority.
fn common_source_boundary_endpoint(
    endpoint_parameters: [[[f64; 2]; 2]; 2],
    surface_ranges: [[ParamRange; 2]; 2],
) -> Option<usize> {
    (0..2).find(|&endpoint| {
        (0..2).all(|operand| {
            let height = endpoint_parameters[operand][endpoint][1];
            let range = surface_ranges[operand][1];
            height == range.lo || height == range.hi
        })
    })
}

/// Rebase the common line parameter to zero at one deterministically selected
/// graph endpoint. A jointly represented source boundary is preferred so the
/// exact topology coefficients on both cylinders share the literal zero root;
/// otherwise the canonical low endpoint is used. The source order is chosen
/// by the same complete cylinder/window ordering as lower dispatch, so operand
/// swaps retain the same carrier. A paired residual proof is reissued after
/// the rebase.
fn normalize_carrier_parameter(
    carrier: Line,
    carrier_range: ParamRange,
    anchor_parameters: [[f64; 2]; 2],
    chart_origin_endpoint: usize,
) -> GraphSurfaceIntersectionResult<(Line, ParamRange)> {
    if chart_origin_endpoint >= 2 {
        return Err(unsupported());
    }
    let heights = anchor_parameters.map(|parameters| parameters[1]);
    let sign = match heights[1].total_cmp(&heights[0]) {
        core::cmp::Ordering::Greater => 1.0,
        core::cmp::Ordering::Less => -1.0,
        core::cmp::Ordering::Equal => return Err(unsupported()),
    };
    let chart_origin_height = heights[chart_origin_endpoint];
    let normalized_parameters = heights.map(|height| sign * (height - chart_origin_height));
    let normalized_range = ParamRange::new(normalized_parameters[0], normalized_parameters[1]);
    if !normalized_range.is_finite() || normalized_range.lo >= normalized_range.hi {
        return Err(unsupported());
    }
    let shift = if chart_origin_endpoint == 0 {
        carrier_range.lo
    } else {
        carrier_range.hi
    };
    let origin = carrier.origin() + carrier.dir() * shift;
    let carrier = Line::new(origin, carrier.dir())
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    Ok((carrier, normalized_range))
}

fn cylinder_ruling_trace(
    carrier_range: ParamRange,
    cylinder: Cylinder,
    endpoint_parameters: [[f64; 2]; 2],
    chart_origin_endpoint: usize,
) -> GraphSurfaceIntersectionResult<(Line2d, AffineParamMap1d, CylinderRulingTrace)> {
    if chart_origin_endpoint >= 2 {
        return Err(unsupported());
    }
    let longitude = endpoint_parameters[0][0];
    let heights = endpoint_parameters.map(|parameters| parameters[1]);
    let height_rate = (heights[1] - heights[0]) / carrier_range.width();
    let chart_origin_parameter = if chart_origin_endpoint == 0 {
        carrier_range.lo
    } else {
        carrier_range.hi
    };
    if chart_origin_parameter != 0.0 {
        return Err(unsupported());
    }
    let height_offset = heights[chart_origin_endpoint];
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
