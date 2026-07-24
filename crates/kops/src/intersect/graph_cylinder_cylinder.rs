//! Promotion of exact parallel Cylinder/Cylinder radial relations.
//!
//! The lower analytic solver owns finite-window discovery. This adapter admits
//! either its complete strict-secant result (exactly two transverse ruling-line
//! branches) or an exact proof that the two infinite radial supports are
//! exterior-disjoint. The latter is the only successful empty result in this
//! closed admission, so generic empty or axially clipped lower results cannot
//! masquerade as radial separation. The proof-only classifier additionally
//! exposes exact external tangency, internal tangency, and common-cylinder
//! support without admitting them as graph intersections. Other internal
//! relations, skew axes, and every incomplete family remain explicit typed
//! gaps.

use kcore::predicates::{Orientation, orient3d};
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Vec2;
use kgraph::exact::bounded_polynomial::ExactScalar;
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, CurveDescriptor, CylinderRulingTrace,
    certify_paired_cylinder_cylinder_ruling_residuals,
};

use super::cylinder_cylinder::{
    compare_cylinder_windows, intersect_bounded_cylinders, validate_ranges,
};
use super::error::IntersectionError;
use super::graph_plane_cylinder::canonical_line;
use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult, IntersectionBranchCertificate,
    IntersectionBranchTopology, VerifiedBranchPayload, source_window_parameter_representative,
};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};

const SUPPORTED_PARALLEL_RESULT_REASON: &str = "Cylinder/Cylinder graph promotion requires either exactly two transverse rulings or proven strict exterior radial separation on exact parallel axes";

/// Non-forgeable completion evidence for exact exterior radial separation of
/// one parallel or antiparallel Cylinder/Cylinder graph query.
///
/// The private field prevents downstream code from manufacturing this witness.
/// It is meaningful only while carried by the graph result that owns the source
/// surface identities and complete-empty raw result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParallelCylinderExteriorRadialSeparation {
    _private: (),
}

/// Operand-directed containment proved by an exact internal radial tangency.
///
/// The direction is part of the proof because ordered Boolean operations need
/// to distinguish the containing support from the contained support. Swapping
/// the classifier operands reverses this value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParallelCylinderInternalTangency {
    /// The first cylinder's radial disk contains the second cylinder's disk.
    FirstContainsSecond,
    /// The second cylinder's radial disk contains the first cylinder's disk.
    SecondContainsFirst,
}

impl ParallelCylinderExteriorRadialSeparation {
    pub(super) const fn certified() -> Self {
        Self { _private: () }
    }
}

/// Exact radial relation proved for two parallel Cylinder supports.
///
/// This is a proof-only classification of the two infinite radial supports;
/// it does not inspect or make any claim about bounded axial windows.
/// [`Self::Unresolved`] is deliberately broad and includes radial overlap,
/// containment, nonparallel axes, and arithmetic outside the bounded
/// exact-expansion envelope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParallelCylinderRadialRelation {
    /// The radial supports are separated by a strict positive clearance.
    StrictExterior,
    /// The radial supports have exactly zero external clearance.
    ExactExternalTangent,
    /// The unequal radial supports have exactly zero internal clearance.
    ExactInternalTangent(ParallelCylinderInternalTangency),
    /// Both cylinders have the same infinite radial support.
    ExactCommonSupport,
    /// Neither supported proof completed; this is not a geometric relation.
    Unresolved,
}

fn unsupported() -> GraphSurfaceIntersectionError {
    GraphSurfaceIntersectionError::BranchCertificate(
        kgraph::IntersectionCertificateError::UnsupportedCarrierParameterization {
            reason: SUPPORTED_PARALLEL_RESULT_REASON,
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

/// Discover one admitted result from a deterministic source order.
///
/// Successful emptiness is reserved for the exact exterior radial predicate
/// `d > radius_a + radius_b`. In particular, a complete-empty lower result
/// caused only by disjoint axial windows is not admitted. The lower range
/// validator runs before the global radial shortcut so malformed windows never
/// become certified misses.
///
/// The lower solver anchors each ruling carrier to its first cylinder. Sorting
/// the complete cylinder/window values before dispatch makes that harmless:
/// the same geometric query receives the same raw carriers under operand swap,
/// while `swapped` restores the caller's pcurve provenance afterward.
pub(super) fn intersect_certified_parallel_cylinders(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    tolerances: kcore::tolerance::Tolerances,
) -> GraphSurfaceIntersectionResult<(
    SurfaceSurfaceIntersections,
    Option<ParallelCylinderExteriorRadialSeparation>,
)> {
    require_exact_parallel_cylinder_axes(cylinders)?;
    validate_ranges(ranges[0], ranges[1])
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    if exact_strict_exterior_radial_miss(cylinders) {
        return Ok((
            SurfaceSurfaceIntersections::complete_empty(),
            Some(ParallelCylinderExteriorRadialSeparation::certified()),
        ));
    }
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
    Ok((result, None))
}

/// Classify the supported exact relation of two infinite radial supports.
///
/// For an axis vector `z` that is only floating-point normalized, the squared
/// transverse clearance comparison is division-free:
///
/// `|(origin_b - origin_a) x z|^2 - (radius_a + radius_b)^2 |z|^2`.
///
/// Every source `f64` is treated as its exact dyadic value. Checked expansion
/// arithmetic fails closed outside its fixed safe envelope. The same sign is
/// required with both stored axes, making the result independent of operand
/// order without assuming either axis has exact unit length. Common support
/// requires zero transverse distance under both axes and exactly equal positive
/// radii; axial displacement and azimuth chart orientation are irrelevant.
/// External tangency additionally requires the mathematical radius sum to be
/// represented by the rounded `f64` sum without error; this prevents a rounded
/// sum from becoming false equality evidence. Internal tangency instead
/// compares the exact radius-difference expansion directly, so no rounded
/// scalar difference can become equality evidence. Unequal positive radii and
/// exact equality under both stored axes are required.
pub fn classify_parallel_cylinder_radial_relation(
    cylinders: [Cylinder; 2],
) -> ParallelCylinderRadialRelation {
    if require_exact_parallel_cylinder_axes(cylinders).is_err() {
        return ParallelCylinderRadialRelation::Unresolved;
    }
    let Some(offset) = exact_vector_difference(
        cylinders[1].frame().origin().to_array(),
        cylinders[0].frame().origin().to_array(),
    ) else {
        return ParallelCylinderRadialRelation::Unresolved;
    };

    let [Some(first_metric), Some(second_metric)] = cylinders
        .map(|cylinder| exact_parallel_radial_metric(&offset, cylinder.frame().z().to_array()))
    else {
        return ParallelCylinderRadialRelation::Unresolved;
    };
    let metrics = [first_metric, second_metric];
    if cylinders[0].radius() == cylinders[1].radius()
        && metrics
            .iter()
            .all(|metric| metric.transverse_squared.is_zero())
    {
        return ParallelCylinderRadialRelation::ExactCommonSupport;
    }

    let radii = cylinders.map(|cylinder| cylinder.radius());
    if !radii
        .into_iter()
        .all(|radius| radius.is_finite() && radius > 0.0)
    {
        return ParallelCylinderRadialRelation::Unresolved;
    }
    let [Some(first_radius), Some(second_radius)] = radii.map(exact) else {
        return ParallelCylinderRadialRelation::Unresolved;
    };

    let containment = match radii[0].total_cmp(&radii[1]) {
        std::cmp::Ordering::Greater => Some(ParallelCylinderInternalTangency::FirstContainsSecond),
        std::cmp::Ordering::Less => Some(ParallelCylinderInternalTangency::SecondContainsFirst),
        std::cmp::Ordering::Equal => None,
    };
    if let Some(containment) = containment
        && let Some(internal_clearances) = first_radius
            .sub(&second_radius)
            .ok()
            .and_then(|difference| difference.mul(&difference).ok())
            .and_then(|squared| exact_parallel_radial_clearance_signs(&metrics, &squared))
        && internal_clearances
            .into_iter()
            .all(|clearance| clearance == 0)
    {
        return ParallelCylinderRadialRelation::ExactInternalTangent(containment);
    }

    let Some(radius_sum) = first_radius.add(&second_radius).ok() else {
        return ParallelCylinderRadialRelation::Unresolved;
    };
    let Some(radius_sum_squared) = radius_sum.mul(&radius_sum).ok() else {
        return ParallelCylinderRadialRelation::Unresolved;
    };

    let Some(clearances) = exact_parallel_radial_clearance_signs(&metrics, &radius_sum_squared)
    else {
        return ParallelCylinderRadialRelation::Unresolved;
    };
    if clearances.into_iter().all(|clearance| clearance > 0) {
        return ParallelCylinderRadialRelation::StrictExterior;
    }

    let rounded_radius_sum = cylinders[0].radius() + cylinders[1].radius();
    let radius_sum_is_error_free = exact(rounded_radius_sum)
        .and_then(|rounded| radius_sum.sub(&rounded).ok())
        .is_some_and(|rounding_error| rounding_error.is_zero());
    if radius_sum_is_error_free && clearances.into_iter().all(|clearance| clearance == 0) {
        return ParallelCylinderRadialRelation::ExactExternalTangent;
    }
    ParallelCylinderRadialRelation::Unresolved
}

fn exact_strict_exterior_radial_miss(cylinders: [Cylinder; 2]) -> bool {
    classify_parallel_cylinder_radial_relation(cylinders)
        == ParallelCylinderRadialRelation::StrictExterior
}

struct ExactParallelRadialMetric {
    transverse_squared: ExactScalar,
    axis_squared: ExactScalar,
}

fn exact_parallel_radial_metric(
    offset: &[ExactScalar; 3],
    axis: [f64; 3],
) -> Option<ExactParallelRadialMetric> {
    let axis = exact_vector(axis)?;
    let cross = exact_cross(offset, &axis)?;
    Some(ExactParallelRadialMetric {
        transverse_squared: exact_norm_squared(&cross)?,
        axis_squared: exact_norm_squared(&axis)?,
    })
}

fn exact_parallel_radial_clearance(
    metric: &ExactParallelRadialMetric,
    radius_sum_squared: &ExactScalar,
) -> Option<ExactScalar> {
    metric
        .transverse_squared
        .sub(&radius_sum_squared.mul(&metric.axis_squared).ok()?)
        .ok()
}

fn exact_parallel_radial_clearance_signs(
    metrics: &[ExactParallelRadialMetric; 2],
    radial_squared: &ExactScalar,
) -> Option<[i8; 2]> {
    Some([
        exact_parallel_radial_clearance(&metrics[0], radial_squared)?.sign(),
        exact_parallel_radial_clearance(&metrics[1], radial_squared)?.sign(),
    ])
}

fn exact(value: f64) -> Option<ExactScalar> {
    ExactScalar::from_f64(value).ok()
}

fn exact_vector(value: [f64; 3]) -> Option<[ExactScalar; 3]> {
    Some([exact(value[0])?, exact(value[1])?, exact(value[2])?])
}

fn exact_vector_difference(point: [f64; 3], origin: [f64; 3]) -> Option<[ExactScalar; 3]> {
    let point = exact_vector(point)?;
    let origin = exact_vector(origin)?;
    Some([
        point[0].sub(&origin[0]).ok()?,
        point[1].sub(&origin[1]).ok()?,
        point[2].sub(&origin[2]).ok()?,
    ])
}

fn exact_cross(first: &[ExactScalar; 3], second: &[ExactScalar; 3]) -> Option<[ExactScalar; 3]> {
    let component = |a: usize, b: usize, c: usize, d: usize| {
        first[a]
            .mul(&second[b])
            .ok()?
            .sub(&first[c].mul(&second[d]).ok()?)
            .ok()
    };
    Some([
        component(1, 2, 2, 1)?,
        component(2, 0, 0, 2)?,
        component(0, 1, 1, 0)?,
    ])
}

fn exact_norm_squared(vector: &[ExactScalar; 3]) -> Option<ExactScalar> {
    let mut squared = ExactScalar::zero();
    for component in vector {
        squared = squared.add(&component.mul(component).ok()?).ok()?;
    }
    Some(squared)
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
        for endpoint in &mut endpoint_parameters[operand] {
            endpoint[1] = source_window_parameter_representative(
                endpoint[1],
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
