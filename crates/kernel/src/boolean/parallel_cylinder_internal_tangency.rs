//! Manifold regularized CSG for exact internally tangent finite cylinders.
//!
//! The relation layer has already proved unequal internally tangent radial
//! supports, directed disk containment, strict positive axial overlap, and one
//! complete exact preorder of the four live source endpoints. This adapter
//! consumes only the result classes whose boundary is representable by the
//! current manifold analytic-shell contract:
//!
//! - intersection is the contained-radius band over the axial overlap;
//! - union is a whole containing-source copy when the contained axial window
//!   is itself contained, or one canonical tangent chain when the contained
//!   window has one or two protruding axial tails;
//! - contained-minus-containing is the zero, one, or two contained-radius
//!   bands selected by the shared interval sweep.
//!
//! Containing-minus-contained has a pinched tangent-annulus cross-section
//! throughout the positive overlap and stays an explicit allocation-free
//! topology refusal here.

use core::cmp::Ordering;

use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2};
use kgraph::AffineParamMap1d;
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticFaceSplitPiece, AnalyticPcurveUse,
    AnalyticShellClosedEdge, AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace,
    AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop, AnalyticShellPcurve,
    AnalyticShellSurface, AnalyticShellVertex, AnalyticVertexKey,
};
use ktopo::entity::{EntityRef, FaceDomain, Sense};

use super::axial_interval_sweep::{
    AuthoredAxialEndpoint, AxialEndpointContributor, AxialEndpointContributors,
    AxialIntervalOperand, AxialIntervalPlan, PlannedAxialSpan, plan_axial_interval_difference,
    plan_axial_interval_sweep,
};
use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, StageResult, adapt_operation,
    refused,
};
use super::curved_realize::{
    realize_analytic_shell_inputs, realize_certified_cylinder_source_copies,
};
use super::curved_source::{CertifiedCylinderBoundary, CertifiedCylinderSource};
use super::parallel_cylinder_relation::{
    CertifiedParallelCylinderInternalRadialTangency, ParallelCylinderAxialBoundaryWitness,
};
use super::select::PlanarBooleanOperation;
use crate::BodyId;
use crate::session::PartEdit;

const PERIOD: f64 = core::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InternalTangencyPlanGap {
    RelationBinding,
    IntervalContract,
    ArithmeticGuard,
    Lineage,
}

#[derive(Debug, Clone, Copy)]
struct BoundBoundary {
    operand: AxialIntervalOperand,
    boundary_index: usize,
    boundary: CertifiedCylinderBoundary,
    axial_parameter: f64,
}

#[derive(Debug, Clone, Copy)]
struct TangentUnionBand {
    operand: usize,
    low: BoundBoundary,
    high: BoundBoundary,
    tail_index: Option<usize>,
}

#[derive(Debug, Clone, Copy)]
struct PreparedTangentUnionBand {
    band: TangentUnionBand,
    frame: Frame,
    radius: f64,
    low_center: Point3,
    low_height: f64,
    high_center: Point3,
    high_height: f64,
}

#[derive(Debug, Clone, Copy)]
struct PreparedTangentUnionContact {
    boundary: BoundBoundary,
    outer_edge: AnalyticEdgeKey,
    inner_edge: AnalyticEdgeKey,
    outer_circle: Circle,
    inner_circle: Circle,
    plane: Plane,
    outer_fin_sense: Sense,
    inner_fin_sense: Sense,
}

/// Consume the exact internal-tangency relation without reclassifying radii or
/// reconstructing containment direction from floating-point geometry.
pub(super) fn execute_parallel_cylinder_internal_tangency(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let contained = relation.contained_operand();
    let containing = relation.containing_operand();
    let intersection = plan_axial_interval_sweep(
        adapt_operation(PlanarBooleanOperation::Intersect),
        relation.preorder(),
    );
    let [overlap] = intersection.spans() else {
        return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
    };

    match operation {
        PlanarBooleanOperation::Intersect => {
            if span_is_whole_operand(overlap, contained) {
                return realize_certified_cylinder_source_copies(
                    edit,
                    &[(bodies[contained].clone(), cylinders[contained])],
                    scope,
                );
            }
            realize_internal_tangent_bands(edit, cylinders, relation, &intersection, linear, scope)
        }
        PlanarBooleanOperation::Unite => {
            let contained_tails =
                plan_axial_interval_difference(relation.preorder(), operand_identity(contained));
            match contained_tails.spans() {
                [] => realize_certified_cylinder_source_copies(
                    edit,
                    &[(bodies[containing].clone(), cylinders[containing])],
                    scope,
                ),
                tails @ ([_] | [_, _]) => realize_internal_tangent_union_chain(
                    edit, cylinders, relation, tails, linear, scope,
                ),
                _ => refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
            }
        }
        PlanarBooleanOperation::Subtract if contained == 0 => {
            let difference = plan_axial_interval_sweep(
                adapt_operation(PlanarBooleanOperation::Subtract),
                relation.preorder(),
            );
            realize_internal_tangent_bands(edit, cylinders, relation, &difference, linear, scope)
        }
        PlanarBooleanOperation::Subtract => {
            refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported)
        }
    }
}

fn realize_internal_tangent_union_chain(
    edit: &mut PartEdit<'_>,
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    tails: &[PlannedAxialSpan],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let input = match prepare_internal_tangent_union_chain(cylinders, relation, tails) {
        Ok(input) => input,
        Err(
            InternalTangencyPlanGap::RelationBinding
            | InternalTangencyPlanGap::IntervalContract
            | InternalTangencyPlanGap::ArithmeticGuard
            | InternalTangencyPlanGap::Lineage,
        ) => {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        }
    };
    realize_analytic_shell_inputs(edit, &[input], linear, scope)
}

/// Replace one or two radial steps by one canonical low-to-high tangent chain.
///
/// Each contact owns two bounded complete-period circles with one shared
/// topology vertex at their parameter seams. Every planar shoulder owns one
/// two-fin boundary walk, so each tangent vertex has one degree-four manifold
/// link rather than two pinched face loops. Bands and contacts are derived
/// from the retained exact preorder; authored endpoint order never selects a
/// configuration-specific construction path.
fn prepare_internal_tangent_union_chain(
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    tails: &[PlannedAxialSpan],
) -> Result<AnalyticShellInput, InternalTangencyPlanGap> {
    bind_relation_boundaries(cylinders, relation.boundaries())?;
    if !(1..=2).contains(&tails.len()) {
        return Err(InternalTangencyPlanGap::IntervalContract);
    }
    let contained = relation.contained_operand();
    let containing = relation.containing_operand();
    let contained_operand = operand_identity(contained);
    let containing_operand = operand_identity(containing);
    let outer_source = cylinders
        .get(containing)
        .ok_or(InternalTangencyPlanGap::RelationBinding)?;
    let inner_source = cylinders
        .get(contained)
        .ok_or(InternalTangencyPlanGap::RelationBinding)?;

    let [outer_low, outer_high] = ordered_operand_boundaries(cylinders, relation, containing)?;
    let mut bands = Vec::with_capacity(tails.len() + 1);
    for (tail_index, tail) in tails.iter().enumerate() {
        if !tail.side_operands().contains(contained_operand)
            || tail.side_operands().contains(containing_operand)
        {
            return Err(InternalTangencyPlanGap::IntervalContract);
        }
        let low = single_bound_boundary(bind_boundary_class(cylinders, relation, tail.low())?)?;
        let high = single_bound_boundary(bind_boundary_class(cylinders, relation, tail.high())?)?;
        if compare_boundaries(relation, low, high) != Ordering::Less {
            return Err(InternalTangencyPlanGap::IntervalContract);
        }
        bands.push(TangentUnionBand {
            operand: contained,
            low,
            high,
            tail_index: Some(tail_index),
        });
    }
    bands.push(TangentUnionBand {
        operand: containing,
        low: outer_low,
        high: outer_high,
        tail_index: None,
    });
    bands.sort_by(|first, second| compare_boundaries(relation, first.low, second.low));
    validate_tangent_union_band_chain(&bands, relation, contained, containing)?;

    let outer_source_frame = *outer_source.cylinder().frame();
    let trial_frame = Frame::new(
        outer_low.boundary.center(),
        outer_source_frame.z(),
        outer_source_frame.x(),
    )
    .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let (_, trial_height) = exact_axial_projection(trial_frame, outer_high.boundary.center())
        .ok_or(InternalTangencyPlanGap::ArithmeticGuard)?;
    let axis = if trial_height > 0.0 {
        outer_source_frame.z()
    } else if trial_height < 0.0 {
        -outer_source_frame.z()
    } else {
        return Err(InternalTangencyPlanGap::ArithmeticGuard);
    };
    let axial_frame = Frame::new(outer_low.boundary.center(), axis, outer_source_frame.x())
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let (outer_high_center, outer_height) =
        exact_axial_projection(axial_frame, outer_high.boundary.center())
            .ok_or(InternalTangencyPlanGap::ArithmeticGuard)?;
    if !outer_height.is_finite() || outer_height <= 0.0 {
        return Err(InternalTangencyPlanGap::ArithmeticGuard);
    }

    let outer_low_center = outer_low.boundary.center();
    let (inner_low_center, _) =
        exact_axial_projection(*inner_source.cylinder().frame(), outer_low_center)
            .ok_or(InternalTangencyPlanGap::ArithmeticGuard)?;
    let radial = inner_low_center - outer_low_center;
    let outer_frame = Frame::new(outer_low_center, axis, radial)
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let inner_frame = outer_frame.with_origin(inner_low_center);

    let outer_radius = outer_source.cylinder().radius();
    let inner_radius = inner_source.cylinder().radius();
    if !outer_radius.is_finite()
        || !inner_radius.is_finite()
        || outer_radius <= inner_radius
        || inner_radius <= 0.0
    {
        return Err(InternalTangencyPlanGap::RelationBinding);
    }

    let prepared_bands = bands
        .iter()
        .copied()
        .map(|band| {
            prepare_tangent_union_band(
                band,
                contained,
                outer_frame,
                inner_frame,
                outer_radius,
                inner_radius,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let outer_band = prepared_bands
        .iter()
        .find(|band| band.band.operand == containing)
        .ok_or(InternalTangencyPlanGap::IntervalContract)?;
    if outer_band.low_height != 0.0
        || outer_band.high_height != outer_height
        || outer_band.low_center != outer_low_center
        || outer_band.high_center != outer_high_center
    {
        return Err(InternalTangencyPlanGap::ArithmeticGuard);
    }

    let far_low_edge = AnalyticEdgeKey::new(0);
    let far_high_edge = AnalyticEdgeKey::new(1 + 2 * tails.len() as u64);
    let mut contacts = Vec::with_capacity(tails.len());
    let mut vertices = Vec::with_capacity(tails.len());
    let mut edges = Vec::with_capacity(2 * tails.len());
    for contact_index in 0..tails.len() {
        let left = prepared_bands[contact_index];
        let right = prepared_bands[contact_index + 1];
        let boundary = left.band.high;
        let vertex = AnalyticVertexKey::new(contact_index as u64);
        let outer_edge = AnalyticEdgeKey::new(1 + 2 * contact_index as u64);
        let inner_edge = AnalyticEdgeKey::new(2 + 2 * contact_index as u64);
        let outer_center = if left.band.operand == containing {
            left.high_center
        } else {
            right.low_center
        };
        let inner_center = if left.band.operand == contained {
            left.high_center
        } else {
            right.low_center
        };
        let outer_circle = Circle::new(outer_frame.with_origin(outer_center), outer_radius)
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
        let inner_circle = Circle::new(inner_frame.with_origin(inner_center), inner_radius)
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
        let outward_axis = if right.band.operand == contained {
            axis
        } else {
            -axis
        };
        let plane = Plane::new(
            Frame::new(outer_center, outward_axis, outer_frame.x())
                .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
        );
        let outer_fin_sense = if left.band.operand == containing {
            Sense::Forward
        } else {
            Sense::Reversed
        };
        let inner_fin_sense = match outer_fin_sense {
            Sense::Forward => Sense::Reversed,
            Sense::Reversed => Sense::Forward,
        };
        vertices.push(AnalyticShellVertex::new(vertex, outer_circle.eval(0.0)));
        edges.push(
            AnalyticShellEdge::new(
                outer_edge,
                [vertex, vertex],
                AnalyticShellCurve::Circle(outer_circle),
                outer_circle.param_range(),
            )
            .with_source(EntityRef::Edge(boundary.boundary.edge())),
        );
        edges.push(
            AnalyticShellEdge::new(
                inner_edge,
                [vertex, vertex],
                AnalyticShellCurve::Circle(inner_circle),
                inner_circle.param_range(),
            )
            .with_derived_sources([
                EntityRef::Face(inner_source.side_face()),
                EntityRef::Face(boundary.boundary.cap_face()),
            ]),
        );
        contacts.push(PreparedTangentUnionContact {
            boundary,
            outer_edge,
            inner_edge,
            outer_circle,
            inner_circle,
            plane,
            outer_fin_sense,
            inner_fin_sense,
        });
    }

    let first_band = prepared_bands
        .first()
        .copied()
        .ok_or(InternalTangencyPlanGap::IntervalContract)?;
    let last_band = prepared_bands
        .last()
        .copied()
        .ok_or(InternalTangencyPlanGap::IntervalContract)?;
    if operand_index(first_band.band.low.operand) != first_band.band.operand
        || operand_index(last_band.band.high.operand) != last_band.band.operand
    {
        return Err(InternalTangencyPlanGap::Lineage);
    }
    let far_low_circle = Circle::new(
        first_band.frame.with_origin(first_band.low_center),
        first_band.radius,
    )
    .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let far_high_circle = Circle::new(
        last_band.frame.with_origin(last_band.high_center),
        last_band.radius,
    )
    .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let closed_edges = vec![
        AnalyticShellClosedEdge::new(
            far_low_edge,
            AnalyticShellCurve::Circle(far_low_circle),
            far_low_circle.param_range(),
        )
        .with_source(EntityRef::Edge(first_band.band.low.boundary.edge())),
        AnalyticShellClosedEdge::new(
            far_high_edge,
            AnalyticShellCurve::Circle(far_high_circle),
            far_high_circle.param_range(),
        )
        .with_source(EntityRef::Edge(last_band.band.high.boundary.edge())),
    ];

    let mut faces = Vec::with_capacity(prepared_bands.len() + 2 + contacts.len());
    for (band_index, band) in prepared_bands.iter().copied().enumerate() {
        let low_fin = if band_index == 0 {
            side_fin(far_low_edge, Sense::Forward, band.low_height)?
        } else {
            let contact = contacts[band_index - 1];
            bounded_side_fin(
                contact_edge_for_operand(contact, band.band.operand, contained, containing)?,
                Sense::Forward,
                band.low_height,
            )?
        };
        let high_fin = if band_index + 1 == prepared_bands.len() {
            side_fin(far_high_edge, Sense::Reversed, band.high_height)?
        } else {
            let contact = contacts[band_index];
            bounded_side_fin(
                contact_edge_for_operand(contact, band.band.operand, contained, containing)?,
                Sense::Reversed,
                band.high_height,
            )?
        };
        let cylinder = Cylinder::new(band.frame, band.radius)
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
        let face = AnalyticShellFace::new(
            AnalyticFaceKey::new(band_index as u64),
            AnalyticShellSurface::Cylinder(cylinder),
            Sense::Forward,
            FaceDomain::from_bounds(0.0, PERIOD, band.low_height, band.high_height)
                .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
            vec![
                AnalyticShellLoop::new(vec![low_fin]),
                AnalyticShellLoop::new(vec![high_fin]),
            ],
        );
        let face = if let Some(tail_index) = band.band.tail_index {
            lineaged_side_face(face, inner_source.side_face(), tails.len(), tail_index)?
        } else {
            face.with_source(EntityRef::Face(outer_source.side_face()))
        };
        faces.push(face);
    }

    let cap_key_base = prepared_bands.len() as u64;
    let low_plane = Plane::new(
        Frame::new(first_band.low_center, -axis, outer_frame.x())
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
    );
    let high_plane = Plane::new(
        Frame::new(last_band.high_center, axis, outer_frame.x())
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
    );
    faces.push(tangent_cap_face(
        AnalyticFaceKey::new(cap_key_base),
        far_low_edge,
        low_plane,
        far_low_circle,
        Sense::Reversed,
        first_band.radius,
        first_band.band.low.boundary.cap_face(),
        true,
    )?);
    faces.push(tangent_cap_face(
        AnalyticFaceKey::new(cap_key_base + 1),
        far_high_edge,
        high_plane,
        far_high_circle,
        Sense::Forward,
        last_band.radius,
        last_band.band.high.boundary.cap_face(),
        true,
    )?);
    for (contact_index, contact) in contacts.iter().copied().enumerate() {
        let shoulder = AnalyticShellFace::new(
            AnalyticFaceKey::new(cap_key_base + 2 + contact_index as u64),
            AnalyticShellSurface::Plane(contact.plane),
            Sense::Forward,
            FaceDomain::from_bounds(-outer_radius, outer_radius, -outer_radius, outer_radius)
                .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
            vec![AnalyticShellLoop::new(vec![
                plane_circle_fin(
                    contact.outer_edge,
                    contact.outer_fin_sense,
                    contact.plane,
                    contact.outer_circle,
                    false,
                )?,
                plane_circle_fin(
                    contact.inner_edge,
                    contact.inner_fin_sense,
                    contact.plane,
                    contact.inner_circle,
                    false,
                )?,
            ])],
        )
        .with_source(EntityRef::Face(contact.boundary.boundary.cap_face()));
        faces.push(shoulder);
    }

    Ok(AnalyticShellInput::new(vertices, edges, faces).with_closed_edges(closed_edges))
}

fn ordered_operand_boundaries(
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    operand: usize,
) -> Result<[BoundBoundary; 2], InternalTangencyPlanGap> {
    let identity = operand_identity(operand);
    let source = cylinders
        .get(operand)
        .ok_or(InternalTangencyPlanGap::RelationBinding)?;
    let boundary = |boundary_index| {
        Ok(BoundBoundary {
            operand: identity,
            boundary_index,
            boundary: source.boundaries()[boundary_index],
            axial_parameter: relation
                .axial_parameter(operand, boundary_index)
                .ok_or(InternalTangencyPlanGap::RelationBinding)?,
        })
    };
    let boundaries = [boundary(0)?, boundary(1)?];
    match compare_boundaries(relation, boundaries[0], boundaries[1]) {
        Ordering::Less => Ok(boundaries),
        Ordering::Greater => Ok([boundaries[1], boundaries[0]]),
        Ordering::Equal => Err(InternalTangencyPlanGap::IntervalContract),
    }
}

fn validate_tangent_union_band_chain(
    bands: &[TangentUnionBand],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    contained: usize,
    containing: usize,
) -> Result<(), InternalTangencyPlanGap> {
    if bands.len() < 2 || bands.len() > 3 {
        return Err(InternalTangencyPlanGap::IntervalContract);
    }
    let mut contained_count = 0;
    let mut containing_count = 0;
    for band in bands {
        if compare_boundaries(relation, band.low, band.high) != Ordering::Less {
            return Err(InternalTangencyPlanGap::IntervalContract);
        }
        if band.operand == contained {
            contained_count += 1;
        } else if band.operand == containing {
            containing_count += 1;
        } else {
            return Err(InternalTangencyPlanGap::IntervalContract);
        }
    }
    if containing_count != 1 || contained_count + containing_count != bands.len() {
        return Err(InternalTangencyPlanGap::IntervalContract);
    }
    for pair in bands.windows(2) {
        let [left, right] = pair else {
            return Err(InternalTangencyPlanGap::IntervalContract);
        };
        if left.operand == right.operand
            || !same_bound_boundary(left.high, right.low)
            || operand_index(left.high.operand) != containing
        {
            return Err(InternalTangencyPlanGap::IntervalContract);
        }
    }
    Ok(())
}

fn prepare_tangent_union_band(
    band: TangentUnionBand,
    contained: usize,
    outer_frame: Frame,
    inner_frame: Frame,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<PreparedTangentUnionBand, InternalTangencyPlanGap> {
    let (frame, radius) = if band.operand == contained {
        (inner_frame, inner_radius)
    } else {
        (outer_frame, outer_radius)
    };
    let (low_center, low_height) = exact_axial_projection(frame, band.low.boundary.center())
        .ok_or(InternalTangencyPlanGap::ArithmeticGuard)?;
    let (high_center, high_height) = exact_axial_projection(frame, band.high.boundary.center())
        .ok_or(InternalTangencyPlanGap::ArithmeticGuard)?;
    if !low_height.is_finite() || !high_height.is_finite() || low_height >= high_height {
        return Err(InternalTangencyPlanGap::ArithmeticGuard);
    }
    Ok(PreparedTangentUnionBand {
        band,
        frame,
        radius,
        low_center,
        low_height,
        high_center,
        high_height,
    })
}

fn contact_edge_for_operand(
    contact: PreparedTangentUnionContact,
    operand: usize,
    contained: usize,
    containing: usize,
) -> Result<AnalyticEdgeKey, InternalTangencyPlanGap> {
    if operand == contained {
        Ok(contact.inner_edge)
    } else if operand == containing {
        Ok(contact.outer_edge)
    } else {
        Err(InternalTangencyPlanGap::IntervalContract)
    }
}

fn compare_boundaries(
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    first: BoundBoundary,
    second: BoundBoundary,
) -> Ordering {
    relation
        .preorder()
        .compare(boundary_contributor(first), boundary_contributor(second))
}

fn boundary_contributor(boundary: BoundBoundary) -> AxialEndpointContributor {
    AxialEndpointContributor::new(
        boundary.operand,
        if boundary.boundary_index == 0 {
            AuthoredAxialEndpoint::Start
        } else {
            AuthoredAxialEndpoint::End
        },
    )
}

fn same_bound_boundary(first: BoundBoundary, second: BoundBoundary) -> bool {
    first.operand == second.operand
        && first.boundary_index == second.boundary_index
        && first.boundary.cap_face() == second.boundary.cap_face()
        && first.boundary.edge() == second.boundary.edge()
}

fn single_bound_boundary(
    boundaries: Vec<BoundBoundary>,
) -> Result<BoundBoundary, InternalTangencyPlanGap> {
    let [boundary] = boundaries.as_slice() else {
        return Err(InternalTangencyPlanGap::IntervalContract);
    };
    Ok(*boundary)
}

fn span_is_whole_operand(span: &PlannedAxialSpan, operand: usize) -> bool {
    let operand = operand_identity(operand);
    let start = AxialEndpointContributor::new(operand, AuthoredAxialEndpoint::Start);
    let end = AxialEndpointContributor::new(operand, AuthoredAxialEndpoint::End);
    (span.low().contains(start) && span.high().contains(end))
        || (span.low().contains(end) && span.high().contains(start))
}

fn realize_internal_tangent_bands(
    edit: &mut PartEdit<'_>,
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    plan: &AxialIntervalPlan,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let inputs = match prepare_interval_shells(cylinders, relation, plan) {
        Ok(inputs) => inputs,
        Err(
            InternalTangencyPlanGap::RelationBinding
            | InternalTangencyPlanGap::IntervalContract
            | InternalTangencyPlanGap::ArithmeticGuard
            | InternalTangencyPlanGap::Lineage,
        ) => {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        }
    };
    realize_analytic_shell_inputs(edit, &inputs, linear, scope)
}

fn prepare_interval_shells(
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    plan: &AxialIntervalPlan,
) -> Result<Vec<AnalyticShellInput>, InternalTangencyPlanGap> {
    bind_relation_boundaries(cylinders, relation.boundaries())?;
    if plan.spans().len() > 2 {
        return Err(InternalTangencyPlanGap::IntervalContract);
    }
    plan.spans()
        .iter()
        .enumerate()
        .map(|(index, span)| {
            prepare_interval_shell(
                cylinders,
                relation,
                relation.contained_operand(),
                span,
                plan.spans().len(),
                index,
            )
        })
        .collect()
}

fn bind_relation_boundaries(
    cylinders: [&CertifiedCylinderSource; 2],
    witnesses: &[ParallelCylinderAxialBoundaryWitness; 4],
) -> Result<(), InternalTangencyPlanGap> {
    let mut seen = [[false; 2]; 2];
    for witness in witnesses {
        let source = cylinders
            .get(witness.operand())
            .ok_or(InternalTangencyPlanGap::RelationBinding)?;
        let boundary = source
            .boundaries()
            .get(witness.boundary())
            .ok_or(InternalTangencyPlanGap::RelationBinding)?;
        if seen[witness.operand()][witness.boundary()]
            || boundary.cap_face() != witness.cap_face()
            || boundary.edge() != witness.edge()
        {
            return Err(InternalTangencyPlanGap::RelationBinding);
        }
        seen[witness.operand()][witness.boundary()] = true;
    }
    if seen != [[true; 2]; 2] {
        return Err(InternalTangencyPlanGap::RelationBinding);
    }
    Ok(())
}

fn prepare_interval_shell(
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    contained: usize,
    span: &PlannedAxialSpan,
    span_count: usize,
    span_index: usize,
) -> Result<AnalyticShellInput, InternalTangencyPlanGap> {
    let low = bind_boundary_class(cylinders, relation, span.low())?;
    let high = bind_boundary_class(cylinders, relation, span.high())?;
    let contained_source = cylinders
        .get(contained)
        .ok_or(InternalTangencyPlanGap::RelationBinding)?;
    let (low_center, mut low_parameter) =
        contained_axis_endpoint(contained_source, contained, &low)?;
    let (high_center, mut high_parameter) =
        contained_axis_endpoint(contained_source, contained, &high)?;

    let source_cylinder = contained_source.cylinder();
    let mut frame = *source_cylinder.frame();
    if low_parameter > high_parameter {
        frame = Frame::new(frame.origin(), -frame.z(), frame.x())
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
        low_parameter = -low_parameter;
        high_parameter = -high_parameter;
    }
    if !low_parameter.is_finite() || !high_parameter.is_finite() || low_parameter >= high_parameter
    {
        return Err(InternalTangencyPlanGap::ArithmeticGuard);
    }
    if !axis_parameter_identity_is_exact(low_center, frame.origin(), frame.z(), low_parameter)
        || !axis_parameter_identity_is_exact(high_center, frame.origin(), frame.z(), high_parameter)
    {
        return Err(InternalTangencyPlanGap::ArithmeticGuard);
    }

    let cylinder = Cylinder::new(frame, source_cylinder.radius())
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let circles = [low_center, high_center].map(|center| {
        Circle::new(frame.with_origin(center), source_cylinder.radius())
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)
    });
    let [low_circle, high_circle] = circles;
    let circles = [low_circle?, high_circle?];
    let closed_edges = [
        lineaged_closed_edge(
            AnalyticEdgeKey::new(0),
            circles[0],
            contained,
            contained_source.side_face(),
            &low,
        )?,
        lineaged_closed_edge(
            AnalyticEdgeKey::new(1),
            circles[1],
            contained,
            contained_source.side_face(),
            &high,
        )?,
    ];

    let side_domain = FaceDomain::from_bounds(0.0, PERIOD, low_parameter, high_parameter)
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let side = AnalyticShellFace::new(
        AnalyticFaceKey::new(0),
        AnalyticShellSurface::Cylinder(cylinder),
        Sense::Forward,
        side_domain,
        vec![
            AnalyticShellLoop::new(vec![side_fin(
                AnalyticEdgeKey::new(0),
                Sense::Forward,
                low_parameter,
            )?]),
            AnalyticShellLoop::new(vec![side_fin(
                AnalyticEdgeKey::new(1),
                Sense::Reversed,
                high_parameter,
            )?]),
        ],
    );
    let side = lineaged_side_face(side, contained_source.side_face(), span_count, span_index)?;
    let caps = [
        lineaged_cap_face(
            AnalyticFaceKey::new(1),
            AnalyticEdgeKey::new(0),
            circles[0],
            frame,
            true,
            contained,
            &low,
        )?,
        lineaged_cap_face(
            AnalyticFaceKey::new(2),
            AnalyticEdgeKey::new(1),
            circles[1],
            frame,
            false,
            contained,
            &high,
        )?,
    ];
    Ok(AnalyticShellInput::new(
        Vec::new(),
        Vec::new(),
        vec![side, caps[0].clone(), caps[1].clone()],
    )
    .with_closed_edges(closed_edges.to_vec()))
}

fn bind_boundary_class(
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    contributors: AxialEndpointContributors,
) -> Result<Vec<BoundBoundary>, InternalTangencyPlanGap> {
    let mut boundaries = Vec::with_capacity(2);
    for contributor in contributors.iter() {
        let operand = contributor.operand();
        let operand_index = operand_index(operand);
        let boundary_index = match contributor.endpoint() {
            AuthoredAxialEndpoint::Start => 0,
            AuthoredAxialEndpoint::End => 1,
        };
        if boundaries
            .iter()
            .any(|bound: &BoundBoundary| bound.operand == operand)
        {
            return Err(InternalTangencyPlanGap::RelationBinding);
        }
        boundaries.push(BoundBoundary {
            operand,
            boundary_index,
            boundary: *cylinders[operand_index]
                .boundaries()
                .get(boundary_index)
                .ok_or(InternalTangencyPlanGap::RelationBinding)?,
            axial_parameter: relation
                .axial_parameter(operand_index, boundary_index)
                .ok_or(InternalTangencyPlanGap::RelationBinding)?,
        });
    }
    if boundaries.is_empty() || boundaries.len() > 2 {
        return Err(InternalTangencyPlanGap::RelationBinding);
    }
    Ok(boundaries)
}

fn contained_axis_endpoint(
    source: &CertifiedCylinderSource,
    contained: usize,
    boundaries: &[BoundBoundary],
) -> Result<(Point3, f64), InternalTangencyPlanGap> {
    if let Some(boundary) = boundaries
        .iter()
        .find(|boundary| operand_index(boundary.operand) == contained)
    {
        return Ok((boundary.boundary.center(), boundary.axial_parameter));
    }
    let boundary = boundaries
        .first()
        .ok_or(InternalTangencyPlanGap::RelationBinding)?;
    let frame = *source.cylinder().frame();
    exact_axial_projection(frame, boundary.boundary.center())
        .ok_or(InternalTangencyPlanGap::ArithmeticGuard)
}

fn lineaged_closed_edge(
    key: AnalyticEdgeKey,
    circle: Circle,
    contained: usize,
    contained_side: ktopo::entity::FaceId,
    boundaries: &[BoundBoundary],
) -> Result<AnalyticShellClosedEdge, InternalTangencyPlanGap> {
    let edge = AnalyticShellClosedEdge::new(
        key,
        AnalyticShellCurve::Circle(circle),
        circle.param_range(),
    );
    let source_boundary = boundaries
        .iter()
        .find(|boundary| operand_index(boundary.operand) == contained);
    if let Some(source) = source_boundary {
        return Ok(edge.with_source(EntityRef::Edge(source.boundary.edge())));
    }
    let cutting_boundary = boundaries.first().ok_or(InternalTangencyPlanGap::Lineage)?;
    Ok(edge.with_derived_sources([
        EntityRef::Face(contained_side),
        EntityRef::Face(cutting_boundary.boundary.cap_face()),
    ]))
}

fn lineaged_side_face(
    face: AnalyticShellFace,
    source: ktopo::entity::FaceId,
    span_count: usize,
    span_index: usize,
) -> Result<AnalyticShellFace, InternalTangencyPlanGap> {
    let source = EntityRef::Face(source);
    match span_count {
        1 => Ok(face.with_source(source)),
        2 => {
            let piece = match span_index {
                0 => AnalyticFaceSplitPiece::First,
                1 => AnalyticFaceSplitPiece::Second,
                _ => return Err(InternalTangencyPlanGap::Lineage),
            };
            Ok(face.with_split_lineage(source, piece))
        }
        _ => Err(InternalTangencyPlanGap::Lineage),
    }
}

fn lineaged_cap_face(
    key: AnalyticFaceKey,
    edge: AnalyticEdgeKey,
    circle: Circle,
    cylinder_frame: Frame,
    low: bool,
    contained: usize,
    boundaries: &[BoundBoundary],
) -> Result<AnalyticShellFace, InternalTangencyPlanGap> {
    let center = circle.frame().origin();
    let plane_frame = if low {
        Frame::new(center, -cylinder_frame.z(), cylinder_frame.x())
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?
    } else {
        cylinder_frame.with_origin(center)
    };
    let plane = Plane::new(plane_frame);
    let pcurve = Circle2d::new(Point2::new(0.0, 0.0), circle.radius(), Vec2::new(1.0, 0.0))
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let map = if low {
        AffineParamMap1d::new(-1.0, PERIOD)
    } else {
        AffineParamMap1d::new(1.0, 0.0)
    }
    .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let fin = AnalyticShellFin::new(
        edge,
        if low { Sense::Reversed } else { Sense::Forward },
        AnalyticPcurveUse::new(AnalyticShellPcurve::Circle(pcurve), map)
            .with_closure_winding([0, 0]),
    );
    let radius = circle.radius();
    let face = AnalyticShellFace::new(
        key,
        AnalyticShellSurface::Plane(plane),
        Sense::Forward,
        FaceDomain::from_bounds(-radius, radius, -radius, radius)
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
        vec![AnalyticShellLoop::new(vec![fin])],
    );
    let source = boundaries
        .iter()
        .find(|boundary| operand_index(boundary.operand) == contained)
        .or_else(|| boundaries.first())
        .ok_or(InternalTangencyPlanGap::Lineage)?;
    Ok(face.with_source(EntityRef::Face(source.boundary.cap_face())))
}

fn side_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    height: f64,
) -> Result<AnalyticShellFin, InternalTangencyPlanGap> {
    cylinder_circle_fin(edge, sense, height, true)
}

fn bounded_side_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    height: f64,
) -> Result<AnalyticShellFin, InternalTangencyPlanGap> {
    cylinder_circle_fin(edge, sense, height, false)
}

fn cylinder_circle_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    height: f64,
    endpoint_free: bool,
) -> Result<AnalyticShellFin, InternalTangencyPlanGap> {
    let line = Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0))
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let map =
        AffineParamMap1d::new(1.0, 0.0).map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let use_ = AnalyticPcurveUse::new(AnalyticShellPcurve::Line(line), map);
    Ok(AnalyticShellFin::new(
        edge,
        sense,
        if endpoint_free {
            use_.with_closure_winding([1, 0])
        } else {
            use_
        },
    ))
}

fn tangent_cap_face(
    key: AnalyticFaceKey,
    edge: AnalyticEdgeKey,
    plane: Plane,
    circle: Circle,
    fin_sense: Sense,
    radius: f64,
    source: ktopo::entity::FaceId,
    endpoint_free: bool,
) -> Result<AnalyticShellFace, InternalTangencyPlanGap> {
    Ok(AnalyticShellFace::new(
        key,
        AnalyticShellSurface::Plane(plane),
        Sense::Forward,
        FaceDomain::from_bounds(-radius, radius, -radius, radius)
            .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?,
        vec![AnalyticShellLoop::new(vec![plane_circle_fin(
            edge,
            fin_sense,
            plane,
            circle,
            endpoint_free,
        )?])],
    )
    .with_source(EntityRef::Face(source)))
}

fn plane_circle_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    plane: Plane,
    circle: Circle,
    endpoint_free: bool,
) -> Result<AnalyticShellFin, InternalTangencyPlanGap> {
    let center = plane.frame().to_local(circle.frame().origin());
    let local_x = Vec2::new(
        circle.frame().x().dot(plane.frame().x()),
        circle.frame().x().dot(plane.frame().y()),
    );
    let local_y = Vec2::new(
        circle.frame().y().dot(plane.frame().x()),
        circle.frame().y().dot(plane.frame().y()),
    );
    let scale = if local_x.perp().dot(local_y) > 0.0 {
        1.0
    } else {
        -1.0
    };
    let pcurve = Circle2d::new(Point2::new(center.x, center.y), circle.radius(), local_x)
        .map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let map =
        AffineParamMap1d::new(scale, 0.0).map_err(|_| InternalTangencyPlanGap::ArithmeticGuard)?;
    let use_ = AnalyticPcurveUse::new(AnalyticShellPcurve::Circle(pcurve), map);
    Ok(AnalyticShellFin::new(
        edge,
        sense,
        if endpoint_free {
            use_.with_closure_winding([0, 0])
        } else {
            use_
        },
    ))
}

const fn operand_identity(index: usize) -> AxialIntervalOperand {
    if index == 0 {
        AxialIntervalOperand::Left
    } else {
        AxialIntervalOperand::Right
    }
}

const fn operand_index(operand: AxialIntervalOperand) -> usize {
    match operand {
        AxialIntervalOperand::Left => 0,
        AxialIntervalOperand::Right => 1,
    }
}

/// Reconstruct a point on `frame`'s axis in the exact plane normal to that
/// axis through `point`.
///
/// The ordinary dot projection is tried first. Exact Pythagorean axes can
/// round that projection even when a component quotient recovers the authored
/// parameter exactly, so each nonzero component supplies one additional
/// candidate. No candidate becomes authority unless exact predicates prove
/// both its componentwise axis evaluation and its coplanarity with `point`.
fn exact_axial_projection(frame: Frame, point: Point3) -> Option<(Point3, f64)> {
    let origin = frame.origin();
    let axis = frame.z();
    let delta = point - origin;
    let axis_components = axis.to_array();
    let delta_components = delta.to_array();
    let candidates = [
        delta.dot(axis),
        delta_components[0] / axis_components[0],
        delta_components[1] / axis_components[1],
        delta_components[2] / axis_components[2],
    ];
    candidates.into_iter().find_map(|parameter| {
        if !parameter.is_finite() {
            return None;
        }
        let center = origin + axis * parameter;
        (axis_parameter_identity_is_exact(center, origin, axis, parameter)
            && affine_dot3(axis.to_array(), center.to_array(), point.to_array(), 0.0)
                .is_some_and(|orientation| orientation.sign() == Orientation::Zero))
        .then_some((center, parameter))
    })
}

/// Prove the authored axis evaluation component-by-component. This prevents a
/// rounded projection from becoming reconstruction authority for a cut ring.
fn axis_parameter_identity_is_exact(
    point: Point3,
    origin: Point3,
    axis: kgeom::vec::Vec3,
    parameter: f64,
) -> bool {
    let point = point.to_array();
    let origin = origin.to_array();
    let axis = axis.to_array();
    (0..3).all(|component| {
        affine_dot3(
            [1.0, axis[component], -1.0],
            [origin[component], parameter, point[component]],
            [0.0; 3],
            0.0,
        )
        .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}
