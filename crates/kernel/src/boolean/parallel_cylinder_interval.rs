//! Exact common-support finite-cylinder interval realization.
//!
//! The relation layer has already proved one exact common infinite cylinder
//! support and a strictly positive overlap of two topology-owned finite axial
//! intervals. The pure four-endpoint sweep selects zero, one, or two maximal
//! material spans. This module rebuilds each selected span as a canonical
//! 3F/2E/0V analytic shell and attaches complete cap, ring, and side lineage
//! before committing the complete component batch through one Full check.

use kcore::operation::OperationScope;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2};
use kgraph::AffineParamMap1d;
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticFaceSplitPiece, AnalyticPcurveUse,
    AnalyticShellClosedEdge, AnalyticShellCurve, AnalyticShellFace, AnalyticShellFin,
    AnalyticShellInput, AnalyticShellLoop, AnalyticShellPcurve, AnalyticShellSurface,
};
use ktopo::entity::{EntityRef, FaceDomain, Sense};

use super::axial_interval_sweep::{
    AuthoredAxialEndpoint, AxialEndpointContributors, AxialIntervalOperand, AxialIntervalPlan,
    PlannedAxialSpan, plan_axial_interval_sweep,
};
use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, StageResult, adapt_operation,
    refused,
};
use super::curved_realize::realize_analytic_shell_inputs;
use super::curved_source::{CertifiedCylinderBoundary, CertifiedCylinderSource};
use super::parallel_cylinder_relation::{
    CertifiedParallelCylinderCommonSupport, ParallelCylinderAxialBoundaryWitness,
};
use super::select::PlanarBooleanOperation;
use crate::session::PartEdit;

const PERIOD: f64 = core::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CommonSupportPlanGap {
    RelationBinding,
    SourceTopology,
    ArithmeticGuard,
    Lineage,
}

#[derive(Debug, Clone, Copy)]
struct BoundBoundary {
    operand: AxialIntervalOperand,
    boundary: CertifiedCylinderBoundary,
}

/// Realize regularized CSG over two certified finite intervals on one exact
/// common cylinder support.
pub(super) fn execute_parallel_cylinder_common_support(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderCommonSupport,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let plan = plan_axial_interval_sweep(adapt_operation(operation), relation.preorder());
    let inputs = match prepare_interval_shells(cylinders, relation, &plan) {
        Ok(inputs) => inputs,
        Err(
            CommonSupportPlanGap::RelationBinding
            | CommonSupportPlanGap::SourceTopology
            | CommonSupportPlanGap::ArithmeticGuard
            | CommonSupportPlanGap::Lineage,
        ) => {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        }
    };
    realize_analytic_shell_inputs(edit, &inputs, linear, scope)
}

fn prepare_interval_shells(
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderCommonSupport,
    plan: &AxialIntervalPlan,
) -> Result<Vec<AnalyticShellInput>, CommonSupportPlanGap> {
    bind_relation_boundaries(cylinders, relation.boundaries())?;
    let split_operand = split_side_operand(plan)?;
    plan.spans()
        .iter()
        .enumerate()
        .map(|(index, span)| prepare_interval_shell(cylinders, span, split_operand, index))
        .collect()
}

fn bind_relation_boundaries(
    cylinders: [&CertifiedCylinderSource; 2],
    witnesses: &[ParallelCylinderAxialBoundaryWitness; 4],
) -> Result<(), CommonSupportPlanGap> {
    let mut seen = [[false; 2]; 2];
    for witness in witnesses {
        let source = cylinders
            .get(witness.operand())
            .ok_or(CommonSupportPlanGap::RelationBinding)?;
        let boundary = source
            .boundaries()
            .get(witness.boundary())
            .ok_or(CommonSupportPlanGap::RelationBinding)?;
        if seen[witness.operand()][witness.boundary()]
            || boundary.cap_face() != witness.cap_face()
            || boundary.edge() != witness.edge()
        {
            return Err(CommonSupportPlanGap::RelationBinding);
        }
        seen[witness.operand()][witness.boundary()] = true;
    }
    if seen != [[true; 2]; 2] {
        return Err(CommonSupportPlanGap::RelationBinding);
    }
    Ok(())
}

fn split_side_operand(
    plan: &AxialIntervalPlan,
) -> Result<Option<AxialIntervalOperand>, CommonSupportPlanGap> {
    let [first, second] = plan.spans() else {
        return Ok(None);
    };
    for operand in [AxialIntervalOperand::Left, AxialIntervalOperand::Right] {
        if first.side_operands().iter().eq([operand]) && second.side_operands().iter().eq([operand])
        {
            return Ok(Some(operand));
        }
    }
    Err(CommonSupportPlanGap::Lineage)
}

fn prepare_interval_shell(
    cylinders: [&CertifiedCylinderSource; 2],
    span: &PlannedAxialSpan,
    split_operand: Option<AxialIntervalOperand>,
    split_index: usize,
) -> Result<AnalyticShellInput, CommonSupportPlanGap> {
    let low = bind_boundary_class(cylinders, span.low())?;
    let high = bind_boundary_class(cylinders, span.high())?;
    let low_center = common_boundary_center(&low)?;
    let high_center = common_boundary_center(&high)?;

    let source_cylinder = cylinders[0].cylinder();
    let mut frame = *source_cylinder.frame();
    let mut low_parameter = axial_parameter(frame, low_center);
    let mut high_parameter = axial_parameter(frame, high_center);
    if low_parameter > high_parameter {
        frame = Frame::new(frame.origin(), -frame.z(), frame.x())
            .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
        low_parameter = axial_parameter(frame, low_center);
        high_parameter = axial_parameter(frame, high_center);
    }
    if !low_parameter.is_finite() || !high_parameter.is_finite() || low_parameter >= high_parameter
    {
        return Err(CommonSupportPlanGap::ArithmeticGuard);
    }
    let cylinder = Cylinder::new(frame, source_cylinder.radius())
        .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
    let circles = [low_center, high_center].map(|center| {
        Circle::new(frame.with_origin(center), source_cylinder.radius())
            .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)
    });
    let [low_circle, high_circle] = circles;
    let circles = [low_circle?, high_circle?];
    let closed_edges = [
        lineaged_closed_edge(AnalyticEdgeKey::new(0), circles[0], &low)?,
        lineaged_closed_edge(AnalyticEdgeKey::new(1), circles[1], &high)?,
    ];

    let side_domain = FaceDomain::from_bounds(0.0, PERIOD, low_parameter, high_parameter)
        .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
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
    let side = lineaged_side_face(side, cylinders, span, split_operand, split_index)?;
    let caps = [
        lineaged_cap_face(
            AnalyticFaceKey::new(1),
            AnalyticEdgeKey::new(0),
            circles[0],
            frame,
            true,
            &low,
        )?,
        lineaged_cap_face(
            AnalyticFaceKey::new(2),
            AnalyticEdgeKey::new(1),
            circles[1],
            frame,
            false,
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
    contributors: AxialEndpointContributors,
) -> Result<Vec<BoundBoundary>, CommonSupportPlanGap> {
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
            return Err(CommonSupportPlanGap::RelationBinding);
        }
        boundaries.push(BoundBoundary {
            operand,
            boundary: *cylinders[operand_index]
                .boundaries()
                .get(boundary_index)
                .ok_or(CommonSupportPlanGap::RelationBinding)?,
        });
    }
    if boundaries.is_empty() || boundaries.len() > 2 {
        return Err(CommonSupportPlanGap::RelationBinding);
    }
    Ok(boundaries)
}

fn common_boundary_center(boundaries: &[BoundBoundary]) -> Result<Point3, CommonSupportPlanGap> {
    let center = boundaries
        .first()
        .ok_or(CommonSupportPlanGap::RelationBinding)?
        .boundary
        .center();
    if boundaries
        .iter()
        .any(|boundary| boundary.boundary.center() != center)
    {
        return Err(CommonSupportPlanGap::RelationBinding);
    }
    Ok(center)
}

fn lineaged_closed_edge(
    key: AnalyticEdgeKey,
    circle: Circle,
    boundaries: &[BoundBoundary],
) -> Result<AnalyticShellClosedEdge, CommonSupportPlanGap> {
    let edge = AnalyticShellClosedEdge::new(
        key,
        AnalyticShellCurve::Circle(circle),
        circle.param_range(),
    );
    match boundaries {
        [boundary] => Ok(edge.with_source(EntityRef::Edge(boundary.boundary.edge()))),
        [first, second] => Ok(edge.with_merge_sources([
            EntityRef::Edge(first.boundary.edge()),
            EntityRef::Edge(second.boundary.edge()),
        ])),
        _ => Err(CommonSupportPlanGap::Lineage),
    }
}

fn lineaged_side_face(
    face: AnalyticShellFace,
    cylinders: [&CertifiedCylinderSource; 2],
    span: &PlannedAxialSpan,
    split_operand: Option<AxialIntervalOperand>,
    split_index: usize,
) -> Result<AnalyticShellFace, CommonSupportPlanGap> {
    let operands = span.side_operands();
    match (
        operands.contains(AxialIntervalOperand::Left),
        operands.contains(AxialIntervalOperand::Right),
    ) {
        (true, true) => Ok(face.with_merge_sources([
            EntityRef::Face(cylinders[0].side_face()),
            EntityRef::Face(cylinders[1].side_face()),
        ])),
        (true, false) | (false, true) => {
            let operand = if operands.contains(AxialIntervalOperand::Left) {
                AxialIntervalOperand::Left
            } else {
                AxialIntervalOperand::Right
            };
            let source = EntityRef::Face(cylinders[operand_index(operand)].side_face());
            if split_operand == Some(operand) {
                let piece = match split_index {
                    0 => AnalyticFaceSplitPiece::First,
                    1 => AnalyticFaceSplitPiece::Second,
                    _ => return Err(CommonSupportPlanGap::Lineage),
                };
                Ok(face.with_split_lineage(source, piece))
            } else {
                Ok(face.with_source(source))
            }
        }
        (false, false) => Err(CommonSupportPlanGap::Lineage),
    }
}

fn lineaged_cap_face(
    key: AnalyticFaceKey,
    edge: AnalyticEdgeKey,
    circle: Circle,
    cylinder_frame: Frame,
    low: bool,
    boundaries: &[BoundBoundary],
) -> Result<AnalyticShellFace, CommonSupportPlanGap> {
    let center = circle.frame().origin();
    let plane_frame = if low {
        Frame::new(center, -cylinder_frame.z(), cylinder_frame.x())
            .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?
    } else {
        cylinder_frame.with_origin(center)
    };
    let plane = Plane::new(plane_frame);
    let pcurve = Circle2d::new(Point2::new(0.0, 0.0), circle.radius(), Vec2::new(1.0, 0.0))
        .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
    let map = if low {
        AffineParamMap1d::new(-1.0, PERIOD)
    } else {
        AffineParamMap1d::new(1.0, 0.0)
    }
    .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
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
            .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?,
        vec![AnalyticShellLoop::new(vec![fin])],
    );
    match boundaries {
        [boundary] => Ok(face.with_source(EntityRef::Face(boundary.boundary.cap_face()))),
        [first, second] => Ok(face.with_merge_sources([
            EntityRef::Face(first.boundary.cap_face()),
            EntityRef::Face(second.boundary.cap_face()),
        ])),
        _ => Err(CommonSupportPlanGap::Lineage),
    }
}

fn side_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    height: f64,
) -> Result<AnalyticShellFin, CommonSupportPlanGap> {
    let line = Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0))
        .map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
    let map = AffineParamMap1d::new(1.0, 0.0).map_err(|_| CommonSupportPlanGap::ArithmeticGuard)?;
    Ok(AnalyticShellFin::new(
        edge,
        sense,
        AnalyticPcurveUse::new(AnalyticShellPcurve::Line(line), map).with_closure_winding([1, 0]),
    ))
}

const fn operand_index(operand: AxialIntervalOperand) -> usize {
    match operand {
        AxialIntervalOperand::Left => 0,
        AxialIntervalOperand::Right => 1,
    }
}

fn axial_parameter(frame: Frame, point: Point3) -> f64 {
    (point - frame.origin()).dot(frame.z())
}
