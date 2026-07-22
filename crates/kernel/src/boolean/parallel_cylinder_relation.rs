//! Operation-local theorem for the first parallel-cylinder Boolean slice.
//!
//! The section graph remains the general intersection authority.  This module
//! only recognizes the strict finite lens-prism relation needed by the first
//! Cylinder/Cylinder `Intersect`: same-directed exactly parallel axes, strict
//! transverse radial secancy, one strictly nested axial interval, and one
//! closed section component alternating between two rulings and the two inner
//! cap arcs.  Every join is owned by the section graph's source-edge/root
//! identity; rounded points are never used as topology keys.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3, orient3d};
use kgeom::vec::{Point3, Vec3};
use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId};

use super::curved_source::CertifiedCylinderSource;
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use crate::error::{Error, Result};
use crate::{
    BodySectionGraph, SectionBranch, SectionBranchTopology, SectionCarrier,
    SectionCurveEndpointTopology, SectionCurveFragmentSpan, SectionPeriodicFaceEmbeddingEvidence,
    SectionSite, SectionSourceParameterKey, SectionUvCurve,
};

/// Fixed proof work charged before the first semantic exit.
///
/// Accepted inputs have exactly four branches, fragments, and endpoints and
/// one component.  Oversized inputs are rejected from their lengths before
/// any collection scan, so this constant is a geometry-independent ceiling.
pub(super) const PARALLEL_CYLINDER_RELATION_WORK: u64 = 64;

/// First unmet obligation in the strict parallel-cylinder lens relation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub(super) enum ParallelCylinderRelationGap {
    /// A source value contains non-finite or otherwise unusable arithmetic.
    ArithmeticGuard,
    /// The two authored cylinder axes are not exactly parallel.
    AxesNotExactlyParallel,
    /// Exactly parallel axes have opposed authored directions.
    AxesOppositelySigned,
    /// The radial circles are not certified as a strict two-root secant.
    RadialSecancyNotStrict,
    /// A source's topology-ordered cap interval is not positive on the common axis.
    SourceAxialOrder,
    /// Both source cap intervals describe the same two axial planes.
    AxialIntervalsEqual,
    /// Neither source cap interval is strictly contained by the other.
    AxialIntervalsNotStrictlyNested,
    /// The supplied section is not globally complete and gap-free.
    SectionIncomplete,
    /// Fixed collection counts, component coverage, or alternation did not match.
    SectionLayout,
    /// Periodic or branch face evidence does not bind to the supplied sources.
    SectionOperandBinding,
    /// A branch carrier, pcurve family, range, or residual proof did not match.
    SectionBranchEvidence,
    /// A trim endpoint did not retain the required inner cap-ring provenance.
    SectionEndpointProvenance,
}

/// One inner cap and the unique section arc cut into it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ParallelCylinderCapBoundaryWitness {
    boundary: usize,
    cap_face: RawFaceId,
    edge: RawEdgeId,
    branch: usize,
    fragment: usize,
    root_ordinals: [usize; 2],
}

impl ParallelCylinderCapBoundaryWitness {
    /// Inner source boundary ordinal in authored-axis order.
    pub(super) const fn boundary(&self) -> usize {
        self.boundary
    }

    /// Topology-owned cap face.
    pub(super) const fn cap_face(&self) -> RawFaceId {
        self.cap_face
    }

    /// Vertexless cap-ring edge owning both arc endpoints.
    pub(super) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    /// Section branch carrying the cap circle.
    pub(super) const fn branch(&self) -> usize {
        self.branch
    }

    /// Section fragment carrying the retained cap arc.
    pub(super) const fn fragment(&self) -> usize {
        self.fragment
    }

    /// The two exact roots in intrinsic cap-ring parameter order.
    pub(super) const fn root_ordinals(&self) -> [usize; 2] {
        self.root_ordinals
    }
}

/// One of the two bounded common-axis rulings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ParallelCylinderRulingWitness {
    branch: usize,
    fragment: usize,
    /// Endpoint indices in low/high inner-cap boundary order.
    endpoints: [usize; 2],
    /// Source-root ordinals in low/high inner-cap boundary order.
    root_ordinals: [usize; 2],
}

impl ParallelCylinderRulingWitness {
    /// Section branch carrying this ruling.
    pub(super) const fn branch(&self) -> usize {
        self.branch
    }

    /// Section fragment carrying this bounded ruling.
    pub(super) const fn fragment(&self) -> usize {
        self.fragment
    }

    /// Endpoint indices in low/high inner-cap boundary order.
    pub(super) const fn endpoints(&self) -> [usize; 2] {
        self.endpoints
    }

    /// Source-root ordinals in low/high inner-cap boundary order.
    pub(super) const fn root_ordinals(&self) -> [usize; 2] {
        self.root_ordinals
    }
}

/// Complete operation-local proof of the strict nested lens-prism relation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CertifiedParallelCylinderLensRelation {
    inner_operand: usize,
    outer_operand: usize,
    component: usize,
    cap_boundaries: [ParallelCylinderCapBoundaryWitness; 2],
    rulings: [ParallelCylinderRulingWitness; 2],
}

impl CertifiedParallelCylinderLensRelation {
    /// Operand whose complete cap interval is strictly inside the other.
    pub(super) const fn inner_operand(&self) -> usize {
        self.inner_operand
    }

    /// Operand whose cap interval strictly contains the other.
    pub(super) const fn outer_operand(&self) -> usize {
        self.outer_operand
    }

    /// Unique closed section-component index.
    pub(super) const fn component(&self) -> usize {
        self.component
    }

    /// Low/high inner-cap arc witnesses.
    pub(super) const fn cap_boundaries(&self) -> &[ParallelCylinderCapBoundaryWitness; 2] {
        &self.cap_boundaries
    }

    /// Two rulings sorted by their low/high source-root ordinal pair.
    pub(super) const fn rulings(&self) -> &[ParallelCylinderRulingWitness; 2] {
        &self.rulings
    }
}

/// Certified relation or a typed fail-closed missing obligation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ParallelCylinderRelationOutcome {
    /// Every analytic, topology, and provenance obligation was discharged.
    Certified(Box<CertifiedParallelCylinderLensRelation>),
    /// The first stable relation obligation that could not be discharged.
    Indeterminate(ParallelCylinderRelationGap),
}

/// Certify the exact strict-nesting relation consumed by the first
/// parallel-cylinder Boolean realization slice.
pub(super) fn certify_parallel_cylinder_relation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<ParallelCylinderRelationOutcome> {
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, PARALLEL_CYLINDER_RELATION_WORK)
        .map_err(Error::from)?;

    let (inner_operand, outer_operand) = match certify_source_relation(cylinders) {
        Ok(relation) => relation,
        Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    };
    match certify_section_relation(graph, cylinders, inner_operand, outer_operand) {
        Ok(certificate) => Ok(ParallelCylinderRelationOutcome::Certified(Box::new(
            certificate,
        ))),
        Err(gap) => Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    }
}

fn certify_source_relation(
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<(usize, usize), ParallelCylinderRelationGap> {
    let cylinder_a = cylinders[0].cylinder();
    let cylinder_b = cylinders[1].cylinder();
    let axis_a = cylinder_a.frame().z();
    let axis_b = cylinder_b.frame().z();
    if !finite_vec3(axis_a)
        || !finite_vec3(axis_b)
        || !finite_vec3(cylinder_a.frame().origin())
        || !finite_vec3(cylinder_b.frame().origin())
        || !cylinder_a.radius().is_finite()
        || !cylinder_b.radius().is_finite()
        || cylinders
            .iter()
            .flat_map(|source| source.boundaries())
            .any(|boundary| !finite_vec3(boundary.center()))
    {
        return Err(ParallelCylinderRelationGap::ArithmeticGuard);
    }
    if !vectors_are_exactly_parallel(axis_a, axis_b) {
        return Err(ParallelCylinderRelationGap::AxesNotExactlyParallel);
    }
    let axis_sign = affine_dot3(axis_a.to_array(), axis_b.to_array(), [0.0; 3], 0.0)
        .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?
        .sign();
    if axis_sign != Orientation::Positive {
        return Err(if axis_sign == Orientation::Negative {
            ParallelCylinderRelationGap::AxesOppositelySigned
        } else {
            ParallelCylinderRelationGap::ArithmeticGuard
        });
    }
    certify_strict_radial_secancy(cylinders)?;

    for source in cylinders {
        if axial_compare(
            axis_a,
            source.boundaries()[1].center(),
            source.boundaries()[0].center(),
        )? != Orientation::Positive
        {
            return Err(ParallelCylinderRelationGap::SourceAxialOrder);
        }
    }
    let low = axial_compare(
        axis_a,
        cylinders[1].boundaries()[0].center(),
        cylinders[0].boundaries()[0].center(),
    )?;
    let high = axial_compare(
        axis_a,
        cylinders[1].boundaries()[1].center(),
        cylinders[0].boundaries()[1].center(),
    )?;
    match (low, high) {
        (Orientation::Positive, Orientation::Negative) => Ok((1, 0)),
        (Orientation::Negative, Orientation::Positive) => Ok((0, 1)),
        (Orientation::Zero, Orientation::Zero) => {
            Err(ParallelCylinderRelationGap::AxialIntervalsEqual)
        }
        _ => Err(ParallelCylinderRelationGap::AxialIntervalsNotStrictlyNested),
    }
}

fn certify_strict_radial_secancy(
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    let first = cylinders[0].cylinder();
    let second = cylinders[1].cylinder();
    let axis = interval_vec3(first.frame().z());
    let displacement = interval_sub(
        interval_vec3(second.frame().origin()),
        interval_vec3(first.frame().origin()),
    );
    let cross = interval_cross(displacement, axis);
    let numerator = interval_norm_squared(cross);
    let denominator = interval_norm_squared(axis);
    let distance_squared = numerator
        .checked_div(denominator)
        .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?;
    let radius_a = Interval::point(first.radius());
    let radius_b = Interval::point(second.radius());
    let radius_difference_squared = (radius_a - radius_b).square();
    let radius_sum_squared = (radius_a + radius_b).square();
    if !finite_interval(distance_squared)
        || !finite_interval(radius_difference_squared)
        || !finite_interval(radius_sum_squared)
    {
        return Err(ParallelCylinderRelationGap::ArithmeticGuard);
    }
    if distance_squared.lo() > radius_difference_squared.hi()
        && distance_squared.hi() < radius_sum_squared.lo()
    {
        Ok(())
    } else {
        Err(ParallelCylinderRelationGap::RadialSecancyNotStrict)
    }
}

fn axial_compare(
    axis: Vec3,
    point: Point3,
    origin: Point3,
) -> core::result::Result<Orientation, ParallelCylinderRelationGap> {
    affine_dot3(axis.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
        .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)
}

#[derive(Debug, Clone, Copy)]
struct BoundEndpoint {
    endpoint: usize,
    boundary: usize,
    root_ordinal: usize,
}

#[derive(Debug, Clone, Copy)]
struct EndpointBindRequest<'a> {
    endpoint: usize,
    source_parameter: &'a SectionSourceParameterKey,
    trim: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
struct PendingRuling {
    branch: usize,
    fragment: usize,
    ends: [BoundEndpoint; 2],
}

#[derive(Debug, Clone, Copy)]
struct PendingCapArc {
    branch: usize,
    fragment: usize,
    boundary: usize,
    ends: [BoundEndpoint; 2],
}

fn certify_section_relation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    inner_operand: usize,
    outer_operand: usize,
) -> core::result::Result<CertifiedParallelCylinderLensRelation, ParallelCylinderRelationGap> {
    if graph.completion() != crate::SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Err(ParallelCylinderRelationGap::SectionIncomplete);
    }
    if graph.branches().len() != 4
        || graph.curve_fragments().len() != 4
        || graph.curve_endpoints().len() != 4
        || graph.curve_components().len() != 1
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    certify_periodic_bindings(graph, cylinders)?;

    let component = &graph.curve_components()[0];
    if !component.closed() || component.fragments().len() != 4 {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    let mut covered_fragments = [false; 4];
    let mut covered_branches = [false; 4];
    let mut kinds = [0_u8; 4];
    let mut rulings = Vec::with_capacity(2);
    let mut cap_arcs: [Option<PendingCapArc>; 2] = [None, None];
    for (component_ordinal, &fragment_index) in component.fragments().iter().enumerate() {
        if fragment_index >= 4 || covered_fragments[fragment_index] {
            return Err(ParallelCylinderRelationGap::SectionLayout);
        }
        covered_fragments[fragment_index] = true;
        let fragment = &graph.curve_fragments()[fragment_index];
        let branch_index = fragment.branch();
        if branch_index >= 4 || covered_branches[branch_index] || fragment.source_ordinal() != 0 {
            return Err(ParallelCylinderRelationGap::SectionLayout);
        }
        covered_branches[branch_index] = true;
        let branch = &graph.branches()[branch_index];
        match fragment.span() {
            SectionCurveFragmentSpan::LineSegment { endpoints } => {
                kinds[component_ordinal] = 1;
                if rulings.len() == 2 {
                    return Err(ParallelCylinderRelationGap::SectionLayout);
                }
                rulings.push(certify_ruling(
                    graph,
                    cylinders,
                    inner_operand,
                    outer_operand,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?);
            }
            SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                kinds[component_ordinal] = 2;
                let arc = certify_cap_arc(
                    graph,
                    cylinders,
                    inner_operand,
                    outer_operand,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?;
                if cap_arcs[arc.boundary].replace(arc).is_some() {
                    return Err(ParallelCylinderRelationGap::SectionLayout);
                }
            }
            SectionCurveFragmentSpan::Whole => {
                return Err(ParallelCylinderRelationGap::SectionLayout);
            }
        }
    }
    if !covered_fragments.into_iter().all(|covered| covered)
        || !covered_branches.into_iter().all(|covered| covered)
        || rulings.len() != 2
        || kinds
            .into_iter()
            .enumerate()
            .any(|(index, kind)| kind == kinds[(index + 1) % kinds.len()])
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    let [Some(cap_low), Some(cap_high)] = cap_arcs else {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    };
    let [first_ruling, second_ruling] = rulings.as_slice() else {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    };
    let mut pending_rulings = [*first_ruling, *second_ruling];

    let mut incidence = [0_u8; 4];
    let mut endpoint_kinds = [0_u8; 4];
    let mut endpoint_binding: [Option<(usize, usize)>; 4] = [None; 4];
    for ruling in pending_rulings {
        for end in ruling.ends {
            record_endpoint(
                end,
                1,
                &mut incidence,
                &mut endpoint_kinds,
                &mut endpoint_binding,
            )?;
        }
    }
    for arc in [cap_low, cap_high] {
        for end in arc.ends {
            record_endpoint(
                end,
                2,
                &mut incidence,
                &mut endpoint_kinds,
                &mut endpoint_binding,
            )?;
        }
    }
    if incidence.into_iter().any(|count| count != 2)
        || endpoint_kinds.into_iter().any(|kind| kind != 3)
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }

    let mut arc_endpoint_by_root = [[usize::MAX; 2]; 2];
    for arc in [cap_low, cap_high] {
        for end in arc.ends {
            if end.root_ordinal >= 2
                || arc_endpoint_by_root[arc.boundary][end.root_ordinal] != usize::MAX
            {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
            arc_endpoint_by_root[arc.boundary][end.root_ordinal] = end.endpoint;
        }
    }
    if arc_endpoint_by_root
        .into_iter()
        .flatten()
        .any(|endpoint| endpoint == usize::MAX)
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    for ruling in pending_rulings {
        for end in ruling.ends {
            if arc_endpoint_by_root[end.boundary][end.root_ordinal] != end.endpoint {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
        }
    }

    pending_rulings.sort_by_key(|ruling| {
        let by_boundary = ends_by_boundary(ruling.ends);
        [by_boundary[0].root_ordinal, by_boundary[1].root_ordinal]
    });
    let rulings = pending_rulings.map(|ruling| {
        let ends = ends_by_boundary(ruling.ends);
        ParallelCylinderRulingWitness {
            branch: ruling.branch,
            fragment: ruling.fragment,
            endpoints: [ends[0].endpoint, ends[1].endpoint],
            root_ordinals: [ends[0].root_ordinal, ends[1].root_ordinal],
        }
    });
    let cap_boundaries = [cap_low, cap_high].map(|arc| {
        let mut roots = [arc.ends[0].root_ordinal, arc.ends[1].root_ordinal];
        roots.sort_unstable();
        ParallelCylinderCapBoundaryWitness {
            boundary: arc.boundary,
            cap_face: cylinders[inner_operand].boundaries()[arc.boundary].cap_face(),
            edge: cylinders[inner_operand].boundaries()[arc.boundary].edge(),
            branch: arc.branch,
            fragment: arc.fragment,
            root_ordinals: roots,
        }
    });
    Ok(CertifiedParallelCylinderLensRelation {
        inner_operand,
        outer_operand,
        component: 0,
        cap_boundaries,
        rulings,
    })
}

fn certify_periodic_bindings(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    if graph.periodic_face_embeddings().len() != 2 {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let mut seen = [false; 2];
    for evidence in graph.periodic_face_embeddings() {
        let SectionPeriodicFaceEmbeddingEvidence::Certified(evidence) = evidence else {
            return Err(ParallelCylinderRelationGap::SectionOperandBinding);
        };
        let operand = evidence.operand();
        if operand >= 2 || seen[operand] || evidence.face().raw() != cylinders[operand].side_face()
        {
            return Err(ParallelCylinderRelationGap::SectionOperandBinding);
        }
        seen[operand] = true;
    }
    if seen.into_iter().all(|value| value) {
        Ok(())
    } else {
        Err(ParallelCylinderRelationGap::SectionOperandBinding)
    }
}

#[allow(clippy::too_many_arguments)]
fn certify_ruling(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    inner_operand: usize,
    outer_operand: usize,
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionRulingFragmentEnd; 2],
) -> core::result::Result<PendingRuling, ParallelCylinderRelationGap> {
    if branch.faces()[inner_operand].raw() != cylinders[inner_operand].side_face()
        || branch.faces()[outer_operand].raw() != cylinders[outer_operand].side_face()
        || branch.topology() != SectionBranchTopology::Open
        || branch.endpoint_sites() != [0, 1]
        || branch.fragment_sites().len() != 2
        || !matches!(branch.pcurves()[0], SectionUvCurve::Line(_))
        || !matches!(branch.pcurves()[1], SectionUvCurve::Line(_))
    {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let SectionCarrier::Line { origin, direction } = branch.carrier() else {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    };
    if !finite_vec3(origin)
        || !finite_vec3(direction)
        || !vectors_are_exactly_parallel(direction, cylinders[0].cylinder().frame().z())
        || !valid_branch_evidence(branch)
    {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    }
    let mut ends: [Option<BoundEndpoint>; 2] = [None, None];
    for end in endpoints.iter() {
        if !finite_vec3(end.point()) || !end.carrier_parameter().is_finite() {
            return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
        }
        let trims = end.trims();
        if trims[outer_operand].is_some() {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let Some(trim) = trims[inner_operand].as_ref() else {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        };
        if trim.operand() != inner_operand
            || trim.face().raw() != cylinders[inner_operand].side_face()
            || !valid_interval(trim.carrier_parameter().lo(), trim.carrier_parameter().hi())
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let binding = bind_endpoint(
            graph,
            cylinders,
            [inner_operand, outer_operand],
            EndpointBindRequest {
                endpoint: end.endpoint(),
                source_parameter: trim.source_parameter(),
                trim: [trim.edge_parameter().lo(), trim.edge_parameter().hi()],
            },
        )?;
        if ends[binding.boundary].replace(binding).is_some() {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    }
    let [Some(low), Some(high)] = ends else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    Ok(PendingRuling {
        branch: branch_index,
        fragment: fragment_index,
        ends: [low, high],
    })
}

#[allow(clippy::too_many_arguments)]
fn certify_cap_arc(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    inner_operand: usize,
    outer_operand: usize,
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionCurveFragmentEnd; 2],
) -> core::result::Result<PendingCapArc, ParallelCylinderRelationGap> {
    let boundary = cylinders[inner_operand]
        .boundaries()
        .iter()
        .position(|boundary| branch.faces()[inner_operand].raw() == boundary.cap_face())
        .ok_or(ParallelCylinderRelationGap::SectionOperandBinding)?;
    if branch.faces()[outer_operand].raw() != cylinders[outer_operand].side_face()
        || branch.topology() != SectionBranchTopology::Closed
        || !matches!(branch.pcurves()[inner_operand], SectionUvCurve::Circle(_))
        || !matches!(branch.pcurves()[outer_operand], SectionUvCurve::Line(_))
    {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let SectionCarrier::Circle {
        center,
        normal,
        x_direction,
        radius,
    } = branch.carrier()
    else {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    };
    if !finite_vec3(center)
        || !finite_vec3(normal)
        || !finite_vec3(x_direction)
        || !radius.is_finite()
        || radius <= 0.0
        || !vectors_are_exactly_parallel(normal, cylinders[0].cylinder().frame().z())
        || !valid_branch_evidence(branch)
    {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    }
    let mut ends: [Option<BoundEndpoint>; 2] = [None, None];
    for (end_index, end) in endpoints.iter().enumerate() {
        let trim = end.trim();
        if !finite_vec3(end.point())
            || !end.carrier_parameter().is_finite()
            || trim.operand() != inner_operand
            || trim.face().raw() != cylinders[inner_operand].boundaries()[boundary].cap_face()
            || !valid_interval(trim.edge_parameter().lo(), trim.edge_parameter().hi())
            || !valid_interval(trim.pcurve_half_angle().lo(), trim.pcurve_half_angle().hi())
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let binding = bind_endpoint(
            graph,
            cylinders,
            [inner_operand, outer_operand],
            EndpointBindRequest {
                endpoint: end.endpoint(),
                source_parameter: trim.source_parameter(),
                trim: [trim.edge_parameter().lo(), trim.edge_parameter().hi()],
            },
        )?;
        if binding.boundary != boundary {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        ends[end_index] = Some(binding);
    }
    let [Some(first), Some(second)] = ends else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    if first.endpoint == second.endpoint || first.root_ordinal == second.root_ordinal {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    Ok(PendingCapArc {
        branch: branch_index,
        fragment: fragment_index,
        boundary,
        ends: [first, second],
    })
}

fn bind_endpoint(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    operands: [usize; 2],
    request: EndpointBindRequest<'_>,
) -> core::result::Result<BoundEndpoint, ParallelCylinderRelationGap> {
    let [inner_operand, outer_operand] = operands;
    let [trim_lo, trim_hi] = request.trim;
    let endpoint_index = request.endpoint;
    let source_parameter = request.source_parameter;
    let endpoint = graph
        .curve_endpoints()
        .get(endpoint_index)
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let boundary = cylinders[inner_operand]
        .boundaries()
        .iter()
        .position(|boundary| source_parameter.edge().raw() == boundary.edge())
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    if !matches!(
        &sites[inner_operand],
        SectionSite::EdgeInterior(edge) if edge.raw() == cylinders[inner_operand].boundaries()[boundary].edge()
    ) || !matches!(
        &sites[outer_operand],
        SectionSite::FaceInterior(face) if face.raw() == cylinders[outer_operand].side_face()
    ) || source_parameters[inner_operand].as_ref() != Some(source_parameter)
        || source_parameters[outer_operand].is_some()
        || endpoint.edge_parameters()[outer_operand].is_some()
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    let common = endpoint.edge_parameters()[inner_operand]
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let root_enclosure = source_parameter.root_parameter_enclosure();
    if source_parameter.root_ordinal() >= 2
        || !source_parameter.root_parameter().is_finite()
        || !valid_interval(root_enclosure.lo(), root_enclosure.hi())
        || !valid_interval(trim_lo, trim_hi)
        || !valid_interval(common.lo(), common.hi())
        || common.lo() < trim_lo
        || common.hi() > trim_hi
        || !root_enclosure.contains(source_parameter.root_parameter())
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    Ok(BoundEndpoint {
        endpoint: endpoint_index,
        boundary,
        root_ordinal: source_parameter.root_ordinal(),
    })
}

fn valid_branch_evidence(branch: &SectionBranch) -> bool {
    let range = branch.range();
    let evidence = branch.evidence();
    range.is_finite()
        && range.lo < range.hi
        && evidence.tolerance().is_finite()
        && evidence.tolerance() > 0.0
        && evidence
            .residual_bounds()
            .into_iter()
            .all(|bound| bound.is_finite() && bound >= 0.0 && bound <= evidence.tolerance())
}

fn record_endpoint(
    end: BoundEndpoint,
    kind: u8,
    incidence: &mut [u8; 4],
    kinds: &mut [u8; 4],
    bindings: &mut [Option<(usize, usize)>; 4],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    if end.endpoint >= incidence.len() {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    incidence[end.endpoint] = incidence[end.endpoint]
        .checked_add(1)
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    if kinds[end.endpoint] & kind != 0 {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    kinds[end.endpoint] |= kind;
    let binding = (end.boundary, end.root_ordinal);
    if let Some(existing) = bindings[end.endpoint] {
        if existing != binding {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    } else {
        bindings[end.endpoint] = Some(binding);
    }
    Ok(())
}

fn ends_by_boundary(ends: [BoundEndpoint; 2]) -> [BoundEndpoint; 2] {
    if ends[0].boundary == 0 {
        ends
    } else {
        [ends[1], ends[0]]
    }
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

#[derive(Debug, Clone, Copy)]
struct IntervalVec3 {
    x: Interval,
    y: Interval,
    z: Interval,
}

fn interval_vec3(value: Vec3) -> IntervalVec3 {
    IntervalVec3 {
        x: Interval::point(value.x),
        y: Interval::point(value.y),
        z: Interval::point(value.z),
    }
}

fn interval_sub(first: IntervalVec3, second: IntervalVec3) -> IntervalVec3 {
    IntervalVec3 {
        x: first.x - second.x,
        y: first.y - second.y,
        z: first.z - second.z,
    }
}

fn interval_cross(first: IntervalVec3, second: IntervalVec3) -> IntervalVec3 {
    IntervalVec3 {
        x: first.y * second.z - first.z * second.y,
        y: first.z * second.x - first.x * second.z,
        z: first.x * second.y - first.y * second.x,
    }
}

fn interval_norm_squared(value: IntervalVec3) -> Interval {
    value.x.square() + value.y.square() + value.z.square()
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn finite_vec3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn valid_interval(lo: f64, hi: f64) -> bool {
    lo.is_finite() && hi.is_finite() && lo <= hi
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};

    use super::*;
    use crate::{
        BodyId, CylinderRequest, Kernel, PartId, SectionBodiesRequest, SectionCompletion, Session,
    };

    #[derive(Debug, Clone, Copy)]
    enum Placement {
        World,
        Oblique,
    }

    struct Fixture {
        session: Session,
        part: PartId,
        outer: BodyId,
        inner: BodyId,
    }

    fn shared_frame(placement: Placement) -> Frame {
        match placement {
            Placement::World => Frame::world(),
            Placement::Oblique => Frame::new(
                Point3::new(2.5, -1.75, 0.625),
                Vec3::new(0.48, 0.64, 0.6),
                Vec3::new(0.8, -0.6, 0.0),
            )
            .unwrap(),
        }
    }

    fn fixture(placement: Placement) -> Fixture {
        let frame = shared_frame(placement);
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (outer, inner) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let outer = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(-0.5, 0.0, -2.0)),
                    1.0,
                    4.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let inner = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(0.5, 0.0, -1.0)),
                    1.0,
                    2.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (outer, inner)
        };
        Fixture {
            session,
            part,
            outer,
            inner,
        }
    }

    fn section(fixture: &Fixture, swapped: bool) -> BodySectionGraph {
        let (first, second) = if swapped {
            (fixture.inner.clone(), fixture.outer.clone())
        } else {
            (fixture.outer.clone(), fixture.inner.clone())
        };
        fixture
            .session
            .part(fixture.part.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(first, second))
            .unwrap()
            .into_result()
            .unwrap()
    }

    fn extract_source(fixture: &Fixture, body: &BodyId) -> CertifiedCylinderSource {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        match super::super::curved_source::extract_cylinder_source(
            &part.state.store,
            body.raw(),
            &mut scope,
        )
        .unwrap()
        {
            super::super::curved_source::CylinderSourceOutcome::Ready(source) => source,
            other => panic!("unexpected cylinder source outcome: {other:?}"),
        }
    }

    fn sources(fixture: &Fixture, swapped: bool) -> [CertifiedCylinderSource; 2] {
        let outer = extract_source(fixture, &fixture.outer);
        let inner = extract_source(fixture, &fixture.inner);
        if swapped {
            [inner, outer]
        } else {
            [outer, inner]
        }
    }

    fn certify(
        fixture: &Fixture,
        graph: &BodySectionGraph,
        sources: &[CertifiedCylinderSource; 2],
        allowed: u64,
    ) -> Result<ParallelCylinderRelationOutcome> {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let overrides = BudgetPlan::new([LimitSpec::new(
            PLANAR_BOOLEAN_BSP_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults())
            .with_budget_overrides(overrides);
        let mut scope = OperationScope::new(&context);
        certify_parallel_cylinder_relation(graph, [&sources[0], &sources[1]], &mut scope)
    }

    fn certified(
        outcome: ParallelCylinderRelationOutcome,
    ) -> CertifiedParallelCylinderLensRelation {
        match outcome {
            ParallelCylinderRelationOutcome::Certified(certificate) => *certificate,
            other => panic!("expected certified relation, got {other:?}"),
        }
    }

    #[test]
    fn strict_world_and_oblique_relations_are_replay_and_swap_deterministic() {
        for placement in [Placement::World, Placement::Oblique] {
            let fixture = fixture(placement);
            let forward_graph = section(&fixture, false);
            let replay_graph = section(&fixture, false);
            let swapped_graph = section(&fixture, true);
            let forward_sources = sources(&fixture, false);
            let swapped_sources = sources(&fixture, true);

            let forward = certified(
                certify(
                    &fixture,
                    &forward_graph,
                    &forward_sources,
                    PARALLEL_CYLINDER_RELATION_WORK,
                )
                .unwrap(),
            );
            let replay = certified(
                certify(
                    &fixture,
                    &replay_graph,
                    &forward_sources,
                    PARALLEL_CYLINDER_RELATION_WORK,
                )
                .unwrap(),
            );
            let swapped = certified(
                certify(
                    &fixture,
                    &swapped_graph,
                    &swapped_sources,
                    PARALLEL_CYLINDER_RELATION_WORK,
                )
                .unwrap(),
            );
            assert_eq!(forward, replay);
            assert_eq!((forward.inner_operand(), forward.outer_operand()), (1, 0));
            assert_eq!((swapped.inner_operand(), swapped.outer_operand()), (0, 1));
            assert_eq!(forward.component(), 0);
            assert_eq!(swapped.component(), 0);
            assert_eq!(
                forward.cap_boundaries().map(|witness| (
                    witness.boundary(),
                    witness.cap_face(),
                    witness.edge(),
                    witness.root_ordinals()
                )),
                swapped.cap_boundaries().map(|witness| (
                    witness.boundary(),
                    witness.cap_face(),
                    witness.edge(),
                    witness.root_ordinals()
                ))
            );
            assert_eq!(
                forward.rulings().map(|witness| witness.root_ordinals()),
                swapped.rulings().map(|witness| witness.root_ordinals())
            );
            for (boundary, witness) in forward.cap_boundaries().iter().enumerate() {
                assert_eq!(witness.boundary(), boundary);
                assert_eq!(witness.root_ordinals(), [0, 1]);
                assert!(witness.branch() < forward_graph.branches().len());
                assert!(witness.fragment() < forward_graph.curve_fragments().len());
            }
            for witness in forward.rulings() {
                assert!(witness.branch() < forward_graph.branches().len());
                assert!(witness.fragment() < forward_graph.curve_fragments().len());
                assert!(witness.endpoints().into_iter().all(|endpoint| endpoint < 4));
            }
        }
    }

    #[test]
    fn relation_work_accepts_exact_n_and_refuses_n_minus_one() {
        let fixture = fixture(Placement::World);
        let graph = section(&fixture, false);
        let sources = sources(&fixture, false);
        assert!(matches!(
            certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
            ParallelCylinderRelationOutcome::Certified(_)
        ));

        let error = certify(
            &fixture,
            &graph,
            &sources,
            PARALLEL_CYLINDER_RELATION_WORK - 1,
        )
        .unwrap_err();
        let snapshot = error
            .limit()
            .expect("relation must retain exact limit evidence");
        assert_eq!(snapshot.stage, PLANAR_BOOLEAN_BSP_WORK);
        assert_eq!(snapshot.resource, ResourceKind::Work);
        assert_eq!(snapshot.consumed, PARALLEL_CYLINDER_RELATION_WORK);
        assert_eq!(snapshot.allowed, PARALLEL_CYLINDER_RELATION_WORK - 1);
    }

    #[test]
    fn incomplete_layout_binding_and_endpoint_failures_are_typed() {
        let fixture = fixture(Placement::World);
        let graph = section(&fixture, false);
        let sources = sources(&fixture, false);

        let mut incomplete = graph.clone();
        incomplete.completion = SectionCompletion::Indeterminate;
        assert_eq!(
            certify(
                &fixture,
                &incomplete,
                &sources,
                PARALLEL_CYLINDER_RELATION_WORK,
            )
            .unwrap(),
            ParallelCylinderRelationOutcome::Indeterminate(
                ParallelCylinderRelationGap::SectionIncomplete,
            )
        );

        let mut truncated = graph.clone();
        truncated.curve_fragments.pop();
        assert_eq!(
            certify(
                &fixture,
                &truncated,
                &sources,
                PARALLEL_CYLINDER_RELATION_WORK,
            )
            .unwrap(),
            ParallelCylinderRelationOutcome::Indeterminate(
                ParallelCylinderRelationGap::SectionLayout,
            )
        );

        let reversed_sources = [sources[1].clone(), sources[0].clone()];
        assert_eq!(
            certify(
                &fixture,
                &graph,
                &reversed_sources,
                PARALLEL_CYLINDER_RELATION_WORK,
            )
            .unwrap(),
            ParallelCylinderRelationOutcome::Indeterminate(
                ParallelCylinderRelationGap::SectionOperandBinding,
            )
        );

        let mut mismatched_endpoints = graph;
        mismatched_endpoints.curve_endpoints.swap(0, 1);
        assert_eq!(
            certify(
                &fixture,
                &mismatched_endpoints,
                &sources,
                PARALLEL_CYLINDER_RELATION_WORK,
            )
            .unwrap(),
            ParallelCylinderRelationOutcome::Indeterminate(
                ParallelCylinderRelationGap::SectionEndpointProvenance,
            )
        );
    }

    fn analytic_sources(
        first_frame: Frame,
        first_height: f64,
        second_frame: Frame,
        second_height: f64,
    ) -> [CertifiedCylinderSource; 2] {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (first, second) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let first = edit
                .create_cylinder(CylinderRequest::new(first_frame, 1.0, first_height))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let second = edit
                .create_cylinder(CylinderRequest::new(second_frame, 1.0, second_height))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (first, second)
        };
        let part = session.part(part_id).unwrap();
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        [first, second].map(|body| {
            match super::super::curved_source::extract_cylinder_source(
                &part.state.store,
                body.raw(),
                &mut scope,
            )
            .unwrap()
            {
                super::super::curved_source::CylinderSourceOutcome::Ready(source) => source,
                other => panic!("unexpected source extraction: {other:?}"),
            }
        })
    }

    #[test]
    fn analytic_boundary_cases_have_distinct_typed_gaps() {
        let world = Frame::world();
        let equal = analytic_sources(
            world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
            2.0,
            world.with_origin(Point3::new(0.5, 0.0, -1.0)),
            2.0,
        );
        assert_eq!(
            certify_source_relation([&equal[0], &equal[1]]),
            Err(ParallelCylinderRelationGap::AxialIntervalsEqual)
        );

        let partial = analytic_sources(
            world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
            2.0,
            world.with_origin(Point3::new(0.5, 0.0, 0.0)),
            2.0,
        );
        assert_eq!(
            certify_source_relation([&partial[0], &partial[1]]),
            Err(ParallelCylinderRelationGap::AxialIntervalsNotStrictlyNested)
        );

        let tangent = analytic_sources(
            world.with_origin(Point3::new(-1.0, 0.0, -2.0)),
            4.0,
            world.with_origin(Point3::new(1.0, 0.0, -1.0)),
            2.0,
        );
        assert_eq!(
            certify_source_relation([&tangent[0], &tangent[1]]),
            Err(ParallelCylinderRelationGap::RadialSecancyNotStrict)
        );

        let reversed = Frame::new(Point3::new(0.5, 0.0, 1.0), -world.z(), world.x()).unwrap();
        let opposed = analytic_sources(
            world.with_origin(Point3::new(-0.5, 0.0, -2.0)),
            4.0,
            reversed,
            2.0,
        );
        assert_eq!(
            certify_source_relation([&opposed[0], &opposed[1]]),
            Err(ParallelCylinderRelationGap::AxesOppositelySigned)
        );

        let skew = Frame::new(
            Point3::new(0.5, 0.0, -1.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let nonparallel = analytic_sources(
            world.with_origin(Point3::new(-0.5, 0.0, -2.0)),
            4.0,
            skew,
            2.0,
        );
        assert_eq!(
            certify_source_relation([&nonparallel[0], &nonparallel[1]]),
            Err(ParallelCylinderRelationGap::AxesNotExactlyParallel)
        );
    }
}
