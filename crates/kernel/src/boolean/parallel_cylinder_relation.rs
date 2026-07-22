//! Operation-local theorem for the first parallel-cylinder Boolean slice.
//!
//! The section graph remains the general intersection authority.  This module
//! only recognizes the strict finite lens-prism relation needed by the first
//! Cylinder/Cylinder Boolean slices: exactly parallel or antiparallel axes,
//! strict transverse radial secancy, a strict positive axial overlap with two
//! uniquely owned physical ends, and one closed section component alternating
//! between two rulings and the matching cap arcs. Both authored boundary
//! orders are normalized onto one certified common axial coordinate before
//! overlap ownership is decided. Every join is owned by the section graph's
//! source-edge/root identity; rounded points are never used as topology keys.

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
    /// The radial circles are not certified as a strict two-root secant.
    RadialSecancyNotStrict,
    /// A source's authored cap interval is not strictly positive on its own axis.
    SourceAxialOrder,
    /// Both source cap intervals describe the same two axial planes.
    AxialIntervalsEqual,
    /// A physical overlap end is shared by both source cap intervals.
    AxialOverlapEndNotUnique,
    /// The source cap intervals are disjoint or touch without positive overlap.
    AxialOverlapNotStrictlyPositive,
    /// The supplied section is not globally complete and gap-free.
    SectionIncomplete,
    /// Fixed collection counts, component coverage, or alternation did not match.
    SectionLayout,
    /// Periodic or branch face evidence does not bind to the supplied sources.
    SectionOperandBinding,
    /// A branch carrier, pcurve family, range, or residual proof did not match.
    SectionBranchEvidence,
    /// A trim endpoint did not retain the required overlap-end cap-ring provenance.
    SectionEndpointProvenance,
}

/// One physical axial-overlap end and the unique section arc cut into its cap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ParallelCylinderOverlapEndWitness {
    operand: usize,
    boundary: usize,
    cap_face: RawFaceId,
    edge: RawEdgeId,
    branch: usize,
    fragment: usize,
    root_ordinals: [usize; 2],
}

impl ParallelCylinderOverlapEndWitness {
    /// Operand owning this physical overlap end.
    pub(super) const fn operand(&self) -> usize {
        self.operand
    }

    /// Owning source boundary ordinal in authored-axis order.
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
    /// Endpoint indices in low/high physical-overlap-end order.
    endpoints: [usize; 2],
    /// Source-root ordinals in low/high physical-overlap-end order.
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

    /// Endpoint indices in low/high physical-overlap-end order.
    pub(super) const fn endpoints(&self) -> [usize; 2] {
        self.endpoints
    }

    /// Source-root ordinals in low/high physical-overlap-end order.
    pub(super) const fn root_ordinals(&self) -> [usize; 2] {
        self.root_ordinals
    }
}

/// Complete operation-local proof of the strict finite lens-prism relation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CertifiedParallelCylinderLensRelation {
    component: usize,
    overlap_ends: [ParallelCylinderOverlapEndWitness; 2],
    rulings: [ParallelCylinderRulingWitness; 2],
}

impl CertifiedParallelCylinderLensRelation {
    /// Return `[inner, outer]` only when the overlap is strict axial nesting.
    pub(super) const fn strict_nesting_operands(&self) -> Option<[usize; 2]> {
        if self.overlap_ends[0].operand == self.overlap_ends[1].operand {
            let inner = self.overlap_ends[0].operand;
            Some([inner, 1 - inner])
        } else {
            None
        }
    }

    /// Unique closed section-component index.
    pub(super) const fn component(&self) -> usize {
        self.component
    }

    /// Low/high physical-overlap-end cap-arc witnesses.
    pub(super) const fn overlap_ends(&self) -> &[ParallelCylinderOverlapEndWitness; 2] {
        &self.overlap_ends
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

/// Certify the exact strict-overlap relation consumed by the first
/// parallel-cylinder Boolean realization slices.
pub(super) fn certify_parallel_cylinder_relation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<ParallelCylinderRelationOutcome> {
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, PARALLEL_CYLINDER_RELATION_WORK)
        .map_err(Error::from)?;

    let overlap_ends = match certify_source_relation(cylinders) {
        Ok(relation) => relation,
        Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    };
    match certify_section_relation(graph, cylinders, overlap_ends) {
        Ok(certificate) => Ok(ParallelCylinderRelationOutcome::Certified(Box::new(
            certificate,
        ))),
        Err(gap) => Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    }
}

fn certify_source_relation(
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<[SourceOverlapEnd; 2], ParallelCylinderRelationGap> {
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
    let common_axis = match axis_sign {
        Orientation::Positive => axis_a,
        Orientation::Negative => canonical_unoriented_axis(axis_a)?,
        Orientation::Zero => return Err(ParallelCylinderRelationGap::ArithmeticGuard),
    };
    certify_strict_radial_secancy(cylinders)?;

    let mut intervals = [NormalizedSourceInterval::default(); 2];
    for (operand, source) in cylinders.into_iter().enumerate() {
        if axial_compare(
            source.cylinder().frame().z(),
            source.boundaries()[1].center(),
            source.boundaries()[0].center(),
        )? != Orientation::Positive
        {
            return Err(ParallelCylinderRelationGap::SourceAxialOrder);
        }
        intervals[operand] = match axial_compare(
            common_axis,
            source.boundaries()[1].center(),
            source.boundaries()[0].center(),
        )? {
            Orientation::Positive => NormalizedSourceInterval { low: 0, high: 1 },
            Orientation::Negative => NormalizedSourceInterval { low: 1, high: 0 },
            Orientation::Zero => return Err(ParallelCylinderRelationGap::SourceAxialOrder),
        };
    }
    let low = axial_compare(
        common_axis,
        cylinders[1].boundaries()[intervals[1].low].center(),
        cylinders[0].boundaries()[intervals[0].low].center(),
    )?;
    let high = axial_compare(
        common_axis,
        cylinders[1].boundaries()[intervals[1].high].center(),
        cylinders[0].boundaries()[intervals[0].high].center(),
    )?;
    if low == Orientation::Zero && high == Orientation::Zero {
        return Err(ParallelCylinderRelationGap::AxialIntervalsEqual);
    }
    let low_end = match low {
        Orientation::Positive => SourceOverlapEnd {
            operand: 1,
            boundary: intervals[1].low,
        },
        Orientation::Negative => SourceOverlapEnd {
            operand: 0,
            boundary: intervals[0].low,
        },
        Orientation::Zero => {
            return Err(ParallelCylinderRelationGap::AxialOverlapEndNotUnique);
        }
    };
    let high_end = match high {
        Orientation::Positive => SourceOverlapEnd {
            operand: 0,
            boundary: intervals[0].high,
        },
        Orientation::Negative => SourceOverlapEnd {
            operand: 1,
            boundary: intervals[1].high,
        },
        Orientation::Zero => {
            return Err(ParallelCylinderRelationGap::AxialOverlapEndNotUnique);
        }
    };
    if axial_compare(
        common_axis,
        cylinders[high_end.operand].boundaries()[high_end.boundary].center(),
        cylinders[low_end.operand].boundaries()[low_end.boundary].center(),
    )? != Orientation::Positive
    {
        return Err(ParallelCylinderRelationGap::AxialOverlapNotStrictlyPositive);
    }
    Ok([low_end, high_end])
}

fn canonical_unoriented_axis(
    axis: Vec3,
) -> core::result::Result<Vec3, ParallelCylinderRelationGap> {
    for basis in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        let sign = affine_dot3(axis.to_array(), basis, [0.0; 3], 0.0)
            .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?
            .sign();
        match sign {
            Orientation::Positive => return Ok(axis),
            Orientation::Negative => return Ok(-axis),
            Orientation::Zero => {}
        }
    }
    Err(ParallelCylinderRelationGap::ArithmeticGuard)
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SourceOverlapEnd {
    operand: usize,
    boundary: usize,
}

#[derive(Debug, Clone, Copy, Default)]
struct NormalizedSourceInterval {
    low: usize,
    high: usize,
}

#[derive(Debug, Clone, Copy)]
struct BoundEndpoint {
    endpoint: usize,
    overlap_end: usize,
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
    overlap_end: usize,
    ends: [BoundEndpoint; 2],
}

fn certify_section_relation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    overlap_ends: [SourceOverlapEnd; 2],
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
                    overlap_ends,
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
                    overlap_ends,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?;
                if cap_arcs[arc.overlap_end].replace(arc).is_some() {
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
                || arc_endpoint_by_root[arc.overlap_end][end.root_ordinal] != usize::MAX
            {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
            arc_endpoint_by_root[arc.overlap_end][end.root_ordinal] = end.endpoint;
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
            if arc_endpoint_by_root[end.overlap_end][end.root_ordinal] != end.endpoint {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
        }
    }

    pending_rulings.sort_by_key(|ruling| {
        let by_overlap_end = ends_by_overlap_end(ruling.ends);
        [
            by_overlap_end[0].root_ordinal,
            by_overlap_end[1].root_ordinal,
        ]
    });
    let rulings = pending_rulings.map(|ruling| {
        let ends = ends_by_overlap_end(ruling.ends);
        ParallelCylinderRulingWitness {
            branch: ruling.branch,
            fragment: ruling.fragment,
            endpoints: [ends[0].endpoint, ends[1].endpoint],
            root_ordinals: [ends[0].root_ordinal, ends[1].root_ordinal],
        }
    });
    let overlap_ends = [cap_low, cap_high].map(|arc| {
        let mut roots = [arc.ends[0].root_ordinal, arc.ends[1].root_ordinal];
        roots.sort_unstable();
        let source_end = overlap_ends[arc.overlap_end];
        ParallelCylinderOverlapEndWitness {
            operand: source_end.operand,
            boundary: source_end.boundary,
            cap_face: cylinders[source_end.operand].boundaries()[source_end.boundary].cap_face(),
            edge: cylinders[source_end.operand].boundaries()[source_end.boundary].edge(),
            branch: arc.branch,
            fragment: arc.fragment,
            root_ordinals: roots,
        }
    });
    Ok(CertifiedParallelCylinderLensRelation {
        component: 0,
        overlap_ends,
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
    overlap_ends: [SourceOverlapEnd; 2],
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionRulingFragmentEnd; 2],
) -> core::result::Result<PendingRuling, ParallelCylinderRelationGap> {
    if branch.faces()[0].raw() != cylinders[0].side_face()
        || branch.faces()[1].raw() != cylinders[1].side_face()
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
        let (trim_operand, trim) = match trims {
            [Some(trim), None] => (0, trim),
            [None, Some(trim)] => (1, trim),
            _ => return Err(ParallelCylinderRelationGap::SectionEndpointProvenance),
        };
        if trim.operand() != trim_operand
            || trim.face().raw() != cylinders[trim_operand].side_face()
            || !valid_interval(trim.carrier_parameter().lo(), trim.carrier_parameter().hi())
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let binding = bind_endpoint(
            graph,
            cylinders,
            overlap_ends,
            EndpointBindRequest {
                endpoint: end.endpoint(),
                source_parameter: trim.source_parameter(),
                trim: [trim.edge_parameter().lo(), trim.edge_parameter().hi()],
            },
        )?;
        if ends[binding.overlap_end].replace(binding).is_some() {
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
    overlap_ends: [SourceOverlapEnd; 2],
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionCurveFragmentEnd; 2],
) -> core::result::Result<PendingCapArc, ParallelCylinderRelationGap> {
    let mut matching_ends = overlap_ends.into_iter().enumerate().filter(|(_, end)| {
        branch.faces()[end.operand].raw()
            == cylinders[end.operand].boundaries()[end.boundary].cap_face()
    });
    let Some((overlap_end, source_end)) = matching_ends.next() else {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    };
    if matching_ends.next().is_some() {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let cap_operand = source_end.operand;
    let side_operand = 1 - cap_operand;
    if branch.faces()[side_operand].raw() != cylinders[side_operand].side_face()
        || branch.topology() != SectionBranchTopology::Closed
        || !matches!(branch.pcurves()[cap_operand], SectionUvCurve::Circle(_))
        || !matches!(branch.pcurves()[side_operand], SectionUvCurve::Line(_))
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
            || trim.operand() != cap_operand
            || trim.face().raw()
                != cylinders[cap_operand].boundaries()[source_end.boundary].cap_face()
            || !valid_interval(trim.edge_parameter().lo(), trim.edge_parameter().hi())
            || !valid_interval(trim.pcurve_half_angle().lo(), trim.pcurve_half_angle().hi())
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let binding = bind_endpoint(
            graph,
            cylinders,
            overlap_ends,
            EndpointBindRequest {
                endpoint: end.endpoint(),
                source_parameter: trim.source_parameter(),
                trim: [trim.edge_parameter().lo(), trim.edge_parameter().hi()],
            },
        )?;
        if binding.overlap_end != overlap_end {
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
        overlap_end,
        ends: [first, second],
    })
}

fn bind_endpoint(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    overlap_ends: [SourceOverlapEnd; 2],
    request: EndpointBindRequest<'_>,
) -> core::result::Result<BoundEndpoint, ParallelCylinderRelationGap> {
    let [trim_lo, trim_hi] = request.trim;
    let endpoint_index = request.endpoint;
    let source_parameter = request.source_parameter;
    let endpoint = graph
        .curve_endpoints()
        .get(endpoint_index)
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let mut matching_ends = overlap_ends.into_iter().enumerate().filter(|(_, end)| {
        source_parameter.edge().raw() == cylinders[end.operand].boundaries()[end.boundary].edge()
    });
    let Some((overlap_end, source_end)) = matching_ends.next() else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    if matching_ends.next().is_some() {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    let cap_operand = source_end.operand;
    let side_operand = 1 - cap_operand;
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    if !matches!(
        &sites[cap_operand],
        SectionSite::EdgeInterior(edge) if edge.raw() == cylinders[cap_operand].boundaries()[source_end.boundary].edge()
    ) || !matches!(
        &sites[side_operand],
        SectionSite::FaceInterior(face) if face.raw() == cylinders[side_operand].side_face()
    ) || source_parameters[cap_operand].as_ref() != Some(source_parameter)
        || source_parameters[side_operand].is_some()
        || endpoint.edge_parameters()[side_operand].is_some()
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    let common = endpoint.edge_parameters()[cap_operand]
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
        overlap_end,
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
    let binding = (end.overlap_end, end.root_ordinal);
    if let Some(existing) = bindings[end.endpoint] {
        if existing != binding {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    } else {
        bindings[end.endpoint] = Some(binding);
    }
    Ok(())
}

fn ends_by_overlap_end(ends: [BoundEndpoint; 2]) -> [BoundEndpoint; 2] {
    if ends[0].overlap_end == 0 {
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
#[path = "parallel_cylinder_relation/tests.rs"]
mod tests;
