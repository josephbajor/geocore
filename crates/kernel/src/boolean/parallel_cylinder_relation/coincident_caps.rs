//! Operation-local relation for strict lens prisms with shared cap planes.
//!
//! Public Section remains honestly indeterminate: coincident planar cells are
//! two-dimensional contact and therefore are not global SSI components.  This
//! theorem admits only the exact residual graph implied by one or two shared
//! overlap ends.  The two rulings retain every contributing source-ring root;
//! a unique end additionally retains its ordinary Plane/Cylinder cap arc.

use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId};

use super::*;
use crate::{SectionCompletion, SectionPeriodicEmbeddingGap, SectionPeriodicFaceEmbeddingEvidence};

/// One exact topology-owned root on a retained source cap ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ParallelCylinderSourceRootWitness {
    endpoint: usize,
    root_ordinal: usize,
    parameter_bits: u64,
    enclosure_bits: [u64; 2],
}

impl ParallelCylinderSourceRootWitness {
    pub(crate) const fn endpoint(self) -> usize {
        self.endpoint
    }

    pub(crate) const fn root_ordinal(self) -> usize {
        self.root_ordinal
    }

    pub(crate) const fn parameter(self) -> f64 {
        f64::from_bits(self.parameter_bits)
    }

    pub(crate) const fn enclosure(self) -> [f64; 2] {
        [
            f64::from_bits(self.enclosure_bits[0]),
            f64::from_bits(self.enclosure_bits[1]),
        ]
    }
}

/// One source ring contributing an arc to a physical overlap-end profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ParallelCylinderSourceArcWitness {
    operand: usize,
    boundary: usize,
    cap_face: RawFaceId,
    edge: RawEdgeId,
    roots: [ParallelCylinderSourceRootWitness; 2],
}

impl ParallelCylinderSourceArcWitness {
    pub(crate) const fn operand(self) -> usize {
        self.operand
    }

    pub(crate) const fn boundary(self) -> usize {
        self.boundary
    }

    pub(crate) const fn cap_face(self) -> RawFaceId {
        self.cap_face
    }

    pub(crate) const fn edge(self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn roots(self) -> [ParallelCylinderSourceRootWitness; 2] {
        self.roots
    }
}

/// Ordinary Section cap arc retained at a uniquely owned overlap end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ParallelCylinderSectionCapArcWitness {
    branch: usize,
    fragment: usize,
    endpoints: [usize; 2],
}

impl ParallelCylinderSectionCapArcWitness {
    pub(crate) const fn branch(self) -> usize {
        self.branch
    }

    pub(crate) const fn fragment(self) -> usize {
        self.fragment
    }

    pub(crate) const fn endpoints(self) -> [usize; 2] {
        self.endpoints
    }
}

/// Boundary vocabulary for one physical axial-overlap end.
///
/// The representation invariant is `source_count + cap_arc.is_some() == 2`:
/// a unique end has one source arc plus one Section arc, while a shared end
/// has the two source arcs and no fabricated SSI circle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ParallelCylinderCoincidentCapEndWitness {
    sources: [Option<ParallelCylinderSourceArcWitness>; 2],
    cap_arc: Option<ParallelCylinderSectionCapArcWitness>,
}

impl ParallelCylinderCoincidentCapEndWitness {
    pub(crate) const fn source(&self, operand: usize) -> Option<ParallelCylinderSourceArcWitness> {
        if operand < 2 {
            self.sources[operand]
        } else {
            None
        }
    }

    pub(crate) const fn sources(&self) -> &[Option<ParallelCylinderSourceArcWitness>; 2] {
        &self.sources
    }

    pub(crate) const fn cap_arc(&self) -> Option<ParallelCylinderSectionCapArcWitness> {
        self.cap_arc
    }

    pub(crate) fn is_shared(&self) -> bool {
        self.sources.iter().all(Option::is_some) && self.cap_arc.is_none()
    }
}

/// One bounded common-axis ruling in low/high physical-end order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ParallelCylinderCoincidentCapRulingWitness {
    branch: usize,
    fragment: usize,
    endpoints: [usize; 2],
}

impl ParallelCylinderCoincidentCapRulingWitness {
    pub(crate) const fn branch(self) -> usize {
        self.branch
    }

    pub(crate) const fn fragment(self) -> usize {
        self.fragment
    }

    pub(crate) const fn endpoints(self) -> [usize; 2] {
        self.endpoints
    }
}

/// Exact operation-local proof for an Intersect lens with shared cap cells.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedParallelCylinderCoincidentCapRelation {
    overlap_ends: [ParallelCylinderCoincidentCapEndWitness; 2],
    rulings: [ParallelCylinderCoincidentCapRulingWitness; 2],
}

impl CertifiedParallelCylinderCoincidentCapRelation {
    pub(crate) const fn overlap_ends(&self) -> &[ParallelCylinderCoincidentCapEndWitness; 2] {
        &self.overlap_ends
    }

    pub(crate) const fn rulings(&self) -> &[ParallelCylinderCoincidentCapRulingWitness; 2] {
        &self.rulings
    }

    pub(crate) fn unique_end_count(&self) -> usize {
        self.overlap_ends
            .iter()
            .filter(|end| end.cap_arc.is_some())
            .count()
    }

    /// Original graph fragments admitted by the operation-local periodic
    /// projection for one cylinder side. Boundary-coincident overlay arcs are
    /// deliberately absent: they lie on topology-owned source rings rather
    /// than cutting the open side face.
    pub(crate) fn periodic_fragment_subset(&self, operand: usize) -> Vec<usize> {
        let mut fragments = self
            .rulings
            .iter()
            .map(|ruling| ruling.fragment())
            .collect::<Vec<_>>();
        fragments.extend(self.overlap_ends.iter().filter_map(|end| {
            (end.source(operand).is_none())
                .then(|| {
                    end.cap_arc()
                        .map(ParallelCylinderSectionCapArcWitness::fragment)
                })
                .flatten()
        }));
        fragments.sort_unstable();
        fragments
    }
}

#[derive(Debug, Clone, Copy)]
struct PendingCoincidentRuling {
    branch: usize,
    fragment: usize,
    endpoints: [BoundCoincidentEndpoint; 2],
}

#[derive(Debug, Clone, Copy)]
struct BoundCoincidentEndpoint {
    endpoint: usize,
    overlap_end: usize,
}

/// Upgrade metric cap ordering with Section's exact dual-source endpoint proof.
///
/// Rounded authored frames can give two semantically shared cap centers a
/// nonzero world-space dot product.  Section owns the stronger representation:
/// both ruling roots name one exact cap-ring edge on each operand.  Two root
/// ordinals on the same edge pair prove one shared physical end without using
/// proximity or endpoint coordinates.  Graphs outside the fixed coincident-cap
/// envelope are left to the ordinary relation checks.
pub(super) fn reconcile_shared_overlap_ends(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    mut source_ends: [SourceOverlapEnd; 2],
) -> core::result::Result<[SourceOverlapEnd; 2], ParallelCylinderRelationGap> {
    if graph.completion() != SectionCompletion::Indeterminate {
        return Ok(source_ends);
    }
    if graph.branches().len() > 6
        || graph.curve_fragments().len() > 6
        || graph.curve_endpoints().len() != 4
    {
        return Ok(source_ends);
    }

    let mut pairs: Vec<([usize; 2], Vec<(usize, [usize; 2])>)> = Vec::new();
    for fragment in graph.curve_fragments() {
        let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
            continue;
        };
        let branch = graph
            .branches()
            .get(fragment.branch())
            .ok_or(ParallelCylinderRelationGap::SectionLayout)?;
        for end in endpoints.iter() {
            let [Some(first), Some(second)] = end.trims() else {
                continue;
            };
            let trims = [first, second];
            let public = graph
                .curve_endpoints()
                .get(end.endpoint())
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            let SectionCurveEndpointTopology::Trim {
                source_parameters, ..
            } = public.topology()
            else {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            };
            let mut boundaries = [0_usize; 2];
            let mut roots = [0_usize; 2];
            for operand in 0..2 {
                let trim = trims[operand];
                let source = source_parameters[operand]
                    .as_ref()
                    .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
                if trim.operand() != operand
                    || trim.face() != branch.faces()[operand]
                    || trim.source_parameter() != source
                {
                    return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
                }
                let mut matches =
                    cylinders[operand]
                        .boundaries()
                        .iter()
                        .enumerate()
                        .filter(|(_, boundary)| {
                            boundary.edge() == source.edge().raw()
                                && boundary.side_loop() == trim.loop_id().raw()
                                && boundary.side_fin() == trim.fin().raw()
                        });
                let (boundary, _) = matches
                    .next()
                    .filter(|_| matches.next().is_none())
                    .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
                boundaries[operand] = boundary;
                roots[operand] = source.root_ordinal();
            }
            match pairs
                .iter_mut()
                .find(|(candidate, _)| *candidate == boundaries)
            {
                Some((_, endpoints)) => endpoints.push((end.endpoint(), roots)),
                None => pairs.push((boundaries, vec![(end.endpoint(), roots)])),
            }
        }
    }

    let mut upgraded = [false; 2];
    for (boundaries, mut endpoints) in pairs {
        endpoints.sort_unstable();
        endpoints.dedup();
        let mut operand_roots = [
            [
                endpoints.first().map_or(usize::MAX, |value| value.1[0]),
                endpoints.get(1).map_or(usize::MAX, |value| value.1[0]),
            ],
            [
                endpoints.first().map_or(usize::MAX, |value| value.1[1]),
                endpoints.get(1).map_or(usize::MAX, |value| value.1[1]),
            ],
        ];
        operand_roots
            .iter_mut()
            .for_each(|roots| roots.sort_unstable());
        if endpoints.len() != 2
            || endpoints[0].0 == endpoints[1].0
            || operand_roots != [[0, 1], [0, 1]]
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let mut matches = source_ends.iter().enumerate().filter(|(_, end)| {
            (0..2).any(|operand| end.boundary_for(operand) == Some(boundaries[operand]))
        });
        let (end_index, _) = matches
            .next()
            .filter(|_| matches.next().is_none())
            .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
        if upgraded[end_index] {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        source_ends[end_index] = SourceOverlapEnd {
            operand: 0,
            boundary: boundaries[0],
            peer_boundary: Some(boundaries[1]),
        };
        upgraded[end_index] = true;
    }
    let shared_end_count = source_ends
        .iter()
        .filter(|end| end.contributor_count() == 2)
        .count();
    if shared_end_count > 0
        && (graph.branches().len() != 4 + shared_end_count
            || graph.curve_fragments().len() != 4 + shared_end_count)
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    Ok(source_ends)
}

pub(crate) fn certify_coincident_cap_relation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    source_ends: [SourceOverlapEnd; 2],
) -> core::result::Result<CertifiedParallelCylinderCoincidentCapRelation, ParallelCylinderRelationGap>
{
    let shared_end_count = source_ends
        .iter()
        .filter(|end| end.contributor_count() == 2)
        .count();
    if shared_end_count == 0 || graph.completion() != SectionCompletion::Indeterminate {
        return Err(ParallelCylinderRelationGap::SectionIncomplete);
    }
    if graph.branches().len() != 4 + shared_end_count
        || graph.curve_fragments().len() != 4 + shared_end_count
        || graph.curve_endpoints().len() != 4
        || !graph.curve_components().is_empty()
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    certify_expected_gaps(graph, cylinders, source_ends)?;

    let mut covered_branches = vec![false; graph.branches().len()];
    let mut rulings = Vec::with_capacity(2);
    let mut cap_arcs: [[Option<PendingCapArc>; 2]; 2] = [[None; 2]; 2];
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch_index = fragment.branch();
        if branch_index >= covered_branches.len()
            || covered_branches[branch_index]
            || fragment.source_ordinal() != 0
        {
            return Err(ParallelCylinderRelationGap::SectionLayout);
        }
        covered_branches[branch_index] = true;
        let branch = &graph.branches()[branch_index];
        match fragment.span() {
            SectionCurveFragmentSpan::LineSegment { endpoints } => {
                rulings.push(certify_coincident_ruling(
                    graph,
                    cylinders,
                    source_ends,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?);
            }
            SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                let arc = certify_cap_arc(
                    graph,
                    cylinders,
                    source_ends,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?;
                if source_ends[arc.overlap_end]
                    .boundary_for(arc.cap_operand)
                    .is_none()
                    || cap_arcs[arc.overlap_end][arc.cap_operand]
                        .replace(arc)
                        .is_some()
                {
                    return Err(ParallelCylinderRelationGap::SectionLayout);
                }
            }
            SectionCurveFragmentSpan::Whole
            | SectionCurveFragmentSpan::BoundedProcedural { .. } => {
                return Err(ParallelCylinderRelationGap::SectionLayout);
            }
        }
    }
    if rulings.len() != 2
        || source_ends.iter().enumerate().any(|(end, source)| {
            (0..2).any(|operand| {
                cap_arcs[end][operand].is_some() != source.boundary_for(operand).is_some()
            })
        })
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    if covered_branches.into_iter().any(|covered| !covered) {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    certify_periodic_bindings(graph, cylinders, &rulings, &cap_arcs)?;

    let [first, second] = rulings.as_slice() else {
        unreachable!("the ruling count was checked")
    };
    let mut pending_rulings = [*first, *second];
    pending_rulings.sort_by_key(|ruling| {
        let ends = ends_by_physical_end(ruling.endpoints);
        [ends[0].endpoint, ends[1].endpoint]
    });
    let rulings = pending_rulings.map(|ruling| {
        let ends = ends_by_physical_end(ruling.endpoints);
        ParallelCylinderCoincidentCapRulingWitness {
            branch: ruling.branch,
            fragment: ruling.fragment,
            endpoints: [ends[0].endpoint, ends[1].endpoint],
        }
    });

    certify_endpoint_incidence(graph, source_ends, &pending_rulings, &cap_arcs)?;
    let overlap_ends = core::array::from_fn(|end| {
        let cap_arc = (source_ends[end].contributor_count() == 1)
            .then(|| cap_arcs[end].iter().flatten().copied().next())
            .flatten();
        build_end_witness(graph, cylinders, source_ends, end, &rulings, cap_arc)
    });
    let [low, high] = overlap_ends;
    Ok(CertifiedParallelCylinderCoincidentCapRelation {
        overlap_ends: [low?, high?],
        rulings,
    })
}

fn certify_periodic_bindings(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    rulings: &[PendingCoincidentRuling],
    cap_arcs: &[[Option<PendingCapArc>; 2]; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    if graph.periodic_face_embeddings().len() != 2 {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let mut seen = [false; 2];
    for evidence in graph.periodic_face_embeddings() {
        let SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
            operand,
            face,
            gap: SectionPeriodicEmbeddingGap::UnstitchedFragmentPath { fragment },
        } = evidence
        else {
            return Err(ParallelCylinderRelationGap::SectionOperandBinding);
        };
        let operand = *operand;
        if operand >= 2
            || seen[operand]
            || face.raw() != cylinders[operand].side_face()
            || !(rulings.iter().any(|ruling| ruling.fragment == *fragment)
                || cap_arcs
                    .iter()
                    .flatten()
                    .flatten()
                    .any(|arc| arc.cap_operand == 1 - operand && arc.fragment == *fragment))
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

fn certify_expected_gaps(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    source_ends: [SourceOverlapEnd; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    let mut expected = Vec::new();
    for source_end in source_ends {
        if source_end.contributor_count() != 2 {
            continue;
        }
        for cap_operand in 0..2 {
            let boundary = source_end
                .boundary_for(cap_operand)
                .ok_or(ParallelCylinderRelationGap::SectionLayout)?;
            let mut faces = [cylinders[0].side_face(), cylinders[1].side_face()];
            faces[cap_operand] = cylinders[cap_operand].boundaries()[boundary].cap_face();
            expected.push((
                crate::section::GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY,
                faces.to_vec(),
            ));
        }
        let cap_faces = [0, 1].map(|operand| {
            let boundary = source_end.boundary_for(operand).expect("shared end");
            cylinders[operand].boundaries()[boundary].cap_face()
        });
        expected.push((crate::section::GAP_COINCIDENT_FACE_PAIR, cap_faces.to_vec()));
    }
    expected.push((crate::section::GAP_MIXED_FRAGMENT_STITCH, Vec::new()));

    for gap in graph.gaps() {
        let actual_faces = gap
            .faces()
            .iter()
            .map(|face| face.raw())
            .collect::<Vec<_>>();
        let Some(index) = expected.iter().position(|(reason, faces)| {
            *reason == gap.reason() && unordered_faces_equal(faces, &actual_faces)
        }) else {
            return Err(ParallelCylinderRelationGap::SectionIncomplete);
        };
        expected.remove(index);
    }
    if expected.is_empty() {
        Ok(())
    } else {
        Err(ParallelCylinderRelationGap::SectionIncomplete)
    }
}

fn unordered_faces_equal(expected: &[RawFaceId], actual: &[RawFaceId]) -> bool {
    if expected.len() != actual.len() {
        return false;
    }
    let mut matched = vec![false; actual.len()];
    expected.iter().all(|face| {
        actual
            .iter()
            .enumerate()
            .find(|(index, candidate)| !matched[*index] && *candidate == face)
            .is_some_and(|(index, _)| {
                matched[index] = true;
                true
            })
    })
}

#[allow(clippy::too_many_arguments)]
fn certify_coincident_ruling(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    source_ends: [SourceOverlapEnd; 2],
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionRulingFragmentEnd; 2],
) -> core::result::Result<PendingCoincidentRuling, ParallelCylinderRelationGap> {
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
        || (!vectors_are_exactly_parallel(direction, cylinders[0].cylinder().frame().z())
            && !has_certified_axial_cylinder_traces(branch))
        || !valid_branch_evidence(branch)
    {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    }
    let mut bound = [None, None];
    for end in endpoints.iter() {
        if !finite_vec3(end.point()) || !end.carrier_parameter().is_finite() {
            return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
        }
        let mut matching_end = None;
        for (overlap_end, source_end) in source_ends.iter().copied().enumerate() {
            if ruling_endpoint_matches_source_end(graph, cylinders, source_end, branch, end)? {
                if matching_end.replace(overlap_end).is_some() {
                    return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
                }
            }
        }
        let overlap_end =
            matching_end.ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
        if bound[overlap_end]
            .replace(BoundCoincidentEndpoint {
                endpoint: end.endpoint(),
                overlap_end,
            })
            .is_some()
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    }
    let [Some(low), Some(high)] = bound else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    Ok(PendingCoincidentRuling {
        branch: branch_index,
        fragment: fragment_index,
        endpoints: [low, high],
    })
}

fn has_certified_axial_cylinder_traces(branch: &SectionBranch) -> bool {
    branch.pcurves().iter().all(|trace| {
        let SectionUvCurve::Line(line) = trace else {
            return false;
        };
        let origin = line.origin();
        let direction = line.direction();
        origin.x.is_finite()
            && origin.y.is_finite()
            && direction.x == 0.0
            && direction.y.is_finite()
            && direction.y != 0.0
    })
}

fn ruling_endpoint_matches_source_end(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    source_end: SourceOverlapEnd,
    branch: &SectionBranch,
    end: &crate::SectionRulingFragmentEnd,
) -> core::result::Result<bool, ParallelCylinderRelationGap> {
    let endpoint = graph
        .curve_endpoints()
        .get(end.endpoint())
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    for operand in 0..2 {
        match source_end.boundary_for(operand) {
            Some(boundary) => {
                let Some(trim) = &end.trims()[operand] else {
                    return Ok(false);
                };
                let expected_edge = cylinders[operand].boundaries()[boundary].edge();
                let common = endpoint.edge_parameters()[operand]
                    .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
                let root = trim.source_parameter();
                let enclosure = root.root_parameter_enclosure();
                if trim.operand() != operand
                    || trim.face().raw() != cylinders[operand].side_face()
                    || root.edge().raw() != expected_edge
                    || root.root_ordinal() >= 2
                    || !root.root_parameter().is_finite()
                    || !valid_interval(enclosure.lo(), enclosure.hi())
                    || !enclosure.contains(root.root_parameter())
                    || !valid_interval(trim.edge_parameter().lo(), trim.edge_parameter().hi())
                    || !valid_interval(common.lo(), common.hi())
                    || common.lo() < trim.edge_parameter().lo()
                    || common.hi() > trim.edge_parameter().hi()
                    || !matches!(
                        &sites[operand],
                        SectionSite::EdgeInterior(edge) if edge.raw() == expected_edge
                    )
                    || source_parameters[operand].as_ref() != Some(root)
                    || branch.faces()[operand].raw() != cylinders[operand].side_face()
                {
                    return Ok(false);
                }
            }
            None => {
                if end.trims()[operand].is_some()
                    || source_parameters[operand].is_some()
                    || endpoint.edge_parameters()[operand].is_some()
                    || !matches!(
                        &sites[operand],
                        SectionSite::FaceInterior(face)
                            if face.raw() == cylinders[operand].side_face()
                    )
                {
                    return Ok(false);
                }
            }
        }
    }
    Ok(true)
}

fn build_end_witness(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    source_ends: [SourceOverlapEnd; 2],
    overlap_end: usize,
    rulings: &[ParallelCylinderCoincidentCapRulingWitness; 2],
    cap_arc: Option<PendingCapArc>,
) -> core::result::Result<ParallelCylinderCoincidentCapEndWitness, ParallelCylinderRelationGap> {
    let source_end = source_ends[overlap_end];
    let mut sources = [None, None];
    for operand in 0..2 {
        let Some(boundary) = source_end.boundary_for(operand) else {
            continue;
        };
        let expected_edge = cylinders[operand].boundaries()[boundary].edge();
        let mut roots = Vec::with_capacity(2);
        for ruling in rulings {
            let endpoint_index = ruling.endpoints[overlap_end];
            let endpoint = graph
                .curve_endpoints()
                .get(endpoint_index)
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            let SectionCurveEndpointTopology::Trim {
                source_parameters, ..
            } = endpoint.topology()
            else {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            };
            let root = source_parameters[operand]
                .as_ref()
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            let enclosure = root.root_parameter_enclosure();
            if root.edge().raw() != expected_edge
                || root.root_ordinal() >= 2
                || !root.root_parameter().is_finite()
                || !valid_interval(enclosure.lo(), enclosure.hi())
                || !enclosure.contains(root.root_parameter())
            {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
            roots.push(ParallelCylinderSourceRootWitness {
                endpoint: endpoint_index,
                root_ordinal: root.root_ordinal(),
                parameter_bits: root.root_parameter().to_bits(),
                enclosure_bits: [enclosure.lo().to_bits(), enclosure.hi().to_bits()],
            });
        }
        roots.sort_by_key(|root| root.root_ordinal);
        let [first, second] = roots.as_slice() else {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        };
        if first.root_ordinal != 0 || second.root_ordinal != 1 || first.endpoint == second.endpoint
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        sources[operand] = Some(ParallelCylinderSourceArcWitness {
            operand,
            boundary,
            cap_face: cylinders[operand].boundaries()[boundary].cap_face(),
            edge: expected_edge,
            roots: [*first, *second],
        });
    }
    let cap_arc = cap_arc.map(|arc| ParallelCylinderSectionCapArcWitness {
        branch: arc.branch,
        fragment: arc.fragment,
        endpoints: arc.ends.map(|end| end.endpoint),
    });
    let source_count = sources.iter().flatten().count();
    if source_count + usize::from(cap_arc.is_some()) != 2 {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    if let (Some(source), Some(arc)) = (sources.iter().flatten().next(), cap_arc) {
        let expected = source.roots.map(|root| (root.endpoint, root.root_ordinal));
        let mut actual = cap_arc_root_pairs(cap_arcs_from_witness(arc, graph)?);
        actual.sort_unstable();
        let mut expected = expected;
        expected.sort_unstable();
        if actual != expected {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    }
    Ok(ParallelCylinderCoincidentCapEndWitness { sources, cap_arc })
}

fn cap_arcs_from_witness(
    witness: ParallelCylinderSectionCapArcWitness,
    graph: &BodySectionGraph,
) -> core::result::Result<[BoundEndpoint; 2], ParallelCylinderRelationGap> {
    let fragment = graph
        .curve_fragments()
        .get(witness.fragment)
        .ok_or(ParallelCylinderRelationGap::SectionLayout)?;
    let SectionCurveFragmentSpan::Arc { endpoints, .. } = fragment.span() else {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    };
    Ok(endpoints.each_ref().map(|end| BoundEndpoint {
        endpoint: end.endpoint(),
        overlap_end: usize::MAX,
        root_ordinal: end.trim().source_parameter().root_ordinal(),
    }))
}

fn cap_arc_root_pairs(ends: [BoundEndpoint; 2]) -> [(usize, usize); 2] {
    ends.map(|end| (end.endpoint, end.root_ordinal))
}

fn certify_endpoint_incidence(
    graph: &BodySectionGraph,
    source_ends: [SourceOverlapEnd; 2],
    rulings: &[PendingCoincidentRuling; 2],
    cap_arcs: &[[Option<PendingCapArc>; 2]; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    let mut incidence = [0_u8; 4];
    for ruling in rulings {
        for end in ruling.endpoints {
            let slot = incidence
                .get_mut(end.endpoint)
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            *slot = slot
                .checked_add(1)
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
        }
    }
    for arc in cap_arcs.iter().flatten().flatten() {
        for end in arc.ends {
            let slot = incidence
                .get_mut(end.endpoint)
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            *slot = slot
                .checked_add(1)
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
        }
    }
    for ruling in rulings {
        for end in ruling.endpoints {
            let expected = 1_u8
                .checked_add(source_ends[end.overlap_end].contributor_count() as u8)
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            if incidence[end.endpoint] != expected {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
        }
    }
    if graph.curve_endpoints().len() != incidence.len()
        || incidence.into_iter().any(|count| count == 0)
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    Ok(())
}

fn ends_by_physical_end(ends: [BoundCoincidentEndpoint; 2]) -> [BoundCoincidentEndpoint; 2] {
    if ends[0].overlap_end == 0 {
        ends
    } else {
        [ends[1], ends[0]]
    }
}
