//! Bounded procedural Section fragments for certified skew-cylinder spans.
//!
//! Graph endpoints delimit the residual-certified interior by representable
//! guard parameters. Physical topology is owned separately by an exact axial
//! root mapped onto one topology-certified source cap ring. This module keeps
//! both pieces of evidence and resolves the ring root through the same
//! operation-local source-root authority used by adjacent ruling fragments.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kgeom::curve::Curve;
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    PairedSkewCylinderBranchResidualCertificate, SkewCylinderBranchGuardedEnd,
    SkewCylinderBranchPcurveRootCorridorCertificate,
};
use kops::intersect::{
    IntersectionBranchEndpointProof, SkewCylinderAxialBoundaryProof,
    SkewCylinderHalfAngleChartProof, SkewCylinderRootInsideSideProof,
};
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, LoopId as RawLoopId,
};
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;

use super::root_identity::{
    CertifiedSourceRootScalar, RootIdentityAuthority, RootOrderOutcome, RootResolution,
    SourceRootKey, SourceRootQuery,
};
use super::source_annulus::{self, CertifiedSourceAnnulus};
use super::*;

// Pair-atomic staging temporarily owns several kilobyte proof values per
// branch. Indirection keeps the staging discriminant compact; commit moves
// each value into its final accumulator vector without cloning the proofs.
enum PreparedSkewBranch {
    Whole(Box<PreparedWholeSkewBranch>),
    Open(Box<PreparedOpenSkewBranch>),
}

struct PreparedWholeSkewBranch {
    branch: SectionBranch,
    fragment: closed_stitch::ClosedCurveFragment,
    evidence: ClosedFragmentEvidence,
}

struct PreparedOpenSkewBranch {
    branch: SectionBranch,
    fragment: CertifiedBoundedSkewCylinderFragment,
}

enum PreparedSkewFacePair {
    Ready(Vec<PreparedSkewBranch>),
    Gap(&'static str),
}

/// Publish all graph branches for one Cylinder/Cylinder face pair atomically.
///
/// Parallel rulings retain their established per-branch clipper. Once any
/// skew certificate appears, however, every branch, annulus, physical root,
/// and orientation is prepared before a single accumulator vector changes.
#[allow(clippy::too_many_arguments)]
pub(super) fn append_face_pair_branches(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    edges: &[IntersectionBranchEdge],
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    linear: f64,
    root_identity: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    let has_skew = edges.iter().any(|edge| {
        edge.certificate.as_skew_cylinder_two_sheet().is_some()
            || edge.certificate.as_skew_cylinder_open_span().is_some()
    });
    if !has_skew {
        for edge in edges {
            super::cylinder_cylinder_publish::append_branch(
                store,
                raw_faces,
                facades,
                edge,
                vertices,
                surfaces,
                senses,
                root_identity,
                scope,
                acc,
            )?;
        }
        return Ok(());
    }
    if !edges.iter().all(|edge| {
        edge.certificate.as_skew_cylinder_two_sheet().is_some()
            || edge.certificate.as_skew_cylinder_open_span().is_some()
    }) {
        acc.gaps.push(SectionGap {
            reason: GAP_PAIR_UNRESOLVED,
            faces: facades.to_vec(),
        });
        return Ok(());
    }

    // Evaluate both faces even after one semantic refusal so operation work is
    // independent of operand order and malformed-face position.
    let first_annulus =
        source_annulus::certify_source_annulus_window_in_scope(store, &facades[0], linear, scope)?;
    let second_annulus =
        source_annulus::certify_source_annulus_window_in_scope(store, &facades[1], linear, scope)?;
    let (Some(first_annulus), Some(second_annulus)) = (first_annulus, second_annulus) else {
        acc.gaps.push(SectionGap {
            reason: GAP_SKEW_CYLINDER_WHOLE_BAND_UNPROVEN,
            faces: facades.to_vec(),
        });
        return Ok(());
    };
    let annuli = [first_annulus, second_annulus];

    let prepared = prepare_skew_face_pair(
        store,
        raw_faces,
        facades,
        edges,
        vertices,
        surfaces,
        senses,
        &annuli,
        acc.branches.len(),
        root_identity,
        scope,
    )?;
    match prepared {
        PreparedSkewFacePair::Ready(prepared) => commit_prepared_branches(prepared, acc),
        PreparedSkewFacePair::Gap(reason) => acc.gaps.push(SectionGap {
            reason,
            faces: facades.to_vec(),
        }),
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn prepare_skew_face_pair(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    edges: &[IntersectionBranchEdge],
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    annuli: &[CertifiedSourceAnnulus; 2],
    base_branch: usize,
    root_identity: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedSkewFacePair> {
    let mut prepared = Vec::with_capacity(edges.len());
    for edge in edges {
        let branch_index = base_branch + prepared.len();
        if let Some(certificate) = edge.certificate.as_skew_cylinder_two_sheet() {
            let branch = match super::cylinder_cylinder_publish::adapt_skew_two_sheet_branch(
                facades,
                edge,
                vertices,
                surfaces,
                senses,
                certificate,
            ) {
                super::cylinder_cylinder_publish::CylinderCylinderBranchAdaptation::Adapted(
                    branch,
                ) => branch,
                super::cylinder_cylinder_publish::CylinderCylinderBranchAdaptation::OrientationIndeterminate => return Ok(PreparedSkewFacePair::Gap(GAP_CARRIER_ORIENTATION)),
                super::cylinder_cylinder_publish::CylinderCylinderBranchAdaptation::Unsupported => {
                    return Ok(PreparedSkewFacePair::Gap(GAP_PAIR_UNRESOLVED));
                }
            };
            let Some((fragment, evidence)) = prepare_whole_closed_branch(branch_index, &branch)
            else {
                return Ok(PreparedSkewFacePair::Gap(GAP_CLOSED_STITCH));
            };
            prepared.push(PreparedSkewBranch::Whole(Box::new(
                PreparedWholeSkewBranch {
                    branch: *branch,
                    fragment,
                    evidence,
                },
            )));
            continue;
        }
        match prepare_open_span(
            store,
            raw_faces,
            facades,
            edge,
            vertices,
            surfaces,
            senses,
            annuli,
            root_identity,
            scope,
        )? {
            OpenSkewCylinderAdaptation::Prepared(open) => {
                let (branch, fragment) = open.into_parts(branch_index);
                prepared.push(PreparedSkewBranch::Open(Box::new(PreparedOpenSkewBranch {
                    branch,
                    fragment,
                })));
            }
            OpenSkewCylinderAdaptation::OrientationIndeterminate => {
                return Ok(PreparedSkewFacePair::Gap(GAP_CARRIER_ORIENTATION));
            }
            OpenSkewCylinderAdaptation::Unproven => {
                return Ok(PreparedSkewFacePair::Gap(
                    GAP_SKEW_CYLINDER_OPEN_SPAN_CAP_ROOT_UNPROVEN,
                ));
            }
        }
    }

    Ok(PreparedSkewFacePair::Ready(prepared))
}

fn commit_prepared_branches(prepared: Vec<PreparedSkewBranch>, acc: &mut SectionAccumulator) {
    for branch in prepared {
        match branch {
            PreparedSkewBranch::Whole(prepared) => {
                let PreparedWholeSkewBranch {
                    branch,
                    fragment,
                    evidence,
                } = *prepared;
                acc.branches.push(branch);
                acc.closed_fragments.push(fragment);
                acc.closed_fragment_evidence.push(evidence);
            }
            PreparedSkewBranch::Open(prepared) => {
                let PreparedOpenSkewBranch { branch, fragment } = *prepared;
                acc.branches.push(branch);
                acc.bounded_procedural_fragments.push(fragment);
            }
        }
    }
}

fn prepare_whole_closed_branch(
    branch_index: usize,
    branch: &SectionBranch,
) -> Option<(closed_stitch::ClosedCurveFragment, ClosedFragmentEvidence)> {
    let source = closed_stitch::ClosedBranchSource::from_section_branch(branch_index, branch)?;
    Some((
        closed_stitch::ClosedCurveFragment {
            source: source.fragment(0),
            orientation: closed_stitch::ClosedFragmentOrientation::AlongCarrier,
            span: closed_stitch::ClosedFragmentSpan::Whole,
        },
        ClosedFragmentEvidence {
            branch: branch_index,
            ordinal: 0,
            span: ClosedFragmentEvidenceSpan::Whole,
        },
    ))
}

#[derive(Debug, Clone)]
pub(super) struct CertifiedBoundedSkewCylinderFragment {
    pub(super) branch: usize,
    pub(super) ordinal: usize,
    endpoints: [CertifiedBoundedSkewCylinderEnd; 2],
}

#[derive(Debug, Clone)]
struct CertifiedBoundedSkewCylinderEnd {
    source_operand: usize,
    source_face: RawFaceId,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    root: SourceRootKey,
    source_root_scalar: CertifiedSourceRootScalar,
    edge_parameter: Interval,
    authored_height: f64,
    root_point: Point3,
    inside_point: Point3,
    inside_carrier_parameter: f64,
    carrier_root: CertifiedCarrierRoot,
}

#[derive(Debug, Clone, Copy)]
struct CertifiedCarrierRoot {
    chart: SkewCylinderHalfAngleChartProof,
    projective: Interval,
}

pub(super) struct PreparedOpenSkewCylinderSpan {
    pub(super) branch: SectionBranch,
    endpoints: [CertifiedBoundedSkewCylinderEnd; 2],
}

impl PreparedOpenSkewCylinderSpan {
    pub(super) fn into_parts(
        self,
        branch: usize,
    ) -> (SectionBranch, CertifiedBoundedSkewCylinderFragment) {
        (
            self.branch,
            CertifiedBoundedSkewCylinderFragment {
                branch,
                ordinal: 0,
                endpoints: self.endpoints,
            },
        )
    }
}

pub(super) enum OpenSkewCylinderAdaptation {
    Prepared(Box<PreparedOpenSkewCylinderSpan>),
    OrientationIndeterminate,
    Unproven,
}

/// Adapt and source-root-certify one graph-owned non-wrapping open span.
#[allow(clippy::too_many_arguments)]
pub(super) fn prepare_open_span(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    annuli: &[CertifiedSourceAnnulus; 2],
    roots: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
) -> Result<OpenSkewCylinderAdaptation> {
    let Some(open_certificate) = edge.certificate.as_skew_cylinder_open_span_branch() else {
        return Ok(OpenSkewCylinderAdaptation::Unproven);
    };
    let certificate = open_certificate.residual_certificate();
    let Some((carrier, source_pcurves)) = validate_open_graph_edge(edge, certificate) else {
        return Ok(OpenSkewCylinderAdaptation::Unproven);
    };
    let range = edge.carrier_range;
    let graph_start = carrier.eval_derivs(range.lo, 1);
    let Some(flipped) = super::cylinder_cylinder_publish::canonical_flip(
        surfaces[0],
        senses[0],
        surfaces[1],
        senses[1],
        graph_start.d[0],
        graph_start.d[1],
    ) else {
        return Ok(OpenSkewCylinderAdaptation::OrientationIndeterminate);
    };

    let section_carrier = SectionSkewCylinderBranchCarrier::new(carrier, range, flipped);
    let pcurves =
        source_pcurves.map(|pcurve| SectionSkewCylinderBranchPcurve::new(pcurve, range, flipped));
    let Some(embedding_certificate) =
        SectionSkewCylinderEmbeddingCertificate::new(open_certificate, range, flipped)
    else {
        return Ok(OpenSkewCylinderAdaptation::Unproven);
    };
    let section_start = section_carrier.eval_derivs(range.lo, 1);
    if !finite_point_and_tangent(section_start.d[0], section_start.d[1])
        || super::cylinder_cylinder_publish::canonical_flip(
            surfaces[0],
            senses[0],
            surfaces[1],
            senses[1],
            section_start.d[0],
            section_start.d[1],
        ) != Some(false)
    {
        return Ok(OpenSkewCylinderAdaptation::OrientationIndeterminate);
    }

    let graph_slots = if flipped { [1, 0] } else { [0, 1] };
    let mut fragment_sites = Vec::with_capacity(2);
    let mut prepared_ends = Vec::with_capacity(2);
    for (section_slot, graph_slot) in graph_slots.into_iter().enumerate() {
        let Some(vertex) = vertices.get(edge.endpoint_vertices[graph_slot]).copied() else {
            return Ok(OpenSkewCylinderAdaptation::Unproven);
        };
        let IntersectionBranchVertexEvent::BoundaryEndpoint {
            surfaces: boundary_surfaces,
        } = vertex.event
        else {
            return Ok(OpenSkewCylinderAdaptation::Unproven);
        };
        let Some(IntersectionBranchEndpointProof::SkewCylinderAxialRoot(proof)) =
            edge.endpoint_proofs[graph_slot]
        else {
            return Ok(OpenSkewCylinderAdaptation::Unproven);
        };
        let graph_parameter = if graph_slot == 0 { range.lo } else { range.hi };
        if proof.source_operand > 1
            || proof.sheet != certificate.sheet()
            || proof.inside_parameter.to_bits() != graph_parameter.to_bits()
            || boundary_surfaces != core::array::from_fn(|operand| operand == proof.source_operand)
        {
            return Ok(OpenSkewCylinderAdaptation::Unproven);
        }
        let section_parameter = if section_slot == 0 {
            range.lo
        } else {
            range.hi
        };
        let section_point = section_carrier.eval(section_parameter);
        if vertex.point != section_point {
            return Ok(OpenSkewCylinderAdaptation::Unproven);
        }
        let Some(end) = certify_endpoint(
            store,
            raw_faces,
            proof,
            vertex.point,
            section_parameter,
            certificate,
            open_certificate.root_corridors()[graph_slot],
            graph_slot,
            annuli,
            roots,
            scope,
        )?
        else {
            return Ok(OpenSkewCylinderAdaptation::Unproven);
        };
        fragment_sites.push(SectionFragmentSite {
            point: vertex.point,
            surface_parameters: vertex.surface_parameters,
            surface_window_boundaries: boundary_surfaces,
        });
        prepared_ends.push(end);
    }
    let Ok([start, end]) = <Vec<CertifiedBoundedSkewCylinderEnd> as TryInto<
        [CertifiedBoundedSkewCylinderEnd; 2],
    >>::try_into(prepared_ends) else {
        unreachable!("both graph endpoints were visited")
    };
    let Ok([low_site, high_site]) =
        <Vec<SectionFragmentSite> as TryInto<[SectionFragmentSite; 2]>>::try_into(fragment_sites)
    else {
        unreachable!("both graph endpoint sites were visited")
    };

    Ok(OpenSkewCylinderAdaptation::Prepared(Box::new(
        PreparedOpenSkewCylinderSpan {
            branch: SectionBranch {
                faces: facades.clone(),
                carrier: SectionCarrier::SkewCylinderBranch(section_carrier),
                range,
                topology: SectionBranchTopology::Open,
                pcurves: pcurves.map(SectionUvCurve::SkewCylinderBranch),
                fragment_sites: vec![low_site, high_site],
                endpoint_sites: [0, 1],
                evidence: SectionBranchEvidence {
                    residual_bounds: certificate.residual_bounds(),
                    tolerance: certificate.tolerance(),
                },
                skew_cylinder_embedding: Some(Box::new(embedding_certificate)),
                ruling_recertification: None,
                ruling_parameter_flipped: false,
            },
            endpoints: [start, end],
        },
    )))
}

fn validate_open_graph_edge(
    edge: &IntersectionBranchEdge,
    certificate: PairedSkewCylinderBranchResidualCertificate,
) -> Option<(
    kgraph::SkewCylinderBranchCarrier,
    [kgraph::SkewCylinderBranchPcurve; 2],
)> {
    let carrier = edge.carrier.as_skew_cylinder_branch().copied()?;
    let source_pcurves = [
        edge.pcurves[0].as_skew_cylinder_branch().copied()?,
        edge.pcurves[1].as_skew_cylinder_branch().copied()?,
    ];
    let traces = certificate.traces();
    (edge.kind == ContactKind::Transverse
        && edge.topology == IntersectionBranchTopology::Open
        && edge.endpoint_vertices[0] != edge.endpoint_vertices[1]
        && edge.carrier_range == certificate.carrier_range()
        && edge.carrier_range.is_finite()
        && edge.carrier_range.width() < core::f64::consts::TAU
        && carrier == certificate.carrier()
        && carrier.sheet() == certificate.sheet()
        && source_pcurves == traces.map(|trace| trace.pcurve())
        && edge.parameter_maps == certificate.parameter_maps())
    .then_some((carrier, source_pcurves))
}

#[allow(clippy::too_many_arguments)]
fn certify_endpoint(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    proof: kops::intersect::SkewCylinderAxialRootEndpointProof,
    inside_point: Point3,
    inside_carrier_parameter: f64,
    certificate: PairedSkewCylinderBranchResidualCertificate,
    root_corridor: SkewCylinderBranchPcurveRootCorridorCertificate,
    graph_slot: usize,
    annuli: &[CertifiedSourceAnnulus; 2],
    roots: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<CertifiedBoundedSkewCylinderEnd>> {
    let source_operand = proof.source_operand;
    let traces = certificate.traces();
    if source_operand > 1
        || traces[source_operand].pcurve().operand() != 0
        || traces[source_operand].surface() != certificate.carrier().cylinders()[0]
    {
        return Ok(None);
    }
    if !root_corridor_matches_endpoint(
        proof,
        root_corridor,
        graph_slot,
        certificate.carrier_range(),
        traces.map(|trace| trace.pcurve().operand()),
    ) {
        return Ok(None);
    }
    let ring = match proof.boundary {
        SkewCylinderAxialBoundaryProof::Lower => annuli[source_operand].lower(),
        SkewCylinderAxialBoundaryProof::Upper => annuli[source_operand].upper(),
    };
    if ring.face() != raw_faces[source_operand]
        || ring.authored_height().to_bits() != proof.bound.to_bits()
    {
        return Ok(None);
    }
    let Some(carrier_longitude) = carrier_longitude_enclosure(proof) else {
        return Ok(None);
    };
    let Some(observed_edge_parameter) =
        ring.intrinsic_edge_parameter_for_longitude(carrier_longitude)
    else {
        return Ok(None);
    };
    let query = SourceRootQuery::new(ring.edge(), raw_faces[1 - source_operand]);
    let root = match roots.resolve(store, query, observed_edge_parameter, scope)? {
        RootResolution::Resolved(root) => root,
        RootResolution::Indeterminate(_) => return Ok(None),
    };
    let (source_root, source_root_scalar) = match roots.certify_order(store, query, scope)? {
        RootOrderOutcome::Certified(order) => {
            let Some(source_root) = order.roots().get(root.ordinal()).copied() else {
                return Err(inconsistent_topology(
                    "skew source-root ordinal escaped its certified order",
                ));
            };
            let Some(scalar) = order.materialize(root) else {
                return Err(inconsistent_topology(
                    "skew source root has no canonical scalar materialization",
                ));
            };
            (source_root, scalar)
        }
        RootOrderOutcome::Indeterminate(_) => return Ok(None),
    };
    let edge_parameter = Interval::new(
        observed_edge_parameter.lo().min(source_root.lo()),
        observed_edge_parameter.hi().max(source_root.hi()),
    );
    let root_point = ring.circle().eval(source_root_scalar.parameter());
    if !finite_point(root_point)
        || !finite_point(inside_point)
        || !inside_carrier_parameter.is_finite()
    {
        return Ok(None);
    }
    Ok(Some(CertifiedBoundedSkewCylinderEnd {
        source_operand,
        source_face: raw_faces[source_operand],
        loop_id: ring.loop_id(),
        fin: ring.fin(),
        edge: ring.edge(),
        root,
        source_root_scalar,
        edge_parameter,
        authored_height: proof.bound,
        root_point,
        inside_point,
        inside_carrier_parameter,
        carrier_root: CertifiedCarrierRoot {
            chart: proof.half_angle_chart,
            projective: Interval::new(proof.half_angle_bracket[0], proof.half_angle_bracket[1]),
        },
    }))
}

fn root_corridor_matches_endpoint(
    proof: kops::intersect::SkewCylinderAxialRootEndpointProof,
    corridor: SkewCylinderBranchPcurveRootCorridorCertificate,
    graph_slot: usize,
    range: kgeom::param::ParamRange,
    trace_operands: [usize; 2],
) -> bool {
    let expected_end = match graph_slot {
        0 => SkewCylinderBranchGuardedEnd::Lower,
        1 => SkewCylinderBranchGuardedEnd::Upper,
        _ => return false,
    };
    let expected_inside_side = match graph_slot {
        0 => SkewCylinderRootInsideSideProof::After,
        1 => SkewCylinderRootInsideSideProof::Before,
        _ => return false,
    };
    let expected_guard = if graph_slot == 0 { range.lo } else { range.hi };
    let corridor_cell = corridor.corridor();
    let root_pcurves = corridor.root_pcurves();
    let corridor_pcurves = corridor_cell.pcurves();
    if corridor.guarded_end() != expected_end
        || proof.inside_side != expected_inside_side
        || proof.inside_parameter.to_bits() != expected_guard.to_bits()
        || root_pcurves.map(|pcurve| pcurve.operand()) != trace_operands
        || corridor_pcurves.map(|pcurve| pcurve.operand()) != trace_operands
        || proof.source_operand > 1
    {
        return false;
    }
    let root_height = root_pcurves[proof.source_operand];
    if !root_height.stored_uv()[1].contains(proof.bound)
        || !root_height.source_uv()[1].contains(proof.bound)
    {
        return false;
    }
    let Some(projective_root) = exact_projective_root_longitude(proof) else {
        return false;
    };
    periodic_root_interval_matches(projective_root, corridor.root_parameter())
}

fn exact_projective_root_longitude(
    proof: kops::intersect::SkewCylinderAxialRootEndpointProof,
) -> Option<Interval> {
    exact_chart_root_longitude(
        proof.half_angle_chart,
        proof.half_angle_bracket[0],
        proof.half_angle_bracket[1],
    )
}

fn exact_chart_root_longitude(
    chart: SkewCylinderHalfAngleChartProof,
    lo: f64,
    hi: f64,
) -> Option<Interval> {
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return None;
    }
    let first = canonical_angle(match chart {
        SkewCylinderHalfAngleChartProof::Tangent => 2.0 * kcore::math::atan2(lo, 1.0),
        SkewCylinderHalfAngleChartProof::Cotangent => 2.0 * kcore::math::atan2(1.0, lo),
    });
    let second = canonical_angle(match chart {
        SkewCylinderHalfAngleChartProof::Tangent => 2.0 * kcore::math::atan2(hi, 1.0),
        SkewCylinderHalfAngleChartProof::Cotangent => 2.0 * kcore::math::atan2(1.0, hi),
    });
    let [angular_lo, angular_hi] = match chart {
        SkewCylinderHalfAngleChartProof::Tangent => [first, second],
        SkewCylinderHalfAngleChartProof::Cotangent => [second, first],
    };
    (angular_lo.is_finite() && angular_hi.is_finite() && angular_lo <= angular_hi)
        .then_some(Interval::new(angular_lo, angular_hi))
}

fn periodic_root_interval_matches(canonical: Interval, lifted: Interval) -> bool {
    const PERIOD: f64 = core::f64::consts::TAU;
    let turns = ((lifted.lo() - canonical.lo()) / PERIOD).round();
    if !turns.is_finite() {
        return false;
    }
    let shift = turns * PERIOD;
    let expected = Interval::new(canonical.lo() + shift, canonical.hi() + shift);
    expected == lifted
}

fn canonical_angle(parameter: f64) -> f64 {
    let mut parameter = parameter % core::f64::consts::TAU;
    if parameter < 0.0 {
        parameter += core::f64::consts::TAU;
    }
    if parameter == core::f64::consts::TAU || parameter == -0.0 {
        0.0
    } else {
        parameter
    }
}

fn carrier_longitude_enclosure(
    proof: kops::intersect::SkewCylinderAxialRootEndpointProof,
) -> Option<Interval> {
    let projective = Interval::new(proof.half_angle_bracket[0], proof.half_angle_bracket[1]);
    if !finite_interval(projective) || projective.lo() > projective.hi() {
        return None;
    }
    let principal = super::root_identity::twice_atan_interval(projective).ok()?;
    let angle = match proof.half_angle_chart {
        SkewCylinderHalfAngleChartProof::Tangent if principal.hi() < 0.0 => {
            principal + Interval::point(core::f64::consts::TAU)
        }
        SkewCylinderHalfAngleChartProof::Tangent if principal.lo() > 0.0 => principal,
        SkewCylinderHalfAngleChartProof::Tangent => return None,
        SkewCylinderHalfAngleChartProof::Cotangent => {
            Interval::point(core::f64::consts::PI) - principal
        }
    };
    (finite_interval(angle)
        && angle.lo() > 0.0
        && angle.lo() <= angle.hi()
        && angle.hi() < core::f64::consts::TAU)
        .then_some(angle)
}

/// Publish staged fragments and intern their physical roots against rulings.
pub(super) fn publish_fragments(
    part: &PartId,
    branches: &[SectionBranch],
    certified: &[CertifiedBoundedSkewCylinderFragment],
    endpoints: &mut Vec<SectionCurveEndpoint>,
    fragments: &mut Vec<SectionCurveFragment>,
) -> Result<()> {
    for fragment in certified {
        let Some(branch) = branches.get(fragment.branch) else {
            return Err(inconsistent_topology(
                "bounded procedural fragment referenced an unknown branch",
            ));
        };
        if !matches!(branch.carrier, SectionCarrier::SkewCylinderBranch(_)) {
            return Err(inconsistent_topology(
                "bounded procedural fragment referenced a nonprocedural branch",
            ));
        }
        let Some(embedding) = branch.embedding_certificate() else {
            return Err(inconsistent_topology(
                "bounded procedural fragment lost its sealed pcurve embedding",
            ));
        };
        if embedding.range() != branch.range {
            return Err(inconsistent_topology(
                "bounded procedural embedding range diverged from its branch",
            ));
        }
        let mut public_ends = Vec::with_capacity(2);
        for (section_end, evidence) in fragment.endpoints.iter().enumerate() {
            if !published_corridor_matches_end(embedding, section_end, evidence) {
                return Err(inconsistent_topology(
                    "bounded procedural endpoint diverged from its root corridor",
                ));
            }
            let mut sites = [
                SectionSite::FaceInterior(branch.faces[0].clone()),
                SectionSite::FaceInterior(branch.faces[1].clone()),
            ];
            sites[evidence.source_operand] =
                SectionSite::EdgeInterior(EdgeId::new(part.clone(), evidence.edge));
            let mut source_parameters = [None, None];
            source_parameters[evidence.source_operand] =
                Some(SectionSourceParameterKey::from_certified_root(
                    part,
                    evidence.root,
                    evidence.source_root_scalar,
                ));
            let mut edge_parameters = [None, None];
            edge_parameters[evidence.source_operand] = Some(
                SectionEdgeParameterInterval::from_interval(evidence.edge_parameter),
            );
            let candidate = SectionCurveEndpoint {
                topology: SectionCurveEndpointTopology::Trim {
                    sites,
                    source_parameters: source_parameters.clone(),
                },
                edge_parameters,
            };
            let endpoint = super::ruling_publish::intern_endpoint_candidate(candidate, endpoints)?;
            let source_parameter = source_parameters[evidence.source_operand]
                .clone()
                .expect("the source operand was populated");
            let trim = SectionBoundedProceduralTrimProvenance {
                operand: evidence.source_operand,
                face: FaceId::new(part.clone(), evidence.source_face),
                loop_id: LoopId::new(part.clone(), evidence.loop_id),
                fin: FinId::new(part.clone(), evidence.fin),
                source_parameter,
                edge_parameter: SectionEdgeParameterInterval::from_interval(
                    evidence.edge_parameter,
                ),
                carrier_root: SectionSkewCylinderCarrierRootEnclosure {
                    chart: match evidence.carrier_root.chart {
                        SkewCylinderHalfAngleChartProof::Tangent => {
                            SectionSkewCylinderRootChart::TangentHalfAngle
                        }
                        SkewCylinderHalfAngleChartProof::Cotangent => {
                            SectionSkewCylinderRootChart::CotangentHalfAngle
                        }
                    },
                    lo: evidence.carrier_root.projective.lo(),
                    hi: evidence.carrier_root.projective.hi(),
                },
            };
            public_ends.push(SectionBoundedProceduralFragmentEnd {
                endpoint,
                root_point: evidence.root_point,
                inside_point: evidence.inside_point,
                inside_carrier_parameter: evidence.inside_carrier_parameter,
                trim,
            });
        }
        let [start, end] = public_ends.try_into().map_err(|_| {
            inconsistent_topology("bounded procedural fragment did not retain two endpoints")
        })?;
        fragments.push(SectionCurveFragment {
            branch: fragment.branch,
            source_ordinal: fragment.ordinal,
            span: SectionCurveFragmentSpan::BoundedProcedural {
                endpoints: Box::new([start, end]),
            },
        });
    }
    Ok(())
}

fn published_corridor_matches_end(
    embedding: &SectionSkewCylinderEmbeddingCertificate,
    section_end: usize,
    evidence: &CertifiedBoundedSkewCylinderEnd,
) -> bool {
    let Some(source) = embedding.source_root_corridor(section_end) else {
        return false;
    };
    let Some(root) = embedding.root_corridor(section_end) else {
        return false;
    };
    let expected_guard = if section_end == 0 {
        embedding.range().lo
    } else {
        embedding.range().hi
    };
    if evidence.inside_carrier_parameter.to_bits() != expected_guard.to_bits()
        || !root
            .corridor()
            .parameter()
            .contains(evidence.inside_carrier_parameter)
        || root.root_pcurves().iter().any(|pcurve| {
            !pcurve.stored_is_strictly_regular() || !pcurve.source_is_strictly_regular()
        })
    {
        return false;
    }
    let root_pcurves = source.root_pcurves();
    if evidence.source_operand > 1
        || !root_pcurves[evidence.source_operand].stored_uv()[1].contains(evidence.authored_height)
        || !root_pcurves[evidence.source_operand].source_uv()[1].contains(evidence.authored_height)
    {
        return false;
    }
    let Some(projective) = exact_chart_root_longitude(
        evidence.carrier_root.chart,
        evidence.carrier_root.projective.lo(),
        evidence.carrier_root.projective.hi(),
    ) else {
        return false;
    };
    periodic_root_interval_matches(projective, source.root_parameter())
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn finite_point(point: Point3) -> bool {
    point.to_array().into_iter().all(f64::is_finite)
}

fn finite_point_and_tangent(point: Point3, tangent: Vec3) -> bool {
    point
        .to_array()
        .into_iter()
        .chain(tangent.to_array())
        .all(f64::is_finite)
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use ktopo::entity::BodyId as RawBodyId;

    use super::super::skew_cylinder_public::orient_parameter_interval;
    use super::*;
    use crate::{CylinderRequest, Kernel};

    fn assert_reversed_enclosure(
        forward: &SectionSkewCylinderPcurveEnclosure,
        reversed: &SectionSkewCylinderPcurveEnclosure,
    ) {
        assert_eq!(forward.stored_uv(), reversed.stored_uv());
        assert_eq!(forward.source_uv(), reversed.source_uv());
        for (forward, reversed) in forward
            .stored_derivative()
            .iter()
            .zip(reversed.stored_derivative())
            .chain(
                forward
                    .source_derivative()
                    .iter()
                    .zip(reversed.source_derivative()),
            )
        {
            assert_eq!(reversed.lo().to_bits(), (-forward.hi()).to_bits());
            assert_eq!(reversed.hi().to_bits(), (-forward.lo()).to_bits());
        }
    }

    fn assert_open_certificate_reversal(edge: &IntersectionBranchEdge) {
        let source = edge
            .certificate
            .as_skew_cylinder_open_span_branch()
            .expect("bounded skew edge lost its sealed open-span proof");
        let range = edge.carrier_range;
        let forward = SectionSkewCylinderEmbeddingCertificate::new(source, range, false).unwrap();
        let reversed = SectionSkewCylinderEmbeddingCertificate::new(source, range, true).unwrap();
        let last = forward.guarded_cell_count() - 1;
        for section_index in 0..=last {
            let graph_cell = forward.guarded_cell(last - section_index).unwrap();
            let section_cell = reversed.guarded_cell(section_index).unwrap();
            let expected =
                orient_parameter_interval(range, as_interval(graph_cell.parameter()), true);
            assert_eq!(
                section_cell.parameter().lo().to_bits(),
                expected.lo().max(range.lo).to_bits()
            );
            assert_eq!(
                section_cell.parameter().hi().to_bits(),
                expected.hi().min(range.hi).to_bits()
            );
            for operand in 0..2 {
                assert_reversed_enclosure(
                    &graph_cell.pcurves()[operand],
                    &section_cell.pcurves()[operand],
                );
            }
        }
        for section_end in 0..2 {
            let graph_root = forward.root_corridor(1 - section_end).unwrap();
            let section_root = reversed.root_corridor(section_end).unwrap();
            assert_eq!(section_root.section_end(), section_end);
            let expected =
                orient_parameter_interval(range, as_interval(graph_root.root_parameter()), true);
            assert_eq!(section_root.root_parameter(), expected);
            let graph_corridor = graph_root.corridor();
            let section_corridor = section_root.corridor();
            assert_eq!(
                section_corridor.parameter(),
                orient_parameter_interval(range, as_interval(graph_corridor.parameter()), true)
            );
            for operand in 0..2 {
                assert_reversed_enclosure(
                    &graph_root.root_pcurves()[operand],
                    &section_root.root_pcurves()[operand],
                );
                assert_reversed_enclosure(
                    &graph_corridor.pcurves()[operand],
                    &section_corridor.pcurves()[operand],
                );
            }
        }
    }

    fn as_interval(value: SectionSkewCylinderInterval) -> Interval {
        Interval::new(value.lo(), value.hi())
    }

    fn side_face(store: &Store, body: RawBodyId) -> RawFaceId {
        store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|face| {
                matches!(
                    store.surface(store.get(*face).unwrap().surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .expect("finite cylinder must retain one side face")
    }

    #[test]
    fn projective_root_identity_matches_exact_positive_and_negative_period_lifts() {
        let canonical = Interval::new(0.25, 0.5);
        assert!(periodic_root_interval_matches(canonical, canonical));
        for turns in [-2.0, -1.0, 1.0, 2.0] {
            let shift = turns * core::f64::consts::TAU;
            assert!(periodic_root_interval_matches(
                canonical,
                Interval::new(canonical.lo() + shift, canonical.hi() + shift)
            ));
        }
        assert!(!periodic_root_interval_matches(
            canonical,
            Interval::new(0.25_f64.next_down(), 0.5)
        ));
    }

    #[test]
    fn bounded_embedding_layout_remains_indirect_on_section_branch() {
        assert_eq!(
            core::mem::size_of::<Option<Box<SectionSkewCylinderEmbeddingCertificate>>>(),
            core::mem::size_of::<usize>()
        );
    }

    #[test]
    fn malformed_final_open_endpoint_refuses_the_face_pair_without_a_prefix() {
        let frame = Frame::world();
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (first, second) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let first = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(0.0, 0.0, 1.8)),
                    1.0,
                    0.1,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let second = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::new(frame.point_at(-1.25, 0.0, 0.0), frame.x(), frame.y()).unwrap(),
                    2.0,
                    2.5,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (first, second)
        };

        let part = session.part(part_id.clone()).unwrap();
        let store = &part.state.store;
        let raw_faces = [
            side_face(store, first.raw()),
            side_face(store, second.raw()),
        ];
        let face_data = raw_faces.map(|face| store.get(face).unwrap());
        let domains = face_data.map(|face| {
            let domain = face.domain().expect("primitive side face has a domain");
            [domain.u, domain.v]
        });
        let surfaces = face_data.map(|face| store.surface(face.surface()).unwrap());
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                super::super::BodySectionBudgetProfile::v1_defaults()
                    .overlaid(&kops::intersect::GraphSurfaceBudgetProfile::v1_defaults()),
            );
        let mut scope = OperationScope::new(&context);
        let intersections = kops::intersect::intersect_bounded_graph_surfaces_in_scope(
            store.geometry(),
            face_data[0].surface(),
            domains[0],
            face_data[1].surface(),
            domains[1],
            &mut scope,
        )
        .unwrap();
        assert!(intersections.raw.is_complete());
        assert_eq!(intersections.branch_graph.edges.len(), 4);
        assert_eq!(intersections.branch_graph.vertices.len(), 8);
        for edge in &intersections.branch_graph.edges {
            assert_open_certificate_reversal(edge);
        }

        let mut edges = intersections.branch_graph.edges.clone();
        edges[3].endpoint_proofs[1] = None;
        let facades = raw_faces.map(|face| FaceId::new(part_id.clone(), face));
        let mut roots = RootIdentityAuthority::new();
        let mut acc = SectionAccumulator::default();
        append_face_pair_branches(
            store,
            raw_faces,
            &facades,
            &edges,
            &intersections.branch_graph.vertices,
            surfaces,
            face_data.map(|face| face.sense()),
            context.tolerances().linear(),
            &mut roots,
            &mut scope,
            &mut acc,
        )
        .unwrap();

        assert!(acc.branches.is_empty());
        assert!(acc.closed_fragments.is_empty());
        assert!(acc.closed_fragment_evidence.is_empty());
        assert!(acc.bounded_procedural_fragments.is_empty());
        assert_eq!(acc.gaps.len(), 1);
        assert_eq!(
            acc.gaps[0].reason(),
            GAP_SKEW_CYLINDER_OPEN_SPAN_CAP_ROOT_UNPROVEN
        );
        assert_eq!(acc.gaps[0].faces(), facades);

        let mut edges = intersections.branch_graph.edges.clone();
        let Some(IntersectionBranchEndpointProof::SkewCylinderAxialRoot(proof)) =
            &mut edges[3].endpoint_proofs[1]
        else {
            panic!("bounded skew fixture lost its final projective root")
        };
        proof.half_angle_bracket[0] = proof.half_angle_bracket[1];
        let mut roots = RootIdentityAuthority::new();
        let mut acc = SectionAccumulator::default();
        append_face_pair_branches(
            store,
            raw_faces,
            &facades,
            &edges,
            &intersections.branch_graph.vertices,
            surfaces,
            face_data.map(|face| face.sense()),
            context.tolerances().linear(),
            &mut roots,
            &mut scope,
            &mut acc,
        )
        .unwrap();
        assert!(acc.branches.is_empty());
        assert!(acc.closed_fragments.is_empty());
        assert!(acc.closed_fragment_evidence.is_empty());
        assert!(acc.bounded_procedural_fragments.is_empty());
        assert_eq!(acc.gaps.len(), 1);
        assert_eq!(
            acc.gaps[0].reason(),
            GAP_SKEW_CYLINDER_OPEN_SPAN_CAP_ROOT_UNPROVEN
        );
        assert_eq!(acc.gaps[0].faces(), facades);
    }
}
