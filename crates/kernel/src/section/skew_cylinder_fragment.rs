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
use kgraph::PairedSkewCylinderBranchResidualCertificate;
use kops::intersect::{
    IntersectionBranchEndpointProof, SkewCylinderAxialBoundaryProof,
    SkewCylinderHalfAngleChartProof,
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

enum PreparedSkewBranch {
    Whole {
        branch: SectionBranch,
        fragment: closed_stitch::ClosedCurveFragment,
        evidence: ClosedFragmentEvidence,
    },
    Open {
        branch: SectionBranch,
        fragment: CertifiedBoundedSkewCylinderFragment,
    },
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
                ) => *branch,
                super::cylinder_cylinder_publish::CylinderCylinderBranchAdaptation::OrientationIndeterminate => return Ok(PreparedSkewFacePair::Gap(GAP_CARRIER_ORIENTATION)),
                super::cylinder_cylinder_publish::CylinderCylinderBranchAdaptation::Unsupported => {
                    return Ok(PreparedSkewFacePair::Gap(GAP_PAIR_UNRESOLVED));
                }
            };
            let Some((fragment, evidence)) = prepare_whole_closed_branch(branch_index, &branch)
            else {
                return Ok(PreparedSkewFacePair::Gap(GAP_CLOSED_STITCH));
            };
            prepared.push(PreparedSkewBranch::Whole {
                branch,
                fragment,
                evidence,
            });
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
            &annuli,
            root_identity,
            scope,
        )? {
            OpenSkewCylinderAdaptation::Prepared(open) => {
                let (branch, fragment) = open.into_parts(branch_index);
                prepared.push(PreparedSkewBranch::Open { branch, fragment });
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
            PreparedSkewBranch::Whole {
                branch,
                fragment,
                evidence,
            } => {
                acc.branches.push(branch);
                acc.closed_fragments.push(fragment);
                acc.closed_fragment_evidence.push(evidence);
            }
            PreparedSkewBranch::Open { branch, fragment } => {
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
    let Some(certificate) = edge.certificate.as_skew_cylinder_open_span() else {
        return Ok(OpenSkewCylinderAdaptation::Unproven);
    };
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
        root_point,
        inside_point,
        inside_carrier_parameter,
        carrier_root: CertifiedCarrierRoot {
            chart: proof.half_angle_chart,
            projective: Interval::new(proof.half_angle_bracket[0], proof.half_angle_bracket[1]),
        },
    }))
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
        let mut public_ends = Vec::with_capacity(2);
        for evidence in &fragment.endpoints {
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

    use super::*;
    use crate::{CylinderRequest, Kernel};

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
    }
}
