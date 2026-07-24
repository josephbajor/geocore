//! Facade publication for certified analytic section fragments.
//!
//! Closed-carrier stitching owns exact endpoint identities, while the curved
//! clipper retains metric representatives and full trim provenance. This
//! module is the single adaptation boundary that reunites those aligned
//! inputs, interns certified ruling endpoints, and publishes deterministic
//! mixed-family components and intact rings.

use kgeom::vec::Point3;

use super::{
    ClosedFragmentEndEvidence, ClosedFragmentEvidence, ClosedFragmentEvidenceSpan, SectionBranch,
    SectionCarrier, SectionCurveComponent, SectionCurveEndpoint, SectionCurveEndpointTopology,
    SectionCurveFragment, SectionCurveFragmentEnd, SectionCurveFragmentSpan,
    SectionCurveTrimProvenance, SectionEdgeParameterInterval, SectionProjectiveParameterInterval,
    SectionRing, SectionSourceParameterKey, adapt_site, closed_stitch, mixed_stitch,
    ruling_publish,
};
use crate::error::{Error, Result};
use crate::{FaceId, FinId, LoopId, PartId};

pub(super) struct PublishedCurves {
    pub(super) endpoints: Vec<SectionCurveEndpoint>,
    pub(super) fragments: Vec<SectionCurveFragment>,
    pub(super) components: Vec<SectionCurveComponent>,
    pub(super) rings: Vec<SectionRing>,
    pub(super) has_mixed_stitch_defects: bool,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn publish_curves(
    part: &PartId,
    branches: &[SectionBranch],
    closed_fragments: &[closed_stitch::ClosedCurveFragment],
    closed_fragment_evidence: &[ClosedFragmentEvidence],
    ruling_fragments: &[ruling_publish::CertifiedRulingFragment],
    disk_fragments: &[super::disk_publish::CertifiedDiskCapFragment],
    bounded_procedural_fragments: &[
        super::skew_cylinder_fragment::CertifiedBoundedSkewCylinderFragment
    ],
    closed_stitched: &closed_stitch::ClosedStitchResult,
) -> Result<PublishedCurves> {
    if closed_fragments.len() != closed_fragment_evidence.len() {
        return Err(inconsistent_topology(
            "closed section fragment evidence is not index-aligned",
        ));
    }

    let fragment_endpoints = index_fragment_endpoints(closed_fragments.len(), closed_stitched)?;
    let root_scalars = index_closed_root_scalars(
        closed_fragment_evidence,
        &fragment_endpoints,
        closed_stitched.vertices.len(),
    )?;
    let mut endpoints = closed_stitched
        .vertices
        .iter()
        .zip(root_scalars)
        .map(|(vertex, scalars)| adapt_closed_endpoint(part, vertex, scalars))
        .collect::<Result<Vec<_>>>()?;
    let mut fragments = publish_closed_fragments(
        part,
        branches,
        closed_fragment_evidence,
        &fragment_endpoints,
        ruling_fragments.len(),
    )?;

    ruling_publish::publish_fragments(
        part,
        branches,
        ruling_fragments,
        &mut endpoints,
        &mut fragments,
    )?;
    super::disk_publish::publish_fragments(part, disk_fragments, &mut endpoints, &mut fragments)?;
    super::skew_cylinder_fragment::publish_fragments(
        part,
        branches,
        bounded_procedural_fragments,
        &mut endpoints,
        &mut fragments,
    )?;

    let mixed_stitched = mixed_stitch::stitch_curve_fragments(&fragments, endpoints.len())?;
    let components = mixed_stitched
        .components
        .iter()
        .map(|component| SectionCurveComponent {
            fragments: component.fragments.clone(),
            closed: component.closed,
        })
        .collect();
    let rings = publish_rings(closed_fragments, branches.len(), closed_stitched);

    Ok(PublishedCurves {
        endpoints,
        fragments,
        components,
        rings,
        has_mixed_stitch_defects: !mixed_stitched.defects.is_empty(),
    })
}

fn index_closed_root_scalars(
    evidence: &[ClosedFragmentEvidence],
    fragment_endpoints: &[Option<Option<[usize; 2]>>],
    endpoint_count: usize,
) -> Result<Vec<[Option<super::root_identity::CertifiedSourceRootScalar>; 2]>> {
    let mut indexed = vec![[None; 2]; endpoint_count];
    for (fragment, evidence) in evidence.iter().copied().enumerate() {
        let ClosedFragmentEvidenceSpan::Arc { ends, .. } = evidence.span else {
            continue;
        };
        let Some(endpoints) = fragment_endpoints
            .get(fragment)
            .copied()
            .flatten()
            .flatten()
        else {
            return Err(inconsistent_topology(
                "certified closed root scalar lacks stitched endpoint indices",
            ));
        };
        for (endpoint, end) in endpoints.into_iter().zip(ends) {
            let Some(slots) = indexed.get_mut(endpoint) else {
                return Err(inconsistent_topology(
                    "certified closed root scalar referenced an unknown endpoint",
                ));
            };
            for (operand, root) in end.endpoint_roots.into_iter().enumerate() {
                let Some(root) = root else {
                    continue;
                };
                let Some(slot) = slots.get_mut(operand) else {
                    return Err(inconsistent_topology(
                        "certified closed root scalar has an invalid operand",
                    ));
                };
                if slot.is_some_and(|current| current != root.source_root_scalar) {
                    return Err(inconsistent_topology(
                        "one closed source root retained inconsistent scalar materializations",
                    ));
                }
                *slot = Some(root.source_root_scalar);
            }
        }
    }
    Ok(indexed)
}

fn index_fragment_endpoints(
    fragment_count: usize,
    stitched: &closed_stitch::ClosedStitchResult,
) -> Result<Vec<Option<Option<[usize; 2]>>>> {
    let mut endpoints = vec![None; fragment_count];
    for chain in &stitched.chains {
        for fragment in &chain.fragments {
            let Some(slot) = endpoints.get_mut(fragment.input_fragment) else {
                return Err(inconsistent_topology(
                    "closed stitch chain referenced an unknown fragment",
                ));
            };
            *slot = Some(fragment.endpoints);
        }
    }
    Ok(endpoints)
}

fn publish_closed_fragments(
    part: &PartId,
    branches: &[SectionBranch],
    evidence: &[ClosedFragmentEvidence],
    fragment_endpoints: &[Option<Option<[usize; 2]>>],
    ruling_fragment_count: usize,
) -> Result<Vec<SectionCurveFragment>> {
    let mut fragments = Vec::with_capacity(evidence.len() + ruling_fragment_count);
    for (input_index, evidence) in evidence.iter().copied().enumerate() {
        let Some(branch) = branches.get(evidence.branch) else {
            return Err(inconsistent_topology(
                "curved section fragment referenced an unknown branch",
            ));
        };
        let span = match evidence.span {
            ClosedFragmentEvidenceSpan::Whole => SectionCurveFragmentSpan::Whole,
            ClosedFragmentEvidenceSpan::Arc {
                ends,
                wraps_pcurve_seam,
            } => {
                let Some(endpoint_indices) = fragment_endpoints
                    .get(input_index)
                    .copied()
                    .flatten()
                    .flatten()
                else {
                    return Err(inconsistent_topology(
                        "certified curved arc lacks stitched endpoint indices",
                    ));
                };
                let (Some(start), Some(end)) = (
                    adapt_curve_fragment_end(part, branch, endpoint_indices[0], ends[0]),
                    adapt_curve_fragment_end(part, branch, endpoint_indices[1], ends[1]),
                ) else {
                    return Err(inconsistent_topology(
                        "certified curved endpoint has no finite representative",
                    ));
                };
                SectionCurveFragmentSpan::Arc {
                    endpoints: Box::new([start, end]),
                    wraps_pcurve_seam,
                }
            }
        };
        fragments.push(SectionCurveFragment {
            branch: evidence.branch,
            source_ordinal: evidence.ordinal,
            span,
        });
    }
    Ok(fragments)
}

fn publish_rings(
    closed_fragments: &[closed_stitch::ClosedCurveFragment],
    branch_count: usize,
    stitched: &closed_stitch::ClosedStitchResult,
) -> Vec<SectionRing> {
    stitched
        .chains
        .iter()
        .filter_map(|chain| {
            let [fragment] = chain.fragments.as_slice() else {
                return None;
            };
            let input = closed_fragments.get(fragment.input_fragment)?;
            (chain.closed
                && fragment.endpoints.is_none()
                && matches!(input.span, closed_stitch::ClosedFragmentSpan::Whole)
                && fragment.source.branch.index() < branch_count)
                .then_some(SectionRing {
                    branch: fragment.source.branch.index(),
                })
        })
        .collect()
}

fn adapt_closed_endpoint(
    part: &PartId,
    vertex: &closed_stitch::ClosedStitchVertex,
    root_scalars: [Option<super::root_identity::CertifiedSourceRootScalar>; 2],
) -> Result<SectionCurveEndpoint> {
    let topology = match vertex.key {
        closed_stitch::CertifiedClosedEndpointKey::TrimSite {
            site,
            edge_parameter_keys,
        } => {
            let mut source_parameters = [None, None];
            for operand in 0..2 {
                source_parameters[operand] =
                    match (edge_parameter_keys[operand], root_scalars[operand]) {
                        (None, None) => None,
                        (Some(key), Some(scalar)) => {
                            Some(SectionSourceParameterKey::from_certified_root(
                                part,
                                super::root_identity::SourceRootKey::new(
                                    key.edge(),
                                    key.root_ordinal(),
                                ),
                                scalar,
                            ))
                        }
                        _ => {
                            return Err(inconsistent_topology(
                                "closed endpoint root identity and scalar authority disagree",
                            ));
                        }
                    };
            }
            SectionCurveEndpointTopology::Trim {
                sites: [adapt_site(part, site.a), adapt_site(part, site.b)],
                source_parameters,
            }
        }
        closed_stitch::CertifiedClosedEndpointKey::PeriodSeam { branch, site } => {
            if root_scalars != [None, None] {
                return Err(inconsistent_topology(
                    "parameter-seam endpoint retained a physical root scalar",
                ));
            }
            SectionCurveEndpointTopology::ParameterSeam {
                branch: branch.index(),
                site,
            }
        }
    };
    Ok(SectionCurveEndpoint {
        topology,
        edge_parameters: vertex
            .edge_parameters
            .map(|value| value.map(SectionEdgeParameterInterval::from_interval)),
    })
}

fn adapt_curve_fragment_end(
    part: &PartId,
    branch: &SectionBranch,
    endpoint: usize,
    evidence: ClosedFragmentEndEvidence,
) -> Option<SectionCurveFragmentEnd> {
    let site = evidence.trim.site;
    Some(SectionCurveFragmentEnd {
        endpoint,
        point: carrier_point(branch.carrier, site.carrier_parameter)?,
        carrier_parameter: site.carrier_parameter,
        trim: SectionCurveTrimProvenance {
            operand: evidence.trim_operand,
            face: FaceId::new(part.clone(), site.face),
            loop_id: LoopId::new(part.clone(), site.loop_id),
            fin: FinId::new(part.clone(), site.fin),
            source_parameter: SectionSourceParameterKey::from_certified_root(
                part,
                super::root_identity::SourceRootKey::new(site.edge, site.root_ordinal),
                evidence.trim.source_root_scalar,
            ),
            edge_parameter: SectionEdgeParameterInterval::from_interval(site.edge_parameter),
            pcurve_half_angle: SectionProjectiveParameterInterval::from_interval(
                site.pcurve_half_angle,
            ),
        },
    })
}

pub(super) fn carrier_point(carrier: SectionCarrier, parameter: f64) -> Option<Point3> {
    if !parameter.is_finite() {
        return None;
    }
    let point = match carrier {
        SectionCarrier::Line { origin, direction } => origin + direction * parameter,
        SectionCarrier::Circle {
            center,
            normal,
            x_direction,
            radius,
        } => {
            let (sin, cos) = kcore::math::sincos(parameter);
            center + x_direction * (radius * cos) + normal.cross(x_direction) * (radius * sin)
        }
        SectionCarrier::SkewCylinderBranch(carrier) => carrier.eval(parameter),
    };
    [point.x, point.y, point.z]
        .into_iter()
        .all(f64::is_finite)
        .then_some(point)
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}
