//! Section publication for exact skew-cylinder branch contacts.
//!
//! A through-contact is retained beside its positive-length branch. It
//! resolves every exact axial root onto the topology-owned cap ring without
//! manufacturing a stitch vertex, curve endpoint, or degenerate fragment.

use kcore::interval::Interval;
use kgraph::{PersistentSkewCylinderAxialBoundary, PersistentSkewCylinderHalfAngleChart};
use kops::intersect::{IntersectionBranchEdge, SkewCylinderThroughContact};
use ktopo::entity::FaceId as RawFaceId;

use super::source_annulus::CertifiedSourceAnnulus;
use super::*;

#[allow(clippy::too_many_arguments)]
pub(super) fn prepare(
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    edges: &[IntersectionBranchEdge],
    contacts: &[SkewCylinderThroughContact],
    annuli: &[CertifiedSourceAnnulus; 2],
    base_branch: usize,
) -> Result<Option<Vec<SectionThroughContact>>> {
    let mut prepared = Vec::with_capacity(contacts.len());
    for contact in contacts.iter().copied() {
        let family = contact.certificate().family();
        let member_membership = contact.certificate().member_membership();
        let mut matching_branches = edges.iter().enumerate().filter(|(_, edge)| {
            if let Some(membership) = member_membership {
                edge.certificate
                    .as_skew_cylinder_open_span_branch()
                    .and_then(|certificate| certificate.finite_window_family_membership())
                    == Some(membership)
            } else {
                family.sheet_occupancy(contact.sheet())
                    == kgraph::PersistentSkewCylinderFiniteWindowSheetOccupancy::Whole
                    && edge
                        .certificate
                        .as_skew_cylinder_whole_contact()
                        .is_some_and(|certificate| {
                            certificate.finite_window_family() == family
                                && certificate.residual_certificate().sheet() == contact.sheet()
                        })
            }
        });
        let Some((edge_ordinal, _)) = matching_branches.next() else {
            return Ok(None);
        };
        if matching_branches.next().is_some() {
            return Ok(None);
        }

        let point = contact.point();
        let surface_parameters = contact.surface_parameters();
        if !finite_point(point)
            || surface_parameters
                .into_iter()
                .flatten()
                .any(|value| !value.is_finite())
            || contact.root_count() == 0
        {
            return Ok(None);
        }

        let mut seen_operands = [false; 2];
        let mut public_roots = Vec::with_capacity(contact.root_count());
        for ordinal in 0..contact.root_count() {
            let Some(graph_root) = contact.root(ordinal) else {
                return Ok(None);
            };
            let operand = graph_root.tag.source_slot();
            if operand > 1 || seen_operands[operand] || !graph_root.repeated {
                return Ok(None);
            }
            seen_operands[operand] = true;
            let (ring, axial_boundary) = match graph_root.tag.boundary() {
                PersistentSkewCylinderAxialBoundary::Lower => (
                    annuli[operand].lower(),
                    SectionSkewCylinderAxialBoundary::Lower,
                ),
                PersistentSkewCylinderAxialBoundary::Upper => (
                    annuli[operand].upper(),
                    SectionSkewCylinderAxialBoundary::Upper,
                ),
            };
            if graph_root.sheet != contact.sheet()
                || ring.face() != raw_faces[operand]
                || ring.authored_height().to_bits() != graph_root.bound.to_bits()
            {
                return Ok(None);
            }

            let longitude = surface_parameters[operand][0];
            if !longitude.is_finite()
                || (surface_parameters[operand][1] - graph_root.bound).abs() > family.tolerance()
            {
                return Ok(None);
            }

            let chart = match graph_root.half_angle_chart {
                PersistentSkewCylinderHalfAngleChart::Tangent => {
                    SectionSkewCylinderRootChart::TangentHalfAngle
                }
                PersistentSkewCylinderHalfAngleChart::Cotangent => {
                    SectionSkewCylinderRootChart::CotangentHalfAngle
                }
            };
            let projective = graph_root.half_angle_bracket;
            if projective.iter().any(|value| !value.is_finite()) || projective[0] > projective[1] {
                return Ok(None);
            }
            // The projective bracket parameterizes the family's first
            // cylinder. A root may instead belong to the second operand, so
            // topology association uses the certificate's operand-local
            // surface longitude rather than reinterpreting that bracket in a
            // different surface chart.
            let longitude_enclosure = Interval::new(longitude.next_down(), longitude.next_up());
            let Some(edge_parameter) =
                ring.intrinsic_edge_parameter_for_longitude(longitude_enclosure)
            else {
                return Ok(None);
            };
            public_roots.push(SectionThroughContactRoot {
                operand,
                axial_boundary,
                authored_bound: graph_root.bound,
                face: facades[operand].clone(),
                loop_id: LoopId::new(facades[operand].part().clone(), ring.loop_id()),
                fin: FinId::new(facades[operand].part().clone(), ring.fin()),
                surface_longitude: longitude,
                carrier_root: SectionSkewCylinderCarrierRootEnclosure {
                    chart,
                    lo: projective[0],
                    hi: projective[1],
                },
                source_edge: ring.edge(),
                edge_parameter,
            });
        }
        if public_roots.len() != contact.root_count() {
            return Ok(None);
        }
        prepared.push(SectionThroughContact {
            faces: facades.clone(),
            branch: base_branch + edge_ordinal,
            source: contact,
            roots: public_roots,
        });
    }
    Ok(Some(prepared))
}

fn finite_point(point: kgeom::vec::Point3) -> bool {
    point.to_array().into_iter().all(f64::is_finite)
}
