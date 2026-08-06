//! Shared theorem for shells obtained by attaching verified product sweeps.
//!
//! Family code is allowed to propose decompositions, but every field below is
//! geometry or topology that this module checks again.  In particular there
//! are no carried proof booleans and no carried shell verdicts.  The theorem
//! delegates the proposed base to an existing general prover, reconstructs
//! every attachment from live incidence, verifies the complete sweep against
//! every base support, verifies every pairwise separation/contact witness, and
//! derives orientation from the verified endpoint roles.

use super::shell_lemmas::{
    CylinderRingBoundary, CylinderRingBoundaryMode, all_shell_faces_consumed, certified_nonzero,
    certified_parallel, cylinder_ring_boundary, indeterminate, interval_vector_dot,
    proof_work as quadratic_proof_work, proof_work_budget, shell_proof_size,
    support_incident_within_resolution,
};
use super::*;
use crate::analytic_tangency::{
    circles_are_exactly_internal_tangent, point_is_within_circle_endpoint_envelope,
};
use kgeom::curve::Circle;

#[path = "shell_surgery/cap_reaching.rs"]
mod cap_reaching;
#[path = "shell_surgery/cap_reaching_sweep.rs"]
mod cap_reaching_sweep;
#[cfg(test)]
#[path = "shell_surgery/cap_reaching_tests.rs"]
mod cap_reaching_tests;
#[path = "shell_surgery/chord_portal.rs"]
mod chord_portal;
#[cfg(test)]
#[path = "shell_surgery/chord_portal_tests.rs"]
mod chord_portal_tests;
#[path = "shell_surgery/cylindrical_host.rs"]
mod cylindrical_host;
#[cfg(test)]
#[path = "shell_surgery/cylindrical_host_tests.rs"]
mod cylindrical_host_tests;
#[path = "shell_surgery/periodic_host_sweep.rs"]
mod periodic_host_sweep;
#[path = "shell_surgery/portal_cylinder.rs"]
mod portal_cylinder;
#[cfg(test)]
#[path = "shell_surgery/portal_cylinder_tests.rs"]
mod portal_cylinder_tests;
#[path = "shell_surgery/profile_sweep.rs"]
mod profile_sweep;
#[path = "shell_surgery/two_host.rs"]
mod two_host;
#[path = "shell_surgery/two_host_sweep.rs"]
mod two_host_sweep;

/// Cumulative work for the one shared shell-surgery theorem.
pub(crate) const SHELL_SURGERY_WORK: StageId = match StageId::new("ktopo.check.shell-surgery-work")
{
    Ok(stage) => stage,
    Err(_) => panic!("valid shell surgery work stage"),
};

const DEFAULT_SHELL_SURGERY_WORK: u64 = 16_777_216;

pub(super) fn shell_surgery_proof_budget() -> BudgetPlan {
    proof_work_budget(
        SHELL_SURGERY_WORK,
        DEFAULT_SHELL_SURGERY_WORK,
        "built-in shell surgery proof budget is valid",
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EndpointRole {
    Port,
    Cap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FeatureRole {
    Through,
    Boss,
    Pocket,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SupportSide {
    StrictInterior,
    IncidentAt(usize),
}

#[derive(Debug, Clone)]
struct PlanarFacetEvidence {
    face: FaceId,
    outer_loop: LoopId,
    vertices: Vec<VertexId>,
}

#[derive(Debug, Clone)]
struct PlanarBaseEvidence {
    facets: Vec<PlanarFacetEvidence>,
}

#[derive(Debug, Clone, Copy)]
struct AttachmentLoopEvidence {
    side_loop: LoopId,
    planar_face: FaceId,
    planar_loop: LoopId,
    edge: EdgeId,
    profile: Circle,
    role: EndpointRole,
}

#[derive(Debug, Clone, Copy)]
struct SupportReference {
    face: FaceId,
    side: SupportSide,
}

#[derive(Debug, Clone)]
struct ProductSweepEvidence {
    side_face: FaceId,
    cylinder: Cylinder,
    profiles: [AttachmentLoopEvidence; 2],
    translation: Vec3,
    interval: [f64; 2],
    supports: Vec<SupportReference>,
    role: FeatureRole,
}

#[derive(Debug, Clone, Copy)]
struct StrictSeparationEvidence {
    first: usize,
    second: usize,
    direction: Vec3,
    origin: Point3,
    first_range: [f64; 2],
    second_range: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
struct ExactTangencyEvidence {
    first: usize,
    second: usize,
    first_edge: EdgeId,
    second_edge: EdgeId,
    first_circle: Circle,
    second_circle: Circle,
    contact: Point3,
    first_parameter: f64,
    second_parameter: f64,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Constructed by contact-family discovery added later in this migration.
enum PairwiseRelationEvidence {
    Strict(StrictSeparationEvidence),
    Tangent(ExactTangencyEvidence),
}

impl PairwiseRelationEvidence {
    const fn pair(self) -> (usize, usize) {
        match self {
            Self::Strict(evidence) => (evidence.first, evidence.second),
            Self::Tangent(evidence) => (evidence.first, evidence.second),
        }
    }
}

#[derive(Debug, Clone)]
struct ShellSurgeryEvidence {
    shell: ShellId,
    base: PlanarBaseEvidence,
    features: Vec<ProductSweepEvidence>,
    relations: Vec<PairwiseRelationEvidence>,
}

#[derive(Debug, Clone)]
struct ChordPortalFeatureEvidence {
    portal_face: FaceId,
    portal_loop: LoopId,
    feature_faces: Vec<FaceId>,
}

#[derive(Debug, Clone)]
struct ChordPortalSurgeryEvidence {
    shell: ShellId,
    base: PlanarBaseEvidence,
    features: Vec<ChordPortalFeatureEvidence>,
}

#[derive(Debug, Clone)]
struct PeriodicHostSurgeryEvidence {
    shell: ShellId,
    host_face: FaceId,
    cylinder: Cylinder,
    planar_faces: Vec<FaceId>,
}

#[derive(Debug, Clone)]
struct CapReachingSurgeryEvidence {
    shell: ShellId,
    host_face: FaceId,
    cylinder: Cylinder,
    cylinders: Vec<(FaceId, Cylinder)>,
    planar_faces: Vec<FaceId>,
}

#[derive(Debug, Clone)]
struct TwoHostSurgeryEvidence {
    shell: ShellId,
    cylinders: Vec<(FaceId, Cylinder)>,
    planar_faces: Vec<FaceId>,
}

#[derive(Debug, Clone, Copy)]
struct VerifiedSweep {
    evidence: ProductSweepEvidenceSummary,
    boundaries: [CylinderRingBoundary; 2],
    orientation_invalid: bool,
}

#[derive(Debug, Clone, Copy)]
struct ProductSweepEvidenceSummary {
    cylinder: Cylinder,
    edges: [EdgeId; 2],
}

/// Attempt every untrusted surgery proposal and return only a theorem-derived
/// result. Discovery slices propose circular and mixed-profile product sweeps;
/// their evidence remains geometry/topology-only and has no proof authority.
pub(super) fn certify_shell_surgery(
    store: &Store,
    shell_id: ShellId,
    mut scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let circular_proposals = cylindrical_host::discover(store, shell_id)?;
    let chord_proposals = chord_portal::discover(store, shell_id)?;
    let periodic_host_proposals = portal_cylinder::discover(store, shell_id)?;
    let cap_reaching_proposals = cap_reaching::discover(store, shell_id)?;
    let two_host_proposals = two_host::discover(store, shell_id)?;
    if circular_proposals.is_empty()
        && chord_proposals.is_empty()
        && periodic_host_proposals.is_empty()
        && cap_reaching_proposals.is_empty()
        && two_host_proposals.is_empty()
    {
        return Ok(None);
    }
    if let Some(scope) = scope.as_deref_mut() {
        scope.ledger().require_limit(
            SHELL_SURGERY_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let mut work = 0_u64;
        if let Some(proposal) = circular_proposals.first() {
            let Some(circular_work) =
                circular_product_sweep_work(store, shell_id, proposal.features.len())?
            else {
                return Ok(Some(indeterminate()));
            };
            let Some(next) = work.checked_add(circular_work) else {
                return Ok(Some(indeterminate()));
            };
            work = next;
        }
        if !chord_proposals.is_empty() {
            let Some(size) = shell_proof_size(store, shell_id)? else {
                return Ok(Some(indeterminate()));
            };
            let Some(chord_work) = quadratic_proof_work(size, 32, 0, 1) else {
                return Ok(Some(indeterminate()));
            };
            let Some(next) = work.checked_add(chord_work) else {
                return Ok(Some(indeterminate()));
            };
            work = next;
        }
        if !periodic_host_proposals.is_empty() {
            let Some(periodic_work) = periodic_host_sweep::proof_work(store, shell_id)? else {
                return Ok(Some(indeterminate()));
            };
            let Some(next) = work.checked_add(periodic_work) else {
                return Ok(Some(indeterminate()));
            };
            work = next;
        }
        if !cap_reaching_proposals.is_empty() {
            let Some(cap_reaching_work) = cap_reaching_sweep::proof_work(
                store,
                shell_id,
                cap_reaching_proposals[0].cylinders.len(),
            )?
            else {
                return Ok(Some(indeterminate()));
            };
            let Some(next) = work.checked_add(cap_reaching_work) else {
                return Ok(Some(indeterminate()));
            };
            work = next;
        }
        if !two_host_proposals.is_empty() {
            let Some(two_host_work) =
                two_host_sweep::proof_work(store, shell_id, two_host_proposals[0].cylinders.len())?
            else {
                return Ok(Some(indeterminate()));
            };
            let Some(next) = work.checked_add(two_host_work) else {
                return Ok(Some(indeterminate()));
            };
            work = next;
        }
        scope.ledger_mut().charge(SHELL_SURGERY_WORK, work)?;
    }
    for evidence in &circular_proposals {
        if let Some(certification) = certify_evidence(store, shell_id, evidence)? {
            return Ok(Some(certification));
        }
    }
    for evidence in &chord_proposals {
        if let Some(certification) =
            profile_sweep::certify_chord_portal_evidence(store, shell_id, evidence)?
        {
            return Ok(Some(certification));
        }
    }
    for evidence in &periodic_host_proposals {
        if let Some(certification) =
            periodic_host_sweep::certify_periodic_host_sweep_evidence(store, shell_id, evidence)?
        {
            return Ok(Some(certification));
        }
    }
    for evidence in &cap_reaching_proposals {
        if let Some(certification) =
            cap_reaching_sweep::certify_cap_reaching_evidence(store, shell_id, evidence)?
        {
            return Ok(Some(certification));
        }
    }
    for evidence in &two_host_proposals {
        if let Some(verified) =
            two_host_sweep::certify_two_host_evidence(store, shell_id, evidence)?
        {
            if verified.contact {
                let Some(contact_work) = two_host_sweep::contact_work(store, shell_id)? else {
                    return Ok(Some(indeterminate()));
                };
                if let Some(scope) = scope.as_deref_mut() {
                    scope
                        .ledger_mut()
                        .charge(SHELL_SURGERY_WORK, contact_work)?;
                }
            }
            return Ok(Some(verified.certification));
        }
    }
    Ok(None)
}

#[cfg(test)]
pub(super) fn assert_cap_reaching_evidence_claims_are_rechecked(store: &Store, shell: ShellId) {
    let evidence = cap_reaching::discover(store, shell)
        .unwrap()
        .into_iter()
        .next()
        .expect("real cap-reaching topology produces raw evidence");
    assert!(
        cap_reaching_sweep::certify_cap_reaching_evidence(store, shell, &evidence)
            .unwrap()
            .is_some()
    );

    let mut wrong_host = evidence.clone();
    wrong_host.host_face = wrong_host.planar_faces[0];
    assert_eq!(
        cap_reaching_sweep::certify_cap_reaching_evidence(store, shell, &wrong_host).unwrap(),
        None
    );

    let mut wrong_cylinder = evidence.clone();
    wrong_cylinder.cylinder = Cylinder::new(
        *wrong_cylinder.cylinder.frame(),
        wrong_cylinder.cylinder.radius() + 0.25,
    )
    .unwrap();
    assert_eq!(
        cap_reaching_sweep::certify_cap_reaching_evidence(store, shell, &wrong_cylinder).unwrap(),
        None
    );

    let mut missing_cylinder = evidence.clone();
    missing_cylinder.cylinders.pop();
    assert_eq!(
        cap_reaching_sweep::certify_cap_reaching_evidence(store, shell, &missing_cylinder).unwrap(),
        None
    );

    let mut missing_plane = evidence;
    missing_plane.planar_faces.pop();
    assert_eq!(
        cap_reaching_sweep::certify_cap_reaching_evidence(store, shell, &missing_plane).unwrap(),
        None
    );
}

/// Transitional legacy bridge used while two-host routing equality is pinned.
pub(super) fn certify_two_host_candidate(
    store: &Store,
    shell_id: ShellId,
    cylinders: Vec<(FaceId, Cylinder)>,
    planar_faces: Vec<FaceId>,
) -> Result<Option<(ShellCertification, bool)>> {
    Ok(two_host_sweep::certify_two_host_evidence(
        store,
        shell_id,
        &TwoHostSurgeryEvidence {
            shell: shell_id,
            cylinders,
            planar_faces,
        },
    )?
    .map(|verified| (verified.certification, verified.contact)))
}

pub(super) fn two_host_proof_work(
    store: &Store,
    shell_id: ShellId,
    cylinder_count: usize,
) -> Result<Option<u64>> {
    two_host_sweep::proof_work(store, shell_id, cylinder_count)
}

pub(super) fn two_host_contact_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    two_host_sweep::contact_work(store, shell_id)
}

#[cfg(test)]
pub(super) fn assert_two_host_evidence_claims_are_rechecked(store: &Store, shell: ShellId) {
    let evidence = two_host::discover(store, shell)
        .unwrap()
        .into_iter()
        .next()
        .expect("real two-host topology produces raw evidence");
    assert!(
        two_host_sweep::certify_two_host_evidence(store, shell, &evidence)
            .unwrap()
            .is_some()
    );

    let mut wrong_order = evidence.clone();
    wrong_order.cylinders.swap(0, 1);
    assert!(
        two_host_sweep::certify_two_host_evidence(store, shell, &wrong_order)
            .unwrap()
            .is_none()
    );

    let mut missing_cylinder = evidence.clone();
    missing_cylinder.cylinders.pop();
    assert!(
        two_host_sweep::certify_two_host_evidence(store, shell, &missing_cylinder)
            .unwrap()
            .is_none()
    );

    let mut missing_plane = evidence;
    missing_plane.planar_faces.pop();
    assert!(
        two_host_sweep::certify_two_host_evidence(store, shell, &missing_plane)
            .unwrap()
            .is_none()
    );
}

/// Exact structural ceiling retained from the cylindrical-host family while
/// charging it to the one shared theorem stage. Counts are reconstructed from
/// the live shell rather than accepted from discovery evidence.
fn circular_product_sweep_work(
    store: &Store,
    shell_id: ShellId,
    feature_count: usize,
) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let Some(face_count) = u64::try_from(shell.faces.len()).ok() else {
        return Ok(None);
    };
    let Some(features) = u64::try_from(feature_count).ok() else {
        return Ok(None);
    };
    let mut loop_count = 0_u64;
    let mut fin_count = 0_u64;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        let Some(loops) = u64::try_from(face.loops.len()).ok() else {
            return Ok(None);
        };
        let Some(next) = loop_count.checked_add(loops) else {
            return Ok(None);
        };
        loop_count = next;
        for &loop_id in &face.loops {
            let Some(fins) = u64::try_from(store.get(loop_id)?.fins.len()).ok() else {
                return Ok(None);
            };
            let Some(next) = fin_count.checked_add(fins) else {
                return Ok(None);
            };
            fin_count = next;
        }
    }
    let Some(layout) = loop_count.checked_mul(fin_count) else {
        return Ok(None);
    };
    let Some(host_supports) = face_count.checked_mul(fin_count) else {
        return Ok(None);
    };
    let Some(incidence_and_sweeps) = face_count
        .checked_mul(features)
        .and_then(|work| work.checked_mul(8))
    else {
        return Ok(None);
    };
    let Some(feature_pairs) = features
        .checked_mul(features.saturating_sub(1))
        .and_then(|ordered| ordered.checked_div(2))
    else {
        return Ok(None);
    };
    Ok(face_count
        .checked_add(loop_count)
        .and_then(|work| work.checked_add(fin_count))
        .and_then(|work| work.checked_add(layout))
        .and_then(|work| work.checked_add(host_supports))
        .and_then(|work| work.checked_add(incidence_and_sweeps))
        .and_then(|work| work.checked_add(feature_pairs)))
}

fn certify_evidence(
    store: &Store,
    shell_id: ShellId,
    evidence: &ShellSurgeryEvidence,
) -> Result<Option<ShellCertification>> {
    if evidence.shell != shell_id || evidence.features.is_empty() {
        return Ok(None);
    }
    let Some((base_certification, base_faces)) =
        verify_planar_base(store, shell_id, &evidence.base)?
    else {
        return Ok(None);
    };
    if base_certification.embedding != ShellEmbedding::Certified {
        return Ok(None);
    }

    let mut consumed_faces = base_faces.clone();
    let mut verified = Vec::with_capacity(evidence.features.len());
    for feature in &evidence.features {
        let Some(candidate) = verify_product_sweep(store, shell_id, &evidence.base, feature)?
        else {
            return Ok(None);
        };
        consumed_faces.push(feature.side_face);
        for endpoint in feature.profiles {
            if endpoint.role == EndpointRole::Cap {
                consumed_faces.push(endpoint.planar_face);
            }
        }
        verified.push(candidate);
    }
    if !all_shell_faces_consumed(store, shell_id, &consumed_faces)?
        || !verify_pairwise_relations(store, &verified, &evidence.relations)?
    {
        return Ok(None);
    }

    let orientation_invalid = base_certification.orientation != ShellOrientation::Positive
        || verified.iter().any(|feature| feature.orientation_invalid);
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_invalid {
            ShellOrientation::Invalid
        } else {
            ShellOrientation::Positive
        },
    }))
}

fn verify_planar_base(
    store: &Store,
    shell_id: ShellId,
    evidence: &PlanarBaseEvidence,
) -> Result<Option<(ShellCertification, Vec<FaceId>)>> {
    if evidence.facets.len() < 4 {
        return Ok(None);
    }
    let mut faces = Vec::with_capacity(evidence.facets.len());
    let mut facets = Vec::with_capacity(evidence.facets.len());
    for facet in &evidence.facets {
        if faces.contains(&facet.face) {
            return Ok(None);
        }
        let face = store.get(facet.face)?;
        if face.shell != shell_id
            || !face.loops.contains(&facet.outer_loop)
            || certify_face_loop_layout(store, facet.face)? != LoopContainment::Certified
        {
            return Ok(None);
        }
        let Some(vertices) = convex_planar_face_loop_vertices(store, facet.face, facet.outer_loop)?
        else {
            return Ok(None);
        };
        if vertices != facet.vertices {
            return Ok(None);
        }
        faces.push(facet.face);
        facets.push((facet.face, vertices));
    }
    let certification = certify_convex_planar_facets(store, facets, None)?;
    Ok(Some((certification, faces)))
}

fn verify_product_sweep(
    store: &Store,
    shell_id: ShellId,
    base: &PlanarBaseEvidence,
    evidence: &ProductSweepEvidence,
) -> Result<Option<VerifiedSweep>> {
    let face = store.get(evidence.side_face)?;
    let SurfaceGeom::Cylinder(cylinder) = store.get(face.surface)? else {
        return Ok(None);
    };
    if face.shell != shell_id || !same_cylinder(*cylinder, evidence.cylinder) {
        return Ok(None);
    }

    let mut boundaries = Vec::with_capacity(2);
    for endpoint in evidence.profiles {
        let Some(boundary) = cylinder_ring_boundary(
            store,
            shell_id,
            evidence.side_face,
            evidence.cylinder,
            endpoint.side_loop,
            CylinderRingBoundaryMode::CylindricalHost,
        )?
        else {
            return Ok(None);
        };
        let CurveGeom::Circle(profile) = store.get(store.get(boundary.edge)?.curve.ok_or(
            kcore::error::Error::InvalidGeometry {
                reason: "surgery profile edge lost its circle carrier",
            },
        )?)?
        else {
            return Ok(None);
        };
        if boundary.face != endpoint.planar_face
            || boundary.loop_id != endpoint.planar_loop
            || boundary.edge != endpoint.edge
            || !same_circle(*profile, endpoint.profile)
            || actual_endpoint_role(base, endpoint.planar_face, endpoint.planar_loop)
                != Some(endpoint.role)
        {
            return Ok(None);
        }
        boundaries.push(boundary);
    }
    let [first, second] = boundaries.as_slice() else {
        return Ok(None);
    };
    if first.edge == second.edge || first.face == second.face {
        return Ok(None);
    }
    let translation = second.center - first.center;
    let interval = [
        (first.center - evidence.cylinder.frame().origin()).dot(evidence.cylinder.frame().z()),
        (second.center - evidence.cylinder.frame().origin()).dot(evidence.cylinder.frame().z()),
    ];
    if translation != evidence.translation
        || interval.map(f64::to_bits) != evidence.interval.map(f64::to_bits)
        || !certified_nonzero(translation)
        || !certified_parallel(translation, evidence.cylinder.frame().z())
        || !verify_supports(store, base, evidence, [*first, *second])?
    {
        return Ok(None);
    }

    let Some(role) = derive_feature_role(store, evidence, [*first, *second])? else {
        return Ok(None);
    };
    if role != evidence.role {
        return Ok(None);
    }
    let Some(orientation_invalid) = derive_sweep_orientation(
        store,
        evidence.side_face,
        [*first, *second],
        evidence.profiles.map(|profile| profile.role),
        role,
    )?
    else {
        return Ok(None);
    };
    Ok(Some(VerifiedSweep {
        evidence: ProductSweepEvidenceSummary {
            cylinder: evidence.cylinder,
            edges: evidence.profiles.map(|profile| profile.edge),
        },
        boundaries: [*first, *second],
        orientation_invalid,
    }))
}

fn actual_endpoint_role(
    base: &PlanarBaseEvidence,
    face: FaceId,
    loop_id: LoopId,
) -> Option<EndpointRole> {
    if base
        .facets
        .iter()
        .any(|facet| facet.face == face && facet.outer_loop != loop_id)
    {
        Some(EndpointRole::Port)
    } else if base.facets.iter().all(|facet| facet.face != face) {
        Some(EndpointRole::Cap)
    } else {
        None
    }
}

fn verify_supports(
    store: &Store,
    base: &PlanarBaseEvidence,
    feature: &ProductSweepEvidence,
    boundaries: [CylinderRingBoundary; 2],
) -> Result<bool> {
    if feature.supports.len() != base.facets.len() {
        return Ok(false);
    }
    for facet in &base.facets {
        let matching = feature
            .supports
            .iter()
            .filter(|support| support.face == facet.face)
            .collect::<Vec<_>>();
        let [support] = matching.as_slice() else {
            return Ok(false);
        };
        let face = store.get(facet.face)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(false);
        };
        let outward = plane.frame().z() * sense_factor(face.sense);
        let incident = feature
            .profiles
            .iter()
            .enumerate()
            .find_map(|(index, endpoint)| {
                (endpoint.role == EndpointRole::Port && endpoint.planar_face == facet.face)
                    .then_some(index)
            });
        match (support.side, incident) {
            (SupportSide::IncidentAt(claimed), Some(index)) if claimed == index => {
                let other = 1 - index;
                let other_side =
                    interval_vector_dot(outward, boundaries[other].center - plane.frame().origin());
                let terminal_side_valid = match feature.role {
                    FeatureRole::Boss => other_side.lo() > LINEAR_RESOLUTION,
                    FeatureRole::Through | FeatureRole::Pocket => {
                        other_side.hi() < -LINEAR_RESOLUTION
                    }
                };
                if !support_incident_within_resolution(
                    outward,
                    boundaries[index].center,
                    plane.frame().origin(),
                ) || !terminal_side_valid
                    || !axis_is_support_normal(feature.cylinder, outward)
                {
                    return Ok(false);
                }
            }
            (SupportSide::StrictInterior, None) => {
                for boundary in boundaries {
                    if !circle_strictly_inside_support(
                        outward,
                        plane.frame().origin(),
                        feature.cylinder,
                        boundary.center,
                    ) {
                        return Ok(false);
                    }
                }
            }
            _ => return Ok(false),
        }
    }
    Ok(true)
}

fn derive_feature_role(
    store: &Store,
    evidence: &ProductSweepEvidence,
    boundaries: [CylinderRingBoundary; 2],
) -> Result<Option<FeatureRole>> {
    let roles = evidence.profiles.map(|profile| profile.role);
    if roles == [EndpointRole::Port, EndpointRole::Port] {
        return Ok(Some(FeatureRole::Through));
    }
    let port = match roles {
        [EndpointRole::Port, EndpointRole::Cap] => 0,
        [EndpointRole::Cap, EndpointRole::Port] => 1,
        _ => return Ok(None),
    };
    let port_face = store.get(evidence.profiles[port].planar_face)?;
    let SurfaceGeom::Plane(plane) = store.get(port_face.surface)? else {
        return Ok(None);
    };
    let outward = plane.frame().z() * sense_factor(port_face.sense);
    let cap = 1 - port;
    let side = interval_vector_dot(outward, boundaries[cap].center - plane.frame().origin());
    if side.lo() > LINEAR_RESOLUTION {
        Ok(Some(FeatureRole::Boss))
    } else if side.hi() < -LINEAR_RESOLUTION {
        Ok(Some(FeatureRole::Pocket))
    } else {
        Ok(None)
    }
}

fn derive_sweep_orientation(
    store: &Store,
    side_face: FaceId,
    boundaries: [CylinderRingBoundary; 2],
    roles: [EndpointRole; 2],
    role: FeatureRole,
) -> Result<Option<bool>> {
    let axis = match store.get(store.get(side_face)?.surface)? {
        SurfaceGeom::Cylinder(cylinder) => cylinder.frame().z(),
        _ => return Ok(None),
    };
    let (low, high, low_role, high_role) =
        match exact_affine_sign(axis, boundaries[1].center, boundaries[0].center) {
            Some(PredicateOrientation::Positive) => {
                (boundaries[0], boundaries[1], roles[0], roles[1])
            }
            Some(PredicateOrientation::Negative) => {
                (boundaries[1], boundaries[0], roles[1], roles[0])
            }
            _ => return Ok(None),
        };
    let side = store.get(side_face)?;
    let mut invalid = endpoint_orientation_invalid(store, low, low_role)?
        || endpoint_orientation_invalid(store, high, high_role)?
        || low.side_traverses_positive_u != (side.sense == Sense::Forward)
        || high.side_traverses_positive_u != (side.sense == Sense::Reversed);

    match (low_role, high_role, role) {
        (EndpointRole::Port, EndpointRole::Port, FeatureRole::Through) => {
            invalid |= side.sense != Sense::Reversed
                || oriented_axis_alignment(low.axis_alignment, store.get(low.face)?.sense)
                    != Some(-1)
                || oriented_axis_alignment(high.axis_alignment, store.get(high.face)?.sense)
                    != Some(1);
        }
        (EndpointRole::Port, EndpointRole::Cap, FeatureRole::Boss | FeatureRole::Pocket)
        | (EndpointRole::Cap, EndpointRole::Port, FeatureRole::Boss | FeatureRole::Pocket) => {
            let (port, cap, cap_direction) = if low_role == EndpointRole::Port {
                (low, high, 1)
            } else {
                (high, low, -1)
            };
            let Some(port_outward) =
                oriented_axis_alignment(port.axis_alignment, store.get(port.face)?.sense)
            else {
                return Ok(None);
            };
            let Some(cap_outward) =
                oriented_axis_alignment(cap.axis_alignment, store.get(cap.face)?.sense)
            else {
                return Ok(None);
            };
            let outward = cap_direction == port_outward;
            if outward != (role == FeatureRole::Boss) {
                return Ok(None);
            }
            invalid |= side.sense
                != if outward {
                    Sense::Forward
                } else {
                    Sense::Reversed
                }
                || cap_outward != port_outward;
        }
        _ => return Ok(None),
    }
    Ok(Some(invalid))
}

fn endpoint_orientation_invalid(
    store: &Store,
    boundary: CylinderRingBoundary,
    role: EndpointRole,
) -> Result<bool> {
    let face = store.get(boundary.face)?;
    let orientation = certify_loop_orientation(store, boundary.face, boundary.loop_id)?;
    let expected_positive = match role {
        EndpointRole::Port => !face.sense.is_forward(),
        EndpointRole::Cap => face.sense.is_forward(),
    };
    Ok(orientation.is_none_or(|orientation| {
        (orientation == PredicateOrientation::Positive) != expected_positive
    }))
}

fn verify_pairwise_relations(
    store: &Store,
    features: &[VerifiedSweep],
    evidence: &[PairwiseRelationEvidence],
) -> Result<bool> {
    let expected = features
        .len()
        .checked_mul(features.len().saturating_sub(1))
        .map(|ordered| ordered / 2);
    if expected != Some(evidence.len()) {
        return Ok(false);
    }
    for first in 0..features.len() {
        for second in first + 1..features.len() {
            let matching = evidence
                .iter()
                .copied()
                .filter(|relation| relation.pair() == (first, second))
                .collect::<Vec<_>>();
            let [relation] = matching.as_slice() else {
                return Ok(false);
            };
            let valid = match relation {
                PairwiseRelationEvidence::Strict(relation) => {
                    verify_strict_separation(features[first], features[second], *relation)
                }
                PairwiseRelationEvidence::Tangent(relation) => {
                    verify_exact_tangency(store, features[first], features[second], *relation)?
                }
            };
            if !valid {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn verify_strict_separation(
    first: VerifiedSweep,
    second: VerifiedSweep,
    evidence: StrictSeparationEvidence,
) -> bool {
    let first_axis = first.evidence.cylinder.frame().z();
    if evidence.direction != first_axis && evidence.direction != -first_axis {
        return false;
    }
    if !certified_parallel(evidence.direction, second.evidence.cylinder.frame().z()) {
        return false;
    }
    let range = |feature: VerifiedSweep| {
        let values = feature
            .boundaries
            .map(|boundary| (boundary.center - evidence.origin).dot(evidence.direction));
        [values[0].min(values[1]), values[0].max(values[1])]
    };
    let first_range = range(first);
    let second_range = range(second);
    first_range.map(f64::to_bits) == evidence.first_range.map(f64::to_bits)
        && second_range.map(f64::to_bits) == evidence.second_range.map(f64::to_bits)
        && (first_range[1] < second_range[0] || second_range[1] < first_range[0])
}

fn verify_exact_tangency(
    store: &Store,
    first: VerifiedSweep,
    second: VerifiedSweep,
    evidence: ExactTangencyEvidence,
) -> Result<bool> {
    if !first.evidence.edges.contains(&evidence.first_edge)
        || !second.evidence.edges.contains(&evidence.second_edge)
    {
        return Ok(false);
    }
    let Some(first_curve) = store.get(evidence.first_edge)?.curve else {
        return Ok(false);
    };
    let Some(second_curve) = store.get(evidence.second_edge)?.curve else {
        return Ok(false);
    };
    let (CurveGeom::Circle(first_circle), CurveGeom::Circle(second_circle)) =
        (store.get(first_curve)?, store.get(second_curve)?)
    else {
        return Ok(false);
    };
    Ok(same_circle(*first_circle, evidence.first_circle)
        && same_circle(*second_circle, evidence.second_circle)
        && exact_tangency_geometry_matches(
            evidence.first_circle,
            evidence.second_circle,
            evidence.contact,
            evidence.first_parameter,
            evidence.second_parameter,
        ))
}

fn exact_tangency_geometry_matches(
    first_circle: Circle,
    second_circle: Circle,
    contact: Point3,
    first_parameter: f64,
    second_parameter: f64,
) -> bool {
    circles_are_exactly_internal_tangent(first_circle, second_circle)
        && point_is_within_circle_endpoint_envelope(
            contact,
            first_circle,
            first_parameter,
            LINEAR_RESOLUTION,
        )
        && point_is_within_circle_endpoint_envelope(
            contact,
            second_circle,
            second_parameter,
            LINEAR_RESOLUTION,
        )
}

fn circle_strictly_inside_support(
    outward: Vec3,
    support_origin: Point3,
    cylinder: Cylinder,
    center: Point3,
) -> bool {
    let signed = interval_vector_dot(outward, center - support_origin);
    if signed.hi() >= 0.0 {
        return false;
    }
    let radius = Interval::point(cylinder.radius());
    let radial_x = interval_vector_dot(outward, cylinder.frame().x()) * radius;
    let radial_y = interval_vector_dot(outward, cylinder.frame().y()) * radius;
    (radial_x.square() + radial_y.square()).hi() < signed.square().lo()
}

fn axis_is_support_normal(cylinder: Cylinder, normal: Vec3) -> bool {
    normal == cylinder.frame().z()
        || normal == -cylinder.frame().z()
        || exact_vector_dot(normal, cylinder.frame().x()) == Some(PredicateOrientation::Zero)
            && exact_vector_dot(normal, cylinder.frame().y()) == Some(PredicateOrientation::Zero)
}

fn same_cylinder(first: Cylinder, second: Cylinder) -> bool {
    first.frame() == second.frame() && first.radius().to_bits() == second.radius().to_bits()
}

fn same_circle(first: Circle, second: Circle) -> bool {
    first.frame() == second.frame() && first.radius().to_bits() == second.radius().to_bits()
}

#[cfg(test)]
pub(super) fn assert_chord_portal_evidence_claims_are_rechecked(store: &Store, shell: ShellId) {
    let evidence = chord_portal::discover(store, shell)
        .unwrap()
        .into_iter()
        .next()
        .expect("real chord-portal topology produces evidence");
    assert_eq!(
        profile_sweep::certify_chord_portal_evidence(store, shell, &evidence).unwrap(),
        Some(ShellCertification {
            embedding: ShellEmbedding::Certified,
            orientation: ShellOrientation::Positive,
        })
    );

    let mut wrong_outer = evidence.clone();
    let portal_loop = wrong_outer.features[0].portal_loop;
    wrong_outer.base.facets[0].outer_loop = portal_loop;
    assert_eq!(
        profile_sweep::certify_chord_portal_evidence(store, shell, &wrong_outer).unwrap(),
        None
    );

    let mut wrong_portal = evidence.clone();
    wrong_portal.features[0].portal_loop = wrong_portal.base.facets[0].outer_loop;
    assert_eq!(
        profile_sweep::certify_chord_portal_evidence(store, shell, &wrong_portal).unwrap(),
        None
    );

    let mut wrong_feature = evidence;
    wrong_feature.features[0].feature_faces[0] = wrong_feature.base.facets[0].face;
    assert_eq!(
        profile_sweep::certify_chord_portal_evidence(store, shell, &wrong_feature).unwrap(),
        None
    );
}

#[cfg(test)]
pub(super) fn assert_periodic_host_evidence_claims_are_rechecked(store: &Store, shell: ShellId) {
    let evidence = portal_cylinder::discover(store, shell)
        .unwrap()
        .into_iter()
        .next()
        .expect("real portal-cylinder topology produces evidence");
    assert!(
        periodic_host_sweep::certify_periodic_host_sweep_evidence(store, shell, &evidence)
            .unwrap()
            .is_some()
    );

    let mut wrong_host = evidence.clone();
    wrong_host.host_face = wrong_host.planar_faces[0];
    assert_eq!(
        periodic_host_sweep::certify_periodic_host_sweep_evidence(store, shell, &wrong_host)
            .unwrap(),
        None
    );

    let mut wrong_cylinder = evidence.clone();
    wrong_cylinder.cylinder = Cylinder::new(
        *wrong_cylinder.cylinder.frame(),
        wrong_cylinder.cylinder.radius() + 0.25,
    )
    .unwrap();
    assert_eq!(
        periodic_host_sweep::certify_periodic_host_sweep_evidence(store, shell, &wrong_cylinder)
            .unwrap(),
        None
    );

    let mut wrong_planar_faces = evidence;
    wrong_planar_faces.planar_faces.pop();
    assert_eq!(
        periodic_host_sweep::certify_periodic_host_sweep_evidence(
            store,
            shell,
            &wrong_planar_faces,
        )
        .unwrap(),
        None
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cylindrical_host::{
        CylindricalHostBandInput, CylindricalHostEndpoint, CylindricalHostSolidInput,
    };
    use crate::planar::{PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey};
    use kgeom::param::ParamRange;

    fn cube() -> PlanarSolidInput {
        let points = [
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let keys = core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(index as u64));
        let vertices = keys
            .into_iter()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
            .collect();
        let faces = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
        .into_iter()
        .map(|ring| PlanarSolidFace::new(ring.map(|vertex| keys[vertex]).to_vec()))
        .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn two_outward_bands() -> CylindricalHostSolidInput {
        let low = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, -2.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::port(0),
                CylindricalHostEndpoint::cap(),
            ],
        );
        let high = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::cap(),
                CylindricalHostEndpoint::port(1),
            ],
        );
        CylindricalHostSolidInput::new(cube(), vec![high, low])
    }

    fn accepted_evidence(store: &Store, shell: ShellId) -> ShellSurgeryEvidence {
        cylindrical_host::discover(store, shell)
            .unwrap()
            .into_iter()
            .find(|evidence| {
                certify_evidence(store, shell, evidence)
                    .unwrap()
                    .is_some_and(|certification| {
                        certification.embedding == ShellEmbedding::Certified
                            && certification.orientation == ShellOrientation::Positive
                    })
            })
            .expect("one theorem-verified role assignment")
    }

    fn assert_cylindrical_host_shared_route(
        store: &Store,
        shell: ShellId,
        expected_orientation: ShellOrientation,
    ) {
        let shared = certify_shell_surgery(store, shell, None).unwrap();
        assert_eq!(
            shared,
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: expected_orientation,
            })
        );
    }

    #[test]
    fn cylindrical_host_shared_route_preserves_orientation_tamper() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        transaction
            .store_mut()
            .get_mut(output.shell())
            .unwrap()
            .faces
            .reverse();
        let faces = transaction
            .store()
            .get(output.shell())
            .unwrap()
            .faces
            .clone();
        for face in faces {
            transaction
                .store_mut()
                .get_mut(face)
                .unwrap()
                .loops
                .reverse();
        }
        assert_cylindrical_host_shared_route(
            transaction.store(),
            output.shell(),
            ShellOrientation::Positive,
        );

        let side = output.bands()[0].side_face();
        transaction.store_mut().get_mut(side).unwrap().sense = Sense::Reversed;
        assert_cylindrical_host_shared_route(
            transaction.store(),
            output.shell(),
            ShellOrientation::Invalid,
        );
    }

    #[test]
    fn theorem_certifies_data_and_rejects_every_corrupted_claim_kind() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        let evidence = accepted_evidence(transaction.store(), output.shell());
        assert_eq!(
            certify_evidence(transaction.store(), output.shell(), &evidence).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let mut wrong_side = evidence.clone();
        wrong_side.features[0].supports[0].side = match wrong_side.features[0].supports[0].side {
            SupportSide::StrictInterior => SupportSide::IncidentAt(0),
            SupportSide::IncidentAt(_) => SupportSide::StrictInterior,
        };
        assert_eq!(
            certify_evidence(transaction.store(), output.shell(), &wrong_side).unwrap(),
            None
        );

        let mut broken_translation = evidence.clone();
        broken_translation.features[0].translation += Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(
            certify_evidence(transaction.store(), output.shell(), &broken_translation,).unwrap(),
            None
        );

        let mut wrong_role = evidence.clone();
        wrong_role.features[0].role = match wrong_role.features[0].role {
            FeatureRole::Through => FeatureRole::Boss,
            FeatureRole::Boss | FeatureRole::Pocket => FeatureRole::Through,
        };
        assert_eq!(
            certify_evidence(transaction.store(), output.shell(), &wrong_role).unwrap(),
            None
        );

        let mut overlapping_ranges = evidence.clone();
        let PairwiseRelationEvidence::Strict(relation) = &mut overlapping_ranges.relations[0]
        else {
            panic!("fixture uses a strict separation witness")
        };
        relation.second_range = relation.first_range;
        assert_eq!(
            certify_evidence(transaction.store(), output.shell(), &overlapping_ranges,).unwrap(),
            None
        );
    }

    #[test]
    fn exact_tangency_evidence_rejects_a_near_miss() {
        let frame = Frame::world();
        let outer = Circle::new(frame, 2.0).unwrap();
        let tangent = Circle::new(frame.with_origin(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap();
        let contact = Point3::new(2.0, 0.0, 0.0);
        assert!(exact_tangency_geometry_matches(
            outer, tangent, contact, 0.0, 0.0,
        ));

        let near_miss = Circle::new(
            frame.with_origin(Point3::new(1.0_f64.next_up(), 0.0, 0.0)),
            1.0,
        )
        .unwrap();
        assert!(!exact_tangency_geometry_matches(
            outer, near_miss, contact, 0.0, 0.0,
        ));
    }
}
