//! Full region proof for a finite-cylinder outer shell and convex planar cavities.

use crate::convex_containment::{
    PlanarSupport, StrictRelation, axial_lower_relation, axial_upper_relation, exact_affine,
    radial_relation,
};
use crate::entity::{BodyKind, RegionId, RegionKind, Sense, ShellId, VertexId};
use crate::geom::SurfaceGeom;
use crate::shell_proof::{
    CylinderBandShellProof, ShellEmbedding, ShellOrientation, certify_cylinder_band_shell_proof,
    certify_shell_in_scope,
};
use crate::store::Store;
use kcore::error::Result;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::Orientation;
use kgeom::vec::Point3;
use std::collections::HashSet;

/// Cumulative exact decisions for mixed convex region ownership.
pub(crate) const MIXED_CONVEX_REGION_WORK: StageId =
    match StageId::new("ktopo.check.mixed-convex-region-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid mixed convex region work stage"),
    };

const DEFAULT_MIXED_CONVEX_REGION_WORK: u64 = 1_048_576;

pub(crate) fn mixed_convex_region_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        MIXED_CONVEX_REGION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_MIXED_CONVEX_REGION_WORK,
    )])
    .expect("built-in mixed convex region proof budget is valid")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MixedConvexRegionCertification {
    NotApplicable,
    Certified,
    Invalid,
    Indeterminate,
}

#[derive(Debug)]
struct PlanarCavity {
    shell: ShellId,
    face_count: u64,
    vertices: Vec<VertexId>,
    face_vertices: Vec<HashSet<VertexId>>,
}

#[derive(Debug)]
struct ProvenPlanarCavity {
    vertices: Vec<Point3>,
    supports: Vec<PlanarSupport>,
}

pub(crate) fn certify_mixed_convex_region_in_scope(
    store: &Store,
    region_id: RegionId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<MixedConvexRegionCertification> {
    let region = store.get(region_id)?;
    if region.shells.len() < 2 {
        return Ok(MixedConvexRegionCertification::NotApplicable);
    }
    scope.ledger().require_limit(
        MIXED_CONVEX_REGION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let mut band = None;
    let mut planar_shells = Vec::new();
    for &shell in &region.shells {
        if let Some(candidate) = certify_cylinder_band_shell_proof(store, shell)? {
            if band.replace(candidate).is_some() {
                return Ok(MixedConvexRegionCertification::NotApplicable);
            }
        } else if admit_planar_shell(store, shell, scope)? {
            planar_shells.push(shell);
        } else {
            return Ok(MixedConvexRegionCertification::NotApplicable);
        }
    }
    let Some(band) = band else {
        return Ok(MixedConvexRegionCertification::NotApplicable);
    };
    if planar_shells.is_empty() {
        return Ok(MixedConvexRegionCertification::NotApplicable);
    }
    if band.certification.embedding != ShellEmbedding::Certified {
        return Ok(MixedConvexRegionCertification::Indeterminate);
    }

    if band.certification.orientation == ShellOrientation::Negative {
        // Negative-cylinder layouts belong to another representation proof.
        return Ok(MixedConvexRegionCertification::NotApplicable);
    }
    if band.certification.orientation == ShellOrientation::Indeterminate {
        return Ok(MixedConvexRegionCertification::Indeterminate);
    }
    if band.certification.orientation != ShellOrientation::Positive {
        return Ok(MixedConvexRegionCertification::Invalid);
    }
    for &shell in &planar_shells {
        let proof =
            certify_shell_in_scope(store, shell, BodyKind::Solid, RegionKind::Solid, scope)?;
        if proof.embedding != ShellEmbedding::Certified {
            return Ok(MixedConvexRegionCertification::Indeterminate);
        }
    }

    let mut cavities = Vec::with_capacity(planar_shells.len());
    for shell in planar_shells {
        let Some(cavity) = collect_planar_cavity(store, shell, scope)? else {
            return Ok(MixedConvexRegionCertification::Indeterminate);
        };
        cavities.push(cavity);
    }
    charge_pair_work(&cavities, scope)?;

    let mut proven = Vec::with_capacity(cavities.len());
    for cavity in &cavities {
        let proof = match prove_planar_cavity(store, cavity)? {
            ProofRelation::Certified(proof) => proof,
            ProofRelation::Invalid => return Ok(MixedConvexRegionCertification::Invalid),
            ProofRelation::Indeterminate => {
                return Ok(MixedConvexRegionCertification::Indeterminate);
            }
        };
        match prove_cavity_inside_band(&proof, &band) {
            Relation::Certified => {}
            Relation::Invalid => return Ok(MixedConvexRegionCertification::Invalid),
            Relation::Indeterminate => {
                return Ok(MixedConvexRegionCertification::Indeterminate);
            }
        }
        proven.push(proof);
    }
    for first in 0..proven.len() {
        for second in first + 1..proven.len() {
            if !prove_pair_separated(&proven[first], &proven[second])? {
                return Ok(MixedConvexRegionCertification::Indeterminate);
            }
        }
    }
    Ok(MixedConvexRegionCertification::Certified)
}

fn admit_planar_shell(
    store: &Store,
    shell: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<bool> {
    let shell = store.get(shell)?;
    if shell.faces.len() < 4 {
        return Ok(false);
    }
    scope
        .ledger_mut()
        .charge(MIXED_CONVEX_REGION_WORK, as_u64(shell.faces.len())?)?;
    for &face in &shell.faces {
        if !matches!(store.get(store.get(face)?.surface)?, SurfaceGeom::Plane(_)) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn collect_planar_cavity(
    store: &Store,
    shell: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<PlanarCavity>> {
    let shell_entity = store.get(shell)?;
    if shell_entity.faces.len() < 4 {
        return Ok(None);
    }
    let face_count = as_u64(shell_entity.faces.len())?;
    scope
        .ledger_mut()
        .charge(MIXED_CONVEX_REGION_WORK, face_count)?;
    let mut loops = Vec::with_capacity(shell_entity.faces.len());
    let mut use_count = 0_u64;
    for &face_id in &shell_entity.faces {
        let face = store.get(face_id)?;
        if !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
            return Ok(None);
        }
        let [loop_id] = face.loops.as_slice() else {
            return Ok(None);
        };
        let loop_ = store.get(*loop_id)?;
        if loop_.fins.len() < 3 {
            return Ok(None);
        }
        let fin_count = as_u64(loop_.fins.len())?;
        use_count = use_count.checked_add(fin_count).ok_or_else(work_error)?;
        loops.push(*loop_id);
    }
    scope
        .ledger_mut()
        .charge(MIXED_CONVEX_REGION_WORK, use_count)?;
    let capacity = usize::try_from(use_count).map_err(|_| work_error())?;
    let mut seen = HashSet::with_capacity(capacity);
    let mut vertices = Vec::with_capacity(capacity);
    let mut face_vertices = Vec::with_capacity(loops.len());
    for loop_id in loops {
        let mut incident = HashSet::with_capacity(store.get(loop_id)?.fins.len());
        for &fin in &store.get(loop_id)?.fins {
            let Some(vertex) = store.fin_tail(fin)? else {
                return Ok(None);
            };
            incident.insert(vertex);
            if seen.insert(vertex) {
                vertices.push(vertex);
            }
        }
        face_vertices.push(incident);
    }
    if vertices.len() < 4 {
        return Ok(None);
    }
    let vertex_count = as_u64(vertices.len())?;
    let decisions = face_count
        .checked_mul(vertex_count)
        .and_then(|value| value.checked_add(vertex_count.checked_mul(3)?))
        .ok_or_else(work_error)?;
    scope
        .ledger_mut()
        .charge(MIXED_CONVEX_REGION_WORK, decisions)?;
    Ok(Some(PlanarCavity {
        shell,
        face_count,
        vertices,
        face_vertices,
    }))
}

/// Charge both directed support/vertex matrices per unordered cavity pair.
fn charge_pair_work(cavities: &[PlanarCavity], scope: &mut OperationScope<'_, '_>) -> Result<()> {
    let mut work = 0_u64;
    for first in 0..cavities.len() {
        for second in first + 1..cavities.len() {
            let first_vertices = as_u64(cavities[first].vertices.len())?;
            let second_vertices = as_u64(cavities[second].vertices.len())?;
            work = work
                .checked_add(
                    cavities[first]
                        .face_count
                        .checked_mul(second_vertices)
                        .ok_or_else(work_error)?,
                )
                .and_then(|value| {
                    value.checked_add(cavities[second].face_count.checked_mul(first_vertices)?)
                })
                .ok_or_else(work_error)?;
        }
    }
    scope.ledger_mut().charge(MIXED_CONVEX_REGION_WORK, work)?;
    Ok(())
}

enum ProofRelation {
    Certified(ProvenPlanarCavity),
    Invalid,
    Indeterminate,
}

fn prove_planar_cavity(store: &Store, cavity: &PlanarCavity) -> Result<ProofRelation> {
    let vertices = cavity
        .vertices
        .iter()
        .map(|&vertex| store.vertex_position(vertex))
        .collect::<Result<Vec<_>>>()?;
    let mut supports = Vec::with_capacity(usize::try_from(cavity.face_count).unwrap_or(0));
    for (&face_id, incident) in store
        .get(cavity.shell)?
        .faces
        .iter()
        .zip(&cavity.face_vertices)
    {
        let face = store.get(face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(ProofRelation::Indeterminate);
        };
        // The shell is negative, so the underlying convex solid's outward
        // support direction opposes the oriented boundary normal.
        let outward = plane.frame().z()
            * if face.sense == Sense::Forward {
                -1.0
            } else {
                1.0
            };
        let support = PlanarSupport {
            outward,
            origin: plane.frame().origin(),
        };
        let mut nonincident = false;
        for (&vertex_id, &vertex) in cavity.vertices.iter().zip(&vertices) {
            if incident.contains(&vertex_id) {
                continue;
            }
            nonincident = true;
            match exact_affine(outward, vertex, support.origin) {
                Ok(Orientation::Negative) => {}
                Ok(Orientation::Zero) => return Ok(ProofRelation::Indeterminate),
                Ok(Orientation::Positive) => return Ok(ProofRelation::Invalid),
                Err(_) => return Ok(ProofRelation::Indeterminate),
            }
        }
        if !nonincident {
            return Ok(ProofRelation::Indeterminate);
        }
        supports.push(support);
    }
    Ok(ProofRelation::Certified(ProvenPlanarCavity {
        vertices,
        supports,
    }))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Relation {
    Certified,
    Invalid,
    Indeterminate,
}

fn prove_cavity_inside_band(
    cavity: &ProvenPlanarCavity,
    band: &CylinderBandShellProof,
) -> Relation {
    let frame = band.cylinder.frame();
    for &vertex in &cavity.vertices {
        for relation in [
            axial_lower_relation(vertex, band.low_center, frame.z()),
            axial_upper_relation(vertex, band.high_center, frame.z()),
            radial_relation(
                vertex,
                band.low_center,
                frame.x(),
                frame.y(),
                band.cylinder.radius(),
            ),
        ] {
            match relation {
                StrictRelation::Inside => {}
                StrictRelation::Outside => return Relation::Invalid,
                StrictRelation::Indeterminate => return Relation::Indeterminate,
            }
        }
    }
    Relation::Certified
}

fn prove_pair_separated(first: &ProvenPlanarCavity, second: &ProvenPlanarCavity) -> Result<bool> {
    Ok(separated_by_supports(&first.supports, &second.vertices)?
        || separated_by_supports(&second.supports, &first.vertices)?)
}

fn separated_by_supports(supports: &[PlanarSupport], vertices: &[Point3]) -> Result<bool> {
    for support in supports {
        let mut separates = true;
        for &vertex in vertices {
            match exact_affine(support.outward, vertex, support.origin) {
                Ok(Orientation::Positive) => {}
                Ok(Orientation::Negative | Orientation::Zero) => {
                    separates = false;
                    break;
                }
                Err(_) => return Ok(false),
            }
        }
        if separates {
            return Ok(true);
        }
    }
    Ok(false)
}

fn as_u64(value: usize) -> Result<u64> {
    u64::try_from(value).map_err(|_| work_error())
}

fn work_error() -> kcore::error::Error {
    kcore::error::Error::InvalidGeometry {
        reason: "mixed convex region work count overflow",
    }
}
