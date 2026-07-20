//! Full region proof for a convex planar shell containing one cylinder shell.
//!
//! Recognition is topology and representation driven. Exactly two shells must
//! belong to the solid region: one complete planar shell and one complete
//! finite cylinder band. Independent shell certificates establish positive
//! outer and negative cavity winding. Strict support bounds over both endpoint
//! circles then prove the caps and complete linear sweep lie inside, and are
//! separated from, every convex outer support.

use crate::entity::{BodyKind, RegionId, RegionKind, Sense, ShellId, VertexId};
use crate::geom::SurfaceGeom;
use crate::shell_proof::{
    ShellEmbedding, ShellOrientation, certify_cylinder_band_shell_proof, certify_shell_in_scope,
};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kgeom::vec::{Point3, Vec3};

/// Cumulative endpoint-circle/support decisions for cylindrical cavities.
pub(crate) const CYLINDRICAL_CAVITY_REGION_WORK: StageId =
    match StageId::new("ktopo.check.cylindrical-cavity-region-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid cylindrical cavity region work stage"),
    };

const DEFAULT_CYLINDRICAL_CAVITY_REGION_WORK: u64 = 1_048_576;

pub(crate) fn cylindrical_cavity_region_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        CYLINDRICAL_CAVITY_REGION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_CYLINDRICAL_CAVITY_REGION_WORK,
    )])
    .expect("built-in cylindrical cavity region proof budget is valid")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CylindricalCavityRegionCertification {
    NotApplicable,
    Certified,
    Invalid,
    Indeterminate,
}

#[derive(Debug, Clone, Copy)]
struct PlanarSupport {
    outward: Vec3,
    origin: Point3,
}

pub(crate) fn certify_cylindrical_cavity_region_in_scope(
    store: &Store,
    region_id: RegionId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CylindricalCavityRegionCertification> {
    let region = store.get(region_id)?;
    let [first, second] = region.shells.as_slice() else {
        return Ok(CylindricalCavityRegionCertification::NotApplicable);
    };
    let first_band = certify_cylinder_band_shell_proof(store, *first)?;
    let second_band = certify_cylinder_band_shell_proof(store, *second)?;
    let (outer_shell, cavity) = match (first_band, second_band) {
        (None, Some(cavity)) if shell_is_planar(store, *first)? => (*first, cavity),
        (Some(cavity), None) if shell_is_planar(store, *second)? => (*second, cavity),
        (None, None) => return Ok(CylindricalCavityRegionCertification::NotApplicable),
        _ => return Ok(CylindricalCavityRegionCertification::NotApplicable),
    };

    let outer = certify_shell_in_scope(
        store,
        outer_shell,
        BodyKind::Solid,
        RegionKind::Solid,
        scope,
    )?;
    if outer.embedding != ShellEmbedding::Certified
        || cavity.certification.embedding != ShellEmbedding::Certified
    {
        return Ok(CylindricalCavityRegionCertification::Indeterminate);
    }
    match (outer.orientation, cavity.certification.orientation) {
        (ShellOrientation::Positive, ShellOrientation::Negative) => {}
        (ShellOrientation::Indeterminate, _) | (_, ShellOrientation::Indeterminate) => {
            return Ok(CylindricalCavityRegionCertification::Indeterminate);
        }
        _ => return Ok(CylindricalCavityRegionCertification::Invalid),
    }

    scope.ledger().require_limit(
        CYLINDRICAL_CAVITY_REGION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let Some(vertices) = planar_shell_vertices_in_scope(store, outer_shell, scope)? else {
        return Ok(CylindricalCavityRegionCertification::Indeterminate);
    };
    let outer_shell_entity = store.get(outer_shell)?;
    let Some(face_count) = u64::try_from(outer_shell_entity.faces.len()).ok() else {
        return Ok(CylindricalCavityRegionCertification::Indeterminate);
    };
    let Some(vertex_count) = u64::try_from(vertices.len()).ok() else {
        return Ok(CylindricalCavityRegionCertification::Indeterminate);
    };
    // One face visit plus every face/vertex half-space decision prepares the
    // supports; two endpoint-circle decisions per support finish containment.
    let Some(support_work) = face_count
        .checked_mul(vertex_count)
        .and_then(|work| work.checked_add(face_count.checked_mul(3)?))
    else {
        return Ok(CylindricalCavityRegionCertification::Indeterminate);
    };
    scope
        .ledger_mut()
        .charge(CYLINDRICAL_CAVITY_REGION_WORK, support_work)?;
    let Some(supports) = convex_planar_supports(store, outer_shell, &vertices)? else {
        return Ok(CylindricalCavityRegionCertification::Indeterminate);
    };

    for support in supports {
        for center in [cavity.low_center, cavity.high_center] {
            match circle_support_relation(
                support,
                cavity.cylinder.frame().x(),
                cavity.cylinder.frame().y(),
                center,
                cavity.cylinder.radius(),
            ) {
                CircleSupportRelation::Inside => {}
                CircleSupportRelation::Outside => {
                    return Ok(CylindricalCavityRegionCertification::Invalid);
                }
                CircleSupportRelation::Indeterminate => {
                    return Ok(CylindricalCavityRegionCertification::Indeterminate);
                }
            }
        }
    }
    Ok(CylindricalCavityRegionCertification::Certified)
}

fn shell_is_planar(store: &Store, shell_id: ShellId) -> Result<bool> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 4 {
        return Ok(false);
    }
    for &face_id in &shell.faces {
        if !matches!(
            store.get(store.get(face_id)?.surface)?,
            SurfaceGeom::Plane(_)
        ) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn planar_shell_vertices_in_scope(
    store: &Store,
    shell_id: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<Vec<VertexId>>> {
    let shell = store.get(shell_id)?;
    let Some(face_work) = u64::try_from(shell.faces.len()).ok() else {
        return Ok(None);
    };
    scope
        .ledger_mut()
        .charge(CYLINDRICAL_CAVITY_REGION_WORK, face_work)?;
    let mut loops = Vec::with_capacity(shell.faces.len());
    let mut fin_work = 0_u64;
    for &face_id in &shell.faces {
        let [loop_id] = store.get(face_id)?.loops.as_slice() else {
            return Ok(None);
        };
        let loop_ = store.get(*loop_id)?;
        if loop_.fins.len() < 3 {
            return Ok(None);
        }
        let Some(count) = u64::try_from(loop_.fins.len()).ok() else {
            return Ok(None);
        };
        let Some(next_work) = fin_work.checked_add(count) else {
            return Ok(None);
        };
        fin_work = next_work;
        loops.push(*loop_id);
    }
    scope
        .ledger_mut()
        .charge(CYLINDRICAL_CAVITY_REGION_WORK, fin_work)?;
    let mut vertices = Vec::<VertexId>::new();
    for loop_id in loops {
        for &fin_id in &store.get(loop_id)?.fins {
            let Some(vertex) = store.fin_tail(fin_id)? else {
                return Ok(None);
            };
            vertices.push(vertex);
        }
    }
    if vertices.len() < 4 {
        return Ok(None);
    }
    Ok(Some(vertices))
}

fn convex_planar_supports(
    store: &Store,
    shell_id: ShellId,
    vertices: &[VertexId],
) -> Result<Option<Vec<PlanarSupport>>> {
    let shell = store.get(shell_id)?;
    let mut supports = Vec::with_capacity(shell.faces.len());
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(None);
        };
        let outward = plane.frame().z()
            * if face.sense == Sense::Forward {
                1.0
            } else {
                -1.0
            };
        let support = PlanarSupport {
            outward,
            origin: plane.frame().origin(),
        };
        let mut strict = false;
        for &vertex in vertices {
            let point = store.vertex_position(vertex)?;
            let signed = interval_dot(outward, point - support.origin);
            if signed.lo() > 0.0 {
                return Ok(None);
            }
            if signed.hi() < 0.0 {
                strict = true;
            } else if !signed.contains(0.0) {
                return Ok(None);
            }
        }
        if !strict {
            return Ok(None);
        }
        supports.push(support);
    }
    Ok(Some(supports))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CircleSupportRelation {
    Inside,
    Outside,
    Indeterminate,
}

fn circle_support_relation(
    support: PlanarSupport,
    radial_x: Vec3,
    radial_y: Vec3,
    center: Point3,
    radius: f64,
) -> CircleSupportRelation {
    let signed = interval_dot(support.outward, center - support.origin);
    if signed.lo() > 0.0 {
        return CircleSupportRelation::Outside;
    }
    if signed.hi() >= 0.0 {
        return CircleSupportRelation::Indeterminate;
    }
    let radius = Interval::point(radius);
    let radial_x = interval_dot(support.outward, radial_x) * radius;
    let radial_y = interval_dot(support.outward, radial_y) * radius;
    let radial_squared = radial_x.square() + radial_y.square();
    let clearance_squared = signed.square();
    if radial_squared.hi() < clearance_squared.lo() {
        CircleSupportRelation::Inside
    } else if radial_squared.lo() > clearance_squared.hi() {
        CircleSupportRelation::Outside
    } else {
        CircleSupportRelation::Indeterminate
    }
}

fn interval_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
}
