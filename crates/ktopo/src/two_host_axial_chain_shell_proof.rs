//! Transitional two-host family wrapper.
//!
//! The shared shell-surgery theorem owns every proof predicate. This module
//! remains only to pin exact legacy/shared routing and both retired work-stage
//! frontiers before final removal.

use super::shell_lemmas::{indeterminate, proof_work_budget};
use super::*;

#[cfg(test)]
#[path = "two_host_axial_chain_shell_proof/tests.rs"]
mod tests;

pub(crate) const PARALLEL_CYLINDER_CONTACT_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.parallel-cylinder-contact-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid parallel-cylinder contact shell work stage"),
    };
const DEFAULT_PARALLEL_CYLINDER_CONTACT_SHELL_WORK: u64 = 4096;

pub(super) fn axial_contact_proof_budget() -> BudgetPlan {
    proof_work_budget(
        PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
        DEFAULT_PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
        "built-in parallel-cylinder contact shell proof budget is valid",
    )
}

pub(crate) const TWO_HOST_AXIAL_CHAIN_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.two-host-axial-chain-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid two-host axial-chain shell work stage"),
    };
const DEFAULT_TWO_HOST_AXIAL_CHAIN_SHELL_WORK: u64 = 1_048_576;

pub(super) fn two_host_axial_chain_proof_budget() -> BudgetPlan {
    proof_work_budget(
        TWO_HOST_AXIAL_CHAIN_SHELL_WORK,
        DEFAULT_TWO_HOST_AXIAL_CHAIN_SHELL_WORK,
        "built-in two-host axial-chain shell proof budget is valid",
    )
}

pub(super) fn certify_two_host_axial_chain_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    if scope.is_some() {
        return certify_two_host_axial_chain_shell_legacy(store, shell_id, scope);
    }
    let legacy = certify_two_host_axial_chain_shell_legacy(store, shell_id, None)?;
    #[cfg(test)]
    {
        let shared = super::shell_surgery::certify_shell_surgery(store, shell_id, None)?;
        assert_eq!(shared, legacy, "two-host routing value changed");
    }
    Ok(legacy)
}

fn certify_two_host_axial_chain_shell_legacy(
    store: &Store,
    shell_id: ShellId,
    mut scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut cylinders = Vec::new();
    let mut planar_faces = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => cylinders.push((face_id, *cylinder)),
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            _ => return Ok(None),
        }
    }
    if cylinders.len() < 2 {
        return Ok(None);
    }
    if let Some(scope) = scope.as_deref_mut() {
        scope.ledger().require_limit(
            TWO_HOST_AXIAL_CHAIN_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id, cylinders.len())? else {
            return Ok(Some(indeterminate()));
        };
        scope
            .ledger_mut()
            .charge(TWO_HOST_AXIAL_CHAIN_SHELL_WORK, work)?;
    }
    let Some((certification, contact)) =
        super::shell_surgery::certify_two_host_candidate(store, shell_id, cylinders, planar_faces)?
    else {
        return Ok(None);
    };
    if contact {
        let Some(work) = super::shell_surgery::two_host_contact_work(store, shell_id)? else {
            return Ok(Some(indeterminate()));
        };
        if let Some(scope) = scope {
            scope.ledger().require_limit(
                PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
            )?;
            scope
                .ledger_mut()
                .charge(PARALLEL_CYLINDER_CONTACT_SHELL_WORK, work)?;
        }
    }
    Ok(Some(certification))
}

fn proof_work(store: &Store, shell_id: ShellId, cylinder_count: usize) -> Result<Option<u64>> {
    super::shell_surgery::two_host_proof_work(store, shell_id, cylinder_count)
}
