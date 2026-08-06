//! Transitional cap-reaching family wrapper.
//!
//! The shared shell-surgery theorem owns every proof predicate.  This module
//! remains only long enough to pin exact legacy/shared routing and the retired
//! work-stage compatibility frontier.

use super::shell_lemmas::{indeterminate, proof_work_budget};
use super::*;

#[cfg(test)]
#[path = "cap_reaching_cylinder_shell_proof/tests.rs"]
mod tests;

pub(crate) const CAP_REACHING_CYLINDER_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.cap-reaching-cylinder-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid cap-reaching cylinder-shell work stage"),
    };

const DEFAULT_CAP_REACHING_CYLINDER_SHELL_WORK: u64 = 1_048_576;

pub(super) fn cap_reaching_cylinder_proof_budget() -> BudgetPlan {
    proof_work_budget(
        CAP_REACHING_CYLINDER_SHELL_WORK,
        DEFAULT_CAP_REACHING_CYLINDER_SHELL_WORK,
        "built-in cap-reaching cylinder-shell proof budget is valid",
    )
}

pub(super) fn certify_cap_reaching_cylinder_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    if scope.is_some() {
        return certify_cap_reaching_cylinder_shell_legacy(store, shell_id, scope);
    }
    let legacy = certify_cap_reaching_cylinder_shell_legacy(store, shell_id, None)?;
    #[cfg(test)]
    {
        let shared = super::shell_surgery::certify_shell_surgery(store, shell_id, None)?;
        assert_eq!(shared, legacy, "cap-reaching routing value changed");
    }
    Ok(legacy)
}

fn certify_cap_reaching_cylinder_shell_legacy(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
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
    if let Some(scope) = scope {
        scope.ledger().require_limit(
            CAP_REACHING_CYLINDER_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id, cylinders.len())? else {
            return Ok(Some(indeterminate()));
        };
        scope
            .ledger_mut()
            .charge(CAP_REACHING_CYLINDER_SHELL_WORK, work)?;
    }
    for &(host_face, cylinder) in &cylinders {
        if let Some(certification) = super::shell_surgery::certify_cap_reaching_candidate(
            store,
            shell_id,
            host_face,
            cylinder,
            cylinders.clone(),
            planar_faces.clone(),
        )? {
            return Ok(Some(certification));
        }
    }
    Ok(None)
}

fn proof_work(store: &Store, shell_id: ShellId, cylinder_count: usize) -> Result<Option<u64>> {
    super::shell_surgery::cap_reaching_proof_work(store, shell_id, cylinder_count)
}
