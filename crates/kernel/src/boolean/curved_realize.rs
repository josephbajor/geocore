//! Atomic realization of boundaries selected by the curved Boolean spine.

use kcore::operation::{OperationScope, ResourceKind};
use kgeom::frame::Frame;
use ktopo::analytic_shell::{AnalyticShellAssemblyError, AnalyticShellInput};
use ktopo::entity::BodyId as RawBodyId;
use ktopo::transaction::{FullCommitRequirement, Transaction};

use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, PipelineFailure, StageResult,
};
use super::curved_source::CertifiedCylinderSource;
use super::pipeline::{PLANAR_BOOLEAN_REALIZATION_WORK, PLANAR_BOOLEAN_REALIZED_VERTICES};
use crate::BodyId;
use crate::error::Error;
use crate::session::PartEdit;

/// Copy proof-certified finite cylinders as one atomic, Full-certified result.
///
/// Source order is caller-owned and becomes public result-body/report order.
/// Each source body is first bound to its Full-checked finite-cylinder shell;
/// this keeps the exact structural copy bound local to the topology class for
/// which it is established.
pub(super) fn realize_certified_cylinder_source_copies(
    edit: &mut PartEdit<'_>,
    sources: &[(BodyId, &CertifiedCylinderSource)],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if sources.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    let store = &edit.state.store;
    for (body, source) in sources {
        let shell = store.get(source.shell())?;
        let region = store.get(shell.region())?;
        if region.body() != body.raw() {
            return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
                "certified cylinder source was not owned by its realization body",
            ));
        }
    }
    let sources = sources
        .iter()
        .map(|(body, _)| body.raw())
        .collect::<Vec<_>>();
    commit_source_body_copies(edit, &sources, scope)
}

/// Copy complete source bodies selected by the generic boundary spine.
pub(super) fn realize_source_body_copies(
    edit: &mut PartEdit<'_>,
    sources: &[BodyId],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let sources = sources.iter().map(BodyId::raw).collect::<Vec<_>>();
    commit_source_body_copies(edit, &sources, scope)
}

/// Realize independently connected analytic shells as one atomic result.
///
/// Every component is charged and then preflighted by the topology batch
/// adapter before the first allocation. Component order is caller-owned and
/// becomes public result-body/report order after the single Full commit.
pub(super) fn realize_analytic_shell_inputs(
    edit: &mut PartEdit<'_>,
    inputs: &[AnalyticShellInput],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if inputs.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    let vertices = inputs.iter().try_fold(0_u64, |total, input| {
        let count = u64::try_from(input.vertices().len()).map_err(|_| work_overflow())?;
        total.checked_add(count).ok_or_else(work_overflow)
    })?;
    for input in inputs {
        precharge_analytic_shell_work(input, scope)?;
    }
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            vertices,
        )
        .map_err(Error::from)?;

    let part = edit.id.clone();
    let mut transaction = edit.state.store.transaction().map_err(Error::from)?;
    let outputs = match transaction.assemble_analytic_shell_batch(inputs, linear) {
        Ok(outputs) => outputs,
        Err(AnalyticShellAssemblyError::Preflight(_)) => {
            return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
                "analytic shell batch failed complete preflight",
            ));
        }
        Err(AnalyticShellAssemblyError::Store(source)) => return Err(source.into()),
        Err(_) => {
            return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
                "analytic shell assembly returned an unsupported refusal",
            ));
        }
    };
    let bodies = outputs.iter().map(|output| output.body()).collect();
    commit_full(part, transaction, bodies, scope)
}

/// Realize one exterior analytic shell and its cavity shells in a shared
/// solid region.
pub(super) fn realize_analytic_shell_region(
    edit: &mut PartEdit<'_>,
    inputs: &[AnalyticShellInput],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if inputs.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    let vertices = inputs.iter().try_fold(0_u64, |total, input| {
        let count = u64::try_from(input.vertices().len()).map_err(|_| work_overflow())?;
        total.checked_add(count).ok_or_else(work_overflow)
    })?;
    for input in inputs {
        precharge_analytic_shell_work(input, scope)?;
    }
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            vertices,
        )
        .map_err(Error::from)?;

    let part = edit.id.clone();
    let mut transaction = edit.state.store.transaction().map_err(Error::from)?;
    let outputs = match transaction.assemble_analytic_shell_region(inputs, linear) {
        Ok(outputs) => outputs,
        Err(AnalyticShellAssemblyError::Preflight(_)) => {
            return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
                "analytic shell region failed complete preflight",
            ));
        }
        Err(AnalyticShellAssemblyError::Store(source)) => return Err(source.into()),
        Err(_) => {
            return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
                "analytic shell region returned an unsupported refusal",
            ));
        }
    };
    let body = outputs
        .first()
        .map(ktopo::analytic_shell::AnalyticShellOutput::body)
        .ok_or_else(|| {
            refused_error(CurvedBooleanPipelineRefusal::AssemblyContract(
                "analytic shell region produced no exterior shell",
            ))
        })?;
    if outputs.iter().any(|output| output.body() != body) {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "analytic shell region split one solid across bodies",
        ));
    }
    commit_full(part, transaction, vec![body], scope)
}

fn commit_source_body_copies(
    edit: &mut PartEdit<'_>,
    sources: &[RawBodyId],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if sources.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    precharge_source_body_copies(edit, sources, scope)?;
    let part = edit.id.clone();
    let mut transaction = edit.state.store.transaction().map_err(Error::from)?;
    let mut bodies = Vec::with_capacity(sources.len());
    for source in sources {
        let body = transaction
            .copy_body_rigid_with_source(*source, Frame::world())
            .map_err(Error::from_body_copy)?;
        bodies.push(body);
    }
    commit_full(part, transaction, bodies, scope)
}

fn commit_full(
    part: crate::PartId,
    transaction: Transaction<'_>,
    raw_bodies: Vec<RawBodyId>,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let decision = match transaction.commit_full_in_scope(
        &raw_bodies,
        FullCommitRequirement::RequireValid,
        scope,
        0,
    ) {
        Ok(decision) => decision,
        Err(kcore::error::Error::TopologyCheckFailed { fault_count }) => {
            return refused(CurvedBooleanPipelineRefusal::FullTopologyFault { fault_count });
        }
        Err(source) => return Err(source.into()),
    };
    let (journal, full_checks) = decision.into_parts();
    let Some(journal) = journal else {
        return Ok(CurvedBooleanPipelineOutcome::Refused(
            CurvedBooleanPipelineRefusal::FullProofRejected(full_checks),
        ));
    };
    let bodies = raw_bodies
        .into_iter()
        .map(|body| BodyId::new(part.clone(), body))
        .collect();
    Ok(CurvedBooleanPipelineOutcome::Committed(
        super::curved_pipeline::CommittedCurvedBoolean::new(bodies, journal, full_checks),
    ))
}

/// Charge a conservative checked bound for analytic-shell preflight and
/// allocation before opening the transaction.
fn precharge_analytic_shell_work(
    input: &AnalyticShellInput,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<()> {
    let mut loop_count = 0_usize;
    let mut fin_count = 0_usize;
    for face in input.faces() {
        loop_count = loop_count
            .checked_add(face.loops().len())
            .ok_or_else(work_overflow)?;
        for loop_ in face.loops() {
            fin_count = fin_count
                .checked_add(loop_.fins().len())
                .ok_or_else(work_overflow)?;
        }
    }
    let work = analytic_shell_realization_work(
        input.vertices().len(),
        input.edges().len(),
        input.closed_edges().len(),
        input.faces().len(),
        loop_count,
        fin_count,
    )?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, work)
        .map_err(Error::from)?;
    Ok(())
}

/// Conservative structural ceiling for analytic-shell preflight/allocation.
fn analytic_shell_realization_work(
    vertices: usize,
    edges: usize,
    closed_edges: usize,
    faces: usize,
    loops: usize,
    uses: usize,
) -> StageResult<u64> {
    let vertices = u64::try_from(vertices).map_err(|_| work_overflow())?;
    let edges = u64::try_from(edges).map_err(|_| work_overflow())?;
    let closed_edges = u64::try_from(closed_edges).map_err(|_| work_overflow())?;
    let faces = u64::try_from(faces).map_err(|_| work_overflow())?;
    let loops = u64::try_from(loops).map_err(|_| work_overflow())?;
    let uses = u64::try_from(uses).map_err(|_| work_overflow())?;
    let size = 1_u64
        .checked_add(vertices)
        .and_then(|value| value.checked_add(edges))
        .and_then(|value| value.checked_add(closed_edges))
        .and_then(|value| value.checked_add(faces))
        .and_then(|value| value.checked_add(loops))
        .and_then(|value| value.checked_add(uses))
        .ok_or_else(work_overflow)?;
    size.checked_mul(size)
        .and_then(|value| value.checked_add(size.checked_mul(16)?))
        .ok_or_else(work_overflow)
}

/// Charge a conservative identity-copy bound before opening the transaction.
fn precharge_source_body_copies(
    edit: &PartEdit<'_>,
    sources: &[RawBodyId],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<()> {
    let store = &edit.state.store;
    let mut work = 0_u64;
    let mut vertices = 0_u64;
    for source in sources {
        add_copy_work(&mut work, 1)?;
        for region in store.get(*source)?.regions() {
            add_copy_work(&mut work, 1)?;
            for shell in store.get(*region)?.shells() {
                add_copy_work(&mut work, 1)?;
                for face in store.get(*shell)?.faces() {
                    add_copy_work(&mut work, 2)?;
                    for loop_id in store.get(*face)?.loops() {
                        add_copy_work(&mut work, 1)?;
                        for fin in store.get(*loop_id)?.fins() {
                            add_copy_work(
                                &mut work,
                                if store.get(*fin)?.pcurve().is_some() {
                                    2
                                } else {
                                    1
                                },
                            )?;
                        }
                    }
                }
            }
        }
        for edge in store.edges_of_body(*source)? {
            add_copy_work(
                &mut work,
                if store.get(edge)?.curve().is_some() {
                    2
                } else {
                    1
                },
            )?;
        }
        let source_vertices = store.vertices_of_body(*source)?;
        let vertex_count = u64::try_from(source_vertices.len()).map_err(|_| work_overflow())?;
        vertices = vertices
            .checked_add(vertex_count)
            .ok_or_else(work_overflow)?;
        work = work
            .checked_add(vertex_count.checked_mul(2).ok_or_else(work_overflow)?)
            .ok_or_else(work_overflow)?;
    }
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, work)
        .map_err(Error::from)?;
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            vertices,
        )
        .map_err(Error::from)?;
    Ok(())
}

fn add_copy_work(work: &mut u64, amount: usize) -> StageResult<()> {
    let amount = u64::try_from(amount).map_err(|_| work_overflow())?;
    *work = work.checked_add(amount).ok_or_else(work_overflow)?;
    Ok(())
}

fn work_overflow() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow)
}

fn refused_error(refusal: CurvedBooleanPipelineRefusal) -> PipelineFailure {
    PipelineFailure::Refused(refusal)
}

fn refused<T>(refusal: CurvedBooleanPipelineRefusal) -> StageResult<T> {
    Err(refused_error(refusal))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analytic_shell_realization_work_counts_complete_structure() {
        // Half-cylinder shell: V=4, Eb=4, Ec=2, F=4, L=4, U=12, hence N=31.
        assert_eq!(
            analytic_shell_realization_work(4, 4, 2, 4, 4, 12).unwrap(),
            1_457
        );
        assert_eq!(
            analytic_shell_realization_work(4, 4, 0, 4, 4, 12).unwrap(),
            1_305
        );
    }

    #[test]
    fn analytic_shell_realization_work_fails_closed_on_overflow() {
        assert!(analytic_shell_realization_work(usize::MAX, 0, 0, 0, 0, 0).is_err());
        assert!(
            analytic_shell_realization_work(
                (u64::MAX / 2) as usize,
                (u64::MAX / 2) as usize,
                usize::MAX,
                0,
                0,
                0,
            )
            .is_err()
        );
    }
}
