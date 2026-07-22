//! Atomic realization of proof-selected curved Boolean boundaries.
//!
//! Recognition remains in carrier-specific adapters. This module owns the
//! common result proposal ordering, exact precharge, semantic topology
//! assembly, and single Full-checked transaction.

use std::collections::{BTreeMap, BTreeSet};

use kcore::operation::{OperationScope, ResourceKind};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use ktopo::analytic_shell::{AnalyticShellAssemblyError, AnalyticShellInput};
use ktopo::cylindrical_band::CylindricalBandSolidInput;
use ktopo::entity::{BodyId as RawBodyId, FaceId as RawFaceId};
use ktopo::transaction::{FullCommitRequirement, Transaction};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::convex_containment::{
    PreparedMixedConvexContainment, admit_mixed_convex_containment,
    prepare_admitted_mixed_convex_containment,
};
use super::curved_cavity::{PreparedCylindricalCavity, prepare_cylindrical_cavity};
use super::curved_contact::{
    PreparedSupportContact, admit_support_contact, prepare_admitted_support_contact,
};
use super::curved_host_bands::{PreparedCylindricalHostBands, prepare_cylindrical_host_bands};
use super::curved_pipeline::{
    CertifiedRingCut, CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, CurvedFragment,
    CurvedFragmentKey, PipelineFailure, StageResult,
};
use super::curved_source::CertifiedCylinderSource;
use super::curved_support_separation::CertifiedAxialCapContact;
use super::extract::CertifiedConvexPlanarSource;
use super::face_partition::{AxialBoundary, FaceRegionKey};
use super::pipeline::{PLANAR_BOOLEAN_REALIZATION_WORK, PLANAR_BOOLEAN_REALIZED_VERTICES};
use crate::BodyId;
use crate::error::Error;
use crate::session::PartEdit;

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

#[derive(Debug, Clone)]
enum PreparedCurvedResult {
    SupportContact(Box<PreparedSupportContact>),
    MixedConvexContainment(Box<PreparedMixedConvexContainment>),
    CylindricalCavity(Box<PreparedCylindricalCavity>),
    CylindricalHostBands(Box<PreparedCylindricalHostBands>),
    CylindricalBands(Vec<CylindricalBandSolidInput>),
    WholeSources(Vec<RawBodyId>),
}

impl PreparedCurvedResult {
    fn is_empty(&self) -> bool {
        match self {
            Self::SupportContact(_)
            | Self::MixedConvexContainment(_)
            | Self::CylindricalCavity(_)
            | Self::CylindricalHostBands(_) => false,
            Self::CylindricalBands(bands) => bands.is_empty(),
            Self::WholeSources(sources) => sources.is_empty(),
        }
    }
}

pub(super) struct CurvedRealizationRequest<'a> {
    bodies: &'a [BodyId; 2],
    source_boundary_keys: &'a BTreeMap<OperandSide, BTreeSet<CurvedFragmentKey>>,
    planar: &'a CertifiedConvexPlanarSource,
    cylinder: &'a CertifiedCylinderSource,
    cuts: &'a [CertifiedRingCut],
    contact: Option<&'a CertifiedAxialCapContact>,
    selected: Vec<SelectedCurvedFragment>,
}

impl<'a> CurvedRealizationRequest<'a> {
    pub(super) fn new(
        bodies: &'a [BodyId; 2],
        source_boundary_keys: &'a BTreeMap<OperandSide, BTreeSet<CurvedFragmentKey>>,
        planar: &'a CertifiedConvexPlanarSource,
        cylinder: &'a CertifiedCylinderSource,
        cuts: &'a [CertifiedRingCut],
        contact: Option<&'a CertifiedAxialCapContact>,
        selected: Vec<SelectedCurvedFragment>,
    ) -> Self {
        Self {
            bodies,
            source_boundary_keys,
            planar,
            cylinder,
            cuts,
            contact,
            selected,
        }
    }
}

pub(super) fn realize_selected_result(
    edit: &mut PartEdit<'_>,
    request: CurvedRealizationRequest<'_>,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let proposals = prepare_result_proposals(request, scope)?;
    if proposals.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    commit_proposals(edit, proposals, scope)
}

/// Realize one completely authored bounded analytic shell atomically.
///
/// All deterministic post-selection work and the exact proposed vertex
/// high-water are charged before the transaction is opened. The topology
/// producer performs its complete allocation-free preflight before its first
/// insertion; any proposal defect is therefore an honest assembly refusal,
/// while store failures remain execution failures. The shared commit helper
/// then requires a successful persisted Full proof before exposing a journal.
pub(super) fn realize_analytic_shell_input(
    edit: &mut PartEdit<'_>,
    input: &AnalyticShellInput,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    realize_analytic_shell_inputs(edit, core::slice::from_ref(input), linear, scope)
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
    for input in inputs {
        precharge_analytic_shell(input, scope)?;
    }

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

fn prepare_result_proposals(
    request: CurvedRealizationRequest<'_>,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<PreparedCurvedResult> {
    let CurvedRealizationRequest {
        bodies,
        source_boundary_keys,
        planar,
        cylinder,
        cuts,
        contact,
        selected,
    } = request;
    if let Some(contact) = contact {
        let Some(admission) = admit_support_contact(planar, cylinder, *contact, &selected)
            .map_err(assembly_contract)?
        else {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        };
        precharge_planar_curved_assembly(
            admission.host_vertices(),
            admission.host_faces(),
            admission.host_face_uses(),
            admission.semantic_preflight_work(),
            scope,
        )?;
        let prepared = prepare_admitted_support_contact(admission, planar, cylinder, *contact)
            .map_err(assembly_contract)?;
        return Ok(PreparedCurvedResult::SupportContact(Box::new(prepared)));
    }
    if let Some(admission) =
        admit_mixed_convex_containment(source_boundary_keys, planar, cuts, &selected)
            .map_err(assembly_contract)?
    {
        precharge_planar_curved_assembly(
            admission.planar_vertices(),
            admission.planar_faces(),
            admission.planar_face_uses(),
            admission.semantic_preflight_work(),
            scope,
        )?;
        let containment = prepare_admitted_mixed_convex_containment(admission, planar, cylinder)
            .map_err(assembly_contract)?;
        return Ok(PreparedCurvedResult::MixedConvexContainment(Box::new(
            containment,
        )));
    }
    if let Some(source_copies) =
        prepare_whole_source_copies(bodies, source_boundary_keys, &selected)
    {
        return Ok(PreparedCurvedResult::WholeSources(source_copies));
    }
    if let Some(cavity) =
        prepare_cylindrical_cavity(planar, cylinder, cuts, &selected).map_err(assembly_contract)?
    {
        return Ok(PreparedCurvedResult::CylindricalCavity(Box::new(cavity)));
    }
    if let Some(host_bands) = prepare_cylindrical_host_bands(planar, cylinder, cuts, &selected)
        .map_err(assembly_contract)?
    {
        return Ok(PreparedCurvedResult::CylindricalHostBands(Box::new(
            host_bands,
        )));
    }
    prepare_band_proposals(cylinder, cuts, selected).map(PreparedCurvedResult::CylindricalBands)
}

/// Accept a copy only when truth selection retained every canonical cell of
/// each represented source boundary and retained no partial source boundary.
/// Reversed whole boundaries describe cavities, so another adapter owns them.
fn prepare_whole_source_copies(
    bodies: &[BodyId; 2],
    source_boundary_keys: &BTreeMap<OperandSide, BTreeSet<CurvedFragmentKey>>,
    selected: &[SelectedCurvedFragment],
) -> Option<Vec<RawBodyId>> {
    let mut selected_keys = BTreeMap::<OperandSide, BTreeSet<CurvedFragmentKey>>::new();
    for fragment in selected {
        if fragment.orientation() != SelectedOrientation::Preserved {
            return None;
        }
        selected_keys
            .entry(fragment.operand())
            .or_default()
            .insert(fragment.key().clone());
    }
    if selected_keys.is_empty() {
        return Some(Vec::new());
    }

    let mut proposals = Vec::with_capacity(selected_keys.len());
    for (operand, keys) in selected_keys {
        if source_boundary_keys.get(&operand) != Some(&keys) {
            return None;
        }
        let body = match operand {
            OperandSide::Left => bodies[0].raw(),
            OperandSide::Right => bodies[1].raw(),
        };
        proposals.push(body);
    }
    Some(proposals)
}

fn prepare_band_proposals(
    source: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: Vec<SelectedCurvedFragment>,
) -> StageResult<Vec<CylindricalBandSolidInput>> {
    let mut planar_disks = BTreeMap::<usize, RawFaceId>::new();
    let mut source_caps = [None; 2];
    let mut side_bands = Vec::<(AxialBoundary<usize>, AxialBoundary<usize>)>::new();
    for selected in selected {
        let (_, _, fragment, orientation) = selected.into_parts();
        match (fragment, orientation) {
            (
                CurvedFragment::Planar {
                    face,
                    region: FaceRegionKey::PlanarDisk(cut),
                },
                // Band assembly regenerates result-oriented cap geometry;
                // this selected face supplies lineage only, so reversal is
                // admissible here but not for side or source-cap fragments.
                SelectedOrientation::Preserved | SelectedOrientation::Reversed,
            ) => {
                if planar_disks.insert(cut, face).is_some() {
                    return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
                }
            }
            (
                CurvedFragment::CylinderSide {
                    region: FaceRegionKey::AxialBand { lower, upper },
                },
                SelectedOrientation::Preserved,
            ) => side_bands.push((lower, upper)),
            (CurvedFragment::CylinderCap { face, boundary }, SelectedOrientation::Preserved)
                if boundary < 2 =>
            {
                if source_caps[boundary].replace(face).is_some() {
                    return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
                }
            }
            _ => return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
        }
    }

    let cut_data = cuts
        .iter()
        .map(|cut| (cut.key, (cut.exact_order, cut.axial_parameter)))
        .collect::<BTreeMap<_, _>>();
    side_bands.sort_by_key(|(lower, _)| boundary_rank(lower, &cut_data));
    let bounds = source.boundaries();
    let source_parameters = [
        axial_parameter(source, bounds[0].center()).ok_or_else(section_contract)?,
        axial_parameter(source, bounds[1].center()).ok_or_else(section_contract)?,
    ];
    let mut proposals = Vec::with_capacity(side_bands.len());
    for (lower, upper) in side_bands {
        let Some(lower_rank) = boundary_rank(&lower, &cut_data) else {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        };
        let Some(upper_rank) = boundary_rank(&upper, &cut_data) else {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        };
        if lower_rank.checked_add(1) != Some(upper_rank) {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        }
        let low_parameter = boundary_parameter(&lower, source_parameters, &cut_data)
            .ok_or_else(section_contract)?;
        let high_parameter = boundary_parameter(&upper, source_parameters, &cut_data)
            .ok_or_else(section_contract)?;
        let low_face = consume_cap_source(&lower, &mut planar_disks, &mut source_caps)
            .ok_or_else(result_topology_contract)?;
        let high_face = consume_cap_source(&upper, &mut planar_disks, &mut source_caps)
            .ok_or_else(result_topology_contract)?;
        let cylinder = source.cylinder();
        proposals.push(
            CylindricalBandSolidInput::new(
                *cylinder.frame(),
                cylinder.radius(),
                ParamRange::new(low_parameter, high_parameter),
            )
            .with_side_source(source.side_face())
            .with_cap_sources([Some(low_face), Some(high_face)]),
        );
    }
    if !planar_disks.is_empty() || source_caps.iter().any(Option::is_some) {
        return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
    }
    Ok(proposals)
}

fn boundary_rank(
    boundary: &AxialBoundary<usize>,
    cuts: &BTreeMap<usize, (usize, f64)>,
) -> Option<usize> {
    match boundary {
        AxialBoundary::LowerSource => Some(0),
        AxialBoundary::Cut(cut) => cuts.get(cut).and_then(|(order, _)| order.checked_add(1)),
        AxialBoundary::UpperSource => cuts.len().checked_add(1),
    }
}

fn boundary_parameter(
    boundary: &AxialBoundary<usize>,
    source: [f64; 2],
    cuts: &BTreeMap<usize, (usize, f64)>,
) -> Option<f64> {
    match boundary {
        AxialBoundary::LowerSource => Some(source[0]),
        AxialBoundary::Cut(cut) => cuts.get(cut).map(|(_, parameter)| *parameter),
        AxialBoundary::UpperSource => Some(source[1]),
    }
}

fn consume_cap_source(
    boundary: &AxialBoundary<usize>,
    planar_disks: &mut BTreeMap<usize, RawFaceId>,
    source_caps: &mut [Option<RawFaceId>; 2],
) -> Option<RawFaceId> {
    match boundary {
        AxialBoundary::LowerSource => source_caps[0].take(),
        AxialBoundary::Cut(cut) => planar_disks.remove(cut),
        AxialBoundary::UpperSource => source_caps[1].take(),
    }
}

fn commit_proposals(
    edit: &mut PartEdit<'_>,
    proposals: PreparedCurvedResult,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    match &proposals {
        PreparedCurvedResult::SupportContact(_)
        | PreparedCurvedResult::MixedConvexContainment(_) => {}
        PreparedCurvedResult::CylindricalCavity(cavity) => {
            precharge_cylindrical_cavity(cavity, scope)?
        }
        PreparedCurvedResult::CylindricalHostBands(host_bands) => precharge_planar_curved_assembly(
            host_bands.host_vertices(),
            host_bands.host_faces(),
            host_bands.host_face_uses(),
            host_bands.semantic_preflight_work(),
            scope,
        )?,
        PreparedCurvedResult::WholeSources(sources) => {
            precharge_whole_source_copies(edit, sources, scope)?;
        }
        PreparedCurvedResult::CylindricalBands(_) => {}
    }
    let part = edit.id.clone();
    let mut transaction = edit.state.store.transaction().map_err(Error::from)?;
    let mut raw_bodies = match &proposals {
        PreparedCurvedResult::SupportContact(_)
        | PreparedCurvedResult::MixedConvexContainment(_)
        | PreparedCurvedResult::CylindricalCavity(_)
        | PreparedCurvedResult::CylindricalHostBands(_) => Vec::with_capacity(1),
        PreparedCurvedResult::CylindricalBands(bands) => Vec::with_capacity(bands.len()),
        PreparedCurvedResult::WholeSources(sources) => Vec::with_capacity(sources.len()),
    };
    match proposals {
        PreparedCurvedResult::SupportContact(proposal) => {
            push_assembled(
                &mut raw_bodies,
                transaction.assemble_cylindrical_host_solid(&proposal.into_input()),
                |output| output.body(),
            )?;
        }
        PreparedCurvedResult::MixedConvexContainment(proposal) => {
            push_assembled(
                &mut raw_bodies,
                transaction.assemble_mixed_convex_multishell_solid(&proposal.into_input()),
                |output| output.body(),
            )?;
        }
        PreparedCurvedResult::CylindricalCavity(proposal) => {
            push_assembled(
                &mut raw_bodies,
                transaction.assemble_cylindrical_cavity_solid(&proposal.into_input()),
                |output| output.body(),
            )?;
        }
        PreparedCurvedResult::CylindricalHostBands(proposal) => {
            push_assembled(
                &mut raw_bodies,
                transaction.assemble_cylindrical_host_solid(&proposal.into_input()),
                |output| output.body(),
            )?;
        }
        PreparedCurvedResult::CylindricalBands(proposals) => {
            for proposal in proposals {
                push_assembled(
                    &mut raw_bodies,
                    transaction.assemble_cylindrical_band_solid(&proposal),
                    |output| output.body(),
                )?;
            }
        }
        PreparedCurvedResult::WholeSources(sources) => {
            for source in sources {
                let body = transaction
                    .copy_body_rigid_with_source(source, Frame::world())
                    .map_err(Error::from_body_copy)?;
                raw_bodies.push(body);
            }
        }
    }
    commit_full(part, transaction, raw_bodies, scope)
}

fn push_assembled<T>(
    bodies: &mut Vec<RawBodyId>,
    assembled: kcore::error::Result<T>,
    body: impl FnOnce(T) -> RawBodyId,
) -> StageResult<()> {
    match assembled {
        Ok(output) => bodies.push(body(output)),
        Err(kcore::error::Error::InvalidGeometry { reason }) => {
            return refused(CurvedBooleanPipelineRefusal::AssemblyContract(reason));
        }
        Err(source) => return Err(source.into()),
    }
    Ok(())
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

/// Charge a source-size-exact conservative bound before mixed curved allocation.
fn precharge_planar_curved_assembly(
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
    semantic_preflight_work: u64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<()> {
    let vertices = u64::try_from(host_vertices).map_err(|_| work_overflow())?;
    let work = planar_curved_realization_work(
        host_vertices,
        host_faces,
        host_face_uses,
        semantic_preflight_work,
    )?;
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

/// Charge a conservative checked bound for analytic-shell preflight and
/// allocation before opening the transaction.
fn precharge_analytic_shell(
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
    let vertices = u64::try_from(input.vertices().len()).map_err(|_| work_overflow())?;
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

/// Conservative structural ceiling for analytic-shell preflight/allocation.
///
/// Let `N = 1 + V + Eb + Ec + F + L + U`, where `Eb` and `Ec` are bounded
/// and endpoint-free closed edge declarations, `L` is the loop count, and `U`
/// is the directed edge-use count. `N^2` covers the producer's worst quadratic
/// loop canonicalization and dominates its ordered-map/dual traversals; `16N`
/// covers the fixed-number validation, certificate, geometry, topology, and
/// lineage operations attached to every authored item.
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

/// Source-size-exact bound for one planar shell participating in curved output.
///
/// `4V + 4F + 6U` covers common planar preparation and `H` is the topology
/// producer's checked semantic preflight bound. Carrier-specific producers
/// document their own formula for `H`; this layer only adds that already-
/// checked term to the common exact base before allocation.
fn planar_curved_realization_work(
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
    semantic_preflight_work: u64,
) -> StageResult<u64> {
    let vertices = u64::try_from(host_vertices).map_err(|_| work_overflow())?;
    let faces = u64::try_from(host_faces).map_err(|_| work_overflow())?;
    let uses = u64::try_from(host_face_uses).map_err(|_| work_overflow())?;
    vertices
        .checked_mul(4)
        .and_then(|value| value.checked_add(faces.checked_mul(4)?))
        .and_then(|value| value.checked_add(uses.checked_mul(6)?))
        .and_then(|value| value.checked_add(semantic_preflight_work))
        .ok_or_else(work_overflow)
}

/// Add the face/vertex half-space matrix used by cavity convexity preflight.
fn precharge_cylindrical_cavity(
    cavity: &PreparedCylindricalCavity,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<()> {
    let vertices = u64::try_from(cavity.host_vertices()).map_err(|_| work_overflow())?;
    let faces = u64::try_from(cavity.host_faces()).map_err(|_| work_overflow())?;
    let convexity_work = vertices.checked_mul(faces).ok_or_else(work_overflow)?;
    let uses = u64::try_from(cavity.host_face_uses()).map_err(|_| work_overflow())?;
    let base_work = vertices
        .checked_mul(4)
        .and_then(|value| value.checked_add(faces.checked_mul(4)?))
        .and_then(|value| value.checked_add(uses.checked_mul(6)?))
        .and_then(|value| value.checked_add(32))
        .ok_or_else(work_overflow)?;
    scope
        .ledger_mut()
        .charge(
            PLANAR_BOOLEAN_REALIZATION_WORK,
            base_work
                .checked_add(convexity_work)
                .ok_or_else(work_overflow)?,
        )
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

/// Charge a conservative identity-copy bound before opening the transaction.
fn precharge_whole_source_copies(
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

fn axial_parameter(source: &CertifiedCylinderSource, point: kgeom::vec::Point3) -> Option<f64> {
    let cylinder = source.cylinder();
    let frame = cylinder.frame();
    let parameter = (point - frame.origin()).dot(frame.z());
    parameter.is_finite().then_some(parameter)
}

fn assembly_contract(reason: &'static str) -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::AssemblyContract(reason))
}

fn section_contract() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::SectionIncomplete)
}

fn result_topology_contract() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported)
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
    fn planar_curved_realization_work_counts_every_structural_term() {
        // Convex block shell: V=8, F=6, U=24.
        assert_eq!(planar_curved_realization_work(8, 6, 24, 80).unwrap(), 280);
        assert_eq!(planar_curved_realization_work(8, 6, 24, 84).unwrap(), 284);
        assert_eq!(planar_curved_realization_work(8, 6, 24, 113).unwrap(), 313);
    }

    #[test]
    fn planar_curved_realization_work_fails_closed_on_overflow() {
        assert!(planar_curved_realization_work(usize::MAX, usize::MAX, 0, 0).is_err());
        assert!(planar_curved_realization_work(1, 0, 0, u64::MAX).is_err());
    }

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
