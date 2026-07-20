//! Strict containment proof for the first bound semantic cavity region.
//!
//! This module composes two independently prepared semantic planar shells.
//! Cross-shell contact is never authorized: every facet pair must have a
//! certified positive projection gap. The positive shell must also define a
//! strict convex half-space envelope containing every ideal vertex of the
//! coherently negative shell.

use crate::entity::{RegionId, SurfaceId};
use crate::semantic_planar_pair_proof::{
    SemanticFacetPairRelation, certify_semantic_cross_shell_facet_disjoint,
    semantic_facet_pair_work,
};
use crate::semantic_planar_shell_proof::{
    SemanticPlanarShellEvidence, SemanticPlanarShellPreparation, SemanticVertexEvidence,
    certify_prepared_semantic_planar_shell_in_scope, prepare_semantic_planar_shell_in_scope,
};
use crate::shell_proof::{ShellEmbedding, ShellOrientation};
use crate::store::Store;
use kcore::error::Result;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{
    Orientation, OrientedPlanePoints, oriented_plane_triple_intersection_side,
};

/// Cumulative work for strict two-shell region separation and containment.
pub(crate) const SEMANTIC_PLANAR_REGION_WORK: StageId =
    match StageId::new("ktopo.check.semantic-planar-region-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid semantic planar region work stage"),
    };

const DEFAULT_SEMANTIC_PLANAR_REGION_WORK: u64 = 1_048_576;

/// Version-1 deterministic budget for semantic planar region proof.
pub(crate) fn semantic_planar_region_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        SEMANTIC_PLANAR_REGION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_SEMANTIC_PLANAR_REGION_WORK,
    )])
    .expect("built-in semantic planar region proof budget is valid")
}

/// Full proof result for the first semantic multi-shell solid-region slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SemanticPlanarRegionCertification {
    /// The region is outside the exact two-shell contract; use another proof.
    NotApplicable,
    /// One positive convex outer shell strictly contains one negative shell.
    Certified,
    /// Complete semantic evidence proves the two shells cannot bound the
    /// requested solid region.
    Invalid,
    /// The representation or a strict separation/containment decision was
    /// not certified.
    Indeterminate,
}

/// Certify one bound semantic planar solid region with an outer and cavity shell.
pub(crate) fn certify_semantic_planar_region_in_scope(
    store: &Store,
    region_id: RegionId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<SemanticPlanarRegionCertification> {
    let region = store.get(region_id)?;
    let [first_id, second_id] = region.shells.as_slice() else {
        return Ok(SemanticPlanarRegionCertification::NotApplicable);
    };
    let (first, second) = match (
        prepare_semantic_planar_shell_in_scope(store, *first_id, scope)?,
        prepare_semantic_planar_shell_in_scope(store, *second_id, scope)?,
    ) {
        (
            SemanticPlanarShellPreparation::Certified(first),
            SemanticPlanarShellPreparation::Certified(second),
        ) => (first, second),
        _ => return Ok(SemanticPlanarRegionCertification::Indeterminate),
    };
    let first_certification = certify_prepared_semantic_planar_shell_in_scope(&first, scope)?;
    let second_certification = certify_prepared_semantic_planar_shell_in_scope(&second, scope)?;
    if first_certification.embedding != ShellEmbedding::Certified
        || second_certification.embedding != ShellEmbedding::Certified
    {
        return Ok(SemanticPlanarRegionCertification::Indeterminate);
    }
    let (positive, negative) = match (
        first_certification.orientation,
        second_certification.orientation,
    ) {
        (ShellOrientation::Positive, ShellOrientation::Negative) => (&first, &second),
        (ShellOrientation::Negative, ShellOrientation::Positive) => (&second, &first),
        (ShellOrientation::Invalid, _) | (_, ShellOrientation::Invalid) => {
            return Ok(SemanticPlanarRegionCertification::Invalid);
        }
        (ShellOrientation::Indeterminate, _) | (_, ShellOrientation::Indeterminate) => {
            return Ok(SemanticPlanarRegionCertification::Indeterminate);
        }
        _ => return Ok(SemanticPlanarRegionCertification::Invalid),
    };

    scope.ledger().require_limit(
        SEMANTIC_PLANAR_REGION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let Some(work) = semantic_planar_region_work(positive, negative) else {
        return Ok(SemanticPlanarRegionCertification::Indeterminate);
    };
    charge_region(scope, work)?;
    for positive_facet in positive.facets() {
        for negative_facet in negative.facets() {
            if certify_semantic_cross_shell_facet_disjoint(
                positive,
                positive_facet,
                negative,
                negative_facet,
            ) != SemanticFacetPairRelation::Disjoint
            {
                return Ok(SemanticPlanarRegionCertification::Indeterminate);
            }
        }
    }

    match certify_strict_convex_containment(positive, negative) {
        StrictContainment::Certified => Ok(SemanticPlanarRegionCertification::Certified),
        StrictContainment::Outside => Ok(SemanticPlanarRegionCertification::Invalid),
        StrictContainment::Indeterminate => Ok(SemanticPlanarRegionCertification::Indeterminate),
    }
}

fn semantic_planar_region_work(
    positive: &SemanticPlanarShellEvidence,
    negative: &SemanticPlanarShellEvidence,
) -> Option<u64> {
    let mut work = 0_u64;
    for positive_facet in positive.facets() {
        for negative_facet in negative.facets() {
            work = work.checked_add(semantic_facet_pair_work(positive_facet, negative_facet)?)?;
        }
    }
    for support in unique_positive_supports(positive) {
        let positive_tests = positive
            .vertices()
            .iter()
            .filter(|vertex| !vertex.surfaces().contains(&support))
            .count();
        work = work.checked_add(u64::try_from(positive_tests).ok()?)?;
        work = work.checked_add(u64::try_from(negative.vertices().len()).ok()?)?;
    }
    Some(work)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StrictContainment {
    Certified,
    Outside,
    Indeterminate,
}

fn certify_strict_convex_containment(
    positive: &SemanticPlanarShellEvidence,
    negative: &SemanticPlanarShellEvidence,
) -> StrictContainment {
    for support_id in unique_positive_supports(positive) {
        let Some(support) = positive.plane_witness(support_id) else {
            return StrictContainment::Indeterminate;
        };
        let mut interior_side = None;
        for vertex in positive
            .vertices()
            .iter()
            .filter(|vertex| !vertex.surfaces().contains(&support_id))
        {
            let Some(side) = ideal_vertex_plane_side(positive, *vertex, support) else {
                return StrictContainment::Indeterminate;
            };
            if side == Orientation::Zero || interior_side.is_some_and(|candidate| candidate != side)
            {
                return StrictContainment::Indeterminate;
            }
            interior_side = Some(side);
        }
        let Some(interior_side) = interior_side else {
            return StrictContainment::Indeterminate;
        };
        for &vertex in negative.vertices() {
            let Some(side) = ideal_vertex_plane_side(negative, vertex, support) else {
                return StrictContainment::Indeterminate;
            };
            if side == Orientation::Zero {
                return StrictContainment::Indeterminate;
            }
            if side != interior_side {
                return StrictContainment::Outside;
            }
        }
    }
    StrictContainment::Certified
}

fn unique_positive_supports(evidence: &SemanticPlanarShellEvidence) -> Vec<SurfaceId> {
    let mut supports = Vec::new();
    for facet in evidence.facets() {
        if !supports.contains(&facet.support()) {
            supports.push(facet.support());
        }
    }
    supports
}

fn ideal_vertex_plane_side(
    evidence: &SemanticPlanarShellEvidence,
    vertex: SemanticVertexEvidence,
    plane: OrientedPlanePoints,
) -> Option<Orientation> {
    let [Some(first), Some(second), Some(third)] = vertex
        .surfaces()
        .map(|surface| evidence.plane_witness(surface))
    else {
        return None;
    };
    let defining = [first, second, third];
    oriented_plane_triple_intersection_side(defining, plane).map(|side| side.sign())
}

fn charge_region(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SEMANTIC_PLANAR_REGION_WORK, amount)?;
    Ok(())
}
