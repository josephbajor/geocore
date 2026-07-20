//! Dispatcher for representation-specific solid-region certificates.

use crate::cylindrical_region_proof::CylindricalCavityRegionCertification;
use crate::entity::RegionId;
use crate::mixed_region_proof::MixedConvexRegionCertification;
use crate::semantic_planar_shell_proof::SemanticPlanarRegionCertification;
use crate::store::Store;
use kcore::error::Result;
use kcore::operation::{BudgetPlan, OperationScope};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RegionCertification {
    NotApplicable,
    Certified,
    Invalid,
    Indeterminate,
}

pub(crate) fn region_proof_budget() -> BudgetPlan {
    crate::cylindrical_region_proof::cylindrical_cavity_region_proof_budget()
        .overlaid(&crate::mixed_region_proof::mixed_convex_region_proof_budget())
}

pub(crate) fn certify_region_in_scope(
    store: &Store,
    region_id: RegionId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RegionCertification> {
    match crate::mixed_region_proof::certify_mixed_convex_region_in_scope(store, region_id, scope)?
    {
        MixedConvexRegionCertification::Certified => {
            return Ok(RegionCertification::Certified);
        }
        MixedConvexRegionCertification::Invalid => {
            return Ok(RegionCertification::Invalid);
        }
        MixedConvexRegionCertification::Indeterminate => {
            return Ok(RegionCertification::Indeterminate);
        }
        MixedConvexRegionCertification::NotApplicable => {}
    }
    match crate::cylindrical_region_proof::certify_cylindrical_cavity_region_in_scope(
        store, region_id, scope,
    )? {
        CylindricalCavityRegionCertification::Certified => {
            return Ok(RegionCertification::Certified);
        }
        CylindricalCavityRegionCertification::Invalid => {
            return Ok(RegionCertification::Invalid);
        }
        CylindricalCavityRegionCertification::Indeterminate => {
            return Ok(RegionCertification::Indeterminate);
        }
        CylindricalCavityRegionCertification::NotApplicable => {}
    }
    Ok(
        match crate::semantic_planar_shell_proof::certify_semantic_planar_region_in_scope(
            store, region_id, scope,
        )? {
            SemanticPlanarRegionCertification::NotApplicable => RegionCertification::NotApplicable,
            SemanticPlanarRegionCertification::Certified => RegionCertification::Certified,
            SemanticPlanarRegionCertification::Invalid => RegionCertification::Invalid,
            SemanticPlanarRegionCertification::Indeterminate => RegionCertification::Indeterminate,
        },
    )
}
