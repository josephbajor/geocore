//! Composed resource policy for graph-owned surface intersection.

use super::graph_cylinder_cylinder_skew::{
    SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK, SKEW_CYLINDER_DISCRIMINANT_WORK,
    SKEW_CYLINDER_TWO_SHEET_EXACT_WORK, SKEW_CYLINDER_TWO_SHEET_WORK,
};
use super::nurbs_surface_march::NurbsSurfaceMarchBudgetProfile;
use kcore::operation::{AccountingMode, BudgetPlan, LimitSpec, ResourceKind, StageId};
use kgraph::EvalBudgetProfile;

const MAX_SPHERICAL_CIRCLE_PROOFS_PER_QUERY: u64 = 4_096;
// Scoped owners such as body Section reuse one ledger across many face pairs.
// Keep the exact per-pair debit atomic while bounding the family aggregate.
const MAX_SKEW_CYLINDER_DISCRIMINANT_PROOFS_PER_SCOPE: u64 = 4_096;
const MAX_NURBS_TRACE_CERTIFICATE_WORK_PER_QUERY: u64 = 134_217_728;
const MAX_NURBS_TRACE_CERTIFICATE_ITEMS_PER_QUERY: u64 = 16_777_216;

/// Stable work stage for fixed whole-branch inverse sphere-chart subdivisions.
pub const SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS: StageId =
    match StageId::new("kops.intersect.spherical-circle-proof-subdivisions") {
        Ok(stage) => stage,
        Err(_) => panic!("valid spherical-circle proof stage"),
    };

/// Stable resource stage for fixed-depth whole-range analytic/NURBS proofs.
pub const NURBS_TRACE_CERTIFICATE_WORK: StageId =
    match StageId::new("kops.intersect.nurbs-trace-certificate-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid NURBS trace-certificate stage"),
    };

/// Version-1 composed budget for graph-owned surface intersection.
#[derive(Debug, Clone, Copy, Default)]
pub struct GraphSurfaceBudgetProfile;

impl GraphSurfaceBudgetProfile {
    /// Graph evaluation, scoped NURBS marching, an aggregate of exact skew
    /// discriminants, and bounded whole-range branch proofs.
    pub fn v1_defaults() -> BudgetPlan {
        let evaluation = EvalBudgetProfile::v1_defaults();
        let marcher = NurbsSurfaceMarchBudgetProfile::v1_defaults();
        BudgetPlan::new(
            evaluation
                .limits()
                .iter()
                .copied()
                .chain(marcher.limits().iter().copied())
                .chain([
                    LimitSpec::new(
                        SKEW_CYLINDER_DISCRIMINANT_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
                            * MAX_SKEW_CYLINDER_DISCRIMINANT_PROOFS_PER_SCOPE,
                    ),
                    LimitSpec::new(
                        SKEW_CYLINDER_TWO_SHEET_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        SKEW_CYLINDER_TWO_SHEET_EXACT_WORK
                            * MAX_SKEW_CYLINDER_DISCRIMINANT_PROOFS_PER_SCOPE,
                    ),
                    LimitSpec::new(
                        SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        kgraph::SPHERICAL_CIRCLE_PROOF_SEGMENTS as u64
                            * MAX_SPHERICAL_CIRCLE_PROOFS_PER_QUERY,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        MAX_NURBS_TRACE_CERTIFICATE_WORK_PER_QUERY,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Items,
                        AccountingMode::HighWater,
                        MAX_NURBS_TRACE_CERTIFICATE_ITEMS_PER_QUERY,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        kgraph::TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
                    ),
                ]),
        )
        .expect("built-in graph surface-intersection budget is valid")
    }
}
