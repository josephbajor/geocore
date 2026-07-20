//! Single-scope dispatch between planar and first curved Boolean pipelines.

use kcore::operation::OperationScope;

use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, cylinder_operand_mask_in_scope, execute_curved_in_scope,
};
use super::pipeline::{PlanarBooleanPipelineOutcome, execute_planar_in_scope, validate_operand};
use super::select::PlanarBooleanOperation;
use crate::BodyId;
use crate::error::Result;
use crate::operation::{OperationOutcome, OperationSettings};
use crate::session::PartEdit;

/// Internal family-tagged outcome adapted by the public Boolean facade.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum BooleanPipelineOutcome {
    Planar(PlanarBooleanPipelineOutcome),
    Curved(CurvedBooleanPipelineOutcome),
}

/// Validate identities, create one operation scope, and dispatch by live
/// analytic surface carriers under that scope's source-extraction budget.
pub(crate) fn execute_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    left: BodyId,
    right: BodyId,
    settings: OperationSettings,
) -> Result<OperationOutcome<BooleanPipelineOutcome>> {
    validate_operand(edit, &left)?;
    validate_operand(edit, &right)?;

    let linear = settings.tolerances().linear();
    let context = settings
        .context(edit.policy)?
        .with_family_budget_defaults(super::BooleanBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let mask = cylinder_operand_mask_in_scope(edit, [&left, &right], &mut scope)?;
    let result = if mask.into_iter().any(|has_cylinder| has_cylinder) {
        execute_curved_in_scope(edit, operation, left, right, mask, linear, &mut scope)
            .map(BooleanPipelineOutcome::Curved)
    } else {
        execute_planar_in_scope(edit, operation, left, right, &mut scope)
            .map(BooleanPipelineOutcome::Planar)
    };
    Ok(scope.finish_typed(result))
}
