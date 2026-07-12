//! Graph-aware intersection execution and facade result adaptation.

use kcore::operation::OperationScope;
use kgeom::project::ProjectionBudgetProfile;

use crate::error::{Error, Result};
use crate::operation::{
    CurveContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint,
    CurveOverlapOrientation, IntersectCurvesRequest, IntersectionCompletion, OperationOutcome,
};
use crate::{CurveId, Part};

impl Part<'_> {
    /// Intersect two graph-owned curves through one facade-owned operation
    /// scope while preserving facade identity and completion evidence.
    ///
    /// Wrong-part and stale identities, invalid settings, and incompatible
    /// budget modes are rejected before the scope starts. Current curve graph
    /// nodes are immutable leaf descriptors, so the solver borrows them
    /// directly without copying or inventing graph-evaluation work. Iterative
    /// projection work is charged to the returned F2 report.
    pub fn intersect_curves(
        &self,
        request: IntersectCurvesRequest,
    ) -> Result<OperationOutcome<CurveCurveIntersections>> {
        let IntersectCurvesRequest {
            first,
            second,
            settings,
        } = request;
        self.curve(first.curve.clone())?;
        self.curve(second.curve.clone())?;

        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(ProjectionBudgetProfile::curve_aggregate_compatibility());
        let mut scope = OperationScope::new(&context);
        let first_node = self
            .state
            .store
            .geometry()
            .curve(first.curve.raw())
            .expect("a validated immutable curve identity remains live");
        let second_node = self
            .state
            .store
            .geometry()
            .curve(second.curve.raw())
            .expect("a validated immutable curve identity remains live");
        let lower = kops::intersect::intersect_bounded_curves_in_scope(
            first_node.as_curve(),
            first.range,
            second_node.as_curve(),
            second.range,
            &mut scope,
        );
        let result = lower
            .map(|lower| adapt_curve_intersections(first.curve, second.curve, lower))
            .map_err(Error::from_intersection);
        Ok(scope.finish_typed(result))
    }
}

pub(crate) fn adapt_curve_intersections(
    first: CurveId,
    second: CurveId,
    lower: kops::intersect::CurveCurveIntersections,
) -> CurveCurveIntersections {
    let lower_completion = lower.completion();
    let points = lower
        .points
        .into_iter()
        .map(|point| CurveCurvePoint {
            point: point.point,
            first_parameter: point.t_a,
            second_parameter: point.t_b,
            residual: point.residual,
            kind: match point.kind {
                kops::intersect::ContactKind::Transverse => CurveContactKind::Transverse,
                kops::intersect::ContactKind::Tangent => CurveContactKind::Tangent,
                kops::intersect::ContactKind::Singular => CurveContactKind::Singular,
                _ => CurveContactKind::Unclassified,
            },
        })
        .collect();
    let overlaps = lower
        .overlaps
        .into_iter()
        .map(|overlap| CurveCurveOverlap {
            first_range: overlap.a,
            second_range: overlap.b,
            orientation: match overlap.orientation {
                kops::intersect::ParamOrientation::Same => CurveOverlapOrientation::Same,
                kops::intersect::ParamOrientation::Reversed => CurveOverlapOrientation::Reversed,
            },
        })
        .collect();
    let completion = if lower_completion.is_complete() {
        IntersectionCompletion::Complete
    } else {
        IntersectionCompletion::Indeterminate {
            reason: lower_completion
                .indeterminate_reason()
                .unwrap_or("intersection completion status is not recognized by this facade"),
        }
    };
    CurveCurveIntersections {
        first,
        second,
        points,
        overlaps,
        completion,
    }
}

#[cfg(test)]
mod tests {
    use kcore::operation::{AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind};
    use kgeom::curve::Ellipse;
    use kgeom::frame::Frame;
    use kgeom::nurbs::NurbsCurve;
    use kgeom::param::ParamRange;
    use kgeom::vec::Point3;
    use ktopo::geom::CurveGeom;

    use super::*;
    use crate::{BoundedCurve, EntityKind, Kernel, KernelError, OperationSettings, Tolerances};

    #[test]
    fn facade_matches_direct_context_and_preserves_limit_report() {
        let first_curve = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
        let second_curve = Ellipse::new(Frame::world(), 2.0, 1.5).unwrap();
        let range = ParamRange::new(0.0, core::f64::consts::TAU);
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (first_id, second_id) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let first = store.insert_curve(CurveGeom::Ellipse(first_curve)).unwrap();
            let second = store
                .insert_curve(CurveGeom::Ellipse(second_curve))
                .unwrap();
            (
                CurveId::new(part_id.clone(), first),
                CurveId::new(part_id.clone(), second),
            )
        };
        let request = || {
            IntersectCurvesRequest::new(
                BoundedCurve::new(first_id.clone(), range),
                BoundedCurve::new(second_id.clone(), range),
            )
        };

        let policy = session.policy().clone();
        let direct_context = OperationContext::new(&policy, Tolerances::default()).unwrap();
        let direct = kops::intersect::intersect_bounded_curves_with_context(
            &first_curve,
            range,
            &second_curve,
            range,
            &direct_context,
        );
        let facade = session
            .part(part_id.clone())
            .unwrap()
            .intersect_curves(request())
            .unwrap();
        let (direct_result, direct_report) = direct.into_parts();
        let expected =
            adapt_curve_intersections(first_id.clone(), second_id.clone(), direct_result.unwrap());
        assert_eq!(facade.result(), Ok(&expected));
        assert_eq!(facade.report(), &direct_report);

        let queries = *facade
            .report()
            .usage()
            .iter()
            .find(|snapshot| {
                snapshot.stage == kgeom::project::CURVE_PROJECTION_QUERIES
                    && snapshot.resource == ResourceKind::Work
            })
            .unwrap();
        assert!(queries.consumed > 1);
        let allowed = queries.consumed - 1;
        let limited = BudgetPlan::new([LimitSpec::new(
            kgeom::project::CURVE_PROJECTION_QUERIES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        let outcome = session
            .part(part_id)
            .unwrap()
            .intersect_curves(
                request().with_settings(OperationSettings::new().with_budget_overrides(limited)),
            )
            .unwrap();
        let result = outcome.result();
        let error = result.as_ref().unwrap_err();
        let crossing = error.limit().unwrap();
        assert!(matches!(error, KernelError::GeometryIntersection { .. }));
        assert_eq!(crossing.stage, kgeom::project::CURVE_PROJECTION_QUERIES);
        assert_eq!(
            (crossing.consumed, crossing.allowed),
            (queries.consumed, allowed)
        );
        assert_eq!(outcome.report().limit_events(), &[crossing]);
    }

    #[test]
    fn stale_curve_identity_is_rejected_before_scope_creation() {
        let curve = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
        let range = ParamRange::new(0.0, core::f64::consts::TAU);
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (stale_id, live_id) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let stale = store.insert_curve(CurveGeom::Ellipse(curve)).unwrap();
            let live = store.insert_curve(CurveGeom::Ellipse(curve)).unwrap();
            let ids = (
                CurveId::new(part_id.clone(), stale),
                CurveId::new(part_id.clone(), live),
            );
            let mut transaction = store.transaction().unwrap();
            transaction.assembly().remove_curve(stale).unwrap();
            transaction.commit_checked(&[]).unwrap();
            ids
        };

        let result = session
            .part(part_id)
            .unwrap()
            .intersect_curves(IntersectCurvesRequest::new(
                BoundedCurve::new(stale_id, range),
                BoundedCurve::new(live_id, range),
            ));
        assert!(matches!(
            result,
            Err(KernelError::StaleEntity {
                kind: EntityKind::Curve
            })
        ));
    }

    #[test]
    fn separated_nurbs_control_hulls_remain_a_proven_miss_through_the_facade() {
        let first = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            None,
        )
        .unwrap();
        let second = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(-1.0, 0.0, 1.0), Point3::new(1.0, 0.0, 1.0)],
            None,
        )
        .unwrap();
        let range = ParamRange::new(0.0, 1.0);
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (first_id, second_id) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let first = store.insert_curve(CurveGeom::Nurbs(first)).unwrap();
            let second = store.insert_curve(CurveGeom::Nurbs(second)).unwrap();
            (
                CurveId::new(part_id.clone(), first),
                CurveId::new(part_id.clone(), second),
            )
        };

        let outcome = session
            .part(part_id)
            .unwrap()
            .intersect_curves(IntersectCurvesRequest::new(
                BoundedCurve::new(first_id.clone(), range),
                BoundedCurve::new(second_id.clone(), range),
            ))
            .unwrap();
        let result = outcome.into_result().unwrap();
        assert_eq!(result.first(), first_id);
        assert_eq!(result.second(), second_id);
        assert!(result.is_proven_empty());
    }
}
