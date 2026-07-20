use super::*;

#[test]
fn public_curved_one_port_realization_budget_is_exact_and_denial_is_failure_atomic() {
    for (operation, fixture) in [
        (
            BooleanOperation::Unite,
            one_ring_cylindrical_boss_fixture as fn() -> BooleanFixture,
        ),
        (
            BooleanOperation::Subtract,
            one_ring_blind_pocket_fixture as fn() -> BooleanFixture,
        ),
    ] {
        let baseline = run_boolean(&mut fixture(), operation, OperationSettings::new());
        assert!(matches!(
            baseline.result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));
        let usage = *baseline
            .report()
            .usage()
            .iter()
            .find(|usage| {
                usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
            })
            .unwrap();
        assert!(usage.consumed > 0);

        let settings_at = |allowed| {
            OperationSettings::new().with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    BOOLEAN_POST_SELECTION_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            )
        };
        let admitted = run_boolean(&mut fixture(), operation, settings_at(usage.consumed));
        assert!(matches!(
            admitted.into_result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));

        let mut denied_fixture = fixture();
        let before = boolean_topology_counts(&denied_fixture);
        let denied = run_boolean(
            &mut denied_fixture,
            operation,
            settings_at(usage.consumed - 1),
        );
        let expected = kernel::LimitSnapshot {
            allowed: usage.consumed - 1,
            ..usage
        };
        assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(boolean_topology_counts(&denied_fixture), before);
        assert_boolean_sources_retained(&denied_fixture, 2);
    }
}
