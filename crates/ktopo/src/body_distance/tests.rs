use super::*;
use crate::tolerance::EntityTolerance;
use kcore::operation::{OperationContext, OperationPolicyError, SessionPolicy};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
use kgeom::vec::Point2;

fn translated_frame(origin: Point3) -> Frame {
    Frame::new(origin, Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
}

fn query(store: &Store, first: BodyId, second: BodyId) -> Result<BodyDistanceOutcome> {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(BodyDistanceBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    certify_body_distance_in_scope(store, first, second, &mut scope)
}

fn certified(outcome: BodyDistanceOutcome) -> ScalarEnclosure {
    certified_evidence(outcome).0
}

fn certified_evidence(outcome: BodyDistanceOutcome) -> (ScalarEnclosure, BodyDistanceUpperWitness) {
    let BodyDistanceOutcome::Certified {
        distance,
        upper_witness,
        full_checks,
    } = outcome
    else {
        panic!("expected certified body distance, got {outcome:?}")
    };
    assert!(
        full_checks
            .iter()
            .all(|report| report.outcome() == CheckOutcome::Valid)
    );
    (distance, upper_witness)
}

#[test]
fn separated_blocks_certify_exact_face_gap_and_swap_bits() {
    let mut store = Store::new();
    let first = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
    let second = crate::make::block(
        &mut store,
        &translated_frame(Point3::new(5.0, 0.0, 0.0)),
        [2.0; 3],
    )
    .unwrap();

    let (forward, forward_witness) = certified_evidence(query(&store, first, second).unwrap());
    let (reversed, reversed_witness) = certified_evidence(query(&store, second, first).unwrap());
    assert_eq!(forward, reversed);
    assert!(forward.lower() <= 3.0 && forward.upper() >= 3.0);
    assert!(
        forward.upper() - forward.lower() <= 8.0 * LINEAR_RESOLUTION,
        "{forward:?}"
    );
    assert_eq!(forward.upper(), forward_witness.distance().upper());
    assert_eq!(forward_witness.distance(), reversed_witness.distance());
    let [forward_a, forward_b] = forward_witness.points();
    let [reversed_b, reversed_a] = reversed_witness.points();
    assert_eq!(forward_a, reversed_a);
    assert_eq!(forward_b, reversed_b);
}

#[test]
fn block_and_cylinder_use_lifted_active_pcurve_upper_witnesses() {
    let mut store = Store::new();
    let block = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
    let cylinder = crate::make::cylinder(
        &mut store,
        &translated_frame(Point3::new(4.0, 1.0, 0.0)),
        1.0,
        2.0,
    )
    .unwrap();

    let distance = certified(query(&store, block, cylinder).unwrap());
    assert!(distance.lower() <= 2.0 && distance.upper() >= 2.0);
    assert!(
        distance.upper() - distance.lower() <= 8.0 * LINEAR_RESOLUTION,
        "{distance:?}"
    );
}

#[test]
fn containment_keeps_zero_as_material_distance_lower_bound() {
    let mut store = Store::new();
    let outer = crate::make::block(&mut store, &Frame::world(), [4.0; 3]).unwrap();
    let inner = crate::make::block(
        &mut store,
        &translated_frame(Point3::new(0.25, 0.0, 0.0)),
        [1.0; 3],
    )
    .unwrap();

    let distance = certified(query(&store, outer, inner).unwrap());
    assert_eq!(distance.lower(), 0.0);
    assert!(distance.upper() > 0.0);
}

#[test]
fn unsupported_sphere_and_sheet_are_typed_and_request_relative() {
    let mut store = Store::new();
    let block = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
    let sphere = crate::make::sphere(
        &mut store,
        &translated_frame(Point3::new(5.0, 0.0, 0.0)),
        1.0,
    )
    .unwrap();
    let sheet = crate::make::planar_sheet(
        &mut store,
        &translated_frame(Point3::new(8.0, 0.0, 0.0)),
        &[
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ],
    )
    .unwrap();

    let sphere_reason = query(&store, block, sphere).unwrap().refusal().unwrap();
    assert!(matches!(
        sphere_reason,
        BodyDistanceRefusal::UnsupportedSurface {
            operand: BodyDistanceOperand::Second,
            ..
        }
    ));
    let swapped_reason = query(&store, sphere, block).unwrap().refusal().unwrap();
    assert!(matches!(
        swapped_reason,
        BodyDistanceRefusal::UnsupportedSurface {
            operand: BodyDistanceOperand::First,
            ..
        }
    ));
    assert_eq!(
        query(&store, block, sheet).unwrap().refusal(),
        Some(BodyDistanceRefusal::NonSolidBody {
            operand: BodyDistanceOperand::Second,
        })
    );
}

#[test]
fn exact_query_refuses_face_tolerance_after_full_validation() {
    let mut store = Store::new();
    let first = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
    let second = crate::make::block(
        &mut store,
        &translated_frame(Point3::new(5.0, 0.0, 0.0)),
        [2.0; 3],
    )
    .unwrap();
    let face = store.faces_of_body(second).unwrap()[0];
    store.get_mut(face).unwrap().tolerance =
        Some(EntityTolerance::operation(LINEAR_RESOLUTION, "distance-test").unwrap());

    let outcome = query(&store, first, second).unwrap();
    assert!(
        outcome
            .full_checks()
            .iter()
            .all(|report| report.outcome() == CheckOutcome::Valid)
    );
    assert_eq!(
        outcome.refusal(),
        Some(BodyDistanceRefusal::TolerantFace {
            operand: BodyDistanceOperand::Second,
            face,
        })
    );
}

#[test]
fn full_valid_solid_with_detached_wire_refuses_material_witnesses() {
    let mut store = Store::new();
    let first = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
    let second = crate::make::block(
        &mut store,
        &translated_frame(Point3::new(5.0, 0.0, 0.0)),
        [2.0; 3],
    )
    .unwrap();
    let wire = crate::make::wire_polyline(
        &mut store,
        &[Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 1.0, 0.0)],
        false,
    )
    .unwrap();
    let wire_edge = store.edges_of_body(wire).unwrap()[0];
    let face = store.faces_of_body(first).unwrap()[0];
    let shell = store.get(face).unwrap().shell;
    store.get_mut(shell).unwrap().edges.push(wire_edge);

    let outcome = query(&store, first, second).unwrap();
    assert!(
        outcome
            .full_checks()
            .iter()
            .all(|report| report.outcome() == CheckOutcome::Valid)
    );
    assert_eq!(
        outcome.refusal(),
        Some(BodyDistanceRefusal::MixedDimensionalBody {
            operand: BodyDistanceOperand::First,
        })
    );
}

fn context_with_distance_limit<'a>(
    session: &'a SessionPolicy,
    allowed: u64,
) -> OperationContext<'a> {
    let override_plan = BudgetPlan::new([LimitSpec::new(
        BODY_DISTANCE_ANALYTIC_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(BodyDistanceBudgetProfile::v1_defaults())
        .with_budget_overrides(override_plan)
}

#[test]
fn analytic_work_accepts_exactly_n_and_rejects_n_minus_one() {
    let mut store = Store::new();
    let block = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
    let cylinder = crate::make::cylinder(
        &mut store,
        &translated_frame(Point3::new(4.0, 1.0, 0.0)),
        1.0,
        2.0,
    )
    .unwrap();
    let expected = body_distance_analytic_work(&store, block, cylinder).unwrap();
    assert!(expected > 0);

    let session = SessionPolicy::v1();
    let exact_context = context_with_distance_limit(&session, expected);
    let mut exact_scope = OperationScope::new(&exact_context);
    assert!(certify_body_distance_in_scope(&store, block, cylinder, &mut exact_scope).is_ok());
    let snapshot = exact_scope
        .ledger()
        .snapshots()
        .into_iter()
        .find(|snapshot| snapshot.stage == BODY_DISTANCE_ANALYTIC_WORK)
        .unwrap();
    assert_eq!(snapshot.consumed, expected);
    assert_eq!(snapshot.allowed, expected);

    let denied_context = context_with_distance_limit(&session, expected - 1);
    let mut denied_scope = OperationScope::new(&denied_context);
    let error =
        certify_body_distance_in_scope(&store, block, cylinder, &mut denied_scope).unwrap_err();
    assert!(matches!(
        error,
        Error::OperationPolicy {
            source: OperationPolicyError::LimitReached(limit)
        } if limit.stage == BODY_DISTANCE_ANALYTIC_WORK
            && limit.consumed == expected
            && limit.allowed == expected - 1
    ));
}

#[test]
fn pair_profile_doubles_only_cumulative_checker_allowances() {
    let single = CheckBudgetProfile::v1_defaults(CheckLevel::Full);
    let pair = BodyDistanceBudgetProfile::v1_defaults();
    for limit in single.limits() {
        let paired = pair
            .limits()
            .iter()
            .find(|candidate| {
                candidate.stage == limit.stage && candidate.resource == limit.resource
            })
            .unwrap();
        let expected = if limit.mode == AccountingMode::Cumulative {
            limit.allowed * 2
        } else {
            limit.allowed
        };
        assert_eq!(paired.allowed, expected);
        assert_eq!(paired.mode, limit.mode);
    }
}
