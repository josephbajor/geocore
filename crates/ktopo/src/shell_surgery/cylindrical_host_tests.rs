use super::*;
use crate::check::CheckOutcome;
use crate::cylindrical_host::{
    CylindricalHostBandInput, CylindricalHostEndpoint, CylindricalHostSolidInput,
};
use crate::entity::Body;
use crate::planar::{PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey};
use crate::transaction::FullCommitRequirement;
use kcore::operation::{LimitSnapshot, OperationContext, SessionPolicy};
use kcore::tolerance::Tolerances;
use kgeom::param::ParamRange;

const TWO_BAND_PROOF_WORK: u64 = 985;
const CYLINDRICAL_HOST_SHELL_WORK: StageId = SHELL_SURGERY_WORK;

fn cube() -> PlanarSolidInput {
    let points = [
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(-1.0, 1.0, -1.0),
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(-1.0, -1.0, 1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(-1.0, 1.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
    ];
    let keys = core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(index as u64));
    let vertices = keys
        .into_iter()
        .zip(points)
        .map(|(key, point)| PlanarSolidVertex::new(key, point))
        .collect();
    let faces = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ]
    .into_iter()
    .map(|ring| PlanarSolidFace::new(ring.map(|vertex| keys[vertex]).to_vec()))
    .collect();
    PlanarSolidInput::new(vertices, faces)
}

fn two_outward_bands() -> CylindricalHostSolidInput {
    let low = CylindricalHostBandInput::new(
        Frame::world().with_origin(Point3::new(0.0, 0.0, -2.0)),
        0.5,
        ParamRange::new(0.0, 1.0),
        [
            CylindricalHostEndpoint::port(0),
            CylindricalHostEndpoint::cap(),
        ],
    );
    let high = CylindricalHostBandInput::new(
        Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
        0.5,
        ParamRange::new(0.0, 1.0),
        [
            CylindricalHostEndpoint::cap(),
            CylindricalHostEndpoint::port(1),
        ],
    );
    CylindricalHostSolidInput::new(cube(), vec![high, low])
}

fn proof_budget(allowed: u64) -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        CYLINDRICAL_HOST_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap()
}

#[derive(Debug, Clone, Copy)]
struct TestBoundary {
    center: Point3,
}

#[derive(Debug, Clone, Copy)]
struct TestBand {
    cylinder: Cylinder,
    low: TestBoundary,
    high: TestBoundary,
}

fn prepared_bands(store: &Store, shell: ShellId) -> Vec<TestBand> {
    let evidence = cylindrical_host::discover(store, shell)
        .unwrap()
        .into_iter()
        .next()
        .expect("cylindrical-host discovery proposes the family");
    evidence
        .features
        .into_iter()
        .map(|feature| {
            let first = TestBoundary {
                center: feature.profiles[0].profile.frame().origin(),
            };
            let second = TestBoundary {
                center: feature.profiles[1].profile.frame().origin(),
            };
            let (low, high) = match exact_affine_sign(
                feature.cylinder.frame().z(),
                second.center,
                first.center,
            ) {
                Some(PredicateOrientation::Positive) => (first, second),
                Some(PredicateOrientation::Negative) => (second, first),
                _ => panic!("fixture sweep has a strict axial order"),
            };
            TestBand {
                cylinder: feature.cylinder,
                low,
                high,
            }
        })
        .collect()
}

fn parallel_axial_slabs_are_strictly_separated(first: TestBand, second: TestBand) -> bool {
    let Some(alignment) = exact_axis_alignment(first.cylinder.frame(), second.cylinder.frame().z())
    else {
        return false;
    };
    let (second_low, second_high) = if alignment == PredicateOrientation::Positive {
        (second.low.center, second.high.center)
    } else {
        (second.high.center, second.low.center)
    };
    exact_affine_sign(first.cylinder.frame().z(), second_low, first.high.center)
        == Some(PredicateOrientation::Positive)
        || exact_affine_sign(first.cylinder.frame().z(), first.low.center, second_high)
            == Some(PredicateOrientation::Positive)
}

#[test]
fn multiple_outward_bands_are_full_valid_independent_of_storage_order() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_cylindrical_host_solid(&two_outward_bands())
        .unwrap();
    transaction
        .store_mut()
        .get_mut(output.shell())
        .unwrap()
        .faces
        .reverse();
    let faces = transaction
        .store()
        .get(output.shell())
        .unwrap()
        .faces
        .clone();
    for face in faces {
        transaction
            .store_mut()
            .get_mut(face)
            .unwrap()
            .loops
            .reverse();
    }

    let decision = transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(decision.is_committed(), "checks: {:?}", decision.checks());
    assert!(decision.checks().iter().all(|check| {
        check.report().outcome() == CheckOutcome::Valid && check.report().gaps.is_empty()
    }));
}

#[test]
fn multiple_outward_bands_with_wrong_side_sense_are_full_invalid() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_cylindrical_host_solid(&two_outward_bands())
        .unwrap();
    let side = output.bands()[0].side_face();
    transaction.store_mut().get_mut(side).unwrap().sense = Sense::Reversed;

    let decision = transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(!decision.is_committed());
    assert!(
        decision
            .checks()
            .iter()
            .any(|check| check.report().outcome() == CheckOutcome::Invalid)
    );
    assert_eq!(store.count::<Body>(), 0);
}

#[test]
fn touching_and_overlapping_axial_slabs_are_not_certified_separate() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_cylindrical_host_solid(&two_outward_bands())
        .unwrap();
    let bands = prepared_bands(transaction.store(), output.shell());
    assert_eq!(bands.len(), 2);
    assert!(parallel_axial_slabs_are_strictly_separated(
        bands[0], bands[1]
    ));

    let mut touching = bands[1];
    touching.low.center = bands[0].high.center;
    touching.high.center = bands[0].high.center + bands[0].cylinder.frame().z();
    assert!(!parallel_axial_slabs_are_strictly_separated(
        bands[0], touching
    ));

    let mut overlapping = bands[1];
    overlapping.low.center = bands[0].low.center;
    overlapping.high.center = bands[0].high.center;
    assert!(!parallel_axial_slabs_are_strictly_separated(
        bands[0],
        overlapping
    ));
}

#[test]
fn cylindrical_host_shell_work_accepts_exact_n_and_n_minus_one_rolls_back() {
    let mut accepted_store = Store::new();
    let accepted_session = SessionPolicy::v1();
    let accepted_context = OperationContext::new(&accepted_session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(proof_budget(TWO_BAND_PROOF_WORK));
    let mut accepted = accepted_store.transaction().unwrap();
    let accepted_output = accepted
        .assemble_cylindrical_host_solid(&two_outward_bands())
        .unwrap();
    let accepted = accepted
        .commit_full_with_context(
            &[accepted_output.body()],
            FullCommitRequirement::RequireValid,
            &accepted_context,
        )
        .unwrap();
    assert!(accepted.result().as_ref().unwrap().is_committed());
    let usage = accepted
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == CYLINDRICAL_HOST_SHELL_WORK)
        .copied()
        .unwrap();
    assert_eq!(
        (usage.consumed, usage.allowed),
        (TWO_BAND_PROOF_WORK, TWO_BAND_PROOF_WORK)
    );

    let mut denied_store = Store::new();
    let denied_session = SessionPolicy::v1();
    let denied_context = OperationContext::new(&denied_session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(proof_budget(TWO_BAND_PROOF_WORK - 1));
    let mut denied = denied_store.transaction().unwrap();
    let denied_output = denied
        .assemble_cylindrical_host_solid(&two_outward_bands())
        .unwrap();
    let rolled_back_body = denied_output.body();
    let denied = denied
        .commit_full_with_context(
            &[rolled_back_body],
            FullCommitRequirement::RequireValid,
            &denied_context,
        )
        .unwrap();
    let expected = LimitSnapshot {
        stage: CYLINDRICAL_HOST_SHELL_WORK,
        resource: ResourceKind::Work,
        consumed: TWO_BAND_PROOF_WORK,
        allowed: TWO_BAND_PROOF_WORK - 1,
    };
    assert_eq!(
        denied.result().as_ref().unwrap_err().limit(),
        Some(expected)
    );
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(denied_store.count::<Body>(), 0);

    let mut retry = denied_store.transaction().unwrap();
    let retried = retry
        .assemble_cylindrical_host_solid(&two_outward_bands())
        .unwrap();
    assert_eq!(retried.body(), rolled_back_body);
}
