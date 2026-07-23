use super::*;
use crate::entity::{BodyId, RegionKind};
use crate::make::block;
use kgeom::frame::Frame;
use kgeom::surface::{Cylinder, Plane};

fn session_with_limit(allowed: u64) -> kcore::operation::SessionPolicy {
    let budget = BudgetPlan::new([LimitSpec::new(
        BOUNDED_SKEW_LOBE_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    kcore::operation::SessionPolicy::new(
        kcore::operation::SessionPrecision::parasolid(),
        kcore::operation::NumericalPolicy::v1(),
        kcore::operation::ExecutionPolicy::Serial,
        budget,
        kcore::operation::PolicyVersion::V1,
    )
}

fn solid_shell(store: &Store, body: BodyId) -> ShellId {
    let region = store
        .get(body)
        .unwrap()
        .regions
        .iter()
        .copied()
        .find(|region| store.get(*region).unwrap().kind == RegionKind::Solid)
        .unwrap();
    store.get(region).unwrap().shells[0]
}

fn box2(u: [f64; 2], v: [f64; 2]) -> Aabb2 {
    Aabb2 {
        min: Vec2::new(u[0], v[0]),
        max: Vec2::new(u[1], v[1]),
    }
}

#[test]
fn unrelated_shell_is_inapplicable_without_work() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
    let shell = solid_shell(&store, body);
    let policy = session_with_limit(0);
    let context =
        kcore::operation::OperationContext::new(&policy, kcore::tolerance::Tolerances::default())
            .unwrap();
    let mut scope = OperationScope::new(&context);
    assert_eq!(
        certify_bounded_skew_lobe_shell(&store, shell, Some(&mut scope)).unwrap(),
        None,
    );
    assert!(
        scope
            .ledger()
            .snapshots()
            .into_iter()
            .all(|snapshot| snapshot.consumed == 0)
    );
}

#[test]
fn omitted_family_member_must_be_strictly_outside_modulo_period() {
    let period = core::f64::consts::TAU;
    let face = box2([0.5, 1.5], [-1.0, 1.0]);
    assert!(periodic_member_box_outside_face(
        box2([2.0, 2.5], [-0.5, 0.5]),
        face,
    ));
    assert!(periodic_member_box_outside_face(
        box2([0.75, 1.25], [1.1, 1.5]),
        face,
    ));

    let cases = [
        ("overlap", box2([1.0, 2.0], [-0.5, 0.5])),
        ("touch", box2([1.5, 2.0], [-0.5, 0.5])),
        (
            "periodic duplicate",
            box2([0.5 + period, 1.5 + period], [-1.0, 1.0]),
        ),
        (
            "huge ambiguous lift",
            box2(
                [
                    period * (1_u64 << 53) as f64,
                    period * (1_u64 << 53) as f64 + 1.0,
                ],
                [-0.5, 0.5],
            ),
        ),
    ];
    for (name, member) in cases {
        assert!(!periodic_member_box_outside_face(member, face), "{name}");
    }
}

#[test]
fn selected_members_reject_duplicate_nonadjacent_and_mixed_sheet_tamper() {
    let occupancy = PersistentSkewCylinderFiniteWindowSheetOccupancy::Open {
        first_member_ordinal: 4,
        member_count: 3,
    };
    assert!(selected_members_are_adjacent(
        occupancy,
        4,
        SkewCylinderSheet::Upper,
        5,
        SkewCylinderSheet::Upper,
    ));
    let cases = [
        (
            "duplicate ordinal",
            occupancy,
            4,
            SkewCylinderSheet::Upper,
            4,
            SkewCylinderSheet::Upper,
        ),
        (
            "nonadjacent ordinal",
            occupancy,
            4,
            SkewCylinderSheet::Upper,
            6,
            SkewCylinderSheet::Upper,
        ),
        (
            "mixed sheet",
            occupancy,
            4,
            SkewCylinderSheet::Upper,
            5,
            SkewCylinderSheet::Lower,
        ),
        (
            "outside occupancy",
            PersistentSkewCylinderFiniteWindowSheetOccupancy::Outside,
            4,
            SkewCylinderSheet::Upper,
            5,
            SkewCylinderSheet::Upper,
        ),
        (
            "whole occupancy",
            PersistentSkewCylinderFiniteWindowSheetOccupancy::Whole,
            4,
            SkewCylinderSheet::Upper,
            5,
            SkewCylinderSheet::Upper,
        ),
        (
            "before open range",
            occupancy,
            3,
            SkewCylinderSheet::Upper,
            4,
            SkewCylinderSheet::Upper,
        ),
        (
            "after open range",
            occupancy,
            6,
            SkewCylinderSheet::Upper,
            7,
            SkewCylinderSheet::Upper,
        ),
    ];
    for (name, occupancy, first, first_sheet, second, second_sheet) in cases {
        assert!(
            !selected_members_are_adjacent(occupancy, first, first_sheet, second, second_sheet),
            "{name}",
        );
    }
}

#[test]
fn slab_tags_and_bounds_are_exact_and_source_specific() {
    let lower =
        PersistentSkewCylinderAxialBoundTag::new(0, PersistentSkewCylinderAxialBoundary::Lower)
            .unwrap();
    let upper =
        PersistentSkewCylinderAxialBoundTag::new(0, PersistentSkewCylinderAxialBoundary::Upper)
            .unwrap();
    let other_upper =
        PersistentSkewCylinderAxialBoundTag::new(1, PersistentSkewCylinderAxialBoundary::Upper)
            .unwrap();
    assert_eq!(common_slab_source_slot(lower, upper), Some(0));
    assert_eq!(common_slab_source_slot(upper, lower), Some(0));
    assert_eq!(common_slab_source_slot(lower, lower), None);
    assert_eq!(common_slab_source_slot(lower, other_upper), None);

    let window = ParamRange::new(-2.0, 3.0);
    assert!(tagged_bound_matches_window(lower, -2.0, 0, window));
    assert!(tagged_bound_matches_window(upper, 3.0, 0, window));
    assert!(!tagged_bound_matches_window(
        lower,
        f64::from_bits((-2.0_f64).to_bits() + 1),
        0,
        window,
    ));
    assert!(!tagged_bound_matches_window(other_upper, 3.0, 0, window));
}

#[test]
fn axial_plane_alignment_accepts_rigid_roundoff_but_rejects_authored_drift() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let bound = 2.0_f64;
    let exact = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, bound)));
    let one_ulp = Plane::new(Frame::world().with_origin(Point3::new(
        0.0,
        0.0,
        f64::from_bits(bound.to_bits() + 1),
    )));
    let outside = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, bound + 1.0e-10)));
    let large_cylinder = Cylinder::new(
        Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0e8)),
        1.0,
    )
    .unwrap();
    let large_expected = 1.0e8 + bound;
    let oversized_corridor = Plane::new(Frame::world().with_origin(Point3::new(
        0.0,
        0.0,
        f64::from_bits(large_expected.to_bits() + 1),
    )));

    assert!(certified_axial_plane_alignment(cylinder, exact, bound));
    assert!(certified_axial_plane_alignment(cylinder, one_ulp, bound));
    assert!(!certified_axial_plane_alignment(cylinder, outside, bound));
    assert!(!certified_axial_plane_alignment(
        large_cylinder,
        oversized_corridor,
        bound,
    ));
}

#[test]
fn slab_and_radial_boundary_traversal_orientation_is_exact() {
    assert_eq!(
        slab_boundary_orientation(PersistentSkewCylinderAxialBoundary::Lower, true),
        PredicateOrientation::Positive,
    );
    assert_eq!(
        slab_boundary_orientation(PersistentSkewCylinderAxialBoundary::Upper, false),
        PredicateOrientation::Positive,
    );
    assert_eq!(
        slab_boundary_orientation(PersistentSkewCylinderAxialBoundary::Lower, false),
        PredicateOrientation::Negative,
    );
    assert_eq!(
        slab_boundary_orientation(PersistentSkewCylinderAxialBoundary::Upper, true),
        PredicateOrientation::Negative,
    );
    assert_eq!(
        radial_boundary_orientation(true, false),
        PredicateOrientation::Positive,
    );
    assert_eq!(
        radial_boundary_orientation(false, true),
        PredicateOrientation::Positive,
    );
    assert_eq!(
        radial_boundary_orientation(true, true),
        PredicateOrientation::Negative,
    );
    assert_eq!(
        radial_boundary_orientation(false, false),
        PredicateOrientation::Negative,
    );
}

#[test]
fn cap_slab_u_window_replay_rejects_narrowed_source_authority() {
    let period = core::f64::consts::TAU;
    let domain = box2([0.5, 1.0], [-4.0, 4.0]);
    let exact = ParamRange::new(0.0, 2.0);
    assert_eq!(certify_periodic_u_lift(domain, exact), Some(0));
    assert_eq!(
        certify_periodic_u_lift(
            domain,
            ParamRange::new(exact.lo + period, exact.hi + period),
        ),
        Some(1),
    );
    assert_eq!(
        certify_periodic_u_lift(domain, ParamRange::new(0.6, 0.9)),
        None,
    );
    assert_eq!(
        certify_periodic_u_lift(domain, ParamRange::new(0.6 + period, 0.9 + period),),
        None,
    );
}

#[test]
fn shell_work_accepts_exact_n_and_rejects_n_minus_one() {
    let required = proof_work_for_size(31).unwrap();
    assert_eq!(required, 1_457);

    let exact_policy = session_with_limit(required);
    let exact_context = kcore::operation::OperationContext::new(
        &exact_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut exact_scope = OperationScope::new(&exact_context);
    charge_proof_work(&mut exact_scope, required).unwrap();

    let short_policy = session_with_limit(required - 1);
    let short_context = kcore::operation::OperationContext::new(
        &short_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut short_scope = OperationScope::new(&short_context);
    let error = charge_proof_work(&mut short_scope, required).unwrap_err();
    assert_eq!(
        error.limit().map(|limit| limit.stage),
        Some(BOUNDED_SKEW_LOBE_SHELL_WORK),
    );
    assert!(proof_work_for_size(u64::MAX).is_none());
}
