use super::*;
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};

fn perpendicular_pair(offset: f64) -> [Cylinder; 2] {
    [
        Cylinder::new(Frame::world(), 1.0).unwrap(),
        Cylinder::new(
            Frame::new(
                Point3::new(0.0, offset, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
            2.0,
        )
        .unwrap(),
    ]
}

fn provenance(
    source_operand: usize,
    boundary: SkewCylinderAxialBoundary,
    value: f64,
) -> SkewCylinderAxialBoundProvenance {
    SkewCylinderAxialBoundProvenance {
        source_operand,
        boundary,
        value,
    }
}

fn query(
    cylinders: [Cylinder; 2],
    mapping: [usize; 2],
    provenance: SkewCylinderAxialBoundProvenance,
) -> SkewCylinderAxialBoundTopology {
    classify_skew_cylinder_axial_bound(
        cylinders,
        mapping,
        provenance,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
}

fn angular_parameters(topology: &SkewCylinderAxialBoundTopology) -> Vec<f64> {
    topology
        .roots
        .iter()
        .map(|root| root.angular_bracket().representative())
        .collect()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 8.0 * f64::EPSILON,
        "{actual:.17e} != {expected:.17e}"
    );
}

#[test]
fn canonical_axial_bound_labels_upper_sheet_and_exact_open_cells() {
    // Independent oracle:
    // z_s = s sqrt(4 - (sin(u) - 1/2)^2).
    // z_upper = 3/2 at sin(u)=-(sqrt(7)-1)/2.
    let cylinders = perpendicular_pair(0.5);
    let topology = query(
        cylinders,
        [0, 1],
        provenance(0, SkewCylinderAxialBoundary::Lower, 1.5),
    );
    let sine = (7.0_f64.sqrt() - 1.0) / 2.0;
    let gamma = math::atan2(sine, (1.0 - sine * sine).sqrt());
    let expected = [
        core::f64::consts::PI + gamma,
        core::f64::consts::TAU - gamma,
    ];

    assert_eq!(topology.roots.len(), 2);
    for (root, expected) in topology.roots.iter().zip(expected) {
        assert_eq!(root.sheet, SkewCylinderSheet::Upper);
        assert!(!root.repeated);
        assert_eq!(root.provenance, topology.provenance);
        assert_close(root.angular_bracket().representative(), expected);
    }
    assert_eq!(
        topology.open_cell_relations,
        vec![
            [
                SkewCylinderAxialRelation::Below,
                SkewCylinderAxialRelation::Below,
            ],
            [
                SkewCylinderAxialRelation::Below,
                SkewCylinderAxialRelation::Above,
            ],
        ]
    );
    assert_eq!(
        (topology.roots[0].before, topology.roots[0].after),
        (
            SkewCylinderAxialRelation::Above,
            SkewCylinderAxialRelation::Below,
        )
    );
    assert_eq!(
        (topology.roots[1].before, topology.roots[1].after),
        (
            SkewCylinderAxialRelation::Below,
            SkewCylinderAxialRelation::Above,
        )
    );
}

#[test]
fn opposite_axial_bound_with_zero_dz_retains_both_sheet_provenances() {
    // The second cylinder axis is world X, so both sheets have opposite
    // axial coordinate x=cos(u). Its zeroes are exactly pi/2 and 3pi/2.
    let cylinders = perpendicular_pair(0.5);
    let topology = query(
        cylinders,
        [0, 1],
        provenance(1, SkewCylinderAxialBoundary::Lower, 0.0),
    );

    assert_eq!(topology.roots.len(), 4);
    assert_eq!(
        topology
            .roots
            .iter()
            .map(|root| (root.cyclic_ordinal, root.sheet))
            .collect::<Vec<_>>(),
        vec![
            (0, SkewCylinderSheet::Lower),
            (0, SkewCylinderSheet::Upper),
            (1, SkewCylinderSheet::Lower),
            (1, SkewCylinderSheet::Upper),
        ]
    );
    let parameters = angular_parameters(&topology);
    for parameter in &parameters[..2] {
        assert_close(*parameter, core::f64::consts::FRAC_PI_2);
    }
    for parameter in &parameters[2..] {
        assert_close(*parameter, 3.0 * core::f64::consts::FRAC_PI_2);
    }
    assert_eq!(
        topology.open_cell_relations,
        vec![
            [SkewCylinderAxialRelation::Below; 2],
            [SkewCylinderAxialRelation::Above; 2],
        ]
    );
    assert!(topology.roots[..2].iter().all(|root| {
        root.before == SkewCylinderAxialRelation::Above
            && root.after == SkewCylinderAxialRelation::Below
    }));
    assert!(topology.roots[2..].iter().all(|root| {
        root.before == SkewCylinderAxialRelation::Below
            && root.after == SkewCylinderAxialRelation::Above
    }));
}

#[test]
fn opposite_axial_bound_uses_exact_dual_elimination_when_dz_is_nonzero() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.6, 0.0, 0.8),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let topology = query(
        [first, second],
        [0, 1],
        provenance(1, SkewCylinderAxialBoundary::Upper, 2.0),
    );

    // Independent oracle for this frame:
    // w_s=(cos(u)+0.8 s sqrt(4-sin(u)^2))/0.6.
    // w_upper=2 has cos(u)=(10-4 sqrt(7))/3.
    let cosine = (10.0 - 4.0 * 7.0_f64.sqrt()) / 3.0;
    let alpha = math::atan2((1.0 - cosine * cosine).sqrt(), cosine);
    assert_eq!(topology.roots.len(), 2);
    assert!(
        topology
            .roots
            .iter()
            .all(|root| root.sheet == SkewCylinderSheet::Upper && !root.repeated)
    );
    assert_close(topology.roots[0].angular_bracket().representative(), alpha);
    assert_close(
        topology.roots[1].angular_bracket().representative(),
        TAU - alpha,
    );
    assert_eq!(
        topology.open_cell_relations,
        vec![
            [
                SkewCylinderAxialRelation::Below,
                SkewCylinderAxialRelation::Below,
            ],
            [
                SkewCylinderAxialRelation::Below,
                SkewCylinderAxialRelation::Above,
            ],
        ]
    );
}

#[test]
fn repeated_dual_height_root_is_marked_without_a_false_crossing() {
    // cos(u)-1 has one double root at the canonical seam. Both sheets
    // remain strictly below the bound on the sole open cell.
    let topology = query(
        perpendicular_pair(0.0),
        [0, 1],
        provenance(1, SkewCylinderAxialBoundary::Upper, 1.0),
    );
    assert_eq!(topology.roots.len(), 2);
    assert!(topology.roots.iter().all(|root| root.repeated));
    assert!(topology.roots.iter().all(|root| {
        root.angular_bracket() == SkewCylinderAngularRootBracket { lo: 0.0, hi: 0.0 }
            && root.before == SkewCylinderAxialRelation::Below
            && root.after == SkewCylinderAxialRelation::Below
    }));
    assert_eq!(
        topology.open_cell_relations,
        vec![[SkewCylinderAxialRelation::Below; 2]]
    );
}

#[test]
fn caller_operand_permutation_changes_only_retained_provenance() {
    let cylinders = perpendicular_pair(0.5);
    let forward = query(
        cylinders,
        [0, 1],
        provenance(0, SkewCylinderAxialBoundary::Lower, 1.5),
    );
    let swapped = query(
        cylinders,
        [1, 0],
        provenance(1, SkewCylinderAxialBoundary::Lower, 1.5),
    );

    assert_eq!(forward.open_cell_relations, swapped.open_cell_relations);
    assert_eq!(forward.roots.len(), swapped.roots.len());
    for (forward, swapped) in forward.roots.iter().zip(&swapped.roots) {
        assert_eq!(forward.sheet, swapped.sheet);
        assert_eq!(forward.cyclic_ordinal, swapped.cyclic_ordinal);
        assert_eq!(forward.bracket, swapped.bracket);
        assert_eq!(forward.repeated, swapped.repeated);
        assert_eq!(
            (forward.before, forward.after),
            (swapped.before, swapped.after)
        );
        assert_eq!(forward.provenance.source_operand, 0);
        assert_eq!(swapped.provenance.source_operand, 1);
    }
}

#[test]
fn clipped_perpendicular_oracle_completes_all_four_source_bound_queries() {
    let cylinders = perpendicular_pair(0.5);
    for bound in [
        provenance(0, SkewCylinderAxialBoundary::Lower, 1.8),
        provenance(0, SkewCylinderAxialBoundary::Upper, 2.1),
        provenance(1, SkewCylinderAxialBoundary::Lower, -1.25),
        provenance(1, SkewCylinderAxialBoundary::Upper, 0.0),
    ] {
        let result = classify_skew_cylinder_axial_bound(
            cylinders,
            [0, 1],
            bound,
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        );
        assert!(result.is_ok(), "{bound:?}: {result:?}");
    }
}

#[test]
fn rounded_oblique_rigid_copy_preserves_strict_positive_discriminant() {
    let first = Cylinder::new(
        Frame::new(
            Point3::new(1.42, -3.19, -0.7249999999999999),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(1.5, -1.0, 0.625),
            Vec3::new(0.8, -0.6, 0.0),
            Vec3::new(0.36, 0.48, -0.8),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let result = exact_skew_cylinder_discriminant([first, second]);
    let sign = match result {
        Ok(ExactSkewCylinderDiscriminant::Strict(sign)) => Some(sign),
        Ok(ExactSkewCylinderDiscriminant::Harmonic { coefficients, .. }) => {
            classify_cyclic_second_harmonic(&coefficients, CYCLIC_SECOND_HARMONIC_EXACT_WORK)
                .unwrap()
                .strict_full_cycle_sign()
        }
        other => panic!("{other:?}"),
    };
    assert_eq!(sign, Some(StrictSign::Positive));
}

#[test]
fn angular_brackets_lift_deterministically_into_shifted_full_cycles() {
    let topology = query(
        perpendicular_pair(0.0),
        [0, 1],
        provenance(1, SkewCylinderAxialBoundary::Lower, 0.0),
    );
    let first = topology.roots[0].angular_bracket();
    let second = topology.roots[2].angular_bracket();
    let shifted = ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI);

    assert_close(
        first.lift_representative(shifted).unwrap(),
        core::f64::consts::FRAC_PI_2,
    );
    assert_close(
        second.lift_representative(shifted).unwrap(),
        -core::f64::consts::FRAC_PI_2,
    );
    assert_eq!(
        first.lift_before_side(shifted),
        first.lift_after_side(shifted)
    );
    assert_eq!(first.lift_representative(ParamRange::new(0.0, 1.0)), None);
}

#[test]
fn exact_work_and_unsafe_arithmetic_fail_without_partial_topology() {
    let cylinders = perpendicular_pair(0.0);
    let bound = provenance(0, SkewCylinderAxialBoundary::Lower, 0.0);
    assert_eq!(
        classify_skew_cylinder_axial_bound(
            cylinders,
            [0, 1],
            bound,
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK - 1,
        ),
        Err(SkewCylinderAxialRootFailure::WorkLimit {
            required: SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            provided: SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK - 1,
        })
    );
    let admitted = classify_skew_cylinder_axial_bound(
        cylinders,
        [0, 1],
        bound,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    );
    assert!(admitted.is_ok(), "{admitted:?}");

    let unsafe_bound = provenance(0, SkewCylinderAxialBoundary::Lower, f64::from_bits(1));
    assert!(matches!(
        classify_skew_cylinder_axial_bound(
            cylinders,
            [0, 1],
            unsafe_bound,
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        ),
        Err(SkewCylinderAxialRootFailure::ExactArithmetic(
            RootIsolationFailure::UnsafeArithmeticEnvelope
        ))
    ));
    assert_eq!(
        classify_skew_cylinder_axial_bound(
            cylinders,
            [0, 0],
            bound,
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        ),
        Err(SkewCylinderAxialRootFailure::InvalidSourcePermutation)
    );
}
