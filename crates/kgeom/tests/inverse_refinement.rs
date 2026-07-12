//! Integration coverage for checked inverse knot refinement.

use kgeom::nurbs::{NurbsCurve, checked_refinement_ancestors};
use kgeom::vec::Point3;

fn polynomial_cubic() -> NurbsCurve {
    NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.5),
            Point3::new(2.0, -1.0, 1.0),
            Point3::new(4.0, 1.0, 2.0),
        ],
        None,
    )
    .unwrap()
}

fn rational_line(weight_scale: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, 0.5, 1.0), Point3::new(4.0, 2.0, -1.0)],
        Some(vec![weight_scale, 2.0 * weight_scale]),
    )
    .unwrap()
}

#[test]
fn multiple_insertions_can_be_removed_through_checked_edges() {
    let coarse = polynomial_cubic();
    let refined = coarse.with_knots_refined(&[0.25, 0.25, 0.75]).unwrap();
    let ancestors = checked_refinement_ancestors(&refined, 64).unwrap();
    assert!(ancestors.contains(&coarse));
    assert!(ancestors.iter().any(|ancestor| {
        ancestor.knots().multiplicity(0.25) == 1 && ancestor.knots().multiplicity(0.75) == 1
    }));
}

#[test]
fn one_call_repeated_multiplicity_is_checked_one_edge_at_a_time() {
    let coarse = polynomial_cubic();
    let refined = coarse.with_knot_inserted(0.25, 2).unwrap();
    let ancestors = checked_refinement_ancestors(&refined, 16).unwrap();
    assert!(ancestors.contains(&coarse));
}

#[test]
fn degree_one_rational_removal_preserves_global_weight_scale() {
    for weight_scale in [1.0, 2.0, 0.5] {
        let coarse = rational_line(weight_scale);
        let refined = coarse.with_knot_inserted(0.25, 1).unwrap();
        let ancestors = checked_refinement_ancestors(&refined, 4).unwrap();
        assert!(ancestors.contains(&coarse));
        assert!(
            ancestors.iter().any(|candidate| {
                candidate == &coarse && candidate.weights() == coarse.weights()
            })
        );
    }
}
