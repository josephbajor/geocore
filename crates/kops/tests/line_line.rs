//! Bounded analytic line/line intersection behavior.

use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, ParamOrientation, intersect_bounded_lines};

fn line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

#[test]
fn transverse_and_disjoint_lines() {
    let a = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let b = line([0.5, -1.0, 0.0], [0.0, 1.0, 0.0]);
    let hit = intersect_bounded_lines(
        &a,
        ParamRange::new(0.0, 1.0),
        &b,
        ParamRange::new(0.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert_eq!(hit.points[0].point, Point3::new(0.5, 0.0, 0.0));
    assert_eq!(hit.points[0].residual, 0.0);

    let miss = intersect_bounded_lines(
        &a,
        ParamRange::new(0.0, 0.25),
        &b,
        ParamRange::new(0.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn collinear_overlap_tracks_orientation() {
    let a = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let same = line([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let reversed = line([3.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
    for (b, expected) in [
        (same, ParamOrientation::Same),
        (reversed, ParamOrientation::Reversed),
    ] {
        let hit = intersect_bounded_lines(
            &a,
            ParamRange::new(0.0, 3.0),
            &b,
            ParamRange::new(0.0, 2.0),
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(hit.overlaps.len(), 1);
        assert_eq!(hit.overlaps[0].a, ParamRange::new(1.0, 3.0));
        assert_eq!(hit.overlaps[0].orientation, expected);
    }
}

#[test]
fn collinear_endpoint_contact_is_isolated() {
    let a = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let b = line([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_lines(
        &a,
        ParamRange::new(0.0, 1.0),
        &b,
        ParamRange::new(0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.overlaps.is_empty());
}
