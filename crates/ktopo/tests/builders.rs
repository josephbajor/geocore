#![allow(
    deprecated,
    reason = "compatibility coverage retains the deprecated v1 tessellation wrapper"
)]

//! Checked builders for non-solid topology used by profiles and interchange.

use kcore::predicates::{Orientation, orient3d};
use kgeom::frame::Frame;
use kgeom::vec::{Point2, Point3, Vec3};
use ktopo::btess::{TessOptions, check_watertight, signed_volume, tessellate_body};
use ktopo::check::{CheckLevel, CheckOutcome, VerificationGapKind, check_body_report};
use ktopo::entity::{BodyKind, Edge, Face, Fin, Loop, Region, Shell, Vertex};
use ktopo::geom::SurfaceGeom;
use ktopo::make;
use ktopo::profile::PlanarProfile;
use ktopo::store::Store;
use ktopo::transaction::MutationKind;

fn concave_clockwise_polygon() -> [Point2; 5] {
    [
        Point2::new(0.0, 0.0),
        Point2::new(0.0, 2.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 2.0),
        Point2::new(2.0, 0.0),
    ]
}

#[test]
fn planar_sheet_normalizes_and_certifies_a_concave_polygon() {
    let mut store = Store::new();
    let profile =
        PlanarProfile::from_polygon(Frame::world(), &concave_clockwise_polygon()).unwrap();
    let made = make::planar_sheet_from_profile_with_journal(&mut store, &profile).unwrap();
    let body = made.body();

    assert_eq!(store.get(body).unwrap().kind, BodyKind::Sheet);
    assert_eq!(store.count::<Face>(), 1);
    assert_eq!(store.count::<Edge>(), 5);
    assert_eq!(store.count::<Vertex>(), 5);
    assert_eq!(store.count::<Fin>(), 5);
    assert_eq!(store.count::<Shell>(), 1);
    assert_eq!(store.count::<Region>(), 1);
    for edge in store.edges_of_body(body).unwrap() {
        let edge = store.get(edge).unwrap();
        assert_eq!(edge.fins.len(), 1);
        assert!(store.get(edge.fins[0]).unwrap().pcurve.is_some());
    }
    assert!(
        made.journal()
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
    assert!(made.journal().lineage().is_empty());

    let fast = check_body_report(&store, body, CheckLevel::Fast).unwrap();
    assert_eq!(fast.outcome(), CheckOutcome::Valid);
    let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");

    let mesh = tessellate_body(
        &store,
        body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: Some(0.25),
        },
    )
    .unwrap();
    assert!(!mesh.triangles.is_empty());
}

#[test]
fn planar_sheet_builds_checks_and_tessellates_polygonal_holes() {
    let outer = [
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ];
    let hole = [
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let profile = PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
    let mut store = Store::new();
    let made = make::planar_sheet_from_profile_with_journal(&mut store, &profile).unwrap();
    let body = made.body();

    assert_eq!(store.count::<Face>(), 1);
    assert_eq!(store.count::<Loop>(), 2);
    assert_eq!(store.count::<Edge>(), 8);
    assert_eq!(store.count::<Vertex>(), 8);
    assert_eq!(store.count::<Fin>(), 8);
    let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");

    let mesh = tessellate_body(
        &store,
        body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: Some(0.5),
        },
    )
    .unwrap();
    assert!(!mesh.triangles.is_empty());
    let area = mesh
        .triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.map(|index| mesh.positions[index as usize]);
            (b - a).cross(c - a).norm() * 0.5
        })
        .sum::<f64>();
    assert!(
        (area - 12.0).abs() <= 1.0e-10,
        "unexpected holed sheet area {area}"
    );
}

#[test]
fn polygonal_profile_extrusion_with_a_hole_is_full_valid_watertight_and_exact() {
    let outer = [
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ];
    let hole = [
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let profile = PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
    let mut store = Store::new();
    let made = make::extrude_profile_with_journal(&mut store, &profile, 2.0).unwrap();
    let body = made.body();

    assert_eq!(store.get(body).unwrap().kind, BodyKind::Solid);
    assert_eq!(store.count::<Region>(), 2);
    assert_eq!(store.count::<Shell>(), 1);
    assert_eq!(store.count::<Face>(), 10);
    assert_eq!(store.count::<Loop>(), 12);
    assert_eq!(store.count::<Edge>(), 24);
    assert_eq!(store.count::<Vertex>(), 16);
    assert_eq!(store.count::<Fin>(), 48);
    for edge in store.edges_of_body(body).unwrap() {
        let edge = store.get(edge).unwrap();
        assert_eq!(edge.fins.len(), 2);
        assert_ne!(
            store.get(edge.fins[0]).unwrap().sense,
            store.get(edge.fins[1]).unwrap().sense
        );
        assert!(
            edge.fins
                .iter()
                .all(|&fin| store.get(fin).unwrap().pcurve.is_some())
        );
    }
    assert!(
        made.journal()
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
    let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");

    let mesh = tessellate_body(
        &store,
        body,
        &TessOptions {
            chord_tol: 1.0e-3,
            max_edge_len: Some(0.5),
        },
    )
    .unwrap();
    assert!(
        check_watertight(&mesh).is_empty(),
        "extruded profile mesh is not watertight"
    );
    assert!((signed_volume(&mesh) - 24.0).abs() <= 1.0e-9);
}

#[test]
fn oblique_polygonal_profile_extrusion_is_full_valid_watertight_and_volume_preserving() {
    let outer = [
        Point2::new(-2.0, -1.0),
        Point2::new(2.0, -1.0),
        Point2::new(2.0, 3.0),
        Point2::new(-2.0, 3.0),
    ];
    let hole = [
        Point2::new(-1.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 2.0),
        Point2::new(-1.0, 2.0),
    ];
    let profile = PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
    for translation in [Vec3::new(0.75, -0.5, 2.0), Vec3::new(0.75, -0.5, -2.0)] {
        let mut store = Store::new();
        let made =
            make::extrude_profile_along_with_journal(&mut store, &profile, translation).unwrap();
        let body = made.body();

        assert_eq!(store.count::<Face>(), 10);
        assert_eq!(store.count::<Loop>(), 12);
        assert_eq!(store.count::<Edge>(), 24);
        assert_eq!(store.count::<Vertex>(), 16);
        assert_eq!(store.count::<Fin>(), 48);
        assert!(
            made.journal()
                .mutations()
                .iter()
                .all(|mutation| mutation.kind == MutationKind::Created)
        );
        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");

        let mesh = tessellate_body(
            &store,
            body,
            &TessOptions {
                chord_tol: 1.0e-3,
                max_edge_len: Some(0.5),
            },
        )
        .unwrap();
        assert!(check_watertight(&mesh).is_empty());
        assert!((signed_volume(&mesh) - 24.0).abs() <= 1.0e-9);
    }
}

#[test]
fn axial_profile_extrusion_retains_exact_authored_side_axes() {
    let frame = Frame::new(
        Point3::new(3.0, -2.0, 1.25),
        Vec3::new(0.48, 0.64, 0.6),
        Vec3::new(0.8, -0.6, 0.0),
    )
    .unwrap();
    let polygon = [
        Point2::new(-2.0, -1.0),
        Point2::new(2.0, -1.0),
        Point2::new(2.0, 3.0),
        Point2::new(-2.0, 3.0),
    ];
    let profile = PlanarProfile::from_polygon(frame, &polygon).unwrap();
    let mut store = Store::new();
    let body = make::extrude_profile(&mut store, &profile, 2.0).unwrap();

    let side_normals = store
        .faces_of_body(body)
        .unwrap()
        .into_iter()
        .skip(2)
        .map(
            |face| match store.surface(store.get(face).unwrap().surface).unwrap() {
                SurfaceGeom::Plane(plane) => plane.frame().z(),
                other => panic!("axial polygon extrusion created a non-plane side: {other:?}"),
            },
        )
        .collect::<Vec<_>>();
    assert_eq!(
        side_normals,
        vec![-frame.y(), frame.x(), frame.y(), -frame.x()],
        "authored axis-aligned profile edges must reuse exact signed parent axes"
    );
    assert!(
        side_normals
            .iter()
            .all(|normal| normal.dot(frame.z()) == 0.0),
        "authored side planes lost their exact axial orthogonality"
    );
}

#[test]
fn extrusion_normal_sign_uses_exact_scalar_triples_in_both_directions() {
    let integer_normal = [
        281_474_976_710_666_i64,
        281_474_976_710_672,
        281_474_976_710_675,
    ];
    let integer_translation = [127_i64, -382, 255];
    assert_eq!(exact_integer_dot(integer_normal, integer_translation), 3);

    let frame = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(
            integer_normal[0] as f64,
            integer_normal[1] as f64,
            integer_normal[2] as f64,
        ),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let polygon = [
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let profile = PlanarProfile::from_polygon(frame, &polygon).unwrap();

    for (integer_translation, expected_orientation) in [
        (integer_translation, Orientation::Positive),
        (
            integer_translation.map(|component| -component),
            Orientation::Negative,
        ),
    ] {
        let translation = Vec3::new(
            integer_translation[0] as f64,
            integer_translation[1] as f64,
            integer_translation[2] as f64,
        );
        assert_eq!(translation.dot(frame.z()), 0.0);
        assert_eq!(
            exact_integer_dot(integer_normal, integer_translation).signum(),
            i128::from(expected_orientation.as_i8())
        );
        assert_eq!(
            orient3d(
                frame.x().to_array(),
                frame.y().to_array(),
                translation.to_array(),
                [0.0; 3],
            ),
            expected_orientation
        );

        let mut store = Store::new();
        let made =
            make::extrude_profile_along_with_journal(&mut store, &profile, translation).unwrap();
        let body = made.body();
        assert_eq!(store.get(body).unwrap().kind, BodyKind::Solid);
        assert_eq!(store.count::<Face>(), 6);
        assert_eq!(store.count::<Edge>(), 12);
        assert_eq!(store.count::<Vertex>(), 8);
        for edge in store.edges_of_body(body).unwrap() {
            assert_eq!(store.get(edge).unwrap().fins.len(), 2);
        }
        let fast = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(fast.outcome(), CheckOutcome::Valid, "{fast:?}");
        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert!(full.faults.is_empty(), "{full:?}");

        let mut repeated_store = Store::new();
        let repeated =
            make::extrude_profile_along_with_journal(&mut repeated_store, &profile, translation)
                .unwrap();
        assert_eq!(repeated.body(), body);
        assert_eq!(repeated.journal(), made.journal());
    }

    let mut coplanar_store = Store::new();
    assert!(make::extrude_profile_along(&mut coplanar_store, &profile, frame.x()).is_err());
    assert_eq!(coplanar_store.count::<Region>(), 0);
}

#[test]
fn rejected_profile_extrusions_are_atomic_and_reuse_future_identity() {
    let polygon = [
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let profile = PlanarProfile::from_polygon(Frame::world(), &polygon).unwrap();
    let mut after_failure = Store::new();
    assert!(make::extrude_profile(&mut after_failure, &profile, -1.0).is_err());
    assert!(make::extrude_profile(&mut after_failure, &profile, f64::NAN).is_err());
    assert_eq!(after_failure.count::<Region>(), 0);

    let made_after = make::extrude_profile_with_journal(&mut after_failure, &profile, 2.0).unwrap();
    let mut control = Store::new();
    let made_control = make::extrude_profile_with_journal(&mut control, &profile, 2.0).unwrap();
    assert_eq!(made_after.body(), made_control.body());
    assert_eq!(made_after.journal(), made_control.journal());
}

fn exact_integer_dot(a: [i64; 3], b: [i64; 3]) -> i128 {
    i128::from(a[0]) * i128::from(b[0])
        + i128::from(a[1]) * i128::from(b[1])
        + i128::from(a[2]) * i128::from(b[2])
}

#[test]
fn rejected_oblique_profile_extrusions_are_atomic_and_reuse_future_identity() {
    let polygon = [
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let profile = PlanarProfile::from_polygon(Frame::world(), &polygon).unwrap();
    let mut after_failure = Store::new();
    for translation in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(f64::NAN, 0.0, 1.0),
    ] {
        assert!(make::extrude_profile_along(&mut after_failure, &profile, translation).is_err());
    }
    assert_eq!(after_failure.count::<Region>(), 0);

    let translation = Vec3::new(0.25, -0.5, 2.0);
    let made_after =
        make::extrude_profile_along_with_journal(&mut after_failure, &profile, translation)
            .unwrap();
    let mut control = Store::new();
    let made_control =
        make::extrude_profile_along_with_journal(&mut control, &profile, translation).unwrap();
    assert_eq!(made_after.body(), made_control.body());
    assert_eq!(made_after.journal(), made_control.journal());
}

#[test]
fn planar_sheet_rejects_invalid_boundaries_without_consuming_identity() {
    let bow_tie = [
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
        Point2::new(1.0, 0.0),
    ];
    let collinear = [
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(2.0, 1.0),
        Point2::new(0.0, 1.0),
    ];
    let mut after_failures = Store::new();
    assert!(make::planar_sheet(&mut after_failures, &Frame::world(), &bow_tie).is_err());
    assert!(make::planar_sheet(&mut after_failures, &Frame::world(), &collinear).is_err());
    assert_eq!(after_failures.count::<Region>(), 0);

    let made_after = make::planar_sheet_with_journal(
        &mut after_failures,
        &Frame::world(),
        &concave_clockwise_polygon(),
    )
    .unwrap();
    let mut fresh = Store::new();
    let made_fresh =
        make::planar_sheet_with_journal(&mut fresh, &Frame::world(), &concave_clockwise_polygon())
            .unwrap();
    assert_eq!(made_after.body(), made_fresh.body());
    assert_eq!(made_after.journal(), made_fresh.journal());
}

#[test]
fn wire_polyline_builds_open_and_closed_checked_chains() {
    let points = [
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.5),
        Point3::new(1.0, 0.0, 0.0),
    ];
    for (closed, expected_edges) in [(false, 2), (true, 3)] {
        let mut store = Store::new();
        let made = make::wire_polyline_with_journal(&mut store, &points, closed).unwrap();
        let body = made.body();
        assert_eq!(store.get(body).unwrap().kind, BodyKind::Wire);
        assert_eq!(store.count::<Edge>(), expected_edges);
        assert_eq!(store.count::<Vertex>(), 3);
        assert_eq!(store.count::<Fin>(), 0);
        assert_eq!(store.count::<Region>(), 1);
        assert_eq!(
            check_body_report(&store, body, CheckLevel::Fast)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        );
        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Indeterminate);
        assert_eq!(full.gaps.len(), 1);
        assert_eq!(full.gaps[0].kind, VerificationGapKind::WireSelfIntersection);
        assert!(made.journal().lineage().is_empty());
    }
}

#[test]
fn wire_and_acorn_validation_is_failure_atomic() {
    let mut store = Store::new();
    let repeated = [Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)];
    assert!(make::wire_polyline(&mut store, &repeated, false).is_err());
    assert!(make::acorn(&mut store, Point3::new(501.0, 0.0, 0.0)).is_err());
    assert_eq!(store.count::<Region>(), 0);

    let position = Point3::new(0.25, -0.5, 1.5);
    let made = make::acorn_with_journal(&mut store, position).unwrap();
    let body = made.body();
    assert_eq!(store.get(body).unwrap().kind, BodyKind::Acorn);
    assert_eq!(store.count::<Vertex>(), 1);
    assert_eq!(store.count::<Edge>(), 0);
    assert_eq!(store.count::<Face>(), 0);
    assert_eq!(
        store
            .vertex_position(store.vertices_of_body(body).unwrap()[0])
            .unwrap(),
        position
    );
    assert_eq!(
        check_body_report(&store, body, CheckLevel::Full)
            .unwrap()
            .outcome(),
        CheckOutcome::Valid
    );
    assert!(
        made.journal()
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
}
