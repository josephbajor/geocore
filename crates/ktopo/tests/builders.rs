#![allow(
    deprecated,
    reason = "compatibility coverage retains the deprecated v1 tessellation wrapper"
)]

//! Checked builders for non-solid topology used by profiles and interchange.

use kgeom::frame::Frame;
use kgeom::vec::{Point2, Point3};
use ktopo::btess::{TessOptions, tessellate_body};
use ktopo::check::{CheckLevel, CheckOutcome, VerificationGapKind, check_body_report};
use ktopo::entity::{BodyKind, Edge, Face, Fin, Region, Shell, Vertex};
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
